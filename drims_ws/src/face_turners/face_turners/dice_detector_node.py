#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from drims2_msgs.srv import DiceIdentification
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros

from face_turners.core import utils
from face_turners.core.dice_detector import DiceDetector

import cv2 as cv
import numpy as np

from collections import deque


class DiceDetectorNode(Node):
    def __init__(self):
        super().__init__('dice_detector')
        
        # Input/Output topics
        self.declare_parameter('img_topic', '/color/video/image')
        self.declare_parameter('dice_detection_mask_topic', '/dice_detection_mask')
        self.declare_parameter('dice_detection_img_topic', '/dice_detection_img')
        self.declare_parameter('dice_detection_srv_topic', '/dice_detection_pose')
        input_img_topic = self.get_parameter('img_topic').get_parameter_value().string_value
        dice_detection_mask_topic = self.get_parameter('dice_detection_mask_topic').get_parameter_value().string_value
        dice_detection_img_topic = self.get_parameter('dice_detection_img_topic').get_parameter_value().string_value
        dice_detection_srv_topic = self.get_parameter('dice_detection_srv_topic').get_parameter_value().string_value

        # Camera parameters
        self.declare_parameter('die_size', 0.03)
        self.declare_parameter('base_frame', 'checkerboard')
        self.declare_parameter('K', [1578.55692, 0.0, 952.440456, 0.0, 1582.46225, 545.655012, 0.0, 0.0, 1.0])
        self.declare_parameter('d', [0.07206577, 0.08106335, 0.00300317, 0.00042163, -0.40383728])
        self.declare_parameter('T_c2w', [0.998855, -0.019105, 0.043869, -0.236381, 0.020615, 0.999201, -0.034238, -0.214228, -0.043180, 0.035103, 0.998450, 0.742136, 0.0, 0.0, 0.0, 1.0])
        die_size = self.get_parameter('die_size').get_parameter_value().double_value
        K = np.array(self.get_parameter('K').get_parameter_value().double_array_value).reshape((3, 3))
        d = np.array(self.get_parameter('d').get_parameter_value().double_array_value).reshape((5,))
        self.T_c2w = np.array(self.get_parameter('T_c2w').get_parameter_value().double_array_value).reshape((4, 4))
        self.base_frame_ = self.get_parameter('base_frame').get_parameter_value().string_value

        # Color detection parameters
        self.declare_parameter('hue_range', (20, 40)) # Yellow color range in HSV
        self.declare_parameter('saturation_range', (130, 255))
        self.declare_parameter('value_range', (150, 255))
        hue_range = self.get_parameter('hue_range').get_parameter_value().integer_array_value
        saturation_range = self.get_parameter('saturation_range').get_parameter_value().integer_array_value
        value_range = self.get_parameter('value_range').get_parameter_value().integer_array_value
        lower_hsv = np.array([hue_range[0], saturation_range[0], value_range[0]])
        upper_hsv = np.array([hue_range[1], saturation_range[1], value_range[1]])

        # Parameters for the blobs/dice points detector
        self.declare_parameter('min_point_area', 50)
        self.declare_parameter('max_point_area', 400)
        self.declare_parameter('min_inertia_ratio', 0.7)
        self.declare_parameter('min_circularity', 0.85)
        self.declare_parameter('dbscan_eps', 80.0)
        min_point_area = self.get_parameter('min_point_area').get_parameter_value().integer_value
        max_point_area = self.get_parameter('max_point_area').get_parameter_value().integer_value
        min_inertia_ratio = self.get_parameter('min_inertia_ratio').get_parameter_value().double_value
        min_circularity = self.get_parameter('min_circularity').get_parameter_value().double_value
        dbscan_eps = self.get_parameter('dbscan_eps').get_parameter_value().double_value

        params = cv.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = min_point_area
        params.maxArea = max_point_area
        params.filterByInertia = True
        params.minInertiaRatio = min_inertia_ratio
        params.filterByCircularity = True
        params.minCircularity = min_circularity

        # Initialize the DiceDetector class
        self.dice_detector_ = DiceDetector(
            blob_detector=params,
            dbscan_eps=dbscan_eps,
            lower_hsv=lower_hsv,
            upper_hsv=upper_hsv,
            K=K,
            d=d,
            T_c2w=self.T_c2w,
            die_size=die_size
        )

        self.cv_bridge_ = CvBridge()
        self.frames_stack_ = deque(maxlen=1)
        self.create_subscription(Image, input_img_topic, lambda msg: self.frames_stack_.append(msg), 10) # Avoid processing inside subscribers callbacks

        # Publisher and timer for the dice detection
        self.dice_imgs_pub_ = self.create_publisher(Image, dice_detection_img_topic, 10)
        self.dice_masks_pub_ = self.create_publisher(Image, dice_detection_mask_topic, 10)
        self.create_timer(0.01, self.detec_dice)

        # Create a service for the on-demand pose-detection
        self.last_dice_detections_ = deque(maxlen=1)
        self.dice_detection_srv_ = self.create_service(DiceIdentification, dice_detection_srv_topic, self.dice_identification_cb)

        # Create a TF broadcaster to publish the dice poses
        self.tf_broadcaster_ = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("Dice Detector Node Initialized")

    def detec_dice(self):
        try:
            img_frame = self.frames_stack_.pop()
        except IndexError:
            return
        img = self.cv_bridge_.imgmsg_to_cv2(img_frame, desired_encoding='bgr8')

        dice, dice_imgs, dice_masks = self.dice_detector_.get_dice_from_blobs(img)

        # Publish the detected dice detections
        for i, (die, die_img, die_mask) in enumerate(zip(dice, dice_imgs, dice_masks)):
            self.dice_imgs_pub_.publish(self.cv_bridge_.cv2_to_imgmsg(die_img, encoding='bgr8'))
            self.dice_masks_pub_.publish(self.cv_bridge_.cv2_to_imgmsg(die_mask, encoding='mono8'))

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.base_frame_
            t.child_frame_id = f'die_{i}'
            t.transform = utils.T_to_transform(self.T_c2w @ die.T_d2c)

            self.tf_broadcaster_.sendTransform(t)

        # Save the last detection for the service
        self.last_dice_detections_.append(dice)

    def dice_identification_cb(self, request, response):
        if not self.last_dice_detections_:
            response.success = False
            return response

        die = self.last_dice_detections_[-1][-1]

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.base_frame_
        pose.pose = utils.T_to_pose(self.T_c2w @ die.T_d2c)

        response.face_number = die.face_number
        response.pose = pose
        response.success = True

        self.get_logger().info("Service called -> returning static dice info")
        return response
    

def main(args=None):
    rclpy.init(args=args)
    node = DiceDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
