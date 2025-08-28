#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from face_turners.core.dice_detector import DiceDetector, DieDetection
from face_turners_msgs.msg import DieDetection

import cv2 as cv
import numpy as np

from collections import deque


class DiceDetectorNode(Node):
    def __init__(self):
        super().__init__('dice_detector')
        
        # Input/Output topics
        self.declare_parameter('img_topic', '/color/video/image')
        self.declare_parameter('dice_detection_topic', '/dice_detection')
        input_img_topic = self.get_parameter('img_topic').get_parameter_value().string_value
        dice_detection_topic = self.get_parameter('dice_detection_topic').get_parameter_value().string_value

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
            upper_hsv=upper_hsv
        )

        self.cv_bridge_ = CvBridge()
        self.frames_stack_ = deque(maxlen=1)
        self.create_subscription(Image, input_img_topic, lambda msg: self.frames_stack_.append(msg), 10) # Avoid processing inside subscribers callbacks

        # Publisher and timer for the dice detection
        self.dice_pub_ = self.create_publisher(DieDetection, dice_detection_topic, 10)
        self.create_timer(0.01, self.detec_dice)

        self.get_logger().info("Dice Detector Node Initialized")

    def detec_dice(self):
        try:
            img_frame = self.frames_stack_.pop()
        except IndexError:
            return
        img = self.cv_bridge_.imgmsg_to_cv2(img_frame, desired_encoding='bgr8')

        dice = self.dice_detector_.get_dice_from_blobs(img)

        # Publish the detected dice detections
        stamp = self.get_clock().now().to_msg()
        for i, die in enumerate(dice):
            detection_msg = DieDetection()
            detection_msg.header.frame_id = f"die_{i}"
            detection_msg.header.stamp = stamp
            detection_msg.face_number = die.face_number
            detection_msg.centroid = die.centroid.tolist()
            detection_msg.yaw = float(die.yaw)
            self.dice_pub_.publish(detection_msg)

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
