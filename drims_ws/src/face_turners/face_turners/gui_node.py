#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from face_turners_msgs.msg import DieDetection

import cv2 as cv

from collections import deque


class DetectionGUI(Node):
    def __init__(self):
        super().__init__('detection_gui')

        self.declare_parameter("window_name", "Detection GUI")
        self.win_name_ = self.get_parameter("window_name").get_parameter_value().string_value

        self.declare_parameter('img_topic', '/color/video/image')
        self.declare_parameter('dice_detection_topic', '/dice_detection')
        input_img_topic = self.get_parameter('img_topic').get_parameter_value().string_value
        dice_detection_topic = self.get_parameter('dice_detection_topic').get_parameter_value().string_value

        # GUI Window
        cv.namedWindow(self.win_name_, cv.WINDOW_NORMAL)

        self.img_stack_ = deque(maxlen=1)
        self.img_sub_ = self.create_subscription(Image, input_img_topic, lambda msg: self.img_stack_.append(msg), 10)

        self.die_detection_stack_ = deque(maxlen=1)
        self.die_detection_sub_ = self.create_subscription(DieDetection, dice_detection_topic, lambda msg: self.die_detection_stack_.append(msg), 10)

        self.get_logger().info("GUI node initialized")

    def update_gui(self, max_time_slop: float = 0.05):
        try:
            img_frame = self.img_stack_.pop()
        except IndexError:
            return
        
        try:
            die_detection = self.die_detection_stack_.pop()
        except IndexError:
            die_detection = None

        img = CvBridge().imgmsg_to_cv2(img_frame, desired_encoding='bgr8')

        if die_detection is None:
            # If no die detection is available, just show the image
            cv.imshow(self.win_name_, img)
            cv.waitKey(1)
            return
        
        # If timestamp difference between two msgs is close enough draw the detection
        img_timestamp = img_frame.header.stamp
        img_timestamp = img_timestamp.sec + img_timestamp.nanosec * 1e-9

        die_timestamp = die_detection.header.stamp
        die_timestamp = die_timestamp.sec + die_timestamp.nanosec * 1e-9
        if (img_timestamp - die_timestamp) < max_time_slop:
            die_face_number = str(die_detection.face_number)
            die_centroid = die_detection.centroid

            textsize = cv.getTextSize(die_face_number, cv.FONT_HERSHEY_PLAIN, 3, 2)[0]
            img = cv.putText(
                img, 
                die_face_number,
                (int(die_centroid[0] - textsize[0] / 2), int(die_centroid[1] + textsize[1] / 2)),
                cv.FONT_HERSHEY_PLAIN, 3, (0, 255, 0), 2
            )

        cv.imshow(self.win_name_, img)
        cv.waitKey(1)


def main():
    rclpy.init()
    gui_node = DetectionGUI()

    while rclpy.ok():
        try:
            # Keep OpenCv window in the main thread
            gui_node.update_gui()
            rclpy.spin_once(gui_node, timeout_sec=0)
        except KeyboardInterrupt:
            break

    gui_node.destroy_node()
    rclpy.shutdown()
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()