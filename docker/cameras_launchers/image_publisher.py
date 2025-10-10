import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class StaticImagePublisher(Node):
    def __init__(self):
        super().__init__('static_image_publisher')
        # Create a publisher for sensor_msgs/Image messages on the 'image_topic' topic
        self.publisher_ = self.create_publisher(Image, 'image_topic', 10)
        # Bridge to convert OpenCV images to ROS Image messages
        self.bridge = CvBridge()
        # Timer to call the callback every second
        self.timer = self.create_timer(1.0, self.timer_callback) # Publishes every second
        self.get_logger().info('Static image publisher node started.')

    def timer_callback(self):
        # Path to the image file to publish (update as needed)
        img_path = 'bags/Image1.jpeg'
        # Read the image using OpenCV
        cv_image = cv2.imread(img_path)
        if cv_image is not None:
            # Convert OpenCV image to ROS Image message and publish
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.publisher_.publish(ros_image)
            self.get_logger().info('Published image on /image_topic')
        else:
            # Log an error if the image could not be read
            self.get_logger().error(f'Could not read image from {img_path}')

def main(args=None):
    rclpy.init(args=args)
    static_image_publisher = StaticImagePublisher()
    # Spin once to allow the timer callback to execute, then exit
    rclpy.spin_once(static_image_publisher)  # Publishes the image once and exits
    static_image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()