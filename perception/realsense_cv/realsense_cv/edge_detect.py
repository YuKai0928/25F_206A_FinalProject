#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class CannyEdgeNode(Node):
    def __init__(self):
        super().__init__('canny_edge_node')

        # Parameters
        self.declare_parameter('input_image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('output_image_topic', '/camera/edges_overlay')
        self.declare_parameter('threshold1', 50.0)
        self.declare_parameter('threshold2', 150.0)
        
        input_topic = self.get_parameter('input_image_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_image_topic').get_parameter_value().string_value

        self.bridge = CvBridge()

        # Subscriber to original camera image
        self.image_sub = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )
        # Publisher for edge-overlay image
        self.edge_pub = self.create_publisher(Image, output_topic, 10)

        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Publishing edge overlay on: {output_topic}')

    def image_callback(self, msg: Image):
        self.get_logger().info(f'get image.')
        try:
            # Convert ROS Image message to OpenCV image (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # Convert to grayscale
        # gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Read thresholds from parameters
        t1 = self.get_parameter('threshold1').get_parameter_value().double_value
        t2 = self.get_parameter('threshold2').get_parameter_value().double_value
        # Yellow
        lower_yellow = (20, 80, 80)
        upper_yellow = (35, 255, 255)
        # Orange
        lower_orange = (5, 80, 80)
        upper_orange = (20, 255, 255)

        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
        mask = cv2.bitwise_or(mask_yellow, mask_orange)

        # Clean up the mask
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        # --- Canny on masked grayscale ---
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray_masked = cv2.bitwise_and(gray, gray, mask=mask)

        # Canny edge detection -> single-channel binary image
        blurred = cv2.GaussianBlur(gray_masked, (5, 5), 0)

        edges = cv2.Canny(blurred, t1, t2, L2gradient=True)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=1)

        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        # edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=1)

        # Create a color overlay: draw edges in red on original image
        overlay = cv_image.copy()
        # Where edges are non-zero, color those pixels red [B,G,R] = [0, 0, 255]
        overlay[edges != 0] = [0, 0, 255]

        # Convert overlay image (bgr8) back to ROS Image
        try:
            edge_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # Keep the same header (time/frame)
        edge_msg.header = msg.header

        # Publish
        self.edge_pub.publish(edge_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CannyEdgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()