#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
import cv2
import numpy as np


class CannyEdgeNode(Node):
    def __init__(self):
        super().__init__('canny_edge_node')

        # Parameters
        self.declare_parameter('input_image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('output_image_topic', '/camera/edges_overlay')
        self.declare_parameter('threshold1', 50.0)
        self.declare_parameter('threshold2', 150.0)
        self.declare_parameter('blur_kernel_size', 5)
        self.declare_parameter('blur_sigma', 1.4)

        input_topic = self.get_parameter('input_image_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_image_topic').get_parameter_value().string_value

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

    def ros_image_to_cv2(self, msg: Image):
        """Convert ROS Image message to OpenCV image without using CvBridge"""
        height = msg.height
        width = msg.width
        encoding = msg.encoding

        if encoding == 'bgr8':
            cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
        elif encoding == 'rgb8':
            cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        elif encoding == 'mono8':
            cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width)
        elif encoding == '16UC1':
            cv_image = np.frombuffer(msg.data, dtype=np.uint16).reshape(height, width)
        else:
            self.get_logger().error(f'Unsupported encoding: {encoding}')
            return None

        return cv_image

    def cv2_to_ros_image(self, cv_image, encoding='bgr8'):
        """Convert OpenCV image to ROS Image message without using CvBridge"""
        msg = Image()
        msg.height = cv_image.shape[0]
        msg.width = cv_image.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = cv_image.shape[1] * cv_image.shape[2] if len(cv_image.shape) == 3 else cv_image.shape[1]
        msg.data = cv_image.tobytes()
        return msg
    def fill_mask_holes(self, mask):
        """Fill holes in mask by drawing only outer contours"""
        contours, hierarchy = cv2.findContours(
            mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
        )

        # new empty mask
        filled = np.zeros_like(mask)

        # fill only the outer contours (parent = -1)
        for i, h in enumerate(hierarchy[0]):
            if h[3] == -1:  # h[3] = parent contour index
                cv2.drawContours(filled, contours, i, color=255, thickness=cv2.FILLED)

        return filled
    def image_callback(self, msg: Image):
        # Convert ROS Image message to OpenCV image (BGR)
        cv_image = self.ros_image_to_cv2(msg)
        if cv_image is None:
            return

        # Read parameters
        t1 = self.get_parameter('threshold1').get_parameter_value().double_value
        t2 = self.get_parameter('threshold2').get_parameter_value().double_value
        kernel_size = self.get_parameter('blur_kernel_size').get_parameter_value().integer_value
        sigma = self.get_parameter('blur_sigma').get_parameter_value().double_value

        # 1. Apply Gaussian blur to reduce noise
        blur = cv2.GaussianBlur(cv_image, (kernel_size, kernel_size), sigma)

        # 2. Apply Canny edge detection
        edges = cv2.Canny(blur, t1, t2)

        # 3. Convert edges to BGR for publishing (black and white)
        edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        # Convert to ROS Image
        edge_msg = self.cv2_to_ros_image(edges_bgr, encoding='bgr8')

        # Keep the same header (time/frame)
        edge_msg.header = msg.header

        # Publish
        self.edge_pub.publish(edge_msg)

        self.get_logger().info(f'Published edges (t1={t1}, t2={t2})', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = CannyEdgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()