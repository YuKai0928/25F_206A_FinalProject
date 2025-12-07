#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np


class HSVTunerNode(Node):
    def __init__(self):
        super().__init__('hsv_tuner_node')

        # Parameters (reuse same style as your Canny node)
        self.declare_parameter('input_image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('output_image_topic', '/camera/hsv_tuner_debug')

        # One HSV range that you will tune to include yellow + orange
        self.declare_parameter('lower_h', 3)
        self.declare_parameter('lower_s', 130)
        self.declare_parameter('lower_v', 110)
        self.declare_parameter('upper_h', 15)
        self.declare_parameter('upper_s', 255)
        self.declare_parameter('upper_v', 255)

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

        # Publisher for debug HSV image
        self.image_pub = self.create_publisher(Image, output_topic, 10)

        self.get_logger().info(f'HSV tuner subscribing to: {input_topic}')
        self.get_logger().info(f'Publishing HSV debug image on: {output_topic}')

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # Read HSV params (can be changed at runtime with ros2 param set)
        lh = self.get_parameter('lower_h').get_parameter_value().integer_value
        ls = self.get_parameter('lower_s').get_parameter_value().integer_value
        lv = self.get_parameter('lower_v').get_parameter_value().integer_value
        uh = self.get_parameter('upper_h').get_parameter_value().integer_value
        us = self.get_parameter('upper_s').get_parameter_value().integer_value
        uv = self.get_parameter('upper_v').get_parameter_value().integer_value

        lower = np.array([lh, ls, lv], dtype=np.uint8)
        upper = np.array([uh, us, uv], dtype=np.uint8)

        # Convert to HSV and apply mask
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        # --- 2) Highlight mask: "white-ish" pixels ---
        # # S low, V high
        # lower_highlight = np.array([0,   0,   190], dtype=np.uint8)
        # upper_highlight = np.array([179, 40,  255], dtype=np.uint8)
        # mask_highlight = cv2.inRange(hsv, lower_highlight, upper_highlight)

        # # --- 3) Only keep highlights near the color region ---
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        # # Grow the color mask a bit
        # dilated_color = cv2.dilate(mask, kernel, iterations=1)

        # Keep only highlight pixels that overlap with the grown color area
        # mask_highlight_near_color = cv2.bitwise_and(mask_highlight, dilated_color)

        # --- 4) Combine: color + highlights-near-color ---
        # final_mask = cv2.bitwise_or(mask, mask_highlight_near_color)


        # Optional: a bit of cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7,7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        # Apply mask to original image
        result = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # Build a debug visualization:
        # left: original, middle: masked result, right: binary mask
        try:
            h, w = cv_image.shape[:2]

            # Resize to make the side-by-side image manageable (optional)
            scale = 0.5  # adjust or remove if you prefer full size
            new_w, new_h = int(w * scale), int(h * scale)

            orig_small = cv2.resize(cv_image, (new_w, new_h))
            result_small = cv2.resize(result, (new_w, new_h))
            mask_small = cv2.resize(mask, (new_w, new_h))
            mask_small_bgr = cv2.cvtColor(mask_small, cv2.COLOR_GRAY2BGR)

            debug_image = np.hstack((orig_small, result_small, mask_small_bgr))
        except Exception as e:
            self.get_logger().warn(f"Resize/stack failed: {e}. Sending mask overlay only.")
            # Fallback: simple overlay (highlight masked pixels in yellow)
            debug_image = cv_image.copy()
            debug_image[mask != 0] = [0, 255, 255]

        # Convert back to ROS Image message
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        debug_msg.header = msg.header
        self.image_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HSVTunerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()