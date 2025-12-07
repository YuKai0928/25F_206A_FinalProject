#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import cv2
import numpy as np
import torch


class YOLO5BoxTFBroadcaster(Node):
    def __init__(self):
        super().__init__("yolo5_box_tf_broadcaster")

        # Parameters
        self.declare_parameter("input_image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("model_path", "box_detector_yolov5n.pt")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("box_frame", "detected_box")
        self.declare_parameter("box_width_m", 0.14)
        self.declare_parameter("output_image_topic", "/yolo/annotated")

        img_topic = self.get_parameter("input_image_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        model_path = self.get_parameter("model_path").value

        self.camera_frame = self.get_parameter("camera_frame").value
        self.box_frame = self.get_parameter("box_frame").value
        self.box_width_m = float(self.get_parameter("box_width_m").value)

        # Output image publisher
        self.image_pub = self.create_publisher(Image, 
            self.get_parameter("output_image_topic").value, 10)

        self.bridge = CvBridge()

        # Camera intrinsics
        self.camera_matrix = None
        self.dist_coeffs = None

        # TF broadcaster
        self.br = TransformBroadcaster(self)

        # Load YOLOv5 (Jetson Nano safe)
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Loading YOLOv5 model on {self.device}")

        self.model = torch.hub.load(
            "ultralytics/yolov5",
            "custom",
            path=model_path,
            force_reload=True
        ).to(self.device)

        # Subscribers
        self.create_subscription(Image, img_topic, self.image_callback, 10)
        self.create_subscription(CameraInfo, info_topic, self.camera_info_callback, 10)

        self.last_warn_time = self.get_clock().now()

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera intrinsics received")

    def warn_throttled(self, message, seconds=5.0):
        now = self.get_clock().now()
        if (now - self.last_warn_time).nanoseconds > seconds * 1e9:
            self.get_logger().warn(message)
            self.last_warn_time = now

    def estimate_depth(self, box_width_px):
        fx = self.camera_matrix[0, 0]
        Z = fx * self.box_width_m / box_width_px
        return Z

    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            self.warn_throttled("Waiting for CameraInfo before YOLO detection.")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        # YOLOv5 inference
        results = self.model(frame)
        detections = results.xyxy[0].cpu().numpy()

        if len(detections) == 0:
            return

        candidates = []

        for x1, y1, x2, y2, conf, cls in detections:
            name = self.model.names[int(cls)]
            if name not in ["orange_box", "yellow_box"]:
                continue

            area = (x2 - x1) * (y2 - y1)
            candidates.append((area, x1, y1, x2, y2, name))

        if not candidates:
            return

        _, x1, y1, x2, y2, name = max(candidates, key=lambda x: x[0])
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        box_w_px = (x2 - x1)

        # Depth
        Z = self.estimate_depth(box_w_px)

        # Camera coordinate conversion
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx_cam = self.camera_matrix[0, 2]
        cy_cam = self.camera_matrix[1, 2]

        X = (cx - cx_cam) * Z / fx
        Y = (cy - cy_cam) * Z / fy

        # TF output
        tfmsg = TransformStamped()
        tfmsg.header.stamp = msg.header.stamp
        tfmsg.header.frame_id = self.camera_frame
        tfmsg.child_frame_id = self.box_frame

        tfmsg.transform.translation.x = float(X)
        tfmsg.transform.translation.y = float(Y)
        tfmsg.transform.translation.z = float(Z)

        tfmsg.transform.rotation.x = 0.0
        tfmsg.transform.rotation.y = 0.0
        tfmsg.transform.rotation.z = 0.0
        tfmsg.transform.rotation.w = 1.0

        self.br.sendTransform(tfmsg)

        # Draw bounding box
        frame_out = frame.copy()
        cv2.rectangle(frame_out, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(frame_out, f"{name} {Z:.2f}m",
                    (int(x1), int(y1)-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

        # Publish annotated image
        img_msg = self.bridge.cv2_to_imgmsg(frame_out, "bgr8")
        img_msg.header = msg.header
        self.image_pub.publish(img_msg)


def main():
    rclpy.init()
    node = YOLO5BoxTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()