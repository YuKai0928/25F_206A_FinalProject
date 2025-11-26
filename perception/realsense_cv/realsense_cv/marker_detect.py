#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge, CvBridgeError
from tf2_ros import TransformBroadcaster

import cv2
import numpy as np


def rotation_matrix_to_quaternion(R):
    q = np.empty((4,), dtype=np.float64)
    trace = np.trace(R)

    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        q[3] = 0.25 / s
        q[0] = (R[2, 1] - R[1, 2]) * s
        q[1] = (R[0, 2] - R[2, 0]) * s
        q[2] = (R[1, 0] - R[0, 1]) * s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            q[3] = (R[2, 1] - R[1, 2]) / s
            q[0] = 0.25 * s
            q[1] = (R[0, 1] + R[1, 0]) / s
            q[2] = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            q[3] = (R[0, 2] - R[2, 0]) / s
            q[0] = (R[0, 1] + R[1, 0]) / s
            q[1] = 0.25 * s
            q[2] = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            q[3] = (R[1, 0] - R[0, 1]) / s
            q[0] = (R[0, 2] + R[2, 0]) / s
            q[1] = (R[1, 2] + R[2, 1]) / s
            q[2] = 0.25 * s

    return q[0], q[1], q[2], q[3]


class ArucoTFBroadcaster(Node):
    def __init__(self):
        super().__init__('aruco_tf_broadcaster')

        # Parameters
        self.declare_parameter('input_image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('marker_length_m', 0.02)
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('marker_frame', 'aruco_marker')
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('target_id', -1)

        img_topic = self.get_parameter('input_image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value

        self.marker_length = self.get_parameter('marker_length_m').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.marker_frame = self.get_parameter('marker_frame').value
        dict_name = self.get_parameter('aruco_dict').value
        self.target_id = self.get_parameter('target_id').value

        self.bridge = CvBridge()

        self.camera_matrix = None
        self.dist_coeffs = None

        self.br = TransformBroadcaster(self)

        # Subscribers
        self.image_sub = self.create_subscription(Image, img_topic, self.image_callback, 10)
        self.caminfo_sub = self.create_subscription(CameraInfo, info_topic, self.camera_info_callback, 10)

        # Create detector (OpenCV 4.7+)
        self.get_logger().info("Using OpenCV 4.7+ ArUcoDetector API")
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

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

    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            self.warn_throttled("Waiting for CameraInfo before ArUco detection.")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        #
        # NEW API (OpenCV >= 4.7)
        #
        corners, ids, rejected = self.detector.detectMarkers(gray)

        if ids is None:
            return

        # Pose estimate
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_length, self.camera_matrix, self.dist_coeffs
        )

        ids = ids.flatten()

        for i, marker_id in enumerate(ids):
            if self.target_id >= 0 and marker_id != self.target_id:
                continue

            rvec = rvecs[i][0]
            tvec = tvecs[i][0]

            Rmat, _ = cv2.Rodrigues(rvec)
            qx, qy, qz, qw = rotation_matrix_to_quaternion(Rmat)

            # Build TF
            tfmsg = TransformStamped()
            tfmsg.header.stamp = msg.header.stamp
            tfmsg.header.frame_id = self.camera_frame
            tfmsg.child_frame_id = self.marker_frame

            tfmsg.transform.translation.x = float(tvec[0])
            tfmsg.transform.translation.y = float(tvec[1])
            tfmsg.transform.translation.z = float(tvec[2])

            tfmsg.transform.rotation.x = float(qx)
            tfmsg.transform.rotation.y = float(qy)
            tfmsg.transform.rotation.z = float(qz)
            tfmsg.transform.rotation.w = float(qw)

            self.br.sendTransform(tfmsg)
            break


def main():
    rclpy.init()
    node = ArucoTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()