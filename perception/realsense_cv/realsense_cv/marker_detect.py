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

def lock_z_axis(R):
    # Original axes (columns)
    x_orig = R[:, 0]
    y_orig = R[:, 1]
    z_orig = R[:, 2]

    # Choose +Z or -Z to be closest to original z axis
    if z_orig[2] >= 0:
        z_new = np.array([0.0, 0.0, 1.0])
    else:
        z_new = np.array([0.0, 0.0, -1.0])

    # Project original X axis into XY plane
    x_proj = x_orig.copy()
    x_proj[2] = 0.0
    norm_x = np.linalg.norm(x_proj)

    # If X is almost vertical, fall back to projecting Y
    if norm_x < 1e-6:
        y_proj = y_orig.copy()
        y_proj[2] = 0.0
        norm_y = np.linalg.norm(y_proj)
        if norm_y < 1e-6:
            # Degenerate case: both x and y almost vertical, just pick +X
            x_new = np.array([1.0, 0.0, 0.0])
        else:
            x_new = y_proj / norm_y
    else:
        x_new = x_proj / norm_x

    # Y axis = Z × X to make a right-handed orthonormal basis
    y_new = np.cross(z_new, x_new)
    y_new /= np.linalg.norm(y_new)

    R_locked = np.column_stack((x_new, y_new, z_new))
    return R_locked


class ArucoTFBroadcaster(Node):
    def __init__(self):
        super().__init__('aruco_tf_broadcaster')

        # Parameters
        self.declare_parameter('input_image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('marker_length_m', 0.02)
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('box_frame', 'ar_marker_02')
        self.declare_parameter('tray_frame', 'ar_marker_03')

        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('target_id1', 2)
        self.declare_parameter('target_id2', 3)


        img_topic = self.get_parameter('input_image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value

        self.marker_length = self.get_parameter('marker_length_m').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.box_frame = self.get_parameter('box_frame').value
        self.tray_frame = self.get_parameter('tray_frame').value

        dict_name = self.get_parameter('aruco_dict').value
        self.target_id1 = self.get_parameter('target_id1').value
        self.target_id2 = self.get_parameter('target_id2').value

        self.image_pub = self.create_publisher(Image, 'aruco_annotated', 10)


        self.bridge = CvBridge()

        self.camera_matrix = None
        self.dist_coeffs = None

        self.br = TransformBroadcaster(self)

        # Subscribers
        self.image_sub = self.create_subscription(Image, img_topic, self.image_callback, 10)
        self.caminfo_sub = self.create_subscription(CameraInfo, info_topic, self.camera_info_callback, 10)

        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        self.parameters = cv2.aruco.DetectorParameters_create()
        # self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

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

        # corners, ids, rejected = self.detector.detectMarkers(gray)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)

        if ids is None:
            return

        # Pose estimate
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_length, self.camera_matrix, self.dist_coeffs
        )

        ids = ids.flatten()
        annotated = cv_image.copy()

        for i, marker_id in enumerate(ids):
            self.get_logger().info(f"found id {marker_id}")
            if marker_id != self.target_id1 and marker_id != self.target_id2:
                continue

            rvec = rvecs[i][0]
            tvec = tvecs[i][0]

            Rmat, _ = cv2.Rodrigues(rvec)
            # Force Z axis perpendicular to table
            Rmat = lock_z_axis(Rmat) # lock z axis!

            qx, qy, qz, qw = rotation_matrix_to_quaternion(Rmat)

            # Build TF
            tfmsg = TransformStamped()
            tfmsg.header.stamp = msg.header.stamp
            tfmsg.header.frame_id = self.camera_frame
            if marker_id == self.target_id1:
                tfmsg.child_frame_id = self.box_frame
            else:
                tfmsg.child_frame_id = self.tray_frame
            axis_length = self.marker_length * 0.5  # visible “frame” size

            cv2.drawFrameAxes(
                annotated,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
                axis_length
            )

            tfmsg.transform.translation.x = float(tvec[0])
            tfmsg.transform.translation.y = float(tvec[1])
            tfmsg.transform.translation.z = float(tvec[2])

            tfmsg.transform.rotation.x = float(qx)
            tfmsg.transform.rotation.y = float(qy)
            tfmsg.transform.rotation.z = float(qz)
            tfmsg.transform.rotation.w = float(qw)
            
            self.br.sendTransform(tfmsg)
            for slotInd in range(8,25,10):# generate 4 slide frame for test usage
                self.pseudo_slide_frames(Rmat,tvec,slotInd,msg)

            break
        
    def pseudo_slide_frames(self,RMarker, TMarker, slotInd,msg):
        
        """
        Parent -> Child: (R_pc, t_pc)
        Child  -> New : (R_cn, t_cn)

        Returns Parent -> New: (R_pn, t_pn)
        """
        R_slide_cam = np.array([
            [0,  1, 0],
            [1, 0, 0],
            [0,  0, -1],
        ], dtype=np.float64)
        T_slide_cam = np.array([
            [-0.0528*(25-slotInd)-0.013],
            [0.0387-0.01], # aruco marker center -> short side center
            [0.0],
        ], dtype=np.float64)
        R_pn = RMarker @ R_slide_cam
        t_pn = (RMarker @ T_slide_cam).squeeze(-1) + TMarker
        # print(TMarker.shape,t_pn.shape,T_slide_cam.shape,RMarker.shape,(RMarker @ T_slide_cam).shape)
        # Convert to quaternion
        qx, qy, qz, qw = rotation_matrix_to_quaternion(R_pn)

        # Build and send TF
        tfmsg = TransformStamped()
        tfmsg.header.stamp = msg.header.stamp
        tfmsg.header.frame_id = self.camera_frame
        tfmsg.child_frame_id = f"slide_{slotInd:02d}"

        tfmsg.transform.translation.x = float(t_pn[0])
        tfmsg.transform.translation.y = float(t_pn[1])
        tfmsg.transform.translation.z = float(t_pn[2])

        tfmsg.transform.rotation.x = float(qx)
        tfmsg.transform.rotation.y = float(qy)
        tfmsg.transform.rotation.z = float(qz)
        tfmsg.transform.rotation.w = float(qw)

        self.br.sendTransform(tfmsg)
        return R_pn, t_pn


def main():
    rclpy.init()
    node = ArucoTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()