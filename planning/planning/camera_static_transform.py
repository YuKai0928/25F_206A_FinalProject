#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation

import numpy as np

class ConstantTransformPublisher(Node):
    def __init__(self):
        super().__init__('constant_tf_publisher')
        self.br = StaticTransformBroadcaster(self)

        # Homogeneous transform G_flange->camera_link
        G = np.array([
            [0, -1, 0, -0.018],
            [ 0, 0, -1, -0.06],
            [ 1, 0, 0, 0.073],
            [ 0, 0, 0, 1.0]
        ])

        # Create TransformStamped
        self.transform = TransformStamped()
        # ---------------------------
        # Fill out TransformStamped message
        # --------------------------
        
        self.transform.header.frame_id = 'flange'
        self.transform.child_frame_id = 'camera_link'

        self.transform.transform.translation.x = G[0,3]
        self.transform.transform.translation.y = G[1,3]
        self.transform.transform.translation.z = G[2,3]

        q = Rotation.from_matrix(G[:3,:3]).as_quat()

        self.transform.transform.rotation.x = q[0]
        self.transform.transform.rotation.y = q[1]
        self.transform.transform.rotation.z = q[2]
        self.transform.transform.rotation.w = q[3]


        self.timer = self.create_timer(0.05, self.broadcast_tf)

    def broadcast_tf(self):
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.transform)

def main():
    rclpy.init()
    node = ConstantTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
