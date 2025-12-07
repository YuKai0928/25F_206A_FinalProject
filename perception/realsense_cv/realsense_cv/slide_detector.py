#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_ros import TransformException
import time


class SlideDetector(Node):
    """
    Helper node to detect slides from TF frames.
    Monitors slide_XX frames and returns poses in base frame.
    Tracks which slides have been picked to avoid duplicates.
    """
    
    def __init__(self):
        super().__init__('slide_detector')
        
        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Track which slides we've already picked
        self.picked_slides = set()
        
        # Configuration
        self.declare_parameter('slide_indices', [5, 10, 15, 20])
        self.slide_indices = self.get_parameter('slide_indices').value
        
        self.get_logger().info('Slide Detector initialized')
        self.get_logger().info(f'Monitoring slide indices: {self.slide_indices}')
    
    def wait_for_slide(self, timeout=5.0):
        """
        Wait at current position for slide TF frame to appear.
        Scans for slide frames and returns first unpicked slide found.
        
        Args:
            timeout: Maximum time to wait for slide detection (seconds)
            
        Returns:
            PoseStamped in base frame, or None if no slide found
        """
        start_time = time.time()
        
        self.get_logger().info('Scanning for slides...')
        
        while (time.time() - start_time) < timeout:
            for idx in self.slide_indices:
                # Skip already picked slides
                if idx in self.picked_slides:
                    continue
                
                slide_frame = f"slide_{idx:02d}"
                
                try:
                    # Lookup transform from base to slide frame
                    transform = self.tf_buffer.lookup_transform(
                        'base',
                        slide_frame,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    
                    # Convert to PoseStamped
                    pose = PoseStamped()
                    pose.header.frame_id = 'base'
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.position.x = transform.transform.translation.x
                    pose.pose.position.y = transform.transform.translation.y
                    pose.pose.position.z = transform.transform.translation.z
                    pose.pose.orientation = transform.transform.rotation
                    
                    self.get_logger().info(f'Found slide_{idx:02d} at ({pose.pose.position.x:.3f}, '
                                         f'{pose.pose.position.y:.3f}, {pose.pose.position.z:.3f})')
                    
                    # Mark as picked
                    self.picked_slides.add(idx)
                    
                    return pose
                    
                except TransformException:
                    # Frame not available, continue scanning
                    pass
            
            # Brief pause before next scan cycle
            time.sleep(0.1)
        
        self.get_logger().info('No more slides detected')
        return None
    
    def reset_picked_slides(self):
        """Reset the picked slides set (for testing/restarting)"""
        self.picked_slides.clear()
        self.get_logger().info('Picked slides reset')


def main(args=None):
    rclpy.init(args=args)
    node = SlideDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

