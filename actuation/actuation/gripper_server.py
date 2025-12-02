#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import serial
import time

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('grip_delay', 0.5)
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.grip_delay = self.get_parameter('grip_delay').value
        
        # Connect to Arduino
        try:
            self.serial = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2)  # Wait for Arduino reset
            
            if self.serial.in_waiting:
                ready_msg = self.serial.readline().decode().strip()
                self.get_logger().info(f'Arduino: {ready_msg}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            raise
        
        # Initialize to open position
        self.open_gripper()
        
        # Create service
        self.gripper_service = self.create_service(
            SetBool,
            '/gripper/control',
            self.gripper_service_callback
        )
        
        self.get_logger().info('=' * 40)
        self.get_logger().info('Gripper Controller Initialized')
        self.get_logger().info(f'Serial Port: {self.serial_port}')
        self.get_logger().info('=' * 40)
    
    def send_command(self, cmd):
        """Send command to Arduino."""
        try:
            self.serial.write(cmd.encode())
            self.serial.flush()
            time.sleep(0.1)
            
            if self.serial.in_waiting:
                response = self.serial.readline().decode().strip()
                self.get_logger().info(f'Arduino: {response}')
                
        except Exception as e:
            self.get_logger().error(f'Serial error: {e}')
    
    def open_gripper(self):
        """Open gripper."""
        self.get_logger().info('Opening gripper...')
        self.send_command('O')
        time.sleep(self.grip_delay)
        self.get_logger().info('Gripper opened!')
    
    def close_gripper(self):
        """Close gripper."""
        self.get_logger().info('Closing gripper...')
        self.send_command('C')
        time.sleep(self.grip_delay)
        self.get_logger().info('Gripper closed!')
    
    def gripper_service_callback(self, request, response):
        """Main gripper control service."""
        try:
            if request.data:
                self.close_gripper()
                response.message = 'Gripper closed'
            else:
                self.open_gripper()
                response.message = 'Gripper opened'
            
            response.success = True
            
        except Exception as e:
            self.get_logger().error(f'Gripper control failed: {e}')
            response.success = False
            response.message = f'Error: {str(e)}'
        
        return response
    
    def shutdown(self):
        """Clean shutdown."""
        self.get_logger().info('Shutting down...')
    
        # Close serial connection
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()

def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
