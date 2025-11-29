#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
import Jetson.GPIO as GPIO
import time

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        
        self.declare_parameter('servo_pin', 33)
        self.declare_parameter('pwm_frequency', 50)
        self.declare_parameter('open_duty_cycle', 5.0)
        self.declare_parameter('close_duty_cycle', 10.0)
        self.declare_parameter('grip_delay', 0.5)
        
        self.servo_pin = self.get_parameter('servo_pin').value
        self.pwm_freq = self.get_parameter('pwm_frequency').value
        self.open_duty = self.get_parameter('open_duty_cycle').value
        self.close_duty = self.get_parameter('close_duty_cycle').value
        self.grip_delay = self.get_parameter('grip_delay').value
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        
        self.pwm = GPIO.PWM(self.servo_pin, self.pwm_freq)
        self.pwm.start(0)
        
        self.open_gripper()
        
        # Main control service
        self.gripper_service = self.create_service(
            SetBool,
            '/gripper/control',
            self.gripper_service_callback
        )
        
        
        self.get_logger().info('=' * 40)
        self.get_logger().info('Gripper Controller Initialized')
        self.get_logger().info(f'GPIO Pin: {self.servo_pin}')
        self.get_logger().info(f'Open: {self.open_duty}% | Close: {self.close_duty}%')
        self.get_logger().info('IMPORTANT: Close keeps PWM active for holding!')
        self.get_logger().info('=' * 40)
    
    def open_gripper(self):
        """Open gripper (turns off PWM after movement)."""
        self.get_logger().info('Opening gripper...')
        self.pwm.ChangeDutyCycle(self.open_duty)
        time.sleep(self.grip_delay)
        self.pwm.ChangeDutyCycle(0)  # Turn off (no load when open)
        self.get_logger().info('Gripper opened!')
    
    def close_gripper(self):
        """Close gripper (keeps PWM active for holding torque)."""
        self.get_logger().info('Closing gripper...')
        self.pwm.ChangeDutyCycle(self.close_duty)
        time.sleep(self.grip_delay)
        # Don't turn off! Keep active holding torque
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
        self.pwm.ChangeDutyCycle(0)
        self.pwm.stop()
        GPIO.cleanup()

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
