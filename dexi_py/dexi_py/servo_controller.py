import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import pigpio

# Define constants for the servo
PWM_MIN = 1000  # Minimum valid PWM signal
PWM_MAX = 2000  # Maximum valid PWM signal

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        # Declare and get the servo pin parameter
        self.declare_parameter('servo_pin', 13)
        self.servo_pin = self.get_parameter('servo_pin').value
        
        # Initialize pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error(
                "Failed to connect to pigpio daemon. Start it with 'sudo pigpiod'."
            )
            exit(1)
        
        self.get_logger().info(f'Connected to pigpio daemon. Using GPIO pin {self.servo_pin} for servo control.')
        
        # Create a subscription to the PWM topic
        self.subscription = self.create_subscription(
            Int32,
            '/servo_pwm',
            self.servo_callback,
            10
        )
        self.get_logger().info('Servo controller node initialized. Listening on /servo_pwm.')
    
    def servo_callback(self, msg):
        pwm_value = msg.data
        
        if PWM_MIN <= pwm_value <= PWM_MAX:
            self.pi.set_servo_pulsewidth(self.servo_pin, pwm_value)
            self.get_logger().info(f"Set servo PWM to {pwm_value}μs on pin {self.servo_pin}.")
        else:
            self.get_logger().warn(
                f"Received invalid PWM value: {pwm_value}. Must be between {PWM_MIN} and {PWM_MAX}."
            )
    
    def destroy_node(self):
        # Stop the servo and pigpio on shutdown
        self.pi.set_servo_pulsewidth(self.servo_pin, 0)
        self.pi.stop()
        self.get_logger().info("Servo stopped and pigpio connection closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (Ctrl+C) detected. Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
