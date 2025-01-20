import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from std_msgs.msg import Int32
import pigpio

# Define constants for the servo
PWM_MIN = 1000  # Minimum valid PWM signal
PWM_MAX = 2000  # Maximum valid PWM signal

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        # Declare and get the servo pin parameter with a descriptor
        pin_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description="GPIO pin to which the servo is connected."
        )
        self.declare_parameter('servo_pin', 13, pin_descriptor)
        self.servo_pin = self.get_parameter('servo_pin').value
        
        # Declare a parameter for initial PWM value
        pwm_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description="Initial PWM value for the servo."
        )
        self.declare_parameter('initial_pwm', 1500, pwm_descriptor)
        self.initial_pwm = self.get_parameter('initial_pwm').value

        # Initialize pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error(
                "Failed to connect to pigpio daemon. Start it with 'sudo pigpiod'."
            )
            exit(1)
        
        self.get_logger().info(f'Connected to pigpio daemon. Using GPIO pin {self.servo_pin} for servo control.')

        # Set the initial PWM value
        self.set_pwm(self.servo_pin, self.initial_pwm)
        
        # Create a subscription to the PWM topic
        self.subscription = self.create_subscription(
            Int32,
            '/servo_pwm',
            self.servo_callback,
            10
        )
        
        # Add a parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.get_logger().info('Servo controller node initialized. Listening on /servo_pwm.')

    def servo_callback(self, msg):
        pwm_value = msg.data

        if PWM_MIN <= pwm_value <= PWM_MAX:
            self.set_pwm(self.servo_pin, pwm_value)
            self.get_logger().info(f"Set servo PWM to {pwm_value}μs on pin {self.servo_pin}.")
        else:
            self.get_logger().warn(
                f"Received invalid PWM value: {pwm_value}. Must be between {PWM_MIN} and {PWM_MAX}."
            )
    
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'servo_pin' and param.type == ParameterType.PARAMETER_INTEGER:
                new_pin = param.value
                if new_pin != self.servo_pin:
                    self.get_logger().info(f"Changing GPIO pin from {self.servo_pin} to {new_pin}.")
                    self.set_pwm(self.servo_pin, 0)  # Stop PWM on the old pin
                    self.servo_pin = new_pin
            elif param.name == 'initial_pwm' and param.type == ParameterType.PARAMETER_INTEGER:
                if PWM_MIN <= param.value <= PWM_MAX:
                    self.initial_pwm = param.value
                    self.set_pwm(self.servo_pin, self.initial_pwm)
                    self.get_logger().info(f"Initial PWM set to {self.initial_pwm}μs.")
                else:
                    self.get_logger().warn(
                        f"Invalid initial PWM value: {param.value}. Must be between {PWM_MIN} and {PWM_MAX}."
                    )
        return rclpy.parameter.ParameterEvent()

    def set_pwm(self, pin, pwm_value):
        """Sets the PWM signal on a specified pin."""
        self.pi.set_servo_pulsewidth(pin, pwm_value)
    
    def destroy_node(self):
        # Stop the servo and pigpio on shutdown
        self.set_pwm(self.servo_pin, 0)
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
