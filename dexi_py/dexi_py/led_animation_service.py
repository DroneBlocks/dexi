import rclpy
from rclpy.node import Node
from threading import Thread, Event
from std_srvs.srv import SetBool  # Example service message type (you can define your own if needed)
from time import sleep
from dexi_interfaces.srv import LEDPixelColor

class LEDAnimationService(Node):
    def __init__(self):
        super().__init__('led_animation_service')
        self.current_animation = "idle"  # Default animation type
        self.stop_animation = Event()    # Event to stop the animation loop
        self.new_request = Event()       # Event to signal a new animation request
        self.pixel_request = LEDPixelColor.Request()


        self.num_pixels = 44
        self.middle_index = self.num_pixels // 2
        self.fade_steps = 5
        self.fade_delay = 0.002

        # Define ROYGBIV colors (Red, Orange, Yellow, Green, Blue, Indigo, Violet)
        self.colors = [
            (255, 0, 0),    # Red
            (255, 127, 0),  # Orange
            (255, 255, 0),  # Yellow
            (0, 255, 0),    # Green
            (0, 0, 255),    # Blue
            (75, 0, 130),   # Indigo
            (148, 0, 211)   # Violet
        ]
    
        # Create the service
        self.service = self.create_service(SetBool, 'set_animation', self.set_animation_callback)

        # Start the animation thread
        self.animation_thread = Thread(target=self.animation_loop)
        self.animation_thread.start()

    def send_request(self, index, r, g, b):
        self.request.index = index
        self.request.r = r
        self.request.g = g
        self.request.b = b
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def set_animation_callback(self, request, response):
        animation_type = "blink" if request.data else "idle"
        self.get_logger().info(f"Received request to set animation to: {animation_type}")
        self.current_animation = animation_type

        # Signal that there is a new animation request
        self.new_request.set()

        response.success = True
        response.message = f"Animation set to {animation_type}"
        return response

    def animation_loop(self):
        while rclpy.ok():
            if self.current_animation == "idle":
                self.idle_animation()
            elif self.current_animation == "blink":
                self.blink_animation()

            # Reset the new request flag after handling the change
            if self.new_request.is_set():
                self.get_logger().info("Switching animation...")
                self.new_request.clear()

            # Small sleep to prevent tight loop when idle
            sleep(0.1)

    def idle_animation(self):
        self.fade_between_colors
        self.get_logger().info("Running idle animation...")
        # Perform idle animation here
        sleep(0.5)

    def blink_animation(self):
        self.get_logger().info("Running blink animation...")
        # Perform blink animation here
        sleep(0.5)

    def fade_between_colors(self, index, start_color, end_color, steps, delay=0.002):
        while True: 
            for i in range(len(self.colors)):
                start_color =self. colors[i]
                end_color = self.colors[(i + 1) % len(self.colors)]  # Loop to the first color after the last

                self.get_logger().info(f'Starting ultra-fast chase with color fade from {start_color} to {end_color}')

                # Chase from both ends to the middle with an ultra-fast fade effect
                for j in range(self.middle_index):
                    self.fade_between_colors(j, start_color, end_color, self.fade_steps, self.fade_delay)
                    self.fade_between_colors(num_pixels - j - 1, start_color, end_color, fade_steps, fade_delay)

                # Pause briefly after meeting in the middle
                sleep(0.1)

                # Chase off (fade to off) from the middle to the ends with an ultra-fast fade
                for j in range(self.middle_index):
                    self.fade_between_colors(self.middle_index - j - 1, end_color, (0, 0, 0), self.fade_steps, self.fade_delay)
                    self.fade_between_colors(self.middle_index + j, end_color, (0, 0, 0), self.fade_steps, self.fade_delay)

    def destroy(self):
        # Stop the animation thread when shutting down
        self.stop_animation.set()
        self.animation_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LEDAnimationService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
