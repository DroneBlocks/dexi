import rclpy
import time
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from rpi_ws281x import PixelStrip, Color

# LED strip configuration:
LED_COUNT = 30        # Number of LED pixels.
LED_PIN = 21          # GPIO pin connected to the pixels (18 uses PWM!).
LED_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA = 10          # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255  # Set to 0 for darkest and 255 for brightest
LED_INVERT = False    # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53

class AprilTagDetector(Node):
    
    def __init__(self):
        super().__init__('april_tag_detector')
        self.tags = AprilTagDetectionArray()
        self.tag_sub = self.create_subscription(AprilTagDetectionArray, '/detections', self.detected_tag_cb, 1)
        self.tags_detected = False
        self.current_tags = []
        
        # LED strip setup
        self.strip = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
        self.strip.begin()
        self.clear_leds()

    def detected_tag_cb(self, msg):
        tag_count = len(msg.detections)
        if tag_count > 0 and not self.tags_detected:
            self.current_tags = msg.detections
            self.tags_detected = True
            self.process_tags()
        elif tag_count == 0:
            self.tags_detected = False

    def process_tags(self):
        for tag in self.current_tags:
            if tag.id == 0:
                self.color_wipe(Color(255, 0, 0))
            elif tag.id == 1:
                self.color_wipe(Color(0, 255, 0))

    def color_wipe(self, color, wait_ms=25):
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, color)
            self.strip.show()
            time.sleep(wait_ms / 1000.0)

    def clear_leds(self):
        self.color_wipe(Color(0, 0, 0), 10)

def main(args=None):
    rclpy.init(args=args)
    detector = AprilTagDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    