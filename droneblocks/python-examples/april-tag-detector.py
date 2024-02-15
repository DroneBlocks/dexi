import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray


class AprilTagDetector(Node):
    
    def __init__(self):
        super().__init__('april_tag_detector')
        self.tags = AprilTagDetectionArray()
        self.tag_sub = self.create_subscription(AprilTagDetectionArray, '/detections', self.detected_tag_cb, 1)
        self.tags_detected = False
        self.current_tags = []

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
            print(tag.id)

def main(args=None):
    rclpy.init(args=args)
    detector = AprilTagDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    