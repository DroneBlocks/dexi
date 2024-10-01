import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray

class AprilTagSubscriber(Node):
    
    def __init__(self):
        super().__init__('april_tag_subscriber')
        self.tags = AprilTagDetectionArray()
        self.tag_sub = self.create_subscription(AprilTagDetectionArray, '/detections', self.april_tag_callback, 10)
        # Slow it down
        # self.tag_sub = self.create_subscription(AprilTagDetectionArray, '/throttled/detections', self.april_tag_callback, 10)
        self.tags_detected = False
        self.current_tags = []
        
    def april_tag_callback(self, msg):
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
                self.get_logger().info('detected tag with id=0')
            elif tag.id == 1:
                self.get_logger().info('detected tag with id=1')

def main(args=None):
    rclpy.init(args=args)
    april_tag_subscriber = AprilTagSubscriber()
    rclpy.spin(april_tag_subscriber)
    april_tag_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    