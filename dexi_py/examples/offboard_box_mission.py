# This requires that the px4_offboard_manager node is running
import rclpy
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from dexi_interfaces.msg import OffboardNavCommand

class OffboardBoxMission(Node):
    def __init__(self):
        super().__init__('offboard_box_mission')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.altitude = 2
        self.distance = 3

        self.offboard_control_publisher = self.create_publisher(OffboardNavCommand, '/dexi/offboard_manager', qos_profile)

        time.sleep(3)
        self.start_offboard_heartbeat()
        time.sleep(1)
        self.arm()
        time.sleep(1)
        self.takeoff()

    def start_offboard_heartbeat(self):
        msg = OffboardNavCommand()
        msg.command = "start_offboard_heartbeat"
        self.offboard_control_publisher.publish(msg)
    
    def arm(self):
        msg = OffboardNavCommand()
        msg.command = "arm"
        self.offboard_control_publisher.publish(msg)

    def takeoff(self):
        msg = OffboardNavCommand()
        msg.command = "takeoff"
        msg.distance_or_degrees = self.altitude
        self.offboard_control_publisher.publish(msg)

    def fly_forward(self):
        msg = OffboardNavCommand()
        msg.command = "fly_forward"
        msg.distance_or_degrees = self.distance
        self.offboard_control_publisher.publish(msg)

    def fly_right(self):
        msg = OffboardNavCommand()
        msg.command = "fly_right"
        msg.distance_or_degrees = self.distance
        self.offboard_control_publisher.publish(msg)

    def fly_backward(self):
        msg = OffboardNavCommand()
        msg.command = "fly_backward"
        msg.distance_or_degrees = self.distance
        self.offboard_control_publisher.publish(msg)

    def fly_left(self):
        msg = OffboardNavCommand()
        msg.command = "fly_left"
        msg.distance_or_degrees = self.distance
        self.offboard_control_publisher.publish(msg)

    def stop_offboard_heartbeat(self):
        msg = OffboardNavCommand()
        msg.command = "stop_offboard_heartbeat"
        self.offboard_control_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OffboardBoxMission()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
