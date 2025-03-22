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

        self.altitude = 1.5
        self.distance = 2.0

        self.offboard_control_publisher = self.create_publisher(OffboardNavCommand, '/dexi/offboard_manager', qos_profile)

        self.box_mission()

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

    def land(self):
        msg = OffboardNavCommand()
        msg.command = "land"
        self.offboard_control_publisher.publish(msg)

    def stop_offboard_heartbeat(self):
        msg = OffboardNavCommand()
        msg.command = "stop_offboard_heartbeat"
        self.offboard_control_publisher.publish(msg)

    def box_mission(self):
        time.sleep(3)
        self.start_offboard_heartbeat()

        time.sleep(1)
        self.arm()

        time.sleep(1)
        self.takeoff()

        # Give drone enough time to takeoff and settle
        time.sleep(10)
        self.fly_forward()

        time.sleep(5)
        self.fly_right()

        time.sleep(5)
        self.fly_backward()

        time.sleep(5)
        self.fly_left()

        time.sleep(5)
        self.land()

        time.sleep(5)
        self.stop_offboard_heartbeat()

def main(args=None):
    rclpy.init(args=args)
    node = OffboardBoxMission()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
