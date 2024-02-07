# For testing purposes
# ros2 topic pub /droneblocks/offboard std_msgs/String "data: 'TAKEOFF'" -1
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from dexi_msgs.srv import Navigate
from std_msgs.msg import String

STATE_QOS = rclpy.qos.QoSProfile(
    depth=10, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
)

SENSOR_QOS = rclpy.qos.qos_profile_sensor_data

class Position:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

class OffboardBridge(Node):

    def __init__(self):
        super().__init__('droneblocks_offboard_bridge')
        self.current_state = State()
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, STATE_QOS)
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', SENSOR_QOS)
        self.rate = self.create_rate(1)
        self.current_flight_mode = None
        self.is_drone_armed = False

        self.position = Position()

        self.navigate_subscriber = self.create_subscription(String, '/droneblocks/offboard', self.navigate_cb, STATE_QOS)

        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for set mode service...')


        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for arming service...')


        # TODO: consider starting this timer elsewhere so we don't spam
        # self.publish_timer = self.create_timer(0.05, self.publish_pose) # 20 Hz


    def state_cb(self, msg):
        self.current_state = msg
        self.current_flight_mode = str(self.current_state.mode)
        self.is_drone_armed = msg.armed

    def navigate_cb(self, msg):
        self.get_logger().info(msg.data)
        if msg.data == 'TAKEOFF':
            self.takeoff(2.0)
        if msg.data == 'LAND':
            self.land()

    def set_mode_request(self, mode):
        request = SetMode.Request()
        request.custom_mode = mode
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('set_mode_request future complete')
        return future.result()
    
    def publish_pose(self):
        pose = PoseStamped()
        pose.pose.position.x = self.position.x
        pose.pose.position.y = self.position.y
        pose.pose.position.z = self.position.z
        self.local_pos_pub.publish(pose)

    def takeoff(self, altitude):
        self.position.z = altitude
        # self.set_mode_request('OFFBOARD')
        self.arm_request(True)
        self.get_logger().info('Taking off!')

    # TODO: consider using /mavros/cmd/land service
    def land(self):
        self.position.z = 0.0
        self.get_logger().info('Landing!')

    def arm_request(self, arm=False):
        self.get_logger().info('arm_request called')
        request = CommandBool.Request()
        request.value = arm
        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('arm_request future complete')
        return future.result()

def main(args=None):
    rclpy.init()
    bridge = OffboardBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()