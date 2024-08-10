import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped

STATE_QOS = rclpy.qos.QoSProfile(
    depth=10, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
)

SENSOR_QOS = rclpy.qos.qos_profile_sensor_data

class OffboardMission(Node):

    def __init__(self):
        super().__init__('offboard_mission_node')
        self.current_state = State()
        self.state_sub = self.create_subscription(State, 'mavros/state', self.state_cb, STATE_QOS)
        self.local_pos_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', SENSOR_QOS)
        self.rate = self.create_rate(1)
        self.current_flight_mode = None
        self.is_drone_armed = False

        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for set mode service...')


        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for arming service...')

        self.publish_timer = self.create_timer(0.05, self.publish_pose) # 20 Hz
        self.set_mode_request('OFFBOARD')
        self.arm_request(True)
        self.get_logger().info('Taking off!')   

    def state_cb(self, msg):
        self.current_state = msg
        self.current_flight_mode = str(self.current_state.mode)
        self.is_drone_armed = msg.armed

    def set_mode_request(self, mode):
        request = SetMode.Request()
        request.custom_mode = mode
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def publish_pose(self):
        pose = PoseStamped()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 4.0
        self.local_pos_pub.publish(pose)

    def arm_request(self, arm=False):
        request = CommandBool.Request()
        request.value = arm
        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init()
    mission = OffboardMission()
    rclpy.spin(mission)
    mission.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()