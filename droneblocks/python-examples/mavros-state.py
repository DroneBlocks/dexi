import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State


STATE_QOS = rclpy.qos.QoSProfile(
    depth=10, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
)

class Offboard(Node):
    
  def __init__(self):
      super().__init__('mavros_state_node')
      self.current_state = State()
      self.state_sub = self.create_subscription(State, 'mavros/state', self.state_cb, STATE_QOS)

  def state_cb(self, msg):
      self.get_logger().info(str(msg.mode))
      self.current_state = msg

def main(args=None):
    rclpy.init(args=args)
    offboard = Offboard()
    rclpy.spin(offboard)
    offboard.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    


# if __name__ == "__main__":
#     rclpy.init()

#     state_sub = rclpy.node.create_subscription(State, "mavros/state", callback = state_cb)

#     local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

#     rospy.wait_for_service("/mavros/cmd/arming")
#     arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

#     rospy.wait_for_service("/mavros/set_mode")
#     set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


#     # Setpoint publishing MUST be faster than 2Hz
#     rate = rospy.Rate(20)

#     # Wait for Flight Controller connection
#     while(not rclpy.ok() and not current_state.connected):
#         rate.sleep()

#     pose = PoseStamped()

#     pose.pose.position.x = 0
#     pose.pose.position.y = 0
#     pose.pose.position.z = 2

#     # Send a few setpoints before starting
#     for i in range(100):
#         if not rclpy.ok():
#             break

#         local_pos_pub.publish(pose)
#         rate.sleep()

#     offb_set_mode = SetModeRequest()
#     offb_set_mode.custom_mode = 'OFFBOARD'

#     arm_cmd = CommandBoolRequest()
#     arm_cmd.value = True

#     last_req = rospy.Time.now()

#     while(not rospy.is_shutdown()):
#         if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
#             if(set_mode_client.call(offb_set_mode).mode_sent == True):
#                 rospy.loginfo("OFFBOARD enabled")

#             last_req = rospy.Time.now()
#         else:
#             if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
#                 if(arming_client.call(arm_cmd).success == True):
#                     rospy.loginfo("Vehicle armed")

#                 last_req = rospy.Time.now()

#         local_pos_pub.publish(pose)

#         rate.sleep()

