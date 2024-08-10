import threading
import rclpy

rclpy.init()
node = rclpy.create_node('simple_node')

# Spin in a separate thread
thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
thread.start()

rate = node.create_rate(1)

try:
    while rclpy.ok():
        node.get_logger().info('looping')
        rate.sleep()
except KeyboardInterrupt:
    pass

rclpy.shutdown()
thread.join()