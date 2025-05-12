import rclpy
from rclpy.node import Node                                                    
from sensor_msgs.msg import Imu

class SubscriberNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.sub = self.create_subscription(Imu, 'imu', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info('Robot Imu: "%f, %f, %f"' \
            % (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)) 

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode("imu_sub")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
