
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import numpy as np  

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.pub = self.create_publisher(PoseStamped, '/pose', 10)
        self.timer = self.create_timer(0.2, self.timer_cb)
        self.t = 0.0

    def timer_cb(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        x = 1.0+math.sin(self.t)
        y = 1.0+math.cos(self.t)
        noise_x = np.random.normal(0, 0.05)
        noise_y = np.random.normal(0, 0.05)

        msg.pose.position.x = x + noise_x
        msg.pose.position.y = y + noise_y

        self.pub.publish(msg)
        self.get_logger().info(f"Pose publicada: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")

        self.t += 0.1

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
