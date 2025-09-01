#filter_node 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class SimpleFilter(Node):
    def __init__(self):
        super().__init__('filter_node')

        self.sub = self.create_subscription(
            PoseStamped, '/pose', self.callback, 10
        )

        self.pub = self.create_publisher(PoseStamped, '/scan_filtered', 10)
        self.last_x = None
        self.last_y = None

    def callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y

        if self.last_x is None:
            x_f = x
            y_f = y
        else:
            x_f = 0.5 * x + 0.5 * self.last_x
            y_f = 0.5 * y + 0.5 * self.last_y

        self.last_x = x_f
        self.last_y = y_f

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'

        out.pose.position.x = x_f
        out.pose.position.y = y_f
        out.pose.position.z = 0.0


        out.pose.orientation.w = 1.0
        self.pub.publish(out)
        self.get_logger().info(f'signal filtered by Yerkito: x={x_f:.2f}, y={y_f:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


