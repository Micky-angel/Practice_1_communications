
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        self.declare_parameter('in_topic', '/cmd_vel')
        
        self.declare_parameter('eps_lin', 1e-3)   # umbral para “= 0” en linear.x
        self.declare_parameter('eps_ang', 1e-3)   # umbral para “= 0” en angular.z

        in_topic = str(self.get_parameter('in_topic').value)
        self.sub = self.create_subscription(Twist, in_topic, self.cb_cmd, 10)

        self.get_logger().info(f'Controller escuchando {in_topic}')

    def cb_cmd(self, msg: Twist):
        eps_lin = float(self.get_parameter('eps_lin').value)
        eps_ang = float(self.get_parameter('eps_ang').value)

        v = msg.linear.x
        w = msg.angular.z

        avanzar = abs(v) > eps_lin
        girar   = abs(w) > eps_ang

        if avanzar and not girar:
            self.get_logger().info('AVANZAR, no girar')
        elif not avanzar and girar:
            self.get_logger().info('NO avanza, el robot gira')
        elif not avanzar and not girar:
            self.get_logger().info('NO hace nada')
        else:
            
            self.get_logger().info('AVANZAR y GIRAR')

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


