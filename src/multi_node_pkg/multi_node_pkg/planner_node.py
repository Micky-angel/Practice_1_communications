import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.declare_parameter('in_topic', '/scan_filtered')
        self.declare_parameter('out_topic', '/cmd_vel')
        self.declare_parameter('threshold', 0.8)   
        self.declare_parameter('v_lin', 0.5)      
        self.declare_parameter('w_ang', 0.5)       
        self.in_topic  = str(self.get_parameter('in_topic').value)
        self.out_topic = str(self.get_parameter('out_topic').value)

        self.sub = self.create_subscription(PoseStamped, self.in_topic, self.cb_pose, 10)
        self.pub = self.create_publisher(Twist, self.out_topic, 10)

        self.get_logger().info(
            f'Planner escuchando {self.in_topic} y publicando {self.out_topic}'
        )

    def cb_pose(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y

        thr = float(self.get_parameter('threshold').value)
        v_lin = float(self.get_parameter('v_lin').value)
        w_ang = float(self.get_parameter('w_ang').value)

        cmd = Twist()

        # Regla: si x<thr y y<thr -> girar en sitio; si no, avanzar recto
        if (x < thr) and (y < thr):
            cmd.linear.x  = 0.0
            cmd.angular.z = w_ang
        else:
            cmd.linear.x  = v_lin
            cmd.angular.z = 0.0

        self.pub.publish(cmd)

        self.get_logger().info(
            f'Pose(x={x:.2f}, y={y:.2f}) -> cmd:v={cmd.linear.x:.2f}, w={cmd.angular.z:.2f} \n'
        )

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


