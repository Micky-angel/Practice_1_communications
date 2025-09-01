
import os
import csv
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, Twist


class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')
        self.declare_parameter('pose_topic', '/pose')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('rate_hz', 10.0)  
        self.declare_parameter('folder', '~/roslogs')
        self.declare_parameter('filename', 'el_rincon_del_meca.csv')
        self.declare_parameter('append', False) 

        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.cmd_topic  = str(self.get_parameter('cmd_topic').value)
        self.rate_hz    = float(self.get_parameter('rate_hz').value)
        self.folder     = os.path.expanduser(str(self.get_parameter('folder').value))
        self.filename   = str(self.get_parameter('filename').value)
        self.append     = bool(self.get_parameter('append').value)

        os.makedirs(self.folder, exist_ok=True)
        self.csv_path = os.path.join(self.folder, self.filename)

        self.last_pose = None    
        self.last_cmd  = None 

        self.pose_sub = self.create_subscription(
            PoseStamped, self.pose_topic, self.on_pose, qos_profile_sensor_data
        )
        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_topic, self.on_cmd, 10
        )

        new_file = (not os.path.exists(self.csv_path)) or (not self.append)
        self.csv_file = open(self.csv_path, 'a' if self.append else 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        if new_file:
            self.writer.writerow([
                'wall_time_iso', 'node_time_ns',
                # pose
                'pose_msg_time_ns', 'pose_frame',
                'px','py','pz','q_x','q_y','q_z','q_w',
                # cmd
                'cmd_msg_time_ns',
                'lin_x','lin_y','lin_z','ang_x','ang_y','ang_z'
            ])
            self.csv_file.flush()

        dt = 1.0 / max(1e-3, self.rate_hz)
        self.timer = self.create_timer(dt, self.sample_and_write)

        self.get_logger().info(
            f'Loggeando {self.pose_topic} y {self.cmd_topic} a {self.csv_path} @ {self.rate_hz:.1f} Hz'
        )

    def on_pose(self, msg: PoseStamped):
        self.last_pose = msg

    def on_cmd(self, msg: Twist):
        self.last_cmd = msg

    def sample_and_write(self):
        now_iso = datetime.now().isoformat(timespec='milliseconds')
        now_ns  = self.get_clock().now().nanoseconds

        pose_t_ns = ''
        frame_id  = ''
        px = py = pz = math.nan
        qx = qy = qz = qw = math.nan

        cmd_t_ns = ''
        lx = ly = lz = math.nan
        ax = ay = az = math.nan

        if self.last_pose is not None:
            pose_t_ns = (self.last_pose.header.stamp.sec * 10**9
                         + self.last_pose.header.stamp.nanosec)
            frame_id  = self.last_pose.header.frame_id
            px = self.last_pose.pose.position.x
            py = self.last_pose.pose.position.y
            pz = self.last_pose.pose.position.z
            qx = self.last_pose.pose.orientation.x
            qy = self.last_pose.pose.orientation.y
            qz = self.last_pose.pose.orientation.z
            qw = self.last_pose.pose.orientation.w

        if self.last_cmd is not None:
            cmd_t_ns = now_ns
            lx = self.last_cmd.linear.x
            ly = self.last_cmd.linear.y
            lz = self.last_cmd.linear.z
            ax = self.last_cmd.angular.x
            ay = self.last_cmd.angular.y
            az = self.last_cmd.angular.z

        self.writer.writerow([
            now_iso, now_ns,
            pose_t_ns, frame_id,
            px, py, pz, qx, qy, qz, qw,
            cmd_t_ns,
            lx, ly, lz, ax, ay, az
        ])
        self.csv_file.flush()

    def destroy_node(self):
        try:
            if hasattr(self, 'csv_file') and self.csv_file:
                self.csv_file.flush()
                self.csv_file.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
