#!/usr/bin/env python3
import json
import math
import socket
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, Twist, Vector3Stamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class MasterSlamtecPathUdpNode(Node):
    def __init__(self):
        super().__init__('master_slamtec_path_udp_node')

        self.declare_parameter('target_ip', '192.168.0.140')
        self.declare_parameter('target_port', 5015)
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('yaw_topic', '/imu/processed_yaw')
        self.declare_parameter('odom_topic', '/master/odom')
        self.declare_parameter('path_topic', '/master/path')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'master_base_link')
        self.declare_parameter('update_hz', 50.0)
        self.declare_parameter('send_hz', 15.0)
        self.declare_parameter('path_max_points', 120)
        self.declare_parameter('min_path_delta', 0.01)
        self.declare_parameter('min_yaw_delta', 0.08)
        self.declare_parameter('min_turn_angular', 0.08)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('yaw_is_degrees', True)
        self.declare_parameter('invert_yaw', False)

        self.target_ip = str(self.get_parameter('target_ip').value)
        self.target_port = int(self.get_parameter('target_port').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.yaw_topic = str(self.get_parameter('yaw_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.child_frame_id = str(self.get_parameter('child_frame_id').value)
        self.update_hz = float(self.get_parameter('update_hz').value)
        self.send_hz = float(self.get_parameter('send_hz').value)
        self.path_max_points = int(self.get_parameter('path_max_points').value)
        self.min_path_delta = float(self.get_parameter('min_path_delta').value)
        self.min_yaw_delta = float(self.get_parameter('min_yaw_delta').value)
        self.min_turn_angular = float(self.get_parameter('min_turn_angular').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.yaw_is_degrees = bool(self.get_parameter('yaw_is_degrees').value)
        self.invert_yaw = bool(self.get_parameter('invert_yaw').value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.raw_yaw0 = None
        self.last_update = None
        self.last_cmd_time = 0.0
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.path = []
        self.path_s = 0.0
        self.last_path_x = None
        self.last_path_y = None
        self.last_path_yaw = None
        self.last_path_angular = 0.0

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

        self.create_subscription(Twist, self.cmd_topic, self.on_cmd, 10)
        self.create_subscription(Vector3Stamped, self.yaw_topic, self.on_yaw, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.create_timer(1.0 / max(self.update_hz, 1.0), self.on_update)
        self.create_timer(1.0 / max(self.send_hz, 1.0), self.on_send)

        self.get_logger().info(
            f'path UDP -> {self.target_ip}:{self.target_port}, cmd={self.cmd_topic}, yaw={self.yaw_topic}'
        )

    def on_cmd(self, msg):
        self.cmd_linear = float(msg.linear.x)
        self.cmd_angular = float(msg.angular.z)
        self.last_cmd_time = time.time()

    def on_yaw(self, msg):
        raw = float(msg.vector.z)
        if self.yaw_is_degrees:
            raw = math.radians(raw)
        if self.invert_yaw:
            raw = -raw
        if self.raw_yaw0 is None:
            self.raw_yaw0 = raw
            self.get_logger().info(f'yaw origin set raw={raw:.3f} rad')
        self.yaw = normalize_angle(raw - self.raw_yaw0)

    def active_cmd(self):
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            return 0.0, 0.0
        return self.cmd_linear, self.cmd_angular

    def on_update(self):
        now = time.time()
        if self.last_update is None:
            self.last_update = now
            return
        dt = max(0.0, min(0.1, now - self.last_update))
        self.last_update = now

        linear, angular = self.active_cmd()
        if self.raw_yaw0 is None:
            # Fallback before first IMU message.
            self.yaw = normalize_angle(self.yaw + angular * dt)
        self.x += linear * math.cos(self.yaw) * dt
        self.y += linear * math.sin(self.yaw) * dt

        self.publish_odom_and_path(linear, angular)

    def publish_odom_and_path(self, linear, angular):
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quaternion(self.yaw)
        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular
        self.odom_pub.publish(odom)

        append = self.last_path_x is None
        if not append:
            ds = math.hypot(self.x - self.last_path_x, self.y - self.last_path_y)
            dyaw = abs(normalize_angle(self.yaw - self.last_path_yaw))
            dangular = abs(angular - self.last_path_angular)
            append = ds >= self.min_path_delta or dyaw >= self.min_yaw_delta or dangular >= self.min_turn_angular
        if append:
            if self.last_path_x is not None:
                self.path_s += math.hypot(self.x - self.last_path_x, self.y - self.last_path_y)
            self.last_path_x = self.x
            self.last_path_y = self.y
            self.last_path_yaw = self.yaw
            self.last_path_angular = angular
            if angular > self.min_turn_angular:
                turn = 'left'
            elif angular < -self.min_turn_angular:
                turn = 'right'
            else:
                turn = 'straight'
            self.path.append({
                'x': self.x,
                'y': self.y,
                'yaw': self.yaw,
                's': self.path_s,
                'linear_x': linear,
                'angular_z': angular,
                'turn': turn,
            })
            if len(self.path) > self.path_max_points:
                self.path = self.path[-self.path_max_points:]

            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = self.x
            pose.pose.position.y = self.y
            pose.pose.orientation = yaw_to_quaternion(self.yaw)
            self.path_msg.header.stamp = stamp
            self.path_msg.poses.append(pose)
            if len(self.path_msg.poses) > self.path_max_points:
                self.path_msg.poses = self.path_msg.poses[-self.path_max_points:]
            self.path_pub.publish(self.path_msg)

    def on_send(self):
        self.seq += 1
        linear, angular = self.active_cmd()
        payload = {
            'source': 'master_slamtec_path',
            'seq': self.seq,
            'stamp': time.time(),
            'cmd_vel': {
                'linear_x': linear,
                'angular_z': angular,
            },
            'odom': {
                'x': self.x,
                'y': self.y,
                'yaw': self.yaw,
            },
            'latest_s': self.path_s,
            'path': self.path,
        }
        data = json.dumps(payload, separators=(',', ':')).encode('utf-8')
        self.sock.sendto(data, (self.target_ip, self.target_port))
        if self.seq % max(1, int(self.send_hz)) == 1:
            self.get_logger().info(
                f'tx seq={self.seq} path={len(self.path)} odom=({self.x:.2f},{self.y:.2f},{self.yaw:.2f}) '
                f'cmd=({linear:.2f},{angular:.2f}) bytes={len(data)}'
            )

    def destroy_node(self):
        try:
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = MasterSlamtecPathUdpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
