#!/usr/bin/env python3
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node


def lrc8_with_header(frame_wo_chk: bytes) -> int:
    return (-sum(frame_wo_chk)) & 0xFF


def int16_le_bytes(value: int) -> bytes:
    value = max(-32768, min(32767, int(value)))
    value &= 0xFFFF
    return bytes([value & 0xFF, (value >> 8) & 0xFF])


def build_single_speed_frame(dev_id: int, cmd: int, speed: int) -> bytes:
    header = bytes([0xB7, 0xB8])
    frame_wo_chk = header + bytes([dev_id & 0xFF, cmd & 0xFF, 0x02]) + int16_le_bytes(speed)
    return frame_wo_chk + bytes([lrc8_with_header(frame_wo_chk)])


class MD400TUsb2CmdVelBridge(Node):
    def __init__(self):
        super().__init__('md400t_usb2_cmdvel_bridge')

        self.declare_parameter(
            'right_port',
            '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISNS9-if00-port0',
        )
        self.declare_parameter(
            'left_port',
            '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT94EQPJ-if00-port0',
        )
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('dev_id', 1)
        self.declare_parameter('right_cmd_id', 0x82)
        self.declare_parameter('left_cmd_id', 0x82)
        self.declare_parameter('wheel_base', 0.32)
        self.declare_parameter('speed_scale', 300.0)
        self.declare_parameter('max_speed_cmd', 300)
        self.declare_parameter('min_effective_cmd', 80)
        self.declare_parameter('reverse_right', False)
        self.declare_parameter('reverse_left', False)
        self.declare_parameter('send_hz', 50.0)
        self.declare_parameter('stale_sec', 0.5)

        self.right_port = str(self.get_parameter('right_port').value)
        self.left_port = str(self.get_parameter('left_port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.dev_id = int(self.get_parameter('dev_id').value)
        self.right_cmd_id = int(self.get_parameter('right_cmd_id').value)
        self.left_cmd_id = int(self.get_parameter('left_cmd_id').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.speed_scale = float(self.get_parameter('speed_scale').value)
        self.max_speed_cmd = int(self.get_parameter('max_speed_cmd').value)
        self.min_effective_cmd = int(self.get_parameter('min_effective_cmd').value)
        self.reverse_right = bool(self.get_parameter('reverse_right').value)
        self.reverse_left = bool(self.get_parameter('reverse_left').value)
        self.send_hz = float(self.get_parameter('send_hz').value)
        self.stale_sec = float(self.get_parameter('stale_sec').value)

        self.last_cmd_time = 0.0
        self.last_linear_x = 0.0
        self.last_angular_z = 0.0
        self.prev_right_cmd = None
        self.prev_left_cmd = None

        self.ser_right = serial.Serial(self.right_port, self.baudrate, timeout=0.02)
        self.ser_left = serial.Serial(self.left_port, self.baudrate, timeout=0.02)

        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.timer = self.create_timer(1.0 / max(self.send_hz, 0.1), self.on_timer)

        self.get_logger().info(
            f'[start] LR master bridge right={self.right_port}/0x{self.right_cmd_id:02X}, '
            f'left={self.left_port}/0x{self.left_cmd_id:02X}, id={self.dev_id}'
        )

    def on_cmd_vel(self, msg: Twist):
        self.last_linear_x = float(msg.linear.x)
        self.last_angular_z = float(msg.angular.z)
        self.last_cmd_time = time.time()

    def apply_min_effective(self, value: int) -> int:
        if value == 0:
            return 0
        if abs(value) < self.min_effective_cmd:
            return self.min_effective_cmd if value > 0 else -self.min_effective_cmd
        return value

    def cmd_vel_to_driver_cmd(self, linear_x: float, angular_z: float):
        right_mps = linear_x + (angular_z * self.wheel_base / 2.0)
        left_mps = linear_x - (angular_z * self.wheel_base / 2.0)

        right_cmd = int(round(right_mps * self.speed_scale))
        left_cmd = int(round(left_mps * self.speed_scale))

        right_cmd = max(-self.max_speed_cmd, min(self.max_speed_cmd, right_cmd))
        left_cmd = max(-self.max_speed_cmd, min(self.max_speed_cmd, left_cmd))

        # When driving and turning at the same time, the inner wheel must be
        # allowed to slow down. Boosting both sides to min_effective makes turns
        # look like straight driving.
        if abs(linear_x) < 0.01 or abs(angular_z) < 0.01:
            right_cmd = self.apply_min_effective(right_cmd)
            left_cmd = self.apply_min_effective(left_cmd)

        if self.reverse_right:
            right_cmd = -right_cmd
        if self.reverse_left:
            left_cmd = -left_cmd
        return right_cmd, left_cmd

    def on_timer(self):
        if (time.time() - self.last_cmd_time) > self.stale_sec:
            right_cmd = 0
            left_cmd = 0
        else:
            right_cmd, left_cmd = self.cmd_vel_to_driver_cmd(
                self.last_linear_x,
                self.last_angular_z,
            )

        self.send_speed(self.ser_right, self.right_cmd_id, right_cmd)
        time.sleep(0.002)
        self.send_speed(self.ser_left, self.left_cmd_id, left_cmd)

        if right_cmd != self.prev_right_cmd or left_cmd != self.prev_left_cmd:
            self.get_logger().info(
                f'[driver_cmd] linear_x={self.last_linear_x:.3f}, '
                f'angular_z={self.last_angular_z:.3f}, right={right_cmd}, left={left_cmd}, '
                f'ports=right:{self.right_port}, left:{self.left_port}'
            )
            self.prev_right_cmd = right_cmd
            self.prev_left_cmd = left_cmd

    def send_speed(self, ser, cmd_id: int, speed: int):
        try:
            ser.write(build_single_speed_frame(self.dev_id, cmd_id, speed))
            ser.flush()
        except Exception as exc:
            self.get_logger().warn(f'serial write failed cmd=0x{cmd_id:02X} speed={speed}: {exc}')

    def stop_all(self):
        try:
            self.send_speed(self.ser_right, self.right_cmd_id, 0)
            time.sleep(0.002)
            self.send_speed(self.ser_left, self.left_cmd_id, 0)
        except Exception:
            pass

    def destroy_node(self):
        self.stop_all()
        try:
            if self.ser_right and self.ser_right.is_open:
                self.ser_right.close()
        except Exception:
            pass
        try:
            if self.ser_left and self.ser_left.is_open:
                self.ser_left.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = MD400TUsb2CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
