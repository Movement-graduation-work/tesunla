#!/usr/bin/env python3
import json
import math
import socket
import threading
import time

import rclpy
import serial
from rclpy.node import Node

try:
    import cv2
except Exception:
    cv2 = None

try:
    import RPi.GPIO as GPIO
except Exception:
    GPIO = None


def measure_ultrasonic_cm(echo_pin: int, trig_pin: int, timeout_sec: float = 0.04):
    if GPIO is None:
        return None

    GPIO.output(trig_pin, True)
    time.sleep(0.00001)
    GPIO.output(trig_pin, False)

    deadline = time.time() + timeout_sec
    while GPIO.input(echo_pin) == 0:
        if time.time() > deadline:
            return None

    start = time.time()
    while GPIO.input(echo_pin) == 1:
        if time.time() > deadline:
            return None

    return ((time.time() - start) * 34300.0) / 2.0


def lrc8(data: bytes) -> int:
    return (-sum(data)) & 0xFF


def int16_le_bytes(value: int) -> bytes:
    value = max(-32768, min(32767, int(value)))
    value &= 0xFFFF
    return bytes([value & 0xFF, (value >> 8) & 0xFF])


def build_md400t_dual_speed_frame(dev_id: int, spd1: int, spd2: int, accel: int) -> bytes:
    accel = max(0, min(int(accel), 255))
    header = bytes([0xB7, 0xB8])
    spd1_bytes = int16_le_bytes(spd1)
    spd2_bytes = int16_le_bytes(spd2)
    payload = bytes([0x01, spd1_bytes[0], spd1_bytes[1], 0x01, spd2_bytes[0], spd2_bytes[1], accel])
    frame_wo_chk = header + bytes([dev_id & 0xFF, 0xCF, 0x07]) + payload
    return frame_wo_chk + bytes([lrc8(frame_wo_chk)])


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value, low, high):
    return max(low, min(high, value))


class UdpPathEbimuMotorFollower(Node):
    def __init__(self):
        super().__init__('udp_path_ebimu_motor_follower')

        self.declare_parameter('bind_ip', '0.0.0.0')
        self.declare_parameter('udp_port', 5015)
        self.declare_parameter('imu_port', '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')
        self.declare_parameter('imu_baudrate', 115200)
        self.declare_parameter('motor_port', '/dev/ttyUSB0')
        self.declare_parameter('motor_baudrate', 57600)
        self.declare_parameter('dev_id', 1)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.32)
        self.declare_parameter('speed_scale', 300.0)
        self.declare_parameter('max_speed_cmd', 90)
        self.declare_parameter('min_effective_cmd', 0)
        self.declare_parameter('accel', 3)
        self.declare_parameter('stop_accel', 255)
        self.declare_parameter('stop_repeat', 6)
        self.declare_parameter('send_hz', 30.0)
        self.declare_parameter('follow_gap', 0.20)
        self.declare_parameter('lookahead', 0.12)
        self.declare_parameter('min_path_s', 0.03)
        self.declare_parameter('max_linear', 0.045)
        self.declare_parameter('max_angular', 0.22)
        self.declare_parameter('k_linear', 0.45)
        self.declare_parameter('k_angular', 0.9)
        self.declare_parameter('max_yaw_error', 0.8)
        self.declare_parameter('k_path_yaw', 0.55)
        self.declare_parameter('yaw_deadband', 0.05)
        self.declare_parameter('position_yaw_dist', 0.08)
        self.declare_parameter('allow_yaw_only_rotate', False)
        self.declare_parameter('use_path_yaw', False)
        self.declare_parameter('allow_rotate_in_place', False)
        self.declare_parameter('udp_timeout', 1.0)
        self.declare_parameter('flip_path_direction', False)
        self.declare_parameter('invert_imu_yaw', False)
        self.declare_parameter('invert_turn', False)
        self.declare_parameter('swap_left_right', True)
        self.declare_parameter('reverse_left', False)
        self.declare_parameter('reverse_right', False)
        self.declare_parameter('ultrasonic_enabled', True)
        self.declare_parameter('ultrasonic_stop_cm', 100.0)
        self.declare_parameter('ultrasonic_release_cm', 110.0)
        self.declare_parameter('ultrasonic_timeout_sec', 0.04)
        self.declare_parameter('ultrasonic_pins', '12:6,17:27,22:23,24:25')
        self.declare_parameter('ultrasonic_log_period', 0.5)
        self.declare_parameter('ultrasonic_status_log_period', 1.0)
        self.declare_parameter('ultrasonic_hard_check_period', 0.05)
        self.declare_parameter('camera_guidance_enabled', True)
        self.declare_parameter('camera_guidance_mode', 'color')
        self.declare_parameter('camera_device', '/dev/video0')
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('aruco_marker_id', 0)
        self.declare_parameter('green_h_min', 35)
        self.declare_parameter('green_h_max', 85)
        self.declare_parameter('green_s_min', 80)
        self.declare_parameter('green_v_min', 80)
        self.declare_parameter('green_min_area', 1200.0)
        self.declare_parameter('camera_timeout', 0.5)
        self.declare_parameter('camera_angular_gain', 0.08)
        self.declare_parameter('camera_angular_sign', -1.0)
        self.declare_parameter('camera_log_period', 0.5)
        self.declare_parameter('turn_feedforward_enabled', True)
        self.declare_parameter('turn_feedforward_gain', 0.35)
        self.declare_parameter('turn_feedforward_sign', 1.0)
        self.declare_parameter('turn_feedforward_max', 0.04)
        self.declare_parameter('turn_linear_scale', 0.65)
        self.declare_parameter('min_turn_angular', 0.08)
        self.declare_parameter('log_period', 0.5)

        self.bind_ip = str(self.get_parameter('bind_ip').value)
        self.udp_port = int(self.get_parameter('udp_port').value)
        self.imu_port = str(self.get_parameter('imu_port').value)
        self.imu_baudrate = int(self.get_parameter('imu_baudrate').value)
        self.motor_port = str(self.get_parameter('motor_port').value)
        self.motor_baudrate = int(self.get_parameter('motor_baudrate').value)
        self.dev_id = int(self.get_parameter('dev_id').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.speed_scale = float(self.get_parameter('speed_scale').value)
        self.max_speed_cmd = int(self.get_parameter('max_speed_cmd').value)
        self.min_effective_cmd = int(self.get_parameter('min_effective_cmd').value)
        self.accel = int(self.get_parameter('accel').value)
        self.stop_accel = int(self.get_parameter('stop_accel').value)
        self.stop_repeat = int(self.get_parameter('stop_repeat').value)
        self.send_hz = float(self.get_parameter('send_hz').value)
        self.follow_gap = float(self.get_parameter('follow_gap').value)
        self.lookahead = float(self.get_parameter('lookahead').value)
        self.min_path_s = float(self.get_parameter('min_path_s').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.k_linear = float(self.get_parameter('k_linear').value)
        self.k_angular = float(self.get_parameter('k_angular').value)
        self.max_yaw_error = float(self.get_parameter('max_yaw_error').value)
        self.k_path_yaw = float(self.get_parameter('k_path_yaw').value)
        self.yaw_deadband = float(self.get_parameter('yaw_deadband').value)
        self.position_yaw_dist = float(self.get_parameter('position_yaw_dist').value)
        self.allow_yaw_only_rotate = bool(self.get_parameter('allow_yaw_only_rotate').value)
        self.use_path_yaw = bool(self.get_parameter('use_path_yaw').value)
        self.allow_rotate_in_place = bool(self.get_parameter('allow_rotate_in_place').value)
        self.udp_timeout = float(self.get_parameter('udp_timeout').value)
        self.flip_path_direction = bool(self.get_parameter('flip_path_direction').value)
        self.invert_imu_yaw = bool(self.get_parameter('invert_imu_yaw').value)
        self.invert_turn = bool(self.get_parameter('invert_turn').value)
        self.swap_left_right = bool(self.get_parameter('swap_left_right').value)
        self.reverse_left = bool(self.get_parameter('reverse_left').value)
        self.reverse_right = bool(self.get_parameter('reverse_right').value)
        self.ultrasonic_enabled = bool(self.get_parameter('ultrasonic_enabled').value)
        self.ultrasonic_stop_cm = float(self.get_parameter('ultrasonic_stop_cm').value)
        self.ultrasonic_release_cm = float(self.get_parameter('ultrasonic_release_cm').value)
        self.ultrasonic_timeout_sec = float(self.get_parameter('ultrasonic_timeout_sec').value)
        self.ultrasonic_pins = self.parse_ultrasonic_pins(str(self.get_parameter('ultrasonic_pins').value))
        self.ultrasonic_log_period = float(self.get_parameter('ultrasonic_log_period').value)
        self.ultrasonic_status_log_period = float(self.get_parameter('ultrasonic_status_log_period').value)
        self.ultrasonic_hard_check_period = float(self.get_parameter('ultrasonic_hard_check_period').value)
        self.camera_guidance_enabled = bool(self.get_parameter('camera_guidance_enabled').value)
        self.camera_guidance_mode = str(self.get_parameter('camera_guidance_mode').value).lower()
        self.camera_device = str(self.get_parameter('camera_device').value)
        self.camera_width = int(self.get_parameter('camera_width').value)
        self.camera_height = int(self.get_parameter('camera_height').value)
        self.aruco_marker_id = int(self.get_parameter('aruco_marker_id').value)
        self.green_h_min = int(self.get_parameter('green_h_min').value)
        self.green_h_max = int(self.get_parameter('green_h_max').value)
        self.green_s_min = int(self.get_parameter('green_s_min').value)
        self.green_v_min = int(self.get_parameter('green_v_min').value)
        self.green_min_area = float(self.get_parameter('green_min_area').value)
        self.camera_timeout = float(self.get_parameter('camera_timeout').value)
        self.camera_angular_gain = float(self.get_parameter('camera_angular_gain').value)
        self.camera_angular_sign = float(self.get_parameter('camera_angular_sign').value)
        self.camera_log_period = float(self.get_parameter('camera_log_period').value)
        self.turn_feedforward_enabled = bool(self.get_parameter('turn_feedforward_enabled').value)
        self.turn_feedforward_gain = float(self.get_parameter('turn_feedforward_gain').value)
        self.turn_feedforward_sign = float(self.get_parameter('turn_feedforward_sign').value)
        self.turn_feedforward_max = float(self.get_parameter('turn_feedforward_max').value)
        self.turn_linear_scale = float(self.get_parameter('turn_linear_scale').value)
        self.min_turn_angular = float(self.get_parameter('min_turn_angular').value)
        self.log_period = float(self.get_parameter('log_period').value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.raw_yaw0 = None
        self.have_imu = False
        self.last_pose_update = time.time()

        self.master_path = []
        self.master_latest_s = 0.0
        self.last_udp_time = 0.0
        self.have_alignment = False
        self.master_origin_x = 0.0
        self.master_origin_y = 0.0
        self.master_origin_yaw = 0.0
        self.slave_origin_x = 0.0
        self.slave_origin_y = 0.0
        self.slave_origin_yaw = 0.0

        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        self.last_left = None
        self.last_right = None
        self.last_log_time = 0.0
        self.last_forced_stop_time = 0.0
        self.ultrasonic_distances = [None] * len(self.ultrasonic_pins)
        self.ultrasonic_blocked = False
        self.ultrasonic_min_cm = None
        self.ultrasonic_min_index = -1
        self.last_ultrasonic_log_time = 0.0
        self.last_ultrasonic_status_log_time = 0.0
        self.last_ultrasonic_hard_check_time = 0.0
        self.last_ultrasonic_hard_stop_log_time = 0.0
        self.ultrasonic_clear_count = 0
        self.camera_lock = threading.Lock()
        self.marker_seen = False
        self.marker_error_norm = 0.0
        self.marker_last_time = 0.0
        self.marker_area = 0.0
        self.last_camera_log_time = 0.0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind((self.bind_ip, self.udp_port))

        self.imu_ser = None
        self.motor_ser = None
        self.motor_lock = threading.Lock()
        self.ultrasonic_lock = threading.Lock()
        self.running = True
        self.open_imu()
        self.open_motor()
        self.setup_ultrasonic()

        self.create_timer(0.005, self.read_imu)
        self.create_timer(0.01, self.receive_udp)
        if self.ultrasonic_enabled and self.ultrasonic_pins:
            self.create_timer(0.10, self.read_ultrasonic)
            self.ultrasonic_thread = threading.Thread(target=self.ultrasonic_safety_loop, daemon=True)
            self.ultrasonic_thread.start()
        else:
            self.ultrasonic_thread = None
        if self.camera_guidance_enabled:
            self.camera_thread = threading.Thread(target=self.camera_guidance_loop, daemon=True)
            self.camera_thread.start()
        else:
            self.camera_thread = None
        self.create_timer(1.0 / max(self.send_hz, 1.0), self.control_loop)

        self.get_logger().info(
            f'UDP path follower listen={self.bind_ip}:{self.udp_port}, imu={self.imu_port}, motor={self.motor_port}'
        )
        self.get_logger().warn('AUTONOMOUS XY MODE: ignoring master cmd_vel for driving; following master path x/y only')
        if self.ultrasonic_enabled and self.ultrasonic_pins:
            self.get_logger().warn(
                f'ULTRASONIC E-STOP: stop <= {self.ultrasonic_stop_cm:.1f}cm, '
                f'release >= {self.ultrasonic_release_cm:.1f}cm, pins={self.ultrasonic_pins}'
            )
        if self.camera_guidance_enabled:
            self.get_logger().warn(
                f'CAMERA GUIDANCE: mode={self.camera_guidance_mode}, camera={self.camera_device}; '
                'bearing only, path x/y remains primary'
            )
        if self.turn_feedforward_enabled:
            self.get_logger().warn(
                f'SPATIAL TURN FEED-FORWARD: gain={self.turn_feedforward_gain:.2f}, '
                f'sign={self.turn_feedforward_sign:.1f}, max={self.turn_feedforward_max:.2f}; '
                'uses turn command stored at the followed path position, not live cmd_vel mirroring'
            )

    def parse_ultrasonic_pins(self, text):
        pins = []
        for item in text.split(','):
            item = item.strip()
            if not item:
                continue
            try:
                echo_text, trig_text = item.split(':', 1)
                pins.append((int(echo_text), int(trig_text)))
            except Exception:
                self.get_logger().warn(f'bad ultrasonic pin pair ignored: {item}')
        return pins

    def setup_ultrasonic(self):
        if not self.ultrasonic_enabled:
            return
        if GPIO is None:
            self.ultrasonic_enabled = False
            self.get_logger().error('RPi.GPIO import failed; ultrasonic E-stop disabled')
            return
        if not self.ultrasonic_pins:
            self.ultrasonic_enabled = False
            self.get_logger().error('no ultrasonic pins configured; ultrasonic E-stop disabled')
            return

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for echo_pin, trig_pin in self.ultrasonic_pins:
            GPIO.setup(trig_pin, GPIO.OUT)
            GPIO.setup(echo_pin, GPIO.IN)
            GPIO.output(trig_pin, False)
        time.sleep(0.1)

    def read_ultrasonic(self):
        if not self.ultrasonic_lock.acquire(blocking=False):
            return
        try:
            distances = []
            for echo_pin, trig_pin in self.ultrasonic_pins:
                distances.append(measure_ultrasonic_cm(echo_pin, trig_pin, self.ultrasonic_timeout_sec))
                time.sleep(0.01)
        finally:
            self.ultrasonic_lock.release()

        valid = [(idx, dist) for idx, dist in enumerate(distances) if dist is not None and 0.0 < dist <= 400.0]
        if valid:
            min_idx, min_cm = min(valid, key=lambda item: item[1])
        else:
            min_idx, min_cm = -1, None

        was_blocked = self.ultrasonic_blocked
        if min_cm is None:
            blocked = was_blocked
            self.ultrasonic_clear_count = 0
        elif min_cm <= self.ultrasonic_stop_cm:
            blocked = True
            self.ultrasonic_clear_count = 0
        elif was_blocked:
            if min_cm >= self.ultrasonic_release_cm:
                self.ultrasonic_clear_count += 1
            else:
                self.ultrasonic_clear_count = 0
            blocked = self.ultrasonic_clear_count < 3
        else:
            blocked = False

        self.ultrasonic_distances = distances
        self.ultrasonic_min_index = min_idx
        self.ultrasonic_min_cm = min_cm
        self.ultrasonic_blocked = blocked

        now = time.time()
        if blocked:
            self.last_cmd_linear = 0.0
            self.last_cmd_angular = 0.0
            if now - self.last_ultrasonic_log_time >= self.ultrasonic_log_period:
                self.last_ultrasonic_log_time = now
                self.get_logger().warn(
                    f'ULTRASONIC STOP sensor={min_idx + 1} dist={min_cm:.1f}cm '
                    f'<= {self.ultrasonic_stop_cm:.1f}cm all={self.format_distances(distances)}'
                )
        elif was_blocked:
            self.ultrasonic_clear_count = 0
            self.get_logger().info(
                f'ULTRASONIC CLEAR min={min_cm:.1f}cm all={self.format_distances(distances)}'
                if min_cm is not None else
                f'ULTRASONIC CLEAR all={self.format_distances(distances)}'
            )
        elif now - self.last_ultrasonic_status_log_time >= self.ultrasonic_status_log_period:
            self.last_ultrasonic_status_log_time = now
            if min_cm is None:
                self.get_logger().info(f'ULTRASONIC dist={self.format_distances(distances)} min=None')
            else:
                self.get_logger().info(
                    f'ULTRASONIC dist={self.format_distances(distances)} '
                    f'min=sensor{min_idx + 1}:{min_cm:.1f}cm'
                )

    def format_distances(self, distances):
        return '[' + ','.join('None' if dist is None else f'{dist:.1f}' for dist in distances) + ']'

    def ultrasonic_safety_loop(self):
        while self.running:
            try:
                self.read_ultrasonic()
                if self.ultrasonic_blocked:
                    self.last_cmd_linear = 0.0
                    self.last_cmd_angular = 0.0
                    self.force_stop_motor()
                time.sleep(0.05)
            except Exception as exc:
                self.get_logger().error(f'ultrasonic safety loop failed: {exc}')
                time.sleep(0.1)

    def camera_guidance_loop(self):
        if cv2 is None:
            self.get_logger().error('cv2 unavailable; camera guidance disabled')
            self.camera_guidance_enabled = False
            return

        cap = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            self.get_logger().error(f'camera open failed: {self.camera_device}')
            self.camera_guidance_enabled = False
            return

        aruco_dict = None
        params = None
        detector = None
        if self.camera_guidance_mode == 'aruco':
            if not hasattr(cv2, 'aruco'):
                self.get_logger().error('cv2.aruco unavailable; camera guidance disabled')
                self.camera_guidance_enabled = False
                cap.release()
                return
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            params = cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, 'DetectorParameters') else cv2.aruco.DetectorParameters_create()
            detector = cv2.aruco.ArucoDetector(aruco_dict, params) if hasattr(cv2.aruco, 'ArucoDetector') else None
        self.get_logger().info(f'camera opened for {self.camera_guidance_mode} bearing: {self.camera_device}')

        while self.running:
            ok, frame = cap.read()
            now = time.time()
            if not ok or frame is None:
                with self.camera_lock:
                    self.marker_seen = False
                time.sleep(0.05)
                continue

            found = False
            best_area = -1.0
            best_center_x = 0.0
            if self.camera_guidance_mode == 'aruco':
                if detector is not None:
                    corners, ids, _ = detector.detectMarkers(frame)
                else:
                    corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=params)
                if ids is not None:
                    ids_flat = ids.flatten()
                    for idx, marker_id in enumerate(ids_flat):
                        if int(marker_id) != self.aruco_marker_id:
                            continue
                        pts = corners[idx][0]
                        area = abs(cv2.contourArea(pts))
                        if area > best_area:
                            best_area = area
                            best_center_x = float(pts[:, 0].mean())
                            found = True
            else:
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                lower = (self.green_h_min, self.green_s_min, self.green_v_min)
                upper = (self.green_h_max, 255, 255)
                mask = cv2.inRange(hsv, lower, upper)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, None, iterations=2)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, None, iterations=2)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for contour in contours:
                    area = abs(cv2.contourArea(contour))
                    if area < self.green_min_area or area <= best_area:
                        continue
                    moments = cv2.moments(contour)
                    if abs(moments.get('m00', 0.0)) <= 1e-6:
                        continue
                    best_area = area
                    best_center_x = float(moments['m10'] / moments['m00'])
                    found = True

            with self.camera_lock:
                self.marker_seen = found
                if found:
                    image_center_x = frame.shape[1] / 2.0
                    self.marker_error_norm = (best_center_x - image_center_x) / max(image_center_x, 1.0)
                    self.marker_last_time = now
                    self.marker_area = best_area

            if found and now - self.last_camera_log_time >= self.camera_log_period:
                self.last_camera_log_time = now
                self.get_logger().info(
                    f'CAMERA {self.camera_guidance_mode} bearing err={self.marker_error_norm:+.2f} area={best_area:.0f}'
                )
            time.sleep(0.02)

        cap.release()

    def current_marker_error(self):
        with self.camera_lock:
            age = time.time() - self.marker_last_time
            if self.marker_seen and age <= self.camera_timeout:
                return self.marker_error_norm, age
        return None, None

    def open_imu(self):
        try:
            self.imu_ser = serial.Serial(self.imu_port, self.imu_baudrate, timeout=0.0)
            self.get_logger().info(f'IMU opened: {self.imu_port}')
        except Exception as exc:
            self.imu_ser = None
            self.get_logger().error(f'failed to open IMU {self.imu_port}: {exc}')

    def open_motor(self):
        try:
            self.motor_ser = serial.Serial(self.motor_port, self.motor_baudrate, timeout=0.05)
            self.get_logger().info(f'Motor serial opened: {self.motor_port}')
        except Exception as exc:
            self.motor_ser = None
            self.get_logger().error(f'failed to open motor serial {self.motor_port}: {exc}')

    def parse_ebimu_yaw(self, raw):
        line = raw.strip()
        if line.startswith('*'):
            line = line[1:]
        parts = line.split(',')
        if len(parts) < 3:
            return None
        try:
            return math.radians(float(parts[2]))
        except ValueError:
            return None

    def read_imu(self):
        if self.imu_ser is None or not self.imu_ser.is_open:
            self.open_imu()
            return
        while True:
            try:
                raw = self.imu_ser.readline().decode('ascii', errors='ignore')
            except Exception as exc:
                self.get_logger().warn(f'IMU read failed: {exc}')
                try:
                    self.imu_ser.close()
                except Exception:
                    pass
                self.imu_ser = None
                return
            if not raw:
                return
            yaw = self.parse_ebimu_yaw(raw)
            if yaw is None:
                continue
            if self.invert_imu_yaw:
                yaw = -yaw
            if self.raw_yaw0 is None:
                self.raw_yaw0 = yaw
                self.get_logger().info(f'IMU yaw origin set raw={yaw:.3f} rad')
            self.yaw = normalize_angle(yaw - self.raw_yaw0)
            self.have_imu = True

    def integrate_pose(self):
        now = time.time()
        dt = max(0.0, min(0.1, now - self.last_pose_update))
        self.last_pose_update = now
        self.x += self.last_cmd_linear * math.cos(self.yaw) * dt
        self.y += self.last_cmd_linear * math.sin(self.yaw) * dt

    def receive_udp(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except BlockingIOError:
                return
            except Exception as exc:
                self.get_logger().error(f'UDP receive failed: {exc}')
                return
            try:
                payload = json.loads(data.decode('utf-8'))
            except Exception as exc:
                self.get_logger().warn(f'bad UDP payload: {exc}')
                continue

            self.master_path = payload.get('path') or []
            self.master_latest_s = float(payload.get('latest_s', 0.0))
            self.last_udp_time = time.time()

            if self.have_imu and self.master_path and not self.have_alignment:
                first = self.master_path[0]
                self.master_origin_x = float(first.get('x', 0.0))
                self.master_origin_y = float(first.get('y', 0.0))
                self.master_origin_yaw = float(first.get('yaw', 0.0))
                self.slave_origin_x = self.x
                self.slave_origin_y = self.y
                self.slave_origin_yaw = self.yaw
                self.have_alignment = True
                self.get_logger().warn(
                    f'aligned master_start=({self.master_origin_x:.2f},{self.master_origin_y:.2f},{self.master_origin_yaw:.2f}) '
                    f'slave_start=({self.slave_origin_x:.2f},{self.slave_origin_y:.2f},{self.slave_origin_yaw:.2f})'
                )

    def transform_point(self, point):
        dx = float(point.get('x', 0.0)) - self.master_origin_x
        dy = float(point.get('y', 0.0)) - self.master_origin_y
        yaw_offset = normalize_angle(self.slave_origin_yaw - self.master_origin_yaw)
        if self.flip_path_direction:
            yaw_offset = normalize_angle(yaw_offset + math.pi)
        c = math.cos(yaw_offset)
        s = math.sin(yaw_offset)
        return (
            self.slave_origin_x + c * dx - s * dy,
            self.slave_origin_y + s * dx + c * dy,
            float(point.get('s', 0.0)),
            normalize_angle(float(point.get('yaw', self.master_origin_yaw)) + yaw_offset),
            float(point.get('linear_x', 0.0)),
            float(point.get('angular_z', 0.0)),
            str(point.get('turn', 'straight')),
        )

    def transformed_path(self):
        return [self.transform_point(point) for point in self.master_path]

    def closest_s_on_path(self, points):
        best_s = points[0][2]
        best_error = float('inf')
        for idx in range(1, len(points)):
            x0, y0, s0, *_ = points[idx - 1]
            x1, y1, s1, *_ = points[idx]
            dx = x1 - x0
            dy = y1 - y0
            seg_len_sq = dx * dx + dy * dy
            ratio = 0.0 if seg_len_sq <= 1e-9 else ((self.x - x0) * dx + (self.y - y0) * dy) / seg_len_sq
            ratio = clamp(ratio, 0.0, 1.0)
            px = x0 + dx * ratio
            py = y0 + dy * ratio
            error = math.hypot(self.x - px, self.y - py)
            if error < best_error:
                best_error = error
                best_s = s0 + (s1 - s0) * ratio
        return best_s, best_error

    def interp_yaw(self, yaw0, yaw1, ratio):
        return normalize_angle(yaw0 + normalize_angle(yaw1 - yaw0) * ratio)

    def point_at_s(self, points, target_s):
        if target_s <= points[0][2]:
            idx = 0
            while idx + 1 < len(points) and abs(points[idx + 1][2] - points[0][2]) <= 1e-9:
                idx += 1
            return points[idx]
        for idx in range(1, len(points)):
            x0, y0, s0, yaw0, linear0, angular0, turn0 = points[idx - 1]
            x1, y1, s1, yaw1, linear1, angular1, turn1 = points[idx]
            if s1 >= target_s:
                ds = s1 - s0
                if abs(ds) <= 1e-9:
                    return points[idx]
                ratio = (target_s - s0) / ds
                return (
                    x0 + (x1 - x0) * ratio,
                    y0 + (y1 - y0) * ratio,
                    target_s,
                    self.interp_yaw(yaw0, yaw1, ratio),
                    linear0 + (linear1 - linear0) * ratio,
                    angular0 + (angular1 - angular0) * ratio,
                    turn1 if ratio >= 0.5 else turn0,
                )
        return points[-1]

    def clamp_cmd(self, value):
        return max(-self.max_speed_cmd, min(self.max_speed_cmd, int(value)))

    def apply_min_effective(self, value):
        if value == 0:
            return 0
        if abs(value) < self.min_effective_cmd:
            return self.min_effective_cmd if value > 0 else -self.min_effective_cmd
        return value

    def cmd_vel_to_motor(self, linear_x, angular_z):
        left_mps = linear_x - self.wheel_base * angular_z / 2.0
        right_mps = linear_x + self.wheel_base * angular_z / 2.0
        left = self.apply_min_effective(self.clamp_cmd((left_mps / self.wheel_radius) * self.speed_scale))
        right = self.apply_min_effective(self.clamp_cmd((right_mps / self.wheel_radius) * self.speed_scale))
        if self.invert_turn:
            left, right = right, left
        return self.clamp_cmd(left), self.clamp_cmd(right)

    def force_stop_motor(self):
        if self.motor_ser is None or not self.motor_ser.is_open:
            self.open_motor()
            if self.motor_ser is None or not self.motor_ser.is_open:
                return
        frame = build_md400t_dual_speed_frame(self.dev_id, 0, 0, self.stop_accel)
        try:
            with self.motor_lock:
                for _ in range(max(1, self.stop_repeat)):
                    self.motor_ser.write(frame)
                    self.motor_ser.flush()
                    time.sleep(0.005)
        except Exception as exc:
            self.get_logger().error(f'force stop write failed: {exc}')
            try:
                self.motor_ser.close()
            except Exception:
                pass
            self.motor_ser = None

    def send_motor(self, left, right):
        if self.ultrasonic_enabled and (left != 0 or right != 0):
            now = time.time()
            if (
                self.ultrasonic_pins
                and now - self.last_ultrasonic_hard_check_time >= self.ultrasonic_hard_check_period
            ):
                self.last_ultrasonic_hard_check_time = now
                self.read_ultrasonic()
            if self.ultrasonic_blocked:
                if now - self.last_ultrasonic_hard_stop_log_time >= self.ultrasonic_log_period:
                    self.last_ultrasonic_hard_stop_log_time = now
                    self.get_logger().warn(
                        f'ULTRASONIC HARD STOP before motor command '
                        f'min={self.ultrasonic_min_cm}cm all={self.format_distances(self.ultrasonic_distances)}'
                    )
                self.last_cmd_linear = 0.0
                self.last_cmd_angular = 0.0
                self.force_stop_motor()
                return
        if self.reverse_left:
            left = -left
        if self.reverse_right:
            right = -right
        if self.swap_left_right:
            left, right = right, left
        if self.motor_ser is None or not self.motor_ser.is_open:
            self.open_motor()
            if self.motor_ser is None or not self.motor_ser.is_open:
                return
        try:
            with self.motor_lock:
                self.motor_ser.write(build_md400t_dual_speed_frame(self.dev_id, left, right, self.accel))
                self.motor_ser.flush()
        except Exception as exc:
            self.get_logger().error(f'motor write failed: {exc}')
            try:
                self.motor_ser.close()
            except Exception:
                pass
            self.motor_ser = None

    def stop(self, reason=None):
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        now = time.time()
        if self.ultrasonic_enabled and self.ultrasonic_blocked:
            if now - self.last_forced_stop_time >= 0.10:
                self.last_forced_stop_time = now
                self.force_stop_motor()
        else:
            self.send_motor(0, 0)
        if reason and now - self.last_log_time >= self.log_period:
            self.last_log_time = now
            self.get_logger().info(f'stop: {reason}')

    def control_loop(self):
        self.integrate_pose()
        now = time.time()
        if self.ultrasonic_enabled and self.ultrasonic_pins:
            self.read_ultrasonic()
        if self.ultrasonic_enabled and self.ultrasonic_blocked:
            if self.ultrasonic_min_cm is None:
                self.stop('ultrasonic obstacle')
            else:
                self.stop(
                    f'ultrasonic obstacle sensor={self.ultrasonic_min_index + 1} '
                    f'{self.ultrasonic_min_cm:.1f}cm'
                )
            return
        if not self.have_imu:
            self.stop('waiting for EBIMU')
            return
        if not self.master_path or not self.have_alignment:
            self.stop('waiting for master path')
            return
        if now - self.last_udp_time > self.udp_timeout:
            self.stop('master UDP timeout')
            return
        points = self.transformed_path()
        if len(points) < 2:
            self.stop('waiting for enough path')
            return

        closest_s, cross_track = self.closest_s_on_path(points)
        allowed_s = max(points[0][2], points[-1][2] - self.follow_gap)
        target_s = min(closest_s + self.lookahead, allowed_s)
        tx, ty, _, path_yaw, path_linear, path_angular, path_turn = self.point_at_s(points, target_s)

        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)
        heading_yaw = math.atan2(dy, dx) if dist > 1e-6 else path_yaw
        path_yaw_error = normalize_angle(path_yaw - self.yaw)
        if dist < self.position_yaw_dist and not self.allow_yaw_only_rotate:
            self.stop(f'at target position; yaw-only rotate disabled path_yaw_error={path_yaw_error:.2f}')
            return
        if not self.use_path_yaw:
            yaw_error = normalize_angle(heading_yaw - self.yaw)
        elif points[-1][2] - points[0][2] < self.min_path_s or dist < self.position_yaw_dist:
            yaw_error = path_yaw_error
        else:
            heading_error = normalize_angle(heading_yaw - self.yaw)
            yaw_error = normalize_angle(heading_error + self.k_path_yaw * path_yaw_error)

        if abs(yaw_error) < self.yaw_deadband:
            yaw_error = 0.0

        marker_error, marker_age = self.current_marker_error()
        have_marker = marker_error is not None

        if abs(yaw_error) > self.max_yaw_error:
            if not self.allow_rotate_in_place and not have_marker:
                self.stop(f'heading error too large; rotate-in-place disabled yaw_error={yaw_error:.2f}')
                return
            linear = 0.0
        else:
            turn_scale = max(0.25, 1.0 - abs(yaw_error) / max(self.max_yaw_error, 0.01))
            linear = clamp(self.k_linear * dist * turn_scale, 0.0, self.max_linear)
            # No master velocity feed-forward here. Slave motion is based on
            # its own estimated x/y error to the master path target.
        if linear <= 0.001 and not self.allow_rotate_in_place:
            angular = 0.0
        else:
            angular = clamp(self.k_angular * yaw_error, -self.max_angular, self.max_angular)

        turn_ff = 0.0
        if (
            self.turn_feedforward_enabled
            and linear > 0.001
            and abs(path_angular) >= self.min_turn_angular
        ):
            turn_ff = clamp(
                self.turn_feedforward_sign * self.turn_feedforward_gain * path_angular,
                -self.turn_feedforward_max,
                self.turn_feedforward_max,
            )
            angular = clamp(angular + turn_ff, -self.max_angular, self.max_angular)
            linear *= clamp(self.turn_linear_scale, 0.1, 1.0)
        if have_marker and linear > 0.001:
            camera_angular = clamp(
                self.camera_angular_sign * self.camera_angular_gain * marker_error,
                -self.max_angular,
                self.max_angular,
            )
            angular = camera_angular

        left, right = self.cmd_vel_to_motor(linear, angular)
        self.send_motor(left, right)
        self.last_cmd_linear = linear
        self.last_cmd_angular = angular

        if now - self.last_log_time >= self.log_period:
            self.last_log_time = now
            self.get_logger().info(
                f'follow target=({tx:.2f},{ty:.2f},{path_yaw:.2f}) slave=({self.x:.2f},{self.y:.2f},{self.yaw:.2f}) '
                f'dist={dist:.2f} yaw_error={yaw_error:.2f} path_yaw_error={path_yaw_error:.2f} cross={cross_track:.2f} '
                f'turn={path_turn} path_cmd=({path_linear:.2f},{path_angular:.2f}) ff={turn_ff:.2f} '
                f'aruco={marker_error if have_marker else None} cmd=({linear:.2f},{angular:.2f}) motor=({left},{right}) '
                f's={closest_s:.2f}->{target_s:.2f}/{points[-1][2]:.2f}'
            )

    def destroy_node(self):
        self.running = False
        self.force_stop_motor()
        try:
            self.sock.close()
        except Exception:
            pass
        for ser in (self.imu_ser, self.motor_ser):
            try:
                if ser is not None and ser.is_open:
                    ser.close()
            except Exception:
                pass
        if GPIO is not None and self.ultrasonic_enabled:
            try:
                GPIO.cleanup()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = UdpPathEbimuMotorFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
