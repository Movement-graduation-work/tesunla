# (1) 전역 변수 초기화
import math
import time
import serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


# (2) 함수 정의: 헤더 포함 LRC 계산
def lrc8_with_header(frame_wo_chk: bytes) -> int:
    # (2-1) 전체 프레임 합을 8비트로 줄이기
    s = sum(frame_wo_chk) & 0xFF
    # (2-2) 2의 보수
    return (-s) & 0xFF


# (3) 함수 정의: signed int16 little-endian 변환
def int16_le_bytes(value: int) -> bytes:
    # (3-1) signed int16 범위 체크
    if value < -32768 or value > 32767:
        raise ValueError(f"speed out of range: {value}")

    # (3-2) 16비트 2의 보수 변환
    u16 = value & 0xFFFF
    lo = u16 & 0xFF
    hi = (u16 >> 8) & 0xFF
    return bytes([lo, hi])


# (4) 함수 정의: 속도 명령 프레임 생성
#     형식: B7 B8 ID 82 02 spdL spdH LRC
def build_single_speed_frame(dev_id: int, speed: int) -> bytes:
    # (4-1) 헤더
    header = bytes([0xB7, 0xB8])

    # (4-2) 속도 바이트
    spd = int16_le_bytes(speed)

    # (4-3) 체크섬 제외 전체 프레임
    frame_wo_chk = header + bytes([
        dev_id & 0xFF,
        0x82,   # (4-3-1) CMD = 130 = 0x82
        0x02    # (4-3-2) LEN = 2
    ]) + spd

    # (4-4) LRC
    chk = lrc8_with_header(frame_wo_chk)

    # (4-5) 최종 프레임
    return frame_wo_chk + bytes([chk])


# (5) 함수 정의: Main Data Req 프레임 생성
#     TX 예시: 183 184 1 4 1 193 202
def build_main_data_req_frame(dev_id: int) -> bytes:
    # (5-1) 헤더
    header = bytes([0xB7, 0xB8])

    # (5-2) 체크섬 제외 전체 프레임
    frame_wo_chk = header + bytes([
        dev_id & 0xFF,
        0x04,   # (5-2-1) Read/Req cmd
        0x01,   # (5-2-2) LEN = 1
        0xC1    # (5-2-3) Main Data Req
    ])

    # (5-3) LRC
    chk = lrc8_with_header(frame_wo_chk)

    # (5-4) 최종 프레임
    return frame_wo_chk + bytes([chk])


# (6) 함수 정의: 각도 정규화
def normalize_angle(angle: float) -> float:
    # (6-1) -pi ~ pi 범위로 정규화
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


# (7) 함수 정의: yaw -> quaternion(z, w)
def yaw_to_quaternion(yaw: float):
    # (7-1) 2D yaw용 quaternion
    half = yaw * 0.5
    qz = math.sin(half)
    qw = math.cos(half)
    return qz, qw


class MD400TCmdVelOdomBridge(Node):
    def __init__(self):
        # (8) 노드 생성
        super().__init__('md400t_cmdvel_odom_bridge')

        # (9) 파라미터 선언
        # (9-1) 포트
        self.declare_parameter('port_right', '/dev/ttyUSB0')
        self.declare_parameter('port_left', '/dev/ttyUSB1')

        # (9-2) 통신
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('id_right', 1)
        self.declare_parameter('id_left', 1)

        # (9-3) 차동구동 기구학
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.32)

        # (9-4) 엔코더 보정값
        self.declare_parameter('counts_per_rev_right', 260.0)
        self.declare_parameter('counts_per_rev_left', 260.0)

        # (9-5) cmd_vel -> 드라이버 변환
        self.declare_parameter('speed_scale', 300.0)
        self.declare_parameter('max_speed_cmd', 300)
        self.declare_parameter('min_effective_cmd', 80)

        # (9-6) 방향 반전
        self.declare_parameter('reverse_right_cmd', False)
        self.declare_parameter('reverse_left_cmd', False)

        # (9-7) 엔코더 부호 반전(필요 시만)
        self.declare_parameter('reverse_right_counts', False)
        self.declare_parameter('reverse_left_counts', False)

        # (9-8) 주기/안전
        self.declare_parameter('send_hz', 20.0)
        self.declare_parameter('feedback_hz', 10.0)
        self.declare_parameter('stale_sec', 0.5)

        # (9-9) 프레임 이름
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # (10) 파라미터 읽기
        self.port_right = str(self.get_parameter('port_right').value)
        self.port_left = str(self.get_parameter('port_left').value)

        self.baudrate = int(self.get_parameter('baudrate').value)
        self.id_right = int(self.get_parameter('id_right').value)
        self.id_left = int(self.get_parameter('id_left').value)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)

        self.counts_per_rev_right = float(self.get_parameter('counts_per_rev_right').value)
        self.counts_per_rev_left = float(self.get_parameter('counts_per_rev_left').value)

        self.speed_scale = float(self.get_parameter('speed_scale').value)
        self.max_speed_cmd = int(self.get_parameter('max_speed_cmd').value)
        self.min_effective_cmd = int(self.get_parameter('min_effective_cmd').value)

        self.reverse_right_cmd = bool(self.get_parameter('reverse_right_cmd').value)
        self.reverse_left_cmd = bool(self.get_parameter('reverse_left_cmd').value)

        self.reverse_right_counts = bool(self.get_parameter('reverse_right_counts').value)
        self.reverse_left_counts = bool(self.get_parameter('reverse_left_counts').value)

        self.send_hz = float(self.get_parameter('send_hz').value)
        self.feedback_hz = float(self.get_parameter('feedback_hz').value)
        self.stale_sec = float(self.get_parameter('stale_sec').value)

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        # (11) 내부 상태: 최근 /cmd_vel
        self.last_cmd_time = 0.0
        self.last_linear_x = 0.0
        self.last_angular_z = 0.0

        # (12) 내부 상태: 최근 드라이버 명령
        self.prev_right_cmd = None
        self.prev_left_cmd = None

        # (13) 내부 상태: odom
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.prev_right_pos = None
        self.prev_left_pos = None
        self.prev_feedback_time = None

        # (14) 내부 상태: 피드백 요청 주기 제어
        self.last_feedback_req_time = 0.0
        self.feedback_period = 1.0 / max(self.feedback_hz, 0.1)

        # (15) 시리얼 포트 열기
        self.ser_right = serial.Serial(self.port_right, self.baudrate, timeout=0.02)
        self.ser_left = serial.Serial(self.port_left, self.baudrate, timeout=0.02)

        # (16) ROS I/O
        # (16-1) /cmd_vel 구독
        self.sub_cmd = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.on_cmd_vel,
            10
        )

        # (16-2) /odom 퍼블리셔
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)

        # (16-3) TF 브로드캐스터
        self.tf_broadcaster = TransformBroadcaster(self)

        # (17) 주기 타이머
        period = 1.0 / max(self.send_hz, 0.1)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"[start] "
            f"right={self.port_right}(id={self.id_right}), "
            f"left={self.port_left}(id={self.id_left}), "
            f"wheel_radius={self.wheel_radius}, wheel_base={self.wheel_base}, "
            f"counts_per_rev_right={self.counts_per_rev_right}, "
            f"counts_per_rev_left={self.counts_per_rev_left}"
        )

    def on_cmd_vel(self, msg: Twist):
        # (18) /cmd_vel 저장
        self.last_linear_x = float(msg.linear.x)
        self.last_angular_z = float(msg.angular.z)
        self.last_cmd_time = time.time()

    def on_timer(self):
        # (19) 현재 시간
        now = time.time()

        # (20) stale 처리
        if (now - self.last_cmd_time) > self.stale_sec:
            right_cmd = 0
            left_cmd = 0
        else:
            right_cmd, left_cmd = self.cmd_vel_to_driver_cmd(
                self.last_linear_x,
                self.last_angular_z
            )

        # (21) 방향 반전(cmd)
        if self.reverse_right_cmd:
            right_cmd = -right_cmd
        if self.reverse_left_cmd:
            left_cmd = -left_cmd

        # (22) 값 바뀔 때만 로그
        if right_cmd != self.prev_right_cmd or left_cmd != self.prev_left_cmd:
            self.get_logger().info(
                f"[driver_cmd] right={right_cmd}, left={left_cmd}"
            )
            self.prev_right_cmd = right_cmd
            self.prev_left_cmd = left_cmd

        # (23) 속도 명령 전송
        self.send_speed(self.ser_right, self.id_right, right_cmd)
        self.send_speed(self.ser_left, self.id_left, left_cmd)

        # (24) 피드백 읽기 주기 도달 시 Main Data Req
        if (now - self.last_feedback_req_time) >= self.feedback_period:
            self.last_feedback_req_time = now
            self.poll_feedback_and_update_odom()

    def cmd_vel_to_driver_cmd(self, linear_x: float, angular_z: float):
        # (25) 차동구동 계산
        right_mps = linear_x + (angular_z * self.wheel_base / 2.0)
        left_mps = linear_x - (angular_z * self.wheel_base / 2.0)

        # (25-1) m/s -> 드라이버 speed 값
        right_cmd = int(round(right_mps * self.speed_scale))
        left_cmd = int(round(left_mps * self.speed_scale))

        # (25-2) 최대값 제한
        right_cmd = max(-self.max_speed_cmd, min(self.max_speed_cmd, right_cmd))
        left_cmd = max(-self.max_speed_cmd, min(self.max_speed_cmd, left_cmd))

        # (25-3) 너무 작은 값은 최소치 보정
        if right_cmd != 0 and abs(right_cmd) < self.min_effective_cmd:
            right_cmd = self.min_effective_cmd if right_cmd > 0 else -self.min_effective_cmd

        if left_cmd != 0 and abs(left_cmd) < self.min_effective_cmd:
            left_cmd = self.min_effective_cmd if left_cmd > 0 else -self.min_effective_cmd

        return right_cmd, left_cmd

    def send_speed(self, ser, dev_id: int, speed: int):
        # (26) 속도 프레임 전송
        frame = build_single_speed_frame(dev_id, speed)
        ser.write(frame)
        ser.flush()

    def read_main_data_position(self, ser, dev_id: int):
        # (27) 입력 버퍼 비우기
        ser.reset_input_buffer()

        # (28) Main Data Req 전송
        req = build_main_data_req_frame(dev_id)
        ser.write(req)
        ser.flush()

        # (29) 응답 수신
        resp = self.read_response(ser, expected_len=23, timeout_sec=0.05)
        if resp is None:
            return None

        # (30) 응답 파싱
        pos = self.parse_main_data_position(resp)
        return pos

    def read_response(self, ser, expected_len: int, timeout_sec: float):
        # (31) 응답을 일정 길이까지 읽기
        deadline = time.time() + timeout_sec
        buf = bytearray()

        while time.time() < deadline:
            n = ser.in_waiting
            if n > 0:
                buf.extend(ser.read(n))
                if len(buf) >= expected_len:
                    break
            else:
                time.sleep(0.002)

        if len(buf) < expected_len:
            return None

        return bytes(buf[:expected_len])

    def parse_main_data_position(self, resp: bytes):
        # (32) 최소 길이 검사
        if len(resp) < 23:
            return None

        # (33) 응답 헤더 검사
        #      실제 응답은 B8 B7 로 보였음
        if resp[0] != 0xB8 or resp[1] != 0xB7:
            return None

        # (34) cmd, len 검사
        cmd = resp[3]
        length = resp[4]
        if cmd != 0xC1 or length != 17:
            return None

        # (35) payload 추출
        payload = resp[5:5 + 17]

        # (36) Position 바이트 추출
        #      실측 기반: payload[10], payload[11] = Position little-endian
        pos_lo = payload[10]
        pos_hi = payload[11]
        pos = pos_lo | (pos_hi << 8)

        return pos

    def wrap_delta_16bit(self, new: int, old: int) -> int:
        # (37) 16비트 unsigned wrap-around 처리
        delta = int(new) - int(old)
        if delta > 32767:
            delta -= 65536
        elif delta < -32768:
            delta += 65536
        return delta

    def poll_feedback_and_update_odom(self):
        # (38) 우측/좌측 Position 읽기
        right_pos = self.read_main_data_position(self.ser_right, self.id_right)
        left_pos = self.read_main_data_position(self.ser_left, self.id_left)

        # (39) 읽기 실패 시 종료
        if right_pos is None or left_pos is None:
            self.get_logger().warn("[feedback] failed to read main data position")
            return

        # (40) 첫 샘플이면 기준값만 저장
        now = self.get_clock().now().to_msg()
        now_sec = time.time()

        if self.prev_right_pos is None or self.prev_left_pos is None or self.prev_feedback_time is None:
            self.prev_right_pos = right_pos
            self.prev_left_pos = left_pos
            self.prev_feedback_time = now_sec
            self.get_logger().info(
                f"[feedback init] right_pos={right_pos}, left_pos={left_pos}"
            )
            return

        # (41) delta count 계산
        d_right_counts = self.wrap_delta_16bit(right_pos, self.prev_right_pos)
        d_left_counts = self.wrap_delta_16bit(left_pos, self.prev_left_pos)

        # (42) 엔코더 부호 반전(필요 시)
        if self.reverse_right_counts:
            d_right_counts = -d_right_counts
        if self.reverse_left_counts:
            d_left_counts = -d_left_counts

        # (43) 시간차
        dt = now_sec - self.prev_feedback_time
        if dt <= 0.0:
            dt = 1e-3

        # (44) count -> rev
        d_right_rev = d_right_counts / self.counts_per_rev_right
        d_left_rev = d_left_counts / self.counts_per_rev_left

        # (45) rev -> distance
        wheel_circumference = 2.0 * math.pi * self.wheel_radius
        d_right = d_right_rev * wheel_circumference
        d_left = d_left_rev * wheel_circumference

        # (46) 차동구동 odom 적분
        ds = 0.5 * (d_right + d_left)
        dtheta = (d_right - d_left) / self.wheel_base

        self.x += ds * math.cos(self.yaw + 0.5 * dtheta)
        self.y += ds * math.sin(self.yaw + 0.5 * dtheta)
        self.yaw = normalize_angle(self.yaw + dtheta)

        # (47) 속도 계산
        vx = ds / dt
        wz = dtheta / dt

        # (48) /odom 발행
        self.publish_odom(now, vx, wz)

        # (49) TF 발행
#        self.publish_tf(now)

        # (50) 다음 루프 대비 저장
        self.prev_right_pos = right_pos
        self.prev_left_pos = left_pos
        self.prev_feedback_time = now_sec

        # (51) 디버그 로그
        self.get_logger().info(
            f"[odom] right_pos={right_pos}, left_pos={left_pos}, "
            f"d_right_counts={d_right_counts}, d_left_counts={d_left_counts}, "
            f"x={self.x:.3f}, y={self.y:.3f}, yaw={self.yaw:.3f}"
        )

    def publish_odom(self, stamp, vx: float, wz: float):
        # (52) quaternion 계산
        qz, qw = yaw_to_quaternion(self.yaw)

        # (53) Odometry 메시지 생성
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.z = float(qz)
        msg.pose.pose.orientation.w = float(qw)

        msg.twist.twist.linear.x = float(vx)
        msg.twist.twist.angular.z = float(wz)

        # (54) 발행
        self.pub_odom.publish(msg)

    def publish_tf(self, stamp):
        # (55) quaternion 계산
        qz, qw = yaw_to_quaternion(self.yaw)

        # (56) TransformStamped 생성
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame

        tf.transform.translation.x = float(self.x)
        tf.transform.translation.y = float(self.y)
        tf.transform.translation.z = 0.0

        tf.transform.rotation.z = float(qz)
        tf.transform.rotation.w = float(qw)

        # (57) 발행
        self.tf_broadcaster.sendTransform(tf)

    def stop_all(self):
        # (58) 양쪽 정지
        try:
            self.send_speed(self.ser_right, self.id_right, 0)
            time.sleep(0.01)
            self.send_speed(self.ser_left, self.id_left, 0)
        except Exception:
            pass

    def destroy_node(self):
        # (59) 종료 시 정지 + 포트 닫기
        try:
            self.stop_all()
        except Exception:
            pass

        try:
            if hasattr(self, 'ser_right') and self.ser_right and self.ser_right.is_open:
                self.ser_right.close()
        except Exception:
            pass

        try:
            if hasattr(self, 'ser_left') and self.ser_left and self.ser_left.is_open:
                self.ser_left.close()
        except Exception:
            pass

        super().destroy_node()


# (60) 메인 실행
def main():
    # (60-1) ROS2 시작
    rclpy.init()

    # (60-2) 노드 생성
    node = MD400TCmdVelOdomBridge()

    try:
        # (60-3) spin
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # (60-4) 종료 정리
        node.destroy_node()
        rclpy.shutdown()


# (61) 엔트리포인트
if __name__ == '__main__':
    main()
