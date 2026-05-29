#!/usr/bin/env python3
import os
import cv2
import time
import threading
import numpy as np
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

CENTER_DEADBAND_PX = float(os.environ.get("CENTER_DEADBAND_PX", "50"))

CAMERA_DEVICE = os.environ.get("CAMERA_DEVICE", "/dev/video_usb_cam")
CMD_VEL_TOPIC = os.environ.get("CMD_VEL_TOPIC", "/cmd_vel")

WEB_HOST = "0.0.0.0"
WEB_PORT = int(os.environ.get("WEB_PORT", "8080"))

MARKER_ID = int(os.environ.get("MARKER_ID", "0"))
MARKER_SIZE_M = float(os.environ.get("MARKER_SIZE_M", "0.10"))
FOCAL_LENGTH_PX = float(os.environ.get("FOCAL_LENGTH_PX", "530.0"))

TARGET_DISTANCE_M = float(os.environ.get("TARGET_DISTANCE_M", "0.30"))
DISTANCE_DEADBAND_M = float(os.environ.get("DISTANCE_DEADBAND_M", "0.05"))
EMERGENCY_STOP_DISTANCE_M = float(os.environ.get("EMERGENCY_STOP_DISTANCE_M", "0.20"))

KP_LINEAR = float(os.environ.get("KP_LINEAR", "0.45"))
KP_ANGULAR = float(os.environ.get("KP_ANGULAR", "1.8"))

MAX_LINEAR_MPS = float(os.environ.get("MAX_LINEAR_MPS", "0.10"))
MAX_ANGULAR_RADPS = float(os.environ.get("MAX_ANGULAR_RADPS", "0.35"))

# 50cm 안쪽에서는 좌우 회전을 더 크게 함
NEAR_TURN_START_M = float(os.environ.get("NEAR_TURN_START_M", "0.50"))
NEAR_KP_ANGULAR = float(os.environ.get("NEAR_KP_ANGULAR", "0.70"))
NEAR_MAX_ANGULAR_RADPS = float(os.environ.get("NEAR_MAX_ANGULAR_RADPS", "0.12"))

# 전진 우선 방식 설정
# 거리 30cm보다 멀면 방향 보정 없이 직진
# 30cm 이내로 들어오면 좌우 방향만 맞춤
CENTER_DEADBAND_NORM = float(os.environ.get("CENTER_DEADBAND_NORM", "0.10"))
TURN_SPEED = float(os.environ.get("TURN_SPEED", "0.08"))

ANGULAR_SIGN = float(os.environ.get("ANGULAR_SIGN", "-1.0"))
ALLOW_REVERSE = os.environ.get("ALLOW_REVERSE", "false").lower() == "true"

CONTROL_RATE_HZ = float(os.environ.get("CONTROL_RATE_HZ", "15.0"))

# 시작 후 대기 시간
START_DELAY_SEC = float(os.environ.get("START_DELAY_SEC", "2.0"))

# 마커를 잃어버렸을 때 좌우 탐색 설정
SEARCH_TURN_SPEED = float(os.environ.get("SEARCH_TURN_SPEED", "0.04"))
SEARCH_SWITCH_SEC = float(os.environ.get("SEARCH_SWITCH_SEC", "1.0"))
LOST_SEARCH_TIMEOUT_SEC = float(os.environ.get("LOST_SEARCH_TIMEOUT_SEC", "6.0"))

# 30cm 이내에서 마커를 잃었을 때: 왼쪽 1번, 오른쪽 1번 확인 후 정지
CLOSE_SEARCH_TURN_SPEED = float(os.environ.get("CLOSE_SEARCH_TURN_SPEED", "0.025"))
CLOSE_SEARCH_TURN_SEC = float(os.environ.get("CLOSE_SEARCH_TURN_SEC", "1.2"))

latest_jpeg = None
latest_lock = threading.Lock()


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


class ArucoCmdvelWebNode(Node):
    def __init__(self):
        super().__init__("aruco_cmdvel_web_node")

        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        self.cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera: {CAMERA_DEVICE}")

        if not hasattr(cv2, "aruco"):
            raise RuntimeError("cv2.aruco가 없습니다. python3-opencv 확인 필요")

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        if hasattr(cv2.aruco, "DetectorParameters"):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.detector = None
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.last_seen_time = 0.0
        self.last_log_time = 0.0

        self.start_time = time.time()
        self.last_seen_distance_m = None
        self.last_seen_center_error_norm = 0.0
        self.lost_start_time = None

        self.timer = self.create_timer(1.0 / CONTROL_RATE_HZ, self.control_loop)

        self.get_logger().info("==================================================")
        self.get_logger().info("[ARUCO CMDVEL WEB NODE] Started")
        self.get_logger().info(f"Camera device       : {CAMERA_DEVICE}")
        self.get_logger().info(f"Publish topic       : {CMD_VEL_TOPIC}")
        self.get_logger().info(f"Target distance     : {TARGET_DISTANCE_M} m")
        self.get_logger().info(f"Emergency stop dist : {EMERGENCY_STOP_DISTANCE_M} m")
        self.get_logger().info(f"Web view            : http://<RPi_IP>:{WEB_PORT}")
        self.get_logger().info("==================================================")

    def detect_markers(self, frame):
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(frame)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                frame,
                self.aruco_dict,
                parameters=self.aruco_params
            )
        return corners, ids

    def select_marker(self, corners, ids):
        if ids is None or len(ids) == 0:
            return None

        ids_flat = ids.flatten()
        best_idx = None
        best_area = -1.0

        for i, marker_id in enumerate(ids_flat):
            if int(marker_id) != MARKER_ID:
                continue

            pts = corners[i][0].astype(np.float32)
            area = abs(cv2.contourArea(pts))

            if area > best_area:
                best_area = area
                best_idx = i

        if best_idx is None:
            return None

        return corners[best_idx][0].astype(np.float32), int(ids_flat[best_idx])

    def estimate_distance(self, pts):
        top_width = np.linalg.norm(pts[1] - pts[0])
        bottom_width = np.linalg.norm(pts[2] - pts[3])
        pixel_width = max(1.0, (top_width + bottom_width) / 2.0)

        distance_m = (MARKER_SIZE_M * FOCAL_LENGTH_PX) / pixel_width
        return float(distance_m)

    def make_cmd(self, center_error_norm, distance_m):
        cmd = Twist()

        # 너무 가까우면 무조건 정지
        if distance_m <= EMERGENCY_STOP_DISTANCE_M:
            return cmd

        # 거리 제어: 목표 거리 30cm까지 접근
        distance_error = distance_m - TARGET_DISTANCE_M

        if abs(distance_error) <= DISTANCE_DEADBAND_M:
            linear = 0.0
        else:
            linear = KP_LINEAR * distance_error

        if not ALLOW_REVERSE:
            linear = max(0.0, linear)

        # 50cm ~ 30cm 구간에서는 좌우 회전을 더 크게 함
        if TARGET_DISTANCE_M < distance_m <= NEAR_TURN_START_M:
            angular = ANGULAR_SIGN * NEAR_KP_ANGULAR * center_error_norm
            max_angular = NEAR_MAX_ANGULAR_RADPS
        else:
            angular = ANGULAR_SIGN * KP_ANGULAR * center_error_norm
            max_angular = MAX_ANGULAR_RADPS

        # 30cm 이내에서는 전진은 멈추고 좌우 방향만 맞춤
        if distance_m <= TARGET_DISTANCE_M:
            linear = 0.0

        cmd.linear.x = float(clamp(linear, -MAX_LINEAR_MPS, MAX_LINEAR_MPS))
        cmd.angular.z = float(clamp(angular, -max_angular, max_angular))

        return cmd

        # 거리 제어
        distance_error = distance_m - TARGET_DISTANCE_M

        if abs(distance_error) <= DISTANCE_DEADBAND_M:
            linear = 0.0
        else:
            linear = KP_LINEAR * distance_error

        if not ALLOW_REVERSE:
            linear = max(0.0, linear)

        # 좌우 제어
        angular = ANGULAR_SIGN * KP_ANGULAR * center_error_norm

        cmd.linear.x = float(clamp(linear, -MAX_LINEAR_MPS, MAX_LINEAR_MPS))
        cmd.angular.z = float(clamp(angular, -MAX_ANGULAR_RADPS, MAX_ANGULAR_RADPS))

        return cmd

        # 중앙 근처면 회전하지 않음
        if abs(center_error_norm) < CENTER_DEADBAND_NORM:
            turn = 0.0
        elif center_error_norm > 0:
            # 마커가 화면 오른쪽
            turn = ANGULAR_SIGN * TURN_SPEED
        else:
            # 마커가 화면 왼쪽
            turn = -ANGULAR_SIGN * TURN_SPEED

        # 1단계: 목표 거리보다 멀면 전진하면서 약하게 방향 보정
        if distance_m > TARGET_DISTANCE_M:
            cmd.linear.x = float(MAX_LINEAR_MPS)
            cmd.angular.z = float(turn)
            cmd.angular.z = float(clamp(cmd.angular.z, -MAX_ANGULAR_RADPS, MAX_ANGULAR_RADPS))
            return cmd

        # 2단계: 30cm 이내면 전진 멈추고 좌우 방향만 맞춤
        cmd.linear.x = 0.0
        cmd.angular.z = float(turn)
        cmd.angular.z = float(clamp(cmd.angular.z, -MAX_ANGULAR_RADPS, MAX_ANGULAR_RADPS))
        return cmd

        # 1단계: 목표 거리 30cm보다 멀면 좌우 보정 없이 직진
        if distance_m > TARGET_DISTANCE_M:
            cmd.linear.x = float(MAX_LINEAR_MPS)
            cmd.angular.z = 0.0
            return cmd

        # 2단계: 30cm 이내에 들어오면 전진 정지 후 좌우 방향만 맞춤
        cmd.linear.x = 0.0

        if center_error_norm > CENTER_DEADBAND_NORM:
            # 마커가 화면 오른쪽에 있음 → 우회전
            cmd.angular.z = float(ANGULAR_SIGN * TURN_SPEED)

        elif center_error_norm < -CENTER_DEADBAND_NORM:
            # 마커가 화면 왼쪽에 있음 → 좌회전
            cmd.angular.z = float(-ANGULAR_SIGN * TURN_SPEED)

        else:
            # 중앙 근처면 정지
            cmd.angular.z = 0.0

        cmd.angular.z = float(clamp(cmd.angular.z, -MAX_ANGULAR_RADPS, MAX_ANGULAR_RADPS))
        return cmd

        # 1단계: 목표 거리 30cm보다 멀면 좌우 보정 없이 직진
        if distance_m > TARGET_DISTANCE_M:
            cmd.linear.x = float(MAX_LINEAR_MPS)
            cmd.angular.z = 0.0
            return cmd

        # 2단계: 30cm 이내에 들어오면 전진 정지 후 좌우 방향만 맞춤
        cmd.linear.x = 0.0

        if center_error_norm > CENTER_DEADBAND_NORM:
            # 마커가 화면 오른쪽에 있음 → 우회전
            cmd.angular.z = float(ANGULAR_SIGN * TURN_SPEED)

        elif center_error_norm < -CENTER_DEADBAND_NORM:
            # 마커가 화면 왼쪽에 있음 → 좌회전
            cmd.angular.z = float(-ANGULAR_SIGN * TURN_SPEED)

        else:
            # 중앙 근처면 정지
            cmd.angular.z = 0.0

        cmd.angular.z = float(clamp(cmd.angular.z, -MAX_ANGULAR_RADPS, MAX_ANGULAR_RADPS))
        return cmd

        distance_error = distance_m - TARGET_DISTANCE_M

        if abs(distance_error) <= DISTANCE_DEADBAND_M:
            linear = 0.0
        else:
            linear = KP_LINEAR * distance_error

        if not ALLOW_REVERSE:
            linear = max(0.0, linear)

        angular = ANGULAR_SIGN * KP_ANGULAR * center_error_norm

        cmd.linear.x = float(clamp(linear, -MAX_LINEAR_MPS, MAX_LINEAR_MPS))
        cmd.angular.z = float(clamp(angular, -MAX_ANGULAR_RADPS, MAX_ANGULAR_RADPS))

        return cmd

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def update_web_frame(self, frame):
        global latest_jpeg

        ok, jpeg = cv2.imencode(".jpg", frame)
        if not ok:
            return

        with latest_lock:
            latest_jpeg = jpeg.tobytes()

    def control_loop(self):
        ok, frame = self.cap.read()
        now = time.time()

        if not ok or frame is None:
            self.publish_stop()
            self.get_logger().warn("Camera frame read failed", throttle_duration_sec=1.0)
            return

        h, w = frame.shape[:2]
        image_center_x = w / 2.0

        corners, ids = self.detect_markers(frame)
        selected = self.select_marker(corners, ids)

        if selected is None:
            # 마커가 안 보이면 거리와 상관없이 무조건 정지
            self.publish_stop()

            status_text = "NO MARKER - STOP"

            cv2.line(frame, (int(image_center_x), 0), (int(image_center_x), h), (255, 0, 0), 2)
            cv2.putText(frame, status_text, (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            self.update_web_frame(frame)

            if now - self.last_log_time > 0.5:
                self.get_logger().info(f"[NO MARKER] {status_text}")
                self.last_log_time = now
            return

        pts, detected_id = selected

        center_x = float(np.mean(pts[:, 0]))
        center_y = float(np.mean(pts[:, 1]))

        center_error_px = center_x - image_center_x

        if abs(center_error_px) < CENTER_DEADBAND_PX:
            center_error_norm = 0.0
        else:
            center_error_norm = center_error_px / image_center_x

        distance_m = self.estimate_distance(pts)

        self.last_seen_time = now
        self.last_seen_distance_m = distance_m
        self.last_seen_center_error_norm = center_error_norm
        self.lost_start_time = None

        # 시작 후 2초 동안은 마커가 보여도 정지
        if now - self.start_time < START_DELAY_SEC:
            cmd = Twist()
        else:
            cmd = self.make_cmd(center_error_norm, distance_m)

        self.cmd_pub.publish(cmd)

        cv2.aruco.drawDetectedMarkers(
            frame,
            [pts.reshape(1, 4, 2)],
            np.array([[detected_id]], dtype=np.int32)
        )

        cv2.circle(frame, (int(center_x), int(center_y)), 6, (0, 255, 0), -1)
        cv2.line(frame, (int(image_center_x), 0), (int(image_center_x), h), (255, 0, 0), 2)

        cv2.putText(frame, f"id={detected_id} dist={distance_m:.2f}m",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.putText(frame, f"err_x={center_error_px:+.0f}px vx={cmd.linear.x:+.2f} wz={cmd.angular.z:+.2f}",
                    (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        self.update_web_frame(frame)

        if now - self.last_log_time > 0.25:
            self.get_logger().info(
                f"[FOLLOW] id={detected_id} "
                f"dist={distance_m:.2f}m "
                f"err_x={center_error_px:+.0f}px "
                f"linear={cmd.linear.x:+.2f} "
                f"angular={cmd.angular.z:+.2f}"
            )
            self.last_log_time = now

    def destroy_node(self):
        try:
            self.publish_stop()
        except Exception:
            pass

        try:
            self.cap.release()
        except Exception:
            pass

        super().destroy_node()


class WebHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()

            html = f"""
            <html>
            <head>
                <title>ArUco Follow</title>
                <meta name="viewport" content="width=device-width, initial-scale=1.0">
            </head>
            <body>
                <h2>ArUco Follow Camera</h2>
                <p>Camera: {CAMERA_DEVICE}</p>
                <p>CmdVel Topic: {CMD_VEL_TOPIC}</p>
                <p>Target Distance: {TARGET_DISTANCE_M} m</p>
                <img src="/stream" width="640">
            </body>
            </html>
            """
            self.wfile.write(html.encode())

        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()

            while True:
                with latest_lock:
                    frame = latest_jpeg

                if frame is None:
                    time.sleep(0.05)
                    continue

                try:
                    self.wfile.write(b"--frame\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(frame)))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    time.sleep(0.03)
                except BrokenPipeError:
                    break
        else:
            self.send_error(404)


def start_web_server():
    server = HTTPServer((WEB_HOST, WEB_PORT), WebHandler)
    print("==================================================")
    print("[WEB VIEW]")
    print(f"URL: http://<RPi_IP>:{WEB_PORT}")
    print("==================================================")
    server.serve_forever()


def main():
    web_thread = threading.Thread(target=start_web_server, daemon=True)
    web_thread.start()

    rclpy.init()
    node = None

    try:
        node = ArucoCmdvelWebNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()