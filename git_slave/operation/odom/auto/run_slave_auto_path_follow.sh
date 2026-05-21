#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RCUTILS_COLORIZED_OUTPUT="${RCUTILS_COLORIZED_OUTPUT:-0}"
export RCUTILS_CONSOLE_OUTPUT_FORMAT="${RCUTILS_CONSOLE_OUTPUT_FORMAT:-[{severity}] [{time}] [{name}]: {message}}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

BIND_IP="${BIND_IP:-0.0.0.0}"
UDP_PORT="${UDP_PORT:-5015}"
IMU_PORT="${IMU_PORT:-/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0}"
MOTOR_PORT="${MOTOR_PORT:-/dev/ttyUSB0}"
ULTRASONIC_ENABLED="${ULTRASONIC_ENABLED:-true}"
ULTRASONIC_STOP_CM="${ULTRASONIC_STOP_CM:-50.0}"
ULTRASONIC_RELEASE_CM="${ULTRASONIC_RELEASE_CM:-55.0}"
ULTRASONIC_PINS="${ULTRASONIC_PINS:-12:6,17:27,22:23,24:25}"
ULTRASONIC_STATUS_LOG_PERIOD="${ULTRASONIC_STATUS_LOG_PERIOD:-1.0}"
ULTRASONIC_HARD_CHECK_PERIOD="${ULTRASONIC_HARD_CHECK_PERIOD:-0.05}"
CAMERA_GUIDANCE_ENABLED="${CAMERA_GUIDANCE_ENABLED:-false}"
CAMERA_GUIDANCE_MODE="${CAMERA_GUIDANCE_MODE:-color}"
CAMERA_DEVICE="${CAMERA_DEVICE:-/dev/video0}"
ARUCO_MARKER_ID="${ARUCO_MARKER_ID:-0}"
CAMERA_ANGULAR_GAIN="${CAMERA_ANGULAR_GAIN:-0.08}"
CAMERA_ANGULAR_SIGN="${CAMERA_ANGULAR_SIGN:--1.0}"
GREEN_H_MIN="${GREEN_H_MIN:-35}"
GREEN_H_MAX="${GREEN_H_MAX:-85}"
GREEN_S_MIN="${GREEN_S_MIN:-80}"
GREEN_V_MIN="${GREEN_V_MIN:-80}"
GREEN_MIN_AREA="${GREEN_MIN_AREA:-1200.0}"

pkill -f "[u]dp_path_ebimu_motor_follower.py" 2>/dev/null || true
pkill -f "[u]dp_cmdvel_motor_node.py" 2>/dev/null || true
pkill -f "[u]dp_cmdvel_odom_motor_node.py" 2>/dev/null || true
pkill -f "[u]dp_odom_path_motor_follower.py" 2>/dev/null || true
pkill -f "[s]lave_path_follower_node.py" 2>/dev/null || true
pkill -f "[u]sb_camera_web_view.py" 2>/dev/null || true

echo "=================================================="
echo "[SLAVE AUTO PATH FOLLOW]"
echo "[MODE] autonomous x/y/yaw path follow, not cmd_vel mirroring"
echo "[UDP] listen=${BIND_IP}:${UDP_PORT}"
echo "[IMU] ${IMU_PORT}@115200"
echo "[MOTOR] ${MOTOR_PORT}@57600"
echo "[ULTRASONIC] enabled=${ULTRASONIC_ENABLED}"
echo "[ULTRASONIC] stop<=${ULTRASONIC_STOP_CM}cm, release>=${ULTRASONIC_RELEASE_CM}cm"
echo "[ULTRASONIC] pins echo:trig=${ULTRASONIC_PINS}"
echo "[ULTRASONIC] status log period=${ULTRASONIC_STATUS_LOG_PERIOD}s"
echo "[ULTRASONIC] hard check period=${ULTRASONIC_HARD_CHECK_PERIOD}s"
echo "[CAMERA] guidance=${CAMERA_GUIDANCE_ENABLED}, mode=${CAMERA_GUIDANCE_MODE}, device=${CAMERA_DEVICE}"
echo "[CAMERA] green HSV h=${GREEN_H_MIN}-${GREEN_H_MAX}, s>=${GREEN_S_MIN}, v>=${GREEN_V_MIN}, area>=${GREEN_MIN_AREA}"
echo "[CAMERA] angular_gain=${CAMERA_ANGULAR_GAIN}, angular_sign=${CAMERA_ANGULAR_SIGN}"
echo "[TURN] spatial turn feed-forward enabled=true, gain=0.35, max=0.04"
echo "=================================================="

python3 "$SCRIPT_DIR/udp_path_ebimu_motor_follower.py" \
  --ros-args \
  -p bind_ip:="$BIND_IP" \
  -p udp_port:="$UDP_PORT" \
  -p imu_port:="$IMU_PORT" \
  -p motor_port:="$MOTOR_PORT" \
  -p motor_baudrate:=57600 \
  -p imu_baudrate:=115200 \
  -p wheel_radius:=0.05 \
  -p wheel_base:=0.32 \
  -p speed_scale:=300.0 \
  -p max_speed_cmd:=65 \
  -p min_effective_cmd:=0 \
  -p accel:=2 \
  -p stop_accel:=255 \
  -p stop_repeat:=6 \
  -p follow_gap:=0.45 \
  -p lookahead:=0.10 \
  -p max_linear:=0.018 \
  -p max_angular:=0.05 \
  -p k_linear:=0.30 \
  -p k_angular:=0.20 \
  -p max_yaw_error:=1.2 \
  -p k_path_yaw:=0.0 \
  -p yaw_deadband:=0.04 \
  -p position_yaw_dist:=0.10 \
  -p allow_yaw_only_rotate:=false \
  -p use_path_yaw:=false \
  -p allow_rotate_in_place:=false \
  -p ultrasonic_enabled:="$ULTRASONIC_ENABLED" \
  -p ultrasonic_stop_cm:="$ULTRASONIC_STOP_CM" \
  -p ultrasonic_release_cm:="$ULTRASONIC_RELEASE_CM" \
  -p ultrasonic_pins:="$ULTRASONIC_PINS" \
  -p ultrasonic_status_log_period:="$ULTRASONIC_STATUS_LOG_PERIOD" \
  -p ultrasonic_hard_check_period:="$ULTRASONIC_HARD_CHECK_PERIOD" \
  -p camera_guidance_enabled:="$CAMERA_GUIDANCE_ENABLED" \
  -p camera_guidance_mode:="$CAMERA_GUIDANCE_MODE" \
  -p camera_device:="$CAMERA_DEVICE" \
  -p aruco_marker_id:="$ARUCO_MARKER_ID" \
  -p green_h_min:="$GREEN_H_MIN" \
  -p green_h_max:="$GREEN_H_MAX" \
  -p green_s_min:="$GREEN_S_MIN" \
  -p green_v_min:="$GREEN_V_MIN" \
  -p green_min_area:="$GREEN_MIN_AREA" \
  -p camera_angular_gain:="$CAMERA_ANGULAR_GAIN" \
  -p camera_angular_sign:="$CAMERA_ANGULAR_SIGN" \
  -p turn_feedforward_enabled:=true \
  -p turn_feedforward_gain:=0.35 \
  -p turn_feedforward_sign:=1.0 \
  -p turn_feedforward_max:=0.04 \
  -p turn_linear_scale:=0.65 \
  -p min_turn_angular:=0.08 \
  -p invert_turn:=true \
  -p swap_left_right:=true \
  -p reverse_left:=false \
  -p reverse_right:=false
