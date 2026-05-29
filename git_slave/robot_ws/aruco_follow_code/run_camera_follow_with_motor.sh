#!/usr/bin/env bash
set -e

echo "=================================================="
echo "[SLAVE] Camera ArUco Follow + Existing Motor Node"
echo "=================================================="

cd "$HOME/git_slave/robot_ws"

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export OPENCV_VIDEOIO_PRIORITY_GSTREAMER=0

# 고정된 USB 웹캠 이름
export CAMERA_DEVICE=/dev/video_usb_cam

# 기존 모터 노드가 구독할 토픽
export CMD_VEL_TOPIC=/cmd_vel

# 웹 화면
export WEB_PORT=8080

# ArUco 설정
export MARKER_ID=0
export MARKER_SIZE_M=0.10

# 목표 거리 30cm
export TARGET_DISTANCE_M=0.30
export DISTANCE_DEADBAND_M=0.05
export EMERGENCY_STOP_DISTANCE_M=0.20

# 속도 제한
export MAX_LINEAR_MPS=0.10
export MAX_ANGULAR_RADPS=0.05
export NEAR_TURN_START_M=0.50
export NEAR_KP_ANGULAR=0.70
export NEAR_MAX_ANGULAR_RADPS=0.12

# 제어 게인
export KP_LINEAR=0.20
export KP_ANGULAR=0.25

export CENTER_DEADBAND_PX=50

export CENTER_DEADBAND_NORM=0.08
export TURN_SPEED=0.04

# 방향 반대면 1.0으로 변경
export ANGULAR_SIGN=1.0

echo "[INFO] CAMERA_DEVICE=${CAMERA_DEVICE}"
echo "[INFO] CMD_VEL_TOPIC=${CMD_VEL_TOPIC}"
echo "[INFO] TARGET_DISTANCE_M=${TARGET_DISTANCE_M}"
echo "[INFO] WEB_PORT=${WEB_PORT}"
echo "=================================================="

echo "[INFO] Checking camera..."
if [ ! -e "${CAMERA_DEVICE}" ]; then
  echo "[ERROR] Camera device not found: ${CAMERA_DEVICE}"
  echo "Check: ls -l /dev/video_usb_cam"
  exit 1
fi

echo "[INFO] Stop existing teleop if running..."
pkill -f teleop_twist_keyboard || true

echo "[INFO] Starting existing MD400T motor node..."
ros2 run md400t_driver rpi4_cmdvel_motor_node &
MOTOR_PID=$!

sleep 2

echo "=================================================="
echo "[INFO] Current /cmd_vel info:"
ros2 topic info /cmd_vel || true
echo "=================================================="

echo "[INFO] Starting ArUco camera follow node..."
echo "[INFO] Browser URL: http://$(hostname -I | awk '{print $1}'):${WEB_PORT}"
echo "[INFO] Stop: Ctrl+C"
echo "=================================================="

cleanup() {
  echo ""
  echo "[INFO] Stopping camera follow and motor node..."

  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true

  kill ${MOTOR_PID} 2>/dev/null || true

  echo "[INFO] Stopped."
}

trap cleanup EXIT

# 시작 후 2초 대기
export START_DELAY_SEC=2.0

# 마커 잃어버렸을 때 좌우 탐색
export SEARCH_TURN_SPEED=0.025
export CLOSE_SEARCH_TURN_SPEED=0.025
export CLOSE_SEARCH_TURN_SEC=1.2
export SEARCH_SWITCH_SEC=1.0
export LOST_SEARCH_TIMEOUT_SEC=6.0

python3 "$HOME/git_slave/robot_ws/aruco_follow_code/aruco_follow_cmdvel_web.py"


#!/usr/bin/env bash
# set -e

# echo "=================================================="
# echo "[SLAVE] Camera ArUco Follow + Existing Motor Node"
# echo "=================================================="

# cd "$HOME/git_slave/robot_ws"

# source /opt/ros/humble/setup.bash
# source install/setup.bash

# export ROS_DOMAIN_ID=30
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export ROS_LOCALHOST_ONLY=0
# export OPENCV_VIDEOIO_PRIORITY_GSTREAMER=0

# # 고정된 USB 웹캠 이름
# export CAMERA_DEVICE=/dev/video_usb_cam

# # 기존 모터 노드가 구독할 토픽
# export CMD_VEL_TOPIC=/cmd_vel

# # 웹 화면
# export WEB_PORT=8080

# # ArUco 설정
# export MARKER_ID=0
# export MARKER_SIZE_M=0.10

# # 목표 거리 30cm
# export TARGET_DISTANCE_M=0.30
# export DISTANCE_DEADBAND_M=0.05
# export EMERGENCY_STOP_DISTANCE_M=0.20

# # 속도 제한
# export MAX_LINEAR_MPS=0.04
# export MAX_ANGULAR_RADPS=0.05
# export NEAR_TURN_START_M=0.50
# export NEAR_KP_ANGULAR=0.70
# export NEAR_MAX_ANGULAR_RADPS=0.12

# # 제어 게인
# export KP_LINEAR=0.20
# export KP_ANGULAR=0.25

# export CENTER_DEADBAND_PX=50

# export CENTER_DEADBAND_NORM=0.08
# export TURN_SPEED=0.04

# # 방향 반대면 1.0으로 변경
# export ANGULAR_SIGN=1.0

# echo "[INFO] CAMERA_DEVICE=${CAMERA_DEVICE}"
# echo "[INFO] CMD_VEL_TOPIC=${CMD_VEL_TOPIC}"
# echo "[INFO] TARGET_DISTANCE_M=${TARGET_DISTANCE_M}"
# echo "[INFO] WEB_PORT=${WEB_PORT}"
# echo "=================================================="

# echo "[INFO] Checking camera..."
# if [ ! -e "${CAMERA_DEVICE}" ]; then
#   echo "[ERROR] Camera device not found: ${CAMERA_DEVICE}"
#   echo "Check: ls -l /dev/video_usb_cam"
#   exit 1
# fi

# echo "[INFO] Stop existing teleop if running..."
# pkill -f teleop_twist_keyboard || true

# echo "[INFO] Starting existing MD400T motor node..."

# # 중요:
# # 모터 노드는 별도 세션으로 실행
# # 그래야 버튼 코드에서 카메라 추종 프로세스를 SIGSTOP 해도
# # 모터 노드는 살아있어서 /cmd_vel 0 명령을 받을 수 있음
# setsid ros2 run md400t_driver rpi4_cmdvel_motor_node &
# MOTOR_PID=$!

# sleep 2

# echo "=================================================="
# echo "[INFO] Current ${CMD_VEL_TOPIC} info:"
# ros2 topic info "${CMD_VEL_TOPIC}" || true
# echo "=================================================="

# echo "[INFO] Starting ArUco camera follow node..."
# echo "[INFO] Browser URL: http://$(hostname -I | awk '{print $1}'):${WEB_PORT}"
# echo "[INFO] Stop: Ctrl+C"
# echo "=================================================="

# send_stop_cmd() {
#   echo "[INFO] Sending stop cmd_vel..."

#   timeout 2s ros2 topic pub -r 10 "${CMD_VEL_TOPIC}" geometry_msgs/msg/Twist \
#   "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
# }

# cleanup() {
#   echo ""
#   echo "[INFO] Stopping camera follow and motor node..."

#   send_stop_cmd

#   sleep 0.3

#   if [ -n "${MOTOR_PID}" ]; then
#     echo "[INFO] Killing motor node PID=${MOTOR_PID}"
#     kill "${MOTOR_PID}" 2>/dev/null || true
#     pkill -P "${MOTOR_PID}" 2>/dev/null || true
#   fi

#   echo "[INFO] Stopped."
# }

# trap cleanup EXIT INT TERM

# # 시작 후 2초 대기
# export START_DELAY_SEC=2.0

# # 마커 잃어버렸을 때 좌우 탐색
# export SEARCH_TURN_SPEED=0.025
# export CLOSE_SEARCH_TURN_SPEED=0.025
# export CLOSE_SEARCH_TURN_SEC=1.2
# export SEARCH_SWITCH_SEC=1.0
# export LOST_SEARCH_TIMEOUT_SEC=6.0

# python3 "$HOME/git_slave/robot_ws/aruco_follow_code/aruco_follow_cmdvel_web.py"