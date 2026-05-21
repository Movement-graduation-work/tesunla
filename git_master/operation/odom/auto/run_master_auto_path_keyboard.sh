#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OP_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
ODOM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

SLAVE_IP="${SLAVE_IP:-192.168.0.140}"
SLAVE_PORT="${SLAVE_PORT:-5015}"
RIGHT_PORT="${RIGHT_PORT:-/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISNS9-if00-port0}"
LEFT_PORT="${LEFT_PORT:-/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT94EQPJ-if00-port0}"

pkill -f "[m]d400t_usb2_cmdvel_bridge.py" 2>/dev/null || true
pkill -f "[c]md_vel_udp_sender.py" 2>/dev/null || true
pkill -f "[c]md_vel_odom_udp_sender.py" 2>/dev/null || true
pkill -f "[c]md_odom_path_udp_sender.py" 2>/dev/null || true
pkill -f "[s]lamtec_imu_ros2_node" 2>/dev/null || true
pkill -f "[m]aster_slamtec_path_udp_node.py" 2>/dev/null || true

"$ODOM_DIR/run_slamtec_imu_ros2.sh" > /tmp/master_slamtec_imu.log 2>&1 &
IMU_PID=$!

python3 "$OP_DIR/md400t_usb2_cmdvel_bridge.py" \
  --ros-args \
  -p right_port:="$RIGHT_PORT" \
  -p left_port:="$LEFT_PORT" \
  -p baudrate:=57600 \
  -p dev_id:=1 \
  -p right_cmd_id:=0x82 \
  -p left_cmd_id:=0x82 \
  -p wheel_base:=0.32 \
  -p speed_scale:=400.0 \
  -p max_speed_cmd:=300 \
  -p min_effective_cmd:=80 \
  -p send_hz:=50.0 \
  > /tmp/master_auto_md400t_bridge.log 2>&1 &
BRIDGE_PID=$!

python3 "$SCRIPT_DIR/master_slamtec_path_udp_node.py" \
  --ros-args \
  -p target_ip:="$SLAVE_IP" \
  -p target_port:="$SLAVE_PORT" \
  -p cmd_topic:=/cmd_vel \
  -p yaw_topic:=/imu/processed_yaw \
  -p send_hz:=15.0 \
  -p update_hz:=50.0 \
  -p path_max_points:=160 \
  -p min_path_delta:=0.01 \
  -p min_yaw_delta:=0.06 \
  -p min_turn_angular:=0.08 \
  > /tmp/master_auto_path_udp.log 2>&1 &
PATH_PID=$!

cleanup() {
  python3 - <<'PY' >/dev/null 2>&1 || true
import rclpy
import time
from geometry_msgs.msg import Twist
rclpy.init()
node = rclpy.create_node('master_auto_stop_on_exit')
pub = node.create_publisher(Twist, '/cmd_vel', 10)
time.sleep(0.2)
msg = Twist()
for _ in range(8):
    pub.publish(msg)
    time.sleep(0.05)
node.destroy_node()
rclpy.shutdown()
PY
  kill "$PATH_PID" "$BRIDGE_PID" "$IMU_PID" >/dev/null 2>&1 || true
}
trap cleanup EXIT

sleep 2

for item in \
  "Slamtec IMU:$IMU_PID:/tmp/master_slamtec_imu.log" \
  "master bridge:$BRIDGE_PID:/tmp/master_auto_md400t_bridge.log" \
  "path UDP:$PATH_PID:/tmp/master_auto_path_udp.log"; do
  name="${item%%:*}"
  rest="${item#*:}"
  pid="${rest%%:*}"
  log="${rest#*:}"
  if ! kill -0 "$pid" >/dev/null 2>&1; then
    echo "$name failed to start"
    cat "$log"
    exit 1
  fi
done

echo "[OK] master wheels enabled right=${RIGHT_PORT}, left=${LEFT_PORT}"
echo "[OK] Slamtec IMU -> /imu/processed_yaw"
echo "[OK] autonomous path UDP -> ${SLAVE_IP}:${SLAVE_PORT}"
echo "[LOG] imu      : /tmp/master_slamtec_imu.log"
echo "[LOG] bridge   : /tmp/master_auto_md400t_bridge.log"
echo "[LOG] path udp : /tmp/master_auto_path_udp.log"
echo ""
echo "[MODE] This is path-follow test mode, not wheel-command mirroring."
echo ""

python3 "$OP_DIR/pretty_master_wheel_teleop.py"
