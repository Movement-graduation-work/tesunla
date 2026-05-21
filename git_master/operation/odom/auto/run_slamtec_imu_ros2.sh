#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN="$SCRIPT_DIR/slamtec_imu_ros2_install/lib/slamtec_imu_ros2/slamtec_imu_ros2_node"

if [[ ! -x "$BIN" ]]; then
  echo "Node binary not found. Build first:" >&2
  echo "  cd $SCRIPT_DIR && ./build_slamtec_imu_ros2.sh" >&2
  exit 1
fi

exec "$BIN" --ros-args \
  -p usb_vendor_id:=64719 \
  -p usb_product_id:=61696 \
  -p usb_interface_id:=3 \
  -p usb_tx_endpoint:=5 \
  -p usb_rx_endpoint:=5 \
  -p frame_id:=imu \
  -p publish_hz:=200.0
