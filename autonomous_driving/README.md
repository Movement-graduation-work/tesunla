
## 🔧 하드웨어 구성

| 구성품 | 모델 | 사양 |
|:------:|:----:|:----:|
| 🛞 구동 모터 | MDH180 인휠 모터 | 바퀴 지름 171.5mm |
| 📡 LiDAR | YDLIDAR G4 | 2D, 최대 12m |
| ⚙️ 모터 드라이버 | MD400T | 시리얼 57600 baud |
| 💻 컴퓨터 | NVIDIA Jetson | aarch64 |

차량 제원
- \`wheel_base\` (좌우 바퀴 간격): 0.45m
- \`wheel_radius\` (바퀴 반지름): 0.08575m
- 차체 크기: 약 0.74m × 0.45m

---

📁 파일 구성

| 파일 | 역할 |
|:-----|:-----|
| 🐍 \`md400t_cmdvel_odom_bridge.py\` | MD400T 모터 제어 + odometry 발행 |
| ⚙️ \`my_nav2_params.yaml\` | Nav2 파라미터 (footprint, 속도, costmap) |
| 📡 \`ydlidar_retry.yaml\` | YDLIDAR G4 드라이버 설정 |
| 🗺️ \`ydlidar_2d.lua\` | Cartographer SLAM 설정 |

---

 🚀 실행 순서

> 💡 각 명령은 !!별도의 터미널!!에서 실행합니다. 순서대로 진행하세요.

 0️⃣ 컨테이너 시작


\`\`\`bash
docker start yee_humble_g4
docker exec -it yee_humble_g4 bash
\`\`\`

### 1️⃣ LiDAR

\`\`\`bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
source /root/yee_g4_ws/install/setup.bash
chmod 777 /dev/serial/by-id/*
ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node \
  --ros-args --params-file /root/ydlidar_retry.yaml
\`\`\`

### 2️⃣ Cartographer (SLAM)

\`\`\`bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
ros2 run cartographer_ros cartographer_node \
  -configuration_directory /root/ydlidar_slam/config \
  -configuration_basename ydlidar_2d.lua
\`\`\`

### 3️⃣ Occupancy Grid (\`/map\` 생성)

\`\`\`bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
ros2 run cartographer_ros cartographer_occupancy_grid_node \
  -resolution 0.05 \
  -publish_period_sec 1.0
\`\`\`

### 4️⃣ TF (\`base_link\` → \`laser_frame\`)

\`\`\`bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0.45 \
  --yaw 0 --pitch 0 --roll 0 \
  --frame-id base_link --child-frame-id laser_frame
\`\`\`

> ⚠️ \`yaw\` 값은 LiDAR 장착 방향에 따라 조정 (라디안 단위). 정면 정렬 시 \`0\`.

### 5️⃣ 모터 브리지

\`\`\`bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
python3 /root/md400t_cmdvel_odom_bridge.py --ros-args \
  -p port_right:=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISNS9-if00-port0 \
  -p port_left:=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT94EQPJ-if00-port0 \
  -p baudrate:=57600 \
  -p id_right:=1 \
  -p id_left:=1 \
  -p wheel_base:=0.45 \
  -p wheel_radius:=0.08575 \
  -p counts_per_rev_right:=1280.0 \
  -p counts_per_rev_left:=1280.0 \
  -p speed_scale:=300.0 \
  -p max_speed_cmd:=300 \
  -p min_effective_cmd:=80 \
  -p send_hz:=20.0 \
  -p feedback_hz:=10.0 \
  -p stale_sec:=0.5
\`\`\`

### 6️⃣ Nav2 (자율주행)

\`\`\`bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  params_file:=/root/my_nav2_params.yaml
\`\`\`

### 7️⃣ RViz (별도 컨테이너)

**호스트에서:**
\`\`\`bash
xhost +local:docker
docker run -it --net=host \
  -e DISPLAY=:1.0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --privileged yee_humble_friendbase:latest bash
\`\`\`

**컨테이너 안에서:**
\`\`\`bash
source /opt/ros/humble/setup.bash
rviz2
\`\`\`

---

## 🖥️ RViz 설정

**Fixed Frame**: \`map\`

| 토픽 | 타입 | QoS 설정 |
|:-----|:-----|:---------|
| \`/map\` | Map | Reliable / Transient Local |
| \`/scan\` | LaserScan | Best Effort |
| \`/odom\` | Odometry | - |
| \`/global_costmap/costmap\` | Map | - |
| \`/local_costmap/costmap\` | Map | - |
| \`TF\` | TF | - |

> 📌 **로컬 코스트맵**과 **글로벌 코스트맵**을 반드시 추가해야 합니다.
> - \`/global_costmap/costmap\` → 전체 맵 기준 장애물 영역
> - \`/local_costmap/costmap\` → 로봇 주변 실시간 장애물 영역
