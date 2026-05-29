# ArUco 마커 추종 + GPIO20 버튼 실행 시스템

## 1. 작업 개요

이 문서는 **ArUco 마커 기반 슬레이브 추종 시스템**과 **GPIO20 택트 스위치를 이용한 실행/정지 토글 기능**을 정리한 문서입니다.

최종 목표는 다음과 같습니다.

```text
버튼 1번 누름
→ ArUco 추종 시스템 실행

다시 버튼 누름
→ 로봇 정지 및 일시정지

다시 버튼 누름
→ 다시 추종 동작
```

전체 흐름은 다음과 같습니다.

```text
[GPIO20 택트 스위치]
        ↓
[button_run_toggle.py]
        ↓
[run_camera_follow_with_motor.sh 실행]
        ↓
[aruco_follow_cmdvel_web.py]
        ↓
USB 웹캠으로 ArUco 마커 인식
        ↓
/cmd_vel 발행
        ↓
기존 MD400T 모터 노드
        ↓
MD400T 모터 구동
```

---

## 2. 최종 시스템 구조

```text
마스터 로봇
- 뒤쪽에 ArUco 마커 부착

슬레이브 로봇
- USB 웹캠으로 ArUco 마커 인식
- 마커 거리와 좌우 위치 계산
- /cmd_vel 직접 발행
- 기존 MD400T 모터 노드가 /cmd_vel 구독
- GPIO20 버튼으로 실행/정지 제어
```

---

## 3. 사용 장비

| 구분 | 내용 |
|---|---|
| 제어 보드 | Raspberry Pi 4 |
| OS | Ubuntu 22.04 |
| ROS2 | Humble |
| 카메라 | USB 웹캠 |
| 카메라 고정 이름 | `/dev/video_usb_cam` |
| 마커 | ArUco marker ID 0 |
| 모터 드라이버 | MD400T |
| 모터 노드 | `md400t_driver rpi4_cmdvel_motor_node` |
| 버튼 | 택트 스위치 |
| 버튼 GPIO | GPIO20 |
| 버튼 물리핀 | 38번 |
| GND 물리핀 | 39번 |

---

## 4. 파일 구성

작업 파일들은 다음 경로에 정리합니다.

```text
~/git_slave/robot_ws/aruco_follow_code/
├── aruco_follow_cmdvel_web.py
├── camera_only_web.py
├── run_camera_follow_with_motor.sh
└── button_run_toggle.py
```

| 파일 | 역할 |
|---|---|
| `aruco_follow_cmdvel_web.py` | USB 웹캠으로 ArUco 마커 인식, `/cmd_vel` 발행, 웹 화면 출력 |
| `camera_only_web.py` | 모터 없이 카메라 화면만 웹으로 확인 |
| `run_camera_follow_with_motor.sh` | 기존 모터 노드와 ArUco 추종 노드를 함께 실행 |
| `button_run_toggle.py` | GPIO20 버튼 입력을 감지해서 실행/정지/재동작 제어 |

---

## 5. USB 웹캠 고정 설정

USB 웹캠은 `/dev/video0`, `/dev/video2`처럼 번호가 바뀔 수 있습니다.  
따라서 udev rule을 사용해서 고정 이름을 만듭니다.

고정 이름:

```text
/dev/video_usb_cam
```

확인:

```bash
ls -l /dev/video_usb_cam
```

정상 예시:

```text
/dev/video_usb_cam -> video0
```

### 5.1 USB 웹캠 정보 확인

```bash
udevadm info -q property -n /dev/video0 | grep -E "ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL|ID_V4L_PRODUCT|ID_PATH|ID_V4L_CAPABILITIES"
```

확인된 웹캠 정보:

```text
ID_V4L_PRODUCT=2K HD Camera: 2K HD Camera
ID_V4L_CAPABILITIES=:capture:
ID_VENDOR_ID=1b3f
ID_MODEL_ID=1167
ID_SERIAL=GENERAL_2K_HD_Camera
```

### 5.2 udev rule 작성

```bash
sudo nano /etc/udev/rules.d/99-usb-camera.rules
```

내용:

```text
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="1b3f", ATTRS{idProduct}=="1167", ATTR{index}=="0", SYMLINK+="video_usb_cam"
```

적용:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

USB 웹캠을 뺐다가 다시 꽂은 뒤 확인합니다.

```bash
ls -l /dev/video_usb_cam
```

---

## 6. 택트 스위치 회로

버튼 코드는 내부 풀업을 사용합니다.

```python
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
```

따라서 택트 스위치는 **GPIO20과 GND 사이에만 연결**합니다.  
3.3V는 연결하지 않습니다.

### 6.1 핀 연결

| 택트 스위치 | Raspberry Pi 4 |
|---|---|
| 한쪽 다리 | GPIO20, 물리핀 38 |
| 반대쪽 다리 | GND, 물리핀 39 |

회로도:

```text
Raspberry Pi 4

물리핀 38번 GPIO20  ─────┐
                         │
                    [ 택트 스위치 ]
                         │
물리핀 39번 GND     ─────┘
```

동작 방식:

```text
스위치 안 누름 → GPIO20 = HIGH
스위치 누름   → GPIO20 = GND 연결 → LOW
LOW 감지      → 버튼 눌림
```

---

## 7. ROS2 환경 설정

수동 실행 시 기본 환경은 다음과 같습니다.

```bash
cd ~/git_slave/robot_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export OPENCV_VIDEOIO_PRIORITY_GSTREAMER=0
```

---

## 8. ArUco 추종 실행 bash

실행 파일:

```text
~/git_slave/robot_ws/aruco_follow_code/run_camera_follow_with_motor.sh
```

이 파일은 다음 작업을 수행합니다.

```text
1. ROS2 환경 설정
2. 기존 MD400T 모터 노드 실행
3. ArUco 카메라 추종 Python 코드 실행
4. 웹 화면 제공
```

실행:

```bash
~/git_slave/robot_ws/aruco_follow_code/run_camera_follow_with_motor.sh
```

---

## 9. 주요 설정값

`run_camera_follow_with_motor.sh` 안에서 주요 값을 설정합니다.

```bash
export CAMERA_DEVICE=/dev/video_usb_cam
export CMD_VEL_TOPIC=/cmd_vel

export MARKER_ID=0
export MARKER_SIZE_M=0.10

export TARGET_DISTANCE_M=0.30
export DISTANCE_DEADBAND_M=0.05
export EMERGENCY_STOP_DISTANCE_M=0.20

export MAX_LINEAR_MPS=0.03
export MAX_ANGULAR_RADPS=0.04

export KP_LINEAR=0.15
export KP_ANGULAR=0.25

export ANGULAR_SIGN=1.0
```

---

## 10. ArUco 추종 동작

### 10.1 마커가 보이는 경우

```text
마커가 목표 거리보다 멀다
→ 전진

마커가 화면 중앙보다 오른쪽이다
→ 오른쪽으로 회전

마커가 화면 중앙보다 왼쪽이다
→ 왼쪽으로 회전

마커가 목표 거리 근처다
→ 정지 또는 좌우 보정
```

### 10.2 마커가 안 보이는 경우

```text
마커 안 보임
→ NO MARKER - STOP
→ linear.x = 0
→ angular.z = 0
```

---

## 11. 웹 화면 확인

ArUco 추종 실행 시 웹 화면이 함께 제공됩니다.

터미널에 다음과 같이 출력됩니다.

```text
Browser URL: http://192.168.0.140:8080
```

브라우저에서 접속:

```text
http://라즈베리파이IP:8080
```

예시:

```text
http://192.168.0.140:8080
```

웹 화면에서 확인할 수 있는 값:

| 표시 | 의미 |
|---|---|
| 파란색 세로선 | 카메라 화면 중앙선 |
| 초록색 박스 | 인식된 ArUco 마커 |
| `dist` | 카메라와 마커 사이 거리 |
| `err_x` | 마커 중심과 화면 중앙 사이 좌우 오차 |
| `vx` | 전진 속도 명령 |
| `wz` | 회전 속도 명령 |

---

## 12. `/cmd_vel` 연결 확인

ArUco 노드와 모터 노드가 연결되어 있는지 확인합니다.

```bash
ros2 topic info /cmd_vel -v
```

정상 목표:

```text
Publisher count: 1
Subscription count: 1
```

의미:

```text
Publisher count: 1      aruco_cmdvel_web_node가 /cmd_vel 발행
Subscription count: 1   cmdvel_motor_node가 /cmd_vel 구독
```

노드 확인:

```bash
ros2 node list
```

정상 예시:

```text
/aruco_cmdvel_web_node
/cmdvel_motor_node
```

---

## 13. 버튼 제어 Python 코드

파일 위치:

```text
~/git_slave/robot_ws/aruco_follow_code/button_run_toggle.py
```

기능:

```text
1번째 버튼 누름
→ run_camera_follow_with_motor.sh 실행

2번째 버튼 누름
→ /cmd_vel 0 발행
→ 프로세스 일시정지

3번째 버튼 누름
→ 프로세스 재개

4번째 버튼 누름
→ 다시 일시정지
```

버튼 컨트롤러 수동 실행:

```bash
python3 ~/git_slave/robot_ws/aruco_follow_code/button_run_toggle.py
```

GPIO 권한 문제가 있으면:

```bash
sudo -E python3 ~/git_slave/robot_ws/aruco_follow_code/button_run_toggle.py
```

---

## 14. systemd 자동 실행 설정

버튼 감지 프로그램은 항상 실행 중이어야 합니다.  
따라서 systemd 서비스로 등록합니다.

서비스 파일 생성:

```bash
sudo nano /etc/systemd/system/aruco-button.service
```

내용:

```ini
[Unit]
Description=ArUco Follow GPIO20 Button Controller
After=network.target

[Service]
Type=simple
User=root
WorkingDirectory=/home/slave/git_slave/robot_ws
Environment=HOME=/home/slave
Environment=ROS_DOMAIN_ID=30
Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp
Environment=ROS_LOCALHOST_ONLY=0
ExecStart=/usr/bin/python3 -u /home/slave/git_slave/robot_ws/aruco_follow_code/button_run_toggle.py
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
```

서비스 적용:

```bash
sudo systemctl daemon-reload
sudo systemctl enable aruco-button.service
sudo systemctl start aruco-button.service
```

상태 확인:

```bash
sudo systemctl status aruco-button.service
```

정상 상태:

```text
Active: active (running)
```

로그 확인:

```bash
sudo journalctl -u aruco-button.service -f
```

---

## 15. 버튼 동작 확인

로그 확인 상태에서 버튼을 누릅니다.

```bash
sudo journalctl -u aruco-button.service -f
```

첫 번째 버튼 입력:

```text
[BUTTON] Pressed
[BUTTON] START
[STATE] RUNNING
```

두 번째 버튼 입력:

```text
[BUTTON] Pressed
[BUTTON] PAUSE
[STATE] PAUSED
```

세 번째 버튼 입력:

```text
[BUTTON] Pressed
[BUTTON] RESUME
[STATE] RUNNING
```

---

## 16. 카메라 화면만 보기

모터 제어 없이 카메라 화면만 확인하려면 다음 코드를 실행합니다.

```bash
python3 ~/git_slave/robot_ws/aruco_follow_code/camera_only_web.py
```

브라우저에서 접속:

```text
http://라즈베리파이IP:8080
```

이 코드는 `/cmd_vel`을 발행하지 않고, 모터를 움직이지 않습니다.

---

## 17. 방향이 반대로 움직일 때

마커가 파란선 오른쪽에 있는데 로봇이 왼쪽으로 회전하면 `ANGULAR_SIGN`을 반대로 바꿉니다.

```bash
sed -i 's/export ANGULAR_SIGN=.*/export ANGULAR_SIGN=1.0/' ~/git_slave/robot_ws/aruco_follow_code/run_camera_follow_with_motor.sh
```

또는:

```bash
sed -i 's/export ANGULAR_SIGN=.*/export ANGULAR_SIGN=-1.0/' ~/git_slave/robot_ws/aruco_follow_code/run_camera_follow_with_motor.sh
```

---

## 18. 속도 조절

전진 속도 줄이기:

```bash
sed -i 's/export MAX_LINEAR_MPS=.*/export MAX_LINEAR_MPS=0.02/' ~/git_slave/robot_ws/aruco_follow_code/run_camera_follow_with_motor.sh
```

회전 속도 줄이기:

```bash
sed -i 's/export MAX_ANGULAR_RADPS=.*/export MAX_ANGULAR_RADPS=0.03/' ~/git_slave/robot_ws/aruco_follow_code/run_camera_follow_with_motor.sh
```

거리 반응 줄이기:

```bash
sed -i 's/export KP_LINEAR=.*/export KP_LINEAR=0.10/' ~/git_slave/robot_ws/aruco_follow_code/run_camera_follow_with_motor.sh
```

좌우 반응 줄이기:

```bash
sed -i 's/export KP_ANGULAR=.*/export KP_ANGULAR=0.20/' ~/git_slave/robot_ws/aruco_follow_code/run_camera_follow_with_motor.sh
```

---

## 19. 직접 모터 테스트

ArUco 코드를 끄고 직접 `/cmd_vel`을 보내 모터가 움직이는지 확인합니다.

전진 테스트:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

회전 테스트:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.05}}"
```

정지:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

---

## 20. 서비스 관리 명령어

서비스 시작:

```bash
sudo systemctl start aruco-button.service
```

서비스 중지:

```bash
sudo systemctl stop aruco-button.service
```

서비스 재시작:

```bash
sudo systemctl restart aruco-button.service
```

서비스 자동 실행 등록:

```bash
sudo systemctl enable aruco-button.service
```

자동 실행 해제:

```bash
sudo systemctl disable aruco-button.service
```

로그 확인:

```bash
sudo journalctl -u aruco-button.service -f
```

---

## 21. 최종 요약

최종 시스템은 다음과 같이 동작합니다.

```text
1. 라즈베리파이 부팅
2. aruco-button.service 자동 실행
3. GPIO20 버튼 감지 대기
4. 버튼 1번 누름
   → run_camera_follow_with_motor.sh 실행
   → 기존 MD400T 모터 노드 실행
   → ArUco 카메라 추종 시작
5. 다시 버튼 누름
   → /cmd_vel 0 발행
   → 로봇 일시정지
6. 다시 버튼 누름
   → 다시 추종 동작
```

최종 데이터 흐름:

```text
택트 스위치 GPIO20
→ button_run_toggle.py
→ run_camera_follow_with_motor.sh
→ aruco_follow_cmdvel_web.py
→ /cmd_vel
→ cmdvel_motor_node
→ MD400T
→ Slave Robot 이동
```
