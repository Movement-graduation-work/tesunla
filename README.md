# 테순라 : 무선 추종 군집 자율주행 차 최종 코드

## 실행 방법
---
## Master 잿슨나노

  - cd ~/git_master/operation/odom/auto
  - bash run_master_auto_path_keyboard.sh

## 구성:

  - Slamtec IMU 실행
  - 마스터 바퀴 제어 브리지 실행
  - /cmd_vel 기반 마스터 odometry/path 생성
  - 키보드 조종 실행
---
## Slave - 라파4
  - cd ~/git_slave/operation/odom/auto
  - bash run_slave_auto_path_follow.sh

## 구성:

  - EBIMU yaw 수신
  - 마스터 UDP path 수신
  - 슬레이브 자기 위치 odometry 추정
  - 마스터 path 좌표 추종
  - 초음파 장애물 정지
  - 모터 제어 실행
---
## 사용 로직:

1. 마스터 키보드 주행

  - 마스터는 키보드 입력으로 /cmd_vel을 생성한다.
  - /cmd_vel은 마스터 모터 구동에 사용된다.
  - 동시에 /cmd_vel의 선속도와 각속도는 path 생성에도 사용된다.

  2. 마스터 IMU 기반 Odometry

  - Slamtec IMU의 yaw 값을 사용한다.
  - 마스터의 선속도와 yaw를 적분해 x, y 위치를 계산한다.
  - 계산된 위치는 마스터 odometry와 path로 저장된다.

  3. UDP Path 송신

  - 마스터는 일정 주기로 슬레이브에게 UDP 패킷을 보낸다.
  - UDP 데이터에는 다음 값이 포함된다.

  cmd_vel
  master odom x, y, yaw
  latest_s
  path points

  각 path point에는 다음 값이 들어간다.

  x
  y
  yaw
  s
  linear_x
  angular_z
  turn

  turn 값:

  left
  right
  straight

  4. 슬레이브 좌표 정렬

  - 슬레이브는 첫 번째 마스터 path point를 기준으로 좌표계를 정
    렬한다.
  - 슬레이브의 시작 위치와 마스터의 시작 위치를 맞춘다.
  - 이후 마스터 path를 슬레이브 좌표계로 변환해서 따라간다.

  5. 슬레이브 자율 Path 추종

  - 슬레이브는 마스터의 현재 cmd_vel을 그대로 따라 하지 않는다.
  - 슬레이브는 자신의 위치와 마스터 path 목표 좌표를 비교한다.
  - 목표 좌표까지의 거리와 방향 오차를 계산해 모터 명령을 만든
    다.
  - 따라서 단순 미러링이 아니라 좌표 기반 자율 추종 방식이다.

  6. 위치 기반 회전 추종

  - 마스터가 특정 위치에서 좌회전 또는 우회전하면 path point에
    회전 정보가 저장된다.
  - 슬레이브는 그 위치에 도달했을 때 저장된 angular_z, turn 값을
    참고한다.
  - 이 값을 이용해 직진 중 회전 보정을 추가한다.
  - 즉, 현재 마스터 명령을 즉시 따라 하는 것이 아니라 “마스터가
    그 위치에서 했던 움직임”을 따라간다.

  7. 초음파 안전 정지

  - 슬레이브에는 초음파 센서 4개가 연결되어 있다.
  - 주행 중에도 계속 초음파 거리를 확인한다.
  - 장애물이 가까워지면 좌표 추종보다 안전 정지를 우선한다.

  현재 설정:

  stop <= 50cm
  release >= 55cm

  동작:

  50cm 이하 장애물 감지 -> 즉시 정지
  55cm 이상으로 멀어짐 -> 다시 path 추종 시작

  8. 모터 제어

  - 슬레이브는 계산된 linear_x, angular_z를 좌/우 바퀴 속도로 변
    환한다.
  - 변환된 값은 MD400T 모터 드라이버로 전송된다.
  - 초음파 정지 상태에서는 모터 명령보다 정지 명령이 우선된다.
