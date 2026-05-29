#!/usr/bin/env python3
import os
import signal
import subprocess
import time

import RPi.GPIO as GPIO


BUTTON_PIN = 20

USER_HOME = "/home/slave"
WORKSPACE = f"{USER_HOME}/git_slave/robot_ws"
ARUCO_DIR = f"{WORKSPACE}/aruco_follow_code"
RUN_SCRIPT = f"{ARUCO_DIR}/run_camera_follow_with_motor.sh"

ROS_SETUP = "/opt/ros/humble/setup.bash"
WS_SETUP = f"{WORKSPACE}/install/setup.bash"
CMD_VEL_TOPIC = "/cmd_vel"

process = None
paused = False


def send_stop_cmd():
    cmd = (
        f"source {ROS_SETUP} && "
        f"source {WS_SETUP} && "
        f"export ROS_DOMAIN_ID=30 && "
        f"export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && "
        f"export ROS_LOCALHOST_ONLY=0 && "
        f"ros2 topic pub --once {CMD_VEL_TOPIC} geometry_msgs/msg/Twist "
        f"'{{linear: {{x: 0.0}}, angular: {{z: 0.0}}}}'"
    )

    subprocess.run(
        ["bash", "-lc", cmd],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        timeout=3
    )


def is_running():
    global process
    return process is not None and process.poll() is None


def start_robot():
    global process, paused

    print("[BUTTON] START", flush=True)

    env = os.environ.copy()
    env["HOME"] = USER_HOME
    env["ROS_DOMAIN_ID"] = "30"
    env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    env["ROS_LOCALHOST_ONLY"] = "0"
    env["OPENCV_VIDEOIO_PRIORITY_GSTREAMER"] = "0"

    process = subprocess.Popen(
        ["bash", RUN_SCRIPT],
        cwd=WORKSPACE,
        env=env,
        preexec_fn=os.setsid
    )

    paused = False
    print("[STATE] RUNNING", flush=True)


def pause_robot():
    global paused

    if not is_running():
        print("[WARN] Cannot pause. Process is not running.", flush=True)
        return

    print("[BUTTON] PAUSE", flush=True)

    try:
        send_stop_cmd()
    except Exception as e:
        print(f"[WARN] stop cmd failed: {e}", flush=True)

    time.sleep(0.2)

    os.killpg(os.getpgid(process.pid), signal.SIGSTOP)

    paused = True
    print("[STATE] PAUSED", flush=True)


def resume_robot():
    global paused

    if not is_running():
        print("[WARN] Process not running. Start again.", flush=True)
        start_robot()
        return

    print("[BUTTON] RESUME", flush=True)

    os.killpg(os.getpgid(process.pid), signal.SIGCONT)

    paused = False
    print("[STATE] RUNNING", flush=True)


def handle_button_press():
    global paused

    print("[BUTTON] Pressed", flush=True)

    if not is_running():
        start_robot()
    elif not paused:
        pause_robot()
    else:
        resume_robot()


def cleanup():
    global process

    print("[INFO] Cleanup", flush=True)

    try:
        send_stop_cmd()
    except Exception:
        pass

    if is_running():
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        except Exception:
            pass

    GPIO.cleanup()


def main():
    print("==================================================", flush=True)
    print("[GPIO20 Button Toggle Controller]", flush=True)
    print("1st press : start run_camera_follow_with_motor.sh", flush=True)
    print("next press: pause", flush=True)
    print("next press: resume", flush=True)
    print("==================================================", flush=True)

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    # 버튼 한쪽: GPIO20
    # 버튼 다른쪽: GND
    # 내부 풀업 사용
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    last_state = GPIO.input(BUTTON_PIN)
    last_press_time = 0.0

    try:
        while True:
            current_state = GPIO.input(BUTTON_PIN)
            now = time.time()

            # HIGH -> LOW 변화 감지, 버튼 눌림
            if last_state == 1 and current_state == 0:
                if now - last_press_time > 0.5:
                    last_press_time = now
                    handle_button_press()

            last_state = current_state

            # 실행 프로세스가 종료됐으면 상태만 출력
            if process is not None and process.poll() is not None:
                pass

            time.sleep(0.05)

    except KeyboardInterrupt:
        cleanup()


if __name__ == "__main__":
    main()


## !/usr/bin/env python3
# import os
# import signal
# import subprocess
# import time

# import RPi.GPIO as GPIO


# BUTTON_PIN = 20          # 기존 시작 / 일시정지 / 재개 버튼
# STOP_BUTTON_PIN = 26     # 새로 추가한 강제 정지 버튼

# USER_HOME = "/home/slave"
# WORKSPACE = f"{USER_HOME}/git_slave/robot_ws"
# ARUCO_DIR = f"{WORKSPACE}/aruco_follow_code"
# RUN_SCRIPT = f"{ARUCO_DIR}/run_camera_follow_with_motor.sh"

# ROS_SETUP = "/opt/ros/humble/setup.bash"
# WS_SETUP = f"{WORKSPACE}/install/setup.bash"
# CMD_VEL_TOPIC = "/cmd_vel"

# process = None
# paused = False
# force_stopped = False


# def send_stop_cmd(duration=2, rate=10):
#     """
#     모터 속도를 강제로 0으로 만들기 위해
#     /cmd_vel에 0 명령을 여러 번 보냄
#     """
#     cmd = (
#         f"source {ROS_SETUP} && "
#         f"source {WS_SETUP} && "
#         f"export ROS_DOMAIN_ID=30 && "
#         f"export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && "
#         f"export ROS_LOCALHOST_ONLY=0 && "
#         f"timeout {duration}s ros2 topic pub -r {rate} {CMD_VEL_TOPIC} geometry_msgs/msg/Twist "
#         f"'{{linear: {{x: 0.0}}, angular: {{z: 0.0}}}}'"
#     )

#     result = subprocess.run(
#         ["bash", "-lc", cmd],
#         stdout=subprocess.DEVNULL,
#         stderr=subprocess.PIPE,
#         text=True,
#         timeout=duration + 2
#     )

#     # timeout 명령은 정상적으로 끝나도 return code 124가 나올 수 있음
#     if result.returncode not in (0, 124):
#         print("[WARN] stop cmd stderr:", result.stderr, flush=True)


# def is_running():
#     global process
#     return process is not None and process.poll() is None


# def start_robot():
#     global process, paused, force_stopped

#     print("[BUTTON] START", flush=True)

#     env = os.environ.copy()
#     env["HOME"] = USER_HOME
#     env["ROS_DOMAIN_ID"] = "30"
#     env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
#     env["ROS_LOCALHOST_ONLY"] = "0"
#     env["OPENCV_VIDEOIO_PRIORITY_GSTREAMER"] = "0"

#     process = subprocess.Popen(
#         ["bash", RUN_SCRIPT],
#         cwd=WORKSPACE,
#         env=env,
#         preexec_fn=os.setsid
#     )

#     paused = False
#     force_stopped = False

#     print("[STATE] RUNNING", flush=True)


# def pause_robot():
#     global paused, force_stopped

#     if not is_running():
#         print("[WARN] Cannot pause. Process is not running.", flush=True)
#         return

#     if force_stopped:
#         print("[WARN] Already FORCE STOPPED.", flush=True)
#         return

#     print("[BUTTON] PAUSE", flush=True)

#     try:
#         send_stop_cmd(duration=2, rate=10)
#     except Exception as e:
#         print(f"[WARN] stop cmd failed: {e}", flush=True)

#     time.sleep(0.2)

#     try:
#         os.killpg(os.getpgid(process.pid), signal.SIGSTOP)
#         paused = True
#         print("[STATE] PAUSED", flush=True)
#     except Exception as e:
#         print(f"[WARN] pause failed: {e}", flush=True)


# def resume_robot():
#     global paused, force_stopped

#     if not is_running():
#         print("[WARN] Process not running. Start again.", flush=True)
#         start_robot()
#         return

#     if force_stopped:
#         print("[WARN] Cannot resume with GPIO20. Press GPIO26 to release FORCE STOP.", flush=True)
#         return

#     print("[BUTTON] RESUME", flush=True)

#     try:
#         os.killpg(os.getpgid(process.pid), signal.SIGCONT)
#         paused = False
#         print("[STATE] RUNNING", flush=True)
#     except Exception as e:
#         print(f"[WARN] resume failed: {e}", flush=True)


# def force_stop_robot():
#     global force_stopped, paused

#     if not is_running():
#         print("[WARN] Cannot force stop. Process is not running.", flush=True)
#         return

#     print("[BUTTON GPIO26] FORCE STOP", flush=True)

#     # 먼저 모터에 0 속도 명령을 여러 번 보냄
#     try:
#         send_stop_cmd(duration=2, rate=10)
#     except Exception as e:
#         print(f"[WARN] force stop cmd failed: {e}", flush=True)

#     time.sleep(0.2)

#     # 그 다음 카메라 추종 코드가 다시 속도 명령을 덮어쓰지 못하게 정지
#     try:
#         os.killpg(os.getpgid(process.pid), signal.SIGSTOP)
#         force_stopped = True
#         paused = False
#         print("[STATE] FORCE_STOPPED", flush=True)
#     except Exception as e:
#         print(f"[WARN] force stop failed: {e}", flush=True)


# def release_force_stop_robot():
#     global force_stopped, paused

#     if not is_running():
#         print("[WARN] Process not running. Start again.", flush=True)
#         start_robot()
#         return

#     print("[BUTTON GPIO26] RELEASE FORCE STOP", flush=True)

#     try:
#         os.killpg(os.getpgid(process.pid), signal.SIGCONT)
#         force_stopped = False
#         paused = False
#         print("[STATE] RUNNING", flush=True)
#     except Exception as e:
#         print(f"[WARN] release force stop failed: {e}", flush=True)


# def handle_button_press():
#     global paused

#     print("[BUTTON GPIO20] Pressed", flush=True)

#     if not is_running():
#         start_robot()
#     elif not paused:
#         pause_robot()
#     else:
#         resume_robot()


# def handle_stop_button_press():
#     global force_stopped

#     print("[BUTTON GPIO26] Pressed", flush=True)

#     if not is_running():
#         print("[WARN] Robot is not running. GPIO26 does nothing.", flush=True)
#         return

#     if not force_stopped:
#         force_stop_robot()
#     else:
#         release_force_stop_robot()


# def cleanup():
#     global process

#     print("[INFO] Cleanup", flush=True)

#     try:
#         send_stop_cmd(duration=2, rate=10)
#     except Exception:
#         pass

#     if is_running():
#         try:
#             os.killpg(os.getpgid(process.pid), signal.SIGTERM)
#         except Exception:
#             pass

#     GPIO.cleanup()


# def main():
#     print("==================================================", flush=True)
#     print("[GPIO Button Controller]", flush=True)
#     print("GPIO20 1st press : start run_camera_follow_with_motor.sh", flush=True)
#     print("GPIO20 next press: pause", flush=True)
#     print("GPIO20 next press: resume", flush=True)
#     print("GPIO26 press     : force motor speed 0", flush=True)
#     print("GPIO26 next press: release force stop and run again", flush=True)
#     print("==================================================", flush=True)

#     GPIO.setwarnings(False)
#     GPIO.setmode(GPIO.BCM)

#     # GPIO20 버튼
#     # 버튼 한쪽: GPIO20
#     # 버튼 다른쪽: GND
#     GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

#     # GPIO26 강제 정지 버튼
#     # 버튼 한쪽: GPIO26
#     # 버튼 다른쪽: GND
#     GPIO.setup(STOP_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

#     last_button_state = GPIO.input(BUTTON_PIN)
#     last_stop_button_state = GPIO.input(STOP_BUTTON_PIN)
    
#     print(f"[DEBUG] initial GPIO20={last_button_state}, GPIO26={last_stop_button_state}", flush=True)

#     last_button_press_time = 0.0
#     last_stop_button_press_time = 0.0

#     try:
#         while True:
#             current_button_state = GPIO.input(BUTTON_PIN)
#             current_stop_button_state = GPIO.input(STOP_BUTTON_PIN)
#             now = time.time()

#             # GPIO20 버튼: HIGH -> LOW 감지
#             if last_button_state == 1 and current_button_state == 0:
#                 if now - last_button_press_time > 0.5:
#                     last_button_press_time = now
#                     handle_button_press()

#             # GPIO26 버튼: HIGH -> LOW 감지
#             if last_stop_button_state == 1 and current_stop_button_state == 0:
#                 print("[DEBUG] GPIO26 falling edge detected", flush=True)

#                 if now - last_stop_button_press_time > 0.5:
#                     last_stop_button_press_time = now
#                     handle_stop_button_press()

#             last_button_state = current_button_state
#             last_stop_button_state = current_stop_button_state

#             time.sleep(0.05)

#     except KeyboardInterrupt:
#         cleanup()


# if __name__ == "__main__":
#     main()