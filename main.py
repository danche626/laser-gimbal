"""
main.py - 激光云台系统入口
初始化所有模块 → 主循环（抓帧→检测→状态机→显示）
"""
import sys
import signal
import time
import cv2
import subprocess

try:
    import Jetson.GPIO as GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

from config import Config
from motor import MotorController
from laser import LaserController
from pid import AdaptivePID
from vision import detect_target
from gpio_input import GPIOInput
from state_machine import GimbalStateMachine, State
from display import render_hud

signal.signal(signal.SIGTERM, signal.SIG_IGN)
if hasattr(signal, "SIGHUP"):
    signal.signal(signal.SIGHUP, signal.SIG_IGN)


def main():
    cfg = Config()

    print("[INIT] Starting gimbal system...")
    motor = None
    laser = None
    cap = None

    try:
        motor = MotorController(cfg)
        laser = LaserController(cfg)
        pid_x = AdaptivePID(cfg)
        pid_y = AdaptivePID(cfg)
        keys = GPIOInput(cfg)
        sm = GimbalStateMachine(motor, laser, pid_x, pid_y, cfg)

        # OpenCV 3.2 不支持MJPG解码，先用v4l2-ctl切到YUYV。
        try:
            subprocess.run(
                [
                    "v4l2-ctl", "-d", f"/dev/video{cfg.CAM_INDEX}",
                    "--set-fmt-video="
                    f"width={cfg.CAM_WIDTH},height={cfg.CAM_HEIGHT},pixelformat=YUYV",
                ],
                stderr=subprocess.DEVNULL,
                check=False,
            )
            time.sleep(0.5)
        except FileNotFoundError:
            print("[WARN] v4l2-ctl not found, using camera default format")

        cap = cv2.VideoCapture(cfg.CAM_INDEX)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.CAM_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.CAM_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, cfg.CAM_FPS)
        time.sleep(0.3)
        if not cap.isOpened():
            print("[ERROR] Camera not found")
            return

        if cfg.FULLSCREEN:
            cv2.namedWindow(cfg.WINDOW_NAME, cv2.WND_PROP_FULLSCREEN)
            cv2.setWindowProperty(
                cfg.WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN
            )

        print("[INIT] Ready. KEY1=start, KEY2=mode switch")
        running = True
        while running:
            ret, frame = cap.read()
            if not ret:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            det = detect_target(gray, cfg)
            keys.update()

            sm.update(gray, det, keys)

            render_hud(frame, sm.state, det, sm.info, cfg)
            cv2.imshow(cfg.WINDOW_NAME, frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                running = False
            elif key == ord(" "):
                sm.handle_space()
    finally:
        if motor is not None:
            motor.stop_all()
        if laser is not None:
            laser.off()
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        if GPIO_AVAILABLE:
            GPIO.cleanup()
        print("[EXIT] Done")


if __name__ == "__main__":
    main()
