"""
state_machine.py - 云台状态机（参照 karman robot_cmd.c）
6种状态 + 自动转移 + 各模式处理逻辑
"""
import time
import math
import numpy as np
from enum import IntEnum

from vision import (
    detect_target,
    perspective_center,
    dynamic_laser_offset,
    reset_area_history,
)


class State(IntEnum):
    IDLE = 0
    CALIBRATE = 1
    TRACK_STATIC = 2
    TRACK_MOVING = 3
    SEARCH = 4
    CIRCLE = 5


# IDLE时可选的启动模式
STARTUP_MODES = [State.TRACK_STATIC, State.TRACK_MOVING,
                 State.CIRCLE, State.CALIBRATE]


class GimbalStateMachine:
    def __init__(self, motor, laser, pid_x, pid_y, cfg):
        self.motor = motor
        self.laser = laser
        self.pid_x = pid_x
        self.pid_y = pid_y
        self.cfg = cfg

        self.state = State.IDLE
        self.mode_idx = 0  # 当前选中的启动模式索引

        # 追踪状态
        self.fx = None
        self.fy = None
        self.prev_area = None
        self.lost_count = 0
        self.aligned_frames = 0
        self.cur_distance = -1.0
        self.cur_offset_y = -25

        # Y轴限位
        self.pos_y_deg = 0.0
        self._prev_loop_t = time.time()
        self._loop_dt = 1.0 / cfg.CAM_FPS if cfg.CAM_FPS > 0 else 0.033

        # 搜索模式
        self._search_dir = 0  # 0=负方向, 1=正方向
        self._search_angle = 0.0
        self._search_found_count = 0

        # 画圆模式
        self._circle_angle = 0.0
        self._circle_settled = 0
        self._circle_started = False

        # HUD信息
        self.info = {}

    def update(self, gray, det_raw, keys):
        """每帧调用: 处理输入 → 状态转移 → 执行当前状态逻辑
        gray: 灰度图(用于透视校正)
        det_raw: detect_target()结果或None
        keys: GPIOInput实例(已调用update)
        """
        now = time.time()
        self._loop_dt = min(max(now - self._prev_loop_t, 0.001), 0.2)
        self._prev_loop_t = now

        # ---- 按键处理 ----
        if keys.key1_pressed:
            if self.state == State.IDLE:
                self._enter_mode(STARTUP_MODES[self.mode_idx])
            else:
                self._enter_idle()
            return

        if keys.key2_pressed:
            if self.state == State.IDLE:
                self.mode_idx = (self.mode_idx + 1) % len(STARTUP_MODES)
                print(f"[MODE] → {STARTUP_MODES[self.mode_idx].name}")
            else:
                self._enter_idle()
            return

        # ---- 面积跳变过滤 ----
        det = det_raw
        if det and self.prev_area is not None:
            ratio = det.area / self.prev_area
            if ratio < self.cfg.AREA_JUMP_LOW or ratio > self.cfg.AREA_JUMP_HIGH:
                det = None

        # ---- 状态分发 ----
        handler = {
            State.IDLE: self._handle_idle,
            State.CALIBRATE: self._handle_calibrate,
            State.TRACK_STATIC: self._handle_track,
            State.TRACK_MOVING: self._handle_track,
            State.SEARCH: self._handle_search,
            State.CIRCLE: self._handle_circle,
        }.get(self.state, self._handle_idle)
        handler(gray, det)

        # ---- 更新HUD信息 ----
        self.info = {
            "mode_idx": self.mode_idx,
            "distance": self.cur_distance,
            "offset_y": self.cur_offset_y,
            "fx": self.fx,
            "fy": self.fy,
            "laser_on": self.laser.on,
            "search_dir": self._search_dir,
            "circle_progress": self._circle_angle / (2 * math.pi),
        }

    def handle_space(self):
        """空格键(SSH调试)，等同KEY1"""
        if self.state == State.IDLE:
            self._enter_mode(STARTUP_MODES[self.mode_idx])
        else:
            self._enter_idle()

    # =============== 状态进入/退出 ===============

    def _enter_idle(self):
        self.state = State.IDLE
        self.motor.stop_all()
        self.laser.off()
        self.pid_x.reset()
        self.pid_y.reset()
        self.aligned_frames = 0
        self.fx = self.fy = None
        self.prev_area = None
        reset_area_history()
        self.lost_count = 0
        self._circle_started = False
        print(f"[STATE] → IDLE")

    def _enter_mode(self, mode):
        self.state = mode
        self.lost_count = 0
        self.aligned_frames = 0
        self.fx = self.fy = None
        self.prev_area = None
        reset_area_history()
        self._search_angle = 0.0
        self._search_dir = 0
        self._search_found_count = 0
        self._circle_angle = 0.0
        self._circle_settled = 0
        self._circle_started = False
        print(f"[STATE] → {mode.name}")

    # =============== 各状态处理 ===============

    def _handle_idle(self, gray, det):
        pass

    def _handle_calibrate(self, gray, det):
        """标定模式: 只显示信息，不驱动电机"""
        if det:
            self.cur_distance = det.distance_mm
            self.cur_offset_y = dynamic_laser_offset(det.distance_mm, self.cfg)
            self.prev_area = det.area
            self.fx, self.fy = float(det.cx), float(det.cy)

    def _handle_track(self, gray, det):
        """静靶/动靶追踪核心逻辑"""
        cfg = self.cfg
        h, w = gray.shape
        cx, cy = w // 2, h // 2

        if det:
            self.prev_area = det.area
            self.lost_count = 0

            # 面积测距 + 动态偏移
            self.cur_distance = det.distance_mm
            if self.cur_distance > 0:
                self.cur_offset_y = dynamic_laser_offset(self.cur_distance, cfg)

            # 透视校正(静靶模式使用，动靶可选关闭以省算力)
            tx, ty = det.cx, det.cy
            if self.state == State.TRACK_STATIC:
                pc = perspective_center(gray, det.approx)
                if pc is not None:
                    tx, ty = pc

            # 指数滑动平均
            if self.fx is None:
                self.fx, self.fy = float(tx), float(ty)
            else:
                a = cfg.PID_ALPHA
                self.fx = a * tx + (1 - a) * self.fx
                self.fy = a * ty + (1 - a) * self.fy

            # 误差计算
            err_x = self.fx - cx
            err_y = self.fy - (cy + self.cur_offset_y)

            # PID计算(带KF + 距离自适应)
            spd_x = self.pid_x.compute(err_x, self.cur_distance)
            spd_y = self.pid_y.compute(err_y, self.cur_distance)

            # Y轴限位积分
            self._update_y_limit()

            # 方向映射 + 软限位
            cmd_x = -spd_x
            cmd_y = self._apply_y_limit(-spd_y)

            self.motor.set_velocity(cfg.MOTOR_ADDR_X, cmd_x)
            self.motor.set_velocity(cfg.MOTOR_ADDR_Y, cmd_y)

            # 对准判定（用更严格的激光死区）
            laser_dz = cfg.LASER_DEADZONE
            if abs(err_x) < laser_dz and abs(err_y) < laser_dz:
                self.aligned_frames += 1
                if self.aligned_frames >= cfg.ALIGN_THRESHOLD:
                    self.laser.fire()
            else:
                self.aligned_frames = 0
                self.laser.off()
        else:
            # 丢失处理
            self.lost_count += 1
            self.aligned_frames = 0
            self.laser.off()

            if self.lost_count > 2:
                spd_x = self.pid_x.compute(0, self.cur_distance)
                spd_y = self.pid_y.compute(0, self.cur_distance)
                self.motor.set_velocity(cfg.MOTOR_ADDR_X, -spd_x)
                self.motor.set_velocity(cfg.MOTOR_ADDR_Y, -spd_y)

            if self.lost_count > 15:
                self.motor.stop_all()
                self.pid_x.reset()
                self.pid_y.reset()
                self.fx = self.fy = None
                self.prev_area = None

            # 超时进入搜索
            if self.lost_count > cfg.LOST_ENTER_SEARCH:
                self.state = State.SEARCH
                self._search_angle = 0.0
                self._search_dir = 0
                self._search_found_count = 0
                print("[STATE] → SEARCH (target lost)")

    def _handle_search(self, gray, det):
        """搜索扫描: yaw左右扫描，检测到目标后回到TRACK"""
        cfg = self.cfg

        if det:
            self._search_found_count += 1
            if self._search_found_count >= cfg.SEARCH_CONFIRM_FRAMES:
                self.state = State.TRACK_STATIC
                self.lost_count = 0
                print("[STATE] → TRACK (target found)")
                return
        else:
            self._search_found_count = 0

        # 左右扫描
        if self._search_dir == 0:
            self.motor.set_velocity(cfg.MOTOR_ADDR_X, -cfg.SEARCH_STEP_SPEED)
            self._search_angle -= cfg.SEARCH_STEP_SPEED * cfg.Y_DEG_PER_SPEED_SEC * self._loop_dt
        else:
            self.motor.set_velocity(cfg.MOTOR_ADDR_X, cfg.SEARCH_STEP_SPEED)
            self._search_angle += cfg.SEARCH_STEP_SPEED * cfg.Y_DEG_PER_SPEED_SEC * self._loop_dt

        # 到边界反向
        if self._search_angle > cfg.SEARCH_RANGE_DEG:
            self._search_dir = 0
        elif self._search_angle < -cfg.SEARCH_RANGE_DEG:
            self._search_dir = 1

        # pitch不动
        self.motor.set_velocity(cfg.MOTOR_ADDR_Y, 0)

    def _handle_circle(self, gray, det):
        """画圆模式: 先对准靶心，稳定后激光绕靶心画圆"""
        cfg = self.cfg
        h, w = gray.shape
        cx, cy = w // 2, h // 2

        if det is None:
            self.lost_count += 1
            if self.lost_count > 15:
                self.motor.stop_all()
            return

        self.lost_count = 0
        self.prev_area = det.area
        self.cur_distance = det.distance_mm
        if self.cur_distance > 0:
            self.cur_offset_y = dynamic_laser_offset(self.cur_distance, cfg)

        tx, ty = det.cx, det.cy
        if self.fx is None:
            self.fx, self.fy = float(tx), float(ty)
        else:
            a = cfg.PID_ALPHA
            self.fx = a * tx + (1 - a) * self.fx
            self.fy = a * ty + (1 - a) * self.fy

        # 画圆的目标点: 靶心 + 圆周偏移
        if not self._circle_started:
            # 先对准靶心
            target_x = 0.0
            target_y = 0.0
            err_x = self.fx - cx
            err_y = self.fy - (cy + self.cur_offset_y)
            if abs(err_x) < cfg.PID_DEADZONE and abs(err_y) < cfg.PID_DEADZONE:
                self._circle_settled += 1
            else:
                self._circle_settled = 0
            if self._circle_settled >= cfg.CIRCLE_SETTLE_FRAMES:
                self._circle_started = True
                self._circle_angle = 0.0
                self.laser.fire()
                print("[CIRCLE] Started drawing")
        else:
            # 画圆中: PID setpoint = 圆周点
            r = cfg.CIRCLE_RADIUS_PX
            target_x = r * math.cos(self._circle_angle)
            target_y = r * math.sin(self._circle_angle)
            # 角度递增
            self._circle_angle += (2 * math.pi / cfg.CIRCLE_PERIOD_S) * self._loop_dt
            self.laser.fire()

        # PID: err = (目标位置 - 画面中心) - 圆周偏移
        if self._circle_started:
            err_x = self.fx - cx - target_x
            err_y = self.fy - (cy + self.cur_offset_y) - target_y
        else:
            err_x = self.fx - cx
            err_y = self.fy - (cy + self.cur_offset_y)

        spd_x = self.pid_x.compute(err_x, self.cur_distance)
        spd_y = self.pid_y.compute(err_y, self.cur_distance)

        self._update_y_limit()
        cmd_x = -spd_x
        cmd_y = self._apply_y_limit(-spd_y)
        self.motor.set_velocity(cfg.MOTOR_ADDR_X, cmd_x)
        self.motor.set_velocity(cfg.MOTOR_ADDR_Y, cmd_y)

    # =============== 工具方法 ===============

    def _update_y_limit(self):
        spd_y = self.motor.get_last_speed(self.cfg.MOTOR_ADDR_Y)
        if spd_y != 0:
            self.pos_y_deg += spd_y * self.cfg.Y_DEG_PER_SPEED_SEC * self._loop_dt

    def _apply_y_limit(self, cmd_y):
        """Y轴软限位: 接近边界减速，到达边界停止"""
        cfg = self.cfg
        soft_zone = 5.0  # 接近限位5°开始减速
        if self.pos_y_deg > cfg.Y_LIMIT_UP_DEG:
            if cmd_y > 0:
                return 0
        elif self.pos_y_deg > (cfg.Y_LIMIT_UP_DEG - soft_zone):
            if cmd_y > 0:
                scale = (cfg.Y_LIMIT_UP_DEG - self.pos_y_deg) / soft_zone
                cmd_y = int(cmd_y * max(scale, 0.1))

        if self.pos_y_deg < -cfg.Y_LIMIT_DOWN_DEG:
            if cmd_y < 0:
                return 0
        elif self.pos_y_deg < -(cfg.Y_LIMIT_DOWN_DEG - soft_zone):
            if cmd_y < 0:
                scale = (cfg.Y_LIMIT_DOWN_DEG + self.pos_y_deg) / soft_zone
                cmd_y = int(cmd_y * max(scale, 0.1))
        return cmd_y
