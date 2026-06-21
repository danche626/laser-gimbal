"""
pid.py - 双环PID控制器 + 卡尔曼滤波
参照 karman 项目的 controller.c + kalman_filter.c
"""
import time
import numpy as np


class KalmanFilter1D:
    """一维卡尔曼滤波器，状态 = [位置, 速度]
    平滑视觉误差抖动，同时估计目标运动速度(用于动靶预测)
    """
    def __init__(self, dt=0.015, q_pos=0.5, q_vel=2.0, r_meas=3.0):
        self.dt = dt
        # 状态向量 [pos, vel]
        self.x = np.array([0.0, 0.0])
        # 协方差矩阵
        self.P = np.array([[5.0, 0.0],
                           [0.0, 5.0]])
        # 状态转移矩阵
        self.F = np.array([[1.0, dt],
                           [0.0, 1.0]])
        # 过程噪声
        self.Q = np.array([[q_pos, 0.0],
                           [0.0, q_vel]])
        # 观测矩阵 (只观测位置)
        self.H = np.array([[1.0, 0.0]])
        # 观测噪声
        self.R = np.array([[r_meas]])

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, measurement):
        self.predict()
        y = measurement - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ y).flatten()
        I = np.eye(2)
        self.P = (I - K @ self.H) @ self.P
        return self.x[0]  # 返回滤波后的位置

    def get_velocity(self):
        return self.x[1]

    def reset(self):
        self.x = np.array([0.0, 0.0])
        self.P = np.array([[5.0, 0.0],
                           [0.0, 5.0]])


def _mapping(x, in_min, in_max, out_min, out_max):
    """线性映射（参照 karman 的 mapping 宏）"""
    if in_max == in_min:
        return out_min
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class AdaptivePID:
    """带卡尔曼滤波 + 距离自适应增益的双环PID
    外环(位置): target_v = Kp * error + Kd * d(error)/dt
    内环(速度): output += 0.75 * v_err + Ki * integral(v_err)
    """
    def __init__(self, cfg):
        self.kp = cfg.PID_KP
        self.kd = cfg.PID_KD
        self.ki_vel = cfg.PID_KI_VEL
        self.max_out = cfg.PID_MAX_SPEED
        self.deadzone = cfg.PID_DEADZONE
        self._prev_err = 0.0
        self._output = 0.0
        self._integral = 0.0
        self._prev_t = None
        # 卡尔曼滤波器
        self.kf = KalmanFilter1D(
            dt=cfg.KF_DT,
            q_pos=cfg.KF_PROCESS_POS,
            q_vel=cfg.KF_PROCESS_VEL,
            r_meas=cfg.KF_MEASURE_NOISE
        )
        # 距离自适应
        self._adaptive = cfg.ADAPTIVE_GAIN_ENABLED
        self._d_min = cfg.ADAPTIVE_D_MIN
        self._d_max = cfg.ADAPTIVE_D_MAX
        self._g_max = cfg.ADAPTIVE_GAIN_MAX
        self._g_min = cfg.ADAPTIVE_GAIN_MIN

    def compute(self, raw_error, distance_mm=-1):
        """计算PID输出
        raw_error: 原始像素误差
        distance_mm: 当前距离(用于自适应增益)，-1表示不使用
        """
        # 卡尔曼滤波平滑
        error = self.kf.update(raw_error)

        now = time.time()
        dt = 0.015 if self._prev_t is None else max(now - self._prev_t, 0.001)
        self._prev_t = now

        if abs(error) < self.deadzone:
            self._output *= 0.4
            self._integral = 0
            self._prev_err = 0
            if abs(self._output) < 3:
                self._output = 0
            return int(self._output)

        # 外环: 位置PD → 目标速度
        deriv = (error - self._prev_err) / dt
        target_v = self.kp * error + self.kd * deriv
        target_v = np.clip(target_v, -self.max_out * 1.5, self.max_out * 1.5)
        self._prev_err = error

        # 内环: 速度PI
        v_err = target_v - self._output
        self._integral += v_err * dt
        self._integral = np.clip(self._integral, -200, 200)
        self._output += 0.75 * v_err + self.ki_vel * self._integral
        self._output = np.clip(self._output, -self.max_out, self.max_out)

        # 距离自适应增益
        result = self._output
        if self._adaptive and distance_mm > 0:
            d_clamped = np.clip(distance_mm, self._d_min, self._d_max)
            gain = _mapping(d_clamped, self._d_min, self._d_max,
                            self._g_max, self._g_min)
            result *= gain

        return int(result)

    def reset(self):
        self._prev_err = 0.0
        self._output = 0.0
        self._integral = 0.0
        self._prev_t = None
        self.kf.reset()
