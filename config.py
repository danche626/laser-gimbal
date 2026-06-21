"""
config.py - 激光云台系统参数集中管理
参照 karman 项目 robot_def.h，所有可调参数在此定义
"""


class Config:
    # ======================== 电机参数 ========================
    MOTOR_BAUDRATE = 115200
    MOTOR_ADDR_X = 0x01
    MOTOR_ADDR_Y = 0x02
    MOTOR_ACCEL = 200
    MOTOR_SPEED_THRESHOLD = 3  # 速度变化小于此值不发送

    # ======================== 视觉参数 ========================
    # Astra Pro Plus (奥比中光 FHD-1080P, UVC协议)
    # 320x240 优先帧率，追踪响应更快
    CAM_WIDTH = 320
    CAM_HEIGHT = 240
    CAM_FPS = 30
    CAM_INDEX = 0

    # 反向二值化阈值（检测黑色边框，低于此值的像素变白）
    BINARY_THRESHOLD = 60
    # Canny边缘检测参数
    CANNY_LOWER = 50
    CANNY_UPPER = 150
    # 形态学闭运算核大小（修复激光照射黑框断裂）
    MORPH_CLOSE_SIZE = 20
    # 面积筛选
    AREA_MIN_PIXELS = 800
    AREA_MAX_RATIO = 0.80
    # 矩形几何验证
    ANGLE_TOLERANCE = 30       # 角度容差(度)
    SIDE_RATIO_TOLERANCE = 0.4 # 对边长度比容差
    RECT_ASPECT_MAX = 2.5      # 宽高比上限
    # 帧间面积跳变范围
    AREA_JUMP_LOW = 0.4
    AREA_JUMP_HIGH = 2.5

    # ======================== PID 参数 ========================
    PID_KP = 12.0
    PID_KD = 2.0
    PID_KI_VEL = 8.0
    PID_MAX_SPEED = 1800
    PID_DEADZONE = 8
    PID_ALPHA = 0.7  # 指数滑动平均系数

    # 距离自适应增益（参照 karman 的 kx_max/kx_min）
    # 输出乘以 mapping(distance, D_MIN, D_MAX, GAIN_MAX, GAIN_MIN)
    ADAPTIVE_GAIN_ENABLED = True
    ADAPTIVE_D_MIN = 500    # mm
    ADAPTIVE_D_MAX = 1500   # mm
    ADAPTIVE_GAIN_MAX = 1.3  # 近距离增益放大
    ADAPTIVE_GAIN_MIN = 0.7  # 远距离增益缩小

    # ======================== 卡尔曼滤波参数 ========================
    KF_DT = 0.033           # 采样周期(s)，30fps
    KF_PROCESS_POS = 2.0    # 位置过程噪声（大=信任观测）
    KF_PROCESS_VEL = 5.0    # 速度过程噪声
    KF_MEASURE_NOISE = 1.5  # 观测噪声（小=响应快）

    # ======================== 激光参数 ========================
    PIN_LASER = 12
    ALIGN_THRESHOLD = 8     # 连续N帧对准后点激光
    LASER_DEADZONE = 4      # 激光触发死区(像素)
    LASER_BELOW_CAM_MM = 45.0  # 激光在摄像头下方距离(mm)
    FOCAL_LENGTH_PX = 306.0    # 相机焦距(像素)

    # 面积测距标定（320x240分辨率，需实测后更新）
    CALIB_AREA_PIXELS = 12000  # 标定时的面积(需实测)
    CALIB_DISTANCE_MM = 1000   # 标定距离(mm)

    # ======================== GPIO 参数 ========================
    PIN_KEY1 = 7   # 启停按钮(有上拉)
    PIN_KEY2 = 11  # 模式切换按钮
    DEBOUNCE_MS = 30
    BOOT_IGNORE_MS = 500

    # ======================== 限位参数 ========================
    Y_LIMIT_UP_DEG = 20.0
    Y_LIMIT_DOWN_DEG = 90.0
    Y_DEG_PER_SPEED_SEC = 0.12  # 积分系数

    # ======================== 搜索模式参数 ========================
    SEARCH_RANGE_DEG = 60.0    # 左右扫描范围(度)
    SEARCH_STEP_SPEED = 250    # 扫描时电机速度
    SEARCH_CONFIRM_FRAMES = 8  # 连续检测N帧后切回TRACK
    LOST_ENTER_SEARCH = 30     # 丢失N帧后进入SEARCH

    # ======================== 画圆模式参数 ========================
    CIRCLE_RADIUS_PX = 40      # 画圆半径(像素)
    CIRCLE_PERIOD_S = 12.0     # 一圈周期(秒)
    CIRCLE_SETTLE_FRAMES = 20  # 先对准N帧后再开始画圆

    # ======================== 显示参数 ========================
    FULLSCREEN = True
    WINDOW_NAME = "Track"
