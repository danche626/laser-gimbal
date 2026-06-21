# 激光云台自动瞄准系统

电子设计竞赛 — 光电追踪题目。基于 Jetson Nano 视觉闭环控制，实现 3s 内自动追踪并击中靶标红心。

## 系统架构

```
摄像头(30fps) → 视觉检测 → 卡尔曼滤波 → 自适应PID → 步进电机
                                                      ↓
                                              激光对准判定 → 射击
```

## 硬件

| 组件 | 型号 | 说明 |
|------|------|------|
| 主控 | Jetson Nano B01 | Ubuntu 18.04, OpenCV 3.2 |
| 摄像头 | Astra Pro Plus | 奥比中光, UVC, 320x240@30fps |
| 电机 | YL42M x2 | 步进闭环, Modbus RTU RS485 |
| 激光 | 405nm 10mW | TTL 控制, GPIO Pin12 |
| IMU | MPU6050 | 扩展板自带, I2C 0x68 (上车后启用) |

## 软件模块

```
jetson/gimbal/
├── main.py            # 入口: 初始化 + 主循环
├── config.py          # 参数集中管理
├── vision.py          # 视觉检测: 反向二值化 + 矩形验证
├── motor.py           # 电机控制: Modbus RTU 协议
├── pid.py             # 双环PID + 卡尔曼滤波 + 距离自适应增益
├── laser.py           # 激光控制
├── state_machine.py   # 6状态状态机
├── gpio_input.py      # 按键输入(防抖+边沿检测)
└── display.py         # HUD叠加显示
```

## 状态机

```
IDLE → TRACK_STATIC → (丢目标) → SEARCH → (找到) → TRACK_STATIC
  ↓
CALIBRATE / TRACK_MOVING / CIRCLE
```

- **IDLE**: 待机, KEY2切换模式, KEY1/空格启动
- **TRACK_STATIC**: 静靶追踪, PID闭环 + 激光对准射击
- **TRACK_MOVING**: 动靶追踪, 卡尔曼预测
- **SEARCH**: 丢目标后yaw扫描搜索
- **CIRCLE**: 激光绕靶心画圆
- **CALIBRATE**: 标定模式, 显示距离/面积

## 关键参数

| 参数 | 值 | 说明 |
|------|---|------|
| PID_KP | 12.0 | 位置环比例 |
| PID_KD | 2.0 | 位置环微分 |
| PID_MAX_SPEED | 1800 | 电机最大速度 |
| MOTOR_ACCEL | 200 | 加速度 |
| LASER_DEADZONE | 4px | 激光触发精度 |
| ALIGN_THRESHOLD | 8帧 | 连续对准帧数 |
| Y_LIMIT_UP | 20° | 俯仰上限 |
| Y_LIMIT_DOWN | 90° | 俯仰下限 |

## 视觉检测策略

参考 TI 奖 karman 项目:
1. 高斯模糊降噪
2. THRESH_BINARY_INV 反向二值化 (检测黑色边框)
3. 形态学闭运算 (修复激光照射断裂)
4. RETR_EXTERNAL 只取最外层轮廓 (避免内部红色圆环干扰)
5. 四边形近似 + 角度/边长比验证
6. 帧间面积一致性过滤

## 部署

```bash
# 上传代码
scp -r jetson/gimbal/ yuanji@192.168.0.18:~/gimbal/

# 运行
ssh yuanji@192.168.0.18 "cd ~/gimbal && DISPLAY=:0 python3 main.py"
```

## 后续规划

- [ ] 加装导电滑环 (yaw无限旋转)
- [ ] 装车后启用 MPU6050 IMU 闭环解耦
- [ ] 串级PID: 视觉外环(30Hz) + IMU内环(500Hz)
- [ ] 深度相机测距替代面积法
