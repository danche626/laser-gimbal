# 激光云台项目优化交接文档

本文档用于和后续接手的 AI/开发者交接本轮优化内容。当前修改聚焦在低风险稳定性优化，没有改变 PID 参数、硬件接线、视觉阈值或电机协议。

## 项目背景

仓库：`danche626/laser-gimbal`

本项目是基于 Jetson Nano、OpenCV、双轴步进电机、TTL 激光和按键输入的激光云台自动瞄准系统。主流程为：

```text
摄像头取帧 -> 视觉检测黑色矩形靶 -> PID 云台闭环 -> 对准后控制激光
```

核心文件：

- `main.py`：系统入口，初始化相机、电机、激光、按键、状态机。
- `vision.py`：视觉检测，包含矩形靶识别、透视中心估计、面积测距、激光偏移估计。
- `state_machine.py`：IDLE、标定、静靶追踪、动靶追踪、搜索、画圆状态机。
- `motor.py`：YL42M 步进电机 Modbus RTU 控制。
- `pid.py`：一维卡尔曼滤波和自适应 PID。

## 本轮优化概览

### 1. OpenCV 3/4 轮廓提取兼容

修改文件：`vision.py`

原代码写法：

```python
_, contours, _ = cv2.findContours(...)
```

这个写法只兼容 OpenCV 3。OpenCV 4 的 `cv2.findContours` 只返回两个值，会导致程序启动检测时抛异常。

现已新增：

```python
def _find_contours(binary, mode, method):
    result = cv2.findContours(binary, mode, method)
    return result[1] if len(result) == 3 else result[0]
```

并替换了 `detect_target()` 和 `perspective_center()` 中的轮廓提取调用。

收益：

- OpenCV 3.2 Jetson 环境继续可用。
- OpenCV 4 本地调试/新系统部署不再直接崩溃。

### 2. 状态切换时重置视觉面积历史

修改文件：`vision.py`、`state_machine.py`

原问题：

`vision.py` 中 `_last_area` 是模块级历史状态，用于帧间面积跳变过滤。状态机从 IDLE 进入追踪、退出追踪、重新启动时，旧目标面积可能继续影响新一轮检测，导致刚切换状态后目标被错误过滤。

现已新增：

```python
def reset_area_history():
    _last_area[0] = 0
```

并在以下状态切换点调用：

- `_enter_idle()`
- `_enter_mode()`

同时在 `_enter_mode()` 中重置 `fx/fy` 和 `prev_area`，避免新模式沿用上一轮滤波目标点。

收益：

- 模式切换后第一批检测更干净。
- 减少“明明有靶但刚开始追踪时丢目标”的偶发问题。

### 3. 搜索、画圆、Y 轴限位改用真实帧间隔

修改文件：`state_machine.py`

原问题：

搜索模式和画圆模式写死了 `0.015s`：

```python
dt = 0.015
```

但 README 中相机配置是 `30fps`，理论帧间隔约 `0.033s`。如果实际 FPS 不是 67fps，搜索角度估计、画圆周期和 Y 轴软限位积分都会偏离真实时间。

现已在 `update()` 开头统一计算：

```python
now = time.time()
self._loop_dt = min(max(now - self._prev_loop_t, 0.001), 0.2)
self._prev_loop_t = now
```

并用于：

- 搜索扫描角度累计。
- 画圆角度递增。
- Y 轴软限位位置积分。

收益：

- 行为随真实帧率自适应。
- 当相机帧率下降或系统负载波动时，时间相关动作更稳定。

注意：

- `_loop_dt` 被限制在 `0.001 ~ 0.2s`，避免首次循环或卡顿时出现过大积分。

### 4. 主循环安全清理

修改文件：`main.py`

原问题：

主循环只有正常退出路径才会执行：

```python
motor.stop_all()
laser.off()
cap.release()
cv2.destroyAllWindows()
GPIO.cleanup()
```

如果中途出现异常，存在电机未停止、激光未关闭、相机/GPIO 未释放的风险。

现已将初始化和主循环包进 `try/finally`：

```python
try:
    ...
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
```

收益：

- 视觉、相机或按键逻辑出错时，也会尽量停机并关闭激光。
- 更适合比赛/调试场景，降低异常后的硬件风险。

### 5. 相机格式设置更稳健

修改文件：`main.py`

原代码固定使用 `/dev/video0`：

```python
v4l2-ctl -d /dev/video0 ...
```

现已改为使用配置项：

```python
f"/dev/video{cfg.CAM_INDEX}"
```

同时如果系统没有安装 `v4l2-ctl`，只打印警告并继续使用相机默认格式：

```python
except FileNotFoundError:
    print("[WARN] v4l2-ctl not found, using camera default format")
```

收益：

- `CAM_INDEX` 改动时，格式设置会跟随。
- 本地无 `v4l2-ctl` 的环境也能启动到相机打开阶段，方便调试。

## 已验证内容

在本地仓库目录执行：

```bash
python3 -m py_compile main.py state_machine.py vision.py motor.py pid.py laser.py gpio_input.py display.py config.py
```

结果：通过，无语法错误。

还执行了一个合成图像检测测试：

```python
import numpy as np
import cv2
from config import Config
from vision import detect_target, reset_area_history

cfg = Config()
img = np.full((cfg.CAM_HEIGHT, cfg.CAM_WIDTH), 255, dtype=np.uint8)
cv2.rectangle(img, (90, 70), (230, 170), 0, 8)
reset_area_history()
det = detect_target(img, cfg)
print("detected", det is not None)
```

结果：

```text
detected True
cx 161 cy 121 area 15460 distance 881.02
```

说明：

- `vision.py` 的 OpenCV 兼容封装可运行。
- 基础黑框目标检测路径仍然有效。

## 未验证内容

由于当前环境没有 Jetson Nano、Astra Pro Plus 摄像头、YL42M 电机、RS485 转接器和 TTL 激光，以下内容尚未实机验证：

- 串口是否能正常找到电机设备。
- 电机速度指令方向是否和实际云台方向一致。
- `try/finally` 异常清理时硬件是否立即停止。
- 搜索模式真实扫描范围是否和机械结构匹配。
- 画圆模式真实轨迹是否保持原有期望周期。
- Y 轴软限位积分系数 `Y_DEG_PER_SPEED_SEC` 是否需要重新标定。

后续接手者应优先在低功率/断激光条件下验证电机动作，再启用激光。

## 建议的实机验证步骤

1. 先断开激光或遮光，运行：

   ```bash
   DISPLAY=:0 python3 main.py
   ```

2. 确认初始化日志：

   - 是否找到电机串口。
   - 是否能打开摄像头。
   - 如果没有 `v4l2-ctl`，是否只出现 warning 而不是退出。

3. 在 IDLE 下按 KEY2 切模式，确认模式切换不会误触发电机。

4. 用静态黑框靶测试 `TRACK_STATIC`：

   - 初次启动后应能检测目标，不应被旧面积历史过滤。
   - 云台应向减小误差方向运动。
   - 靶心稳定后才触发激光。

5. 遮挡目标，观察：

   - 短暂丢失时 PID 输出是否逐步收敛。
   - 超过 `LOST_ENTER_SEARCH` 后是否进入 SEARCH。
   - 搜索扫描方向和范围是否合理。

6. 测试 CIRCLE：

   - 应先对准靶心。
   - 稳定 `CIRCLE_SETTLE_FRAMES` 后开始画圆。
   - 一圈周期应接近 `CIRCLE_PERIOD_S`。

7. 手动制造异常或按 `q` 退出，确认：

   - 电机停止。
   - 激光关闭。
   - 摄像头窗口释放。

## 后续优化建议

优先级从高到低：

1. 增加硬件安全开关：

   - 增加 `Config.LASER_ENABLE = False` 默认关闭激光。
   - 只有实机确认后再允许 `laser.fire()` 真正输出 GPIO。

2. 增加 dry-run/mock 模式：

   - 无 Jetson.GPIO、无串口、无摄像头时可以用模拟电机和视频文件调试。
   - 这会大幅提升后续 AI/开发者的本地测试能力。

3. 为 `vision.py` 增加单元测试：

   - 合成矩形。
   - 过小面积。
   - 过大面积。
   - 非矩形轮廓。
   - OpenCV 3/4 `findContours` 返回值模拟。

4. 修正 README 和实际实现不一致处：

   README 中系统架构写的是“卡尔曼滤波 + 自适应 PID + 步进电机”，但硬件/部署路径和实际仓库根目录不完全一致，可补充本仓库当前直接运行方式。

5. 复核状态机中的动靶模式：

   当前 `TRACK_STATIC` 和 `TRACK_MOVING` 都复用 `_handle_track()`，动靶模式没有明显使用 Kalman 速度预测分支。后续可明确动靶预测策略。

6. 增加日志节流：

   硬件调试时如果后续加入更多日志，建议按时间节流，避免影响视觉帧率。

## 当前 Git 状态

本轮修改涉及：

```text
main.py
state_machine.py
vision.py
HANDOFF_OPTIMIZATION.md
```

当前修改尚未提交到 Git 历史。如需交给远端仓库，请后续执行：

```bash
git add main.py state_machine.py vision.py HANDOFF_OPTIMIZATION.md
git commit -m "Improve gimbal runtime robustness"
git push
```

如果没有远端写权限，可以直接把当前工作区交给下一位 AI/开发者继续。
