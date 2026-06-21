"""
display.py - HUD叠加显示
"""
import cv2


# 状态名称映射
STATE_NAMES = {
    0: "IDLE",
    1: "CALIBRATE",
    2: "TRACK:STATIC",
    3: "TRACK:MOVING",
    4: "SEARCH",
    5: "CIRCLE",
}

# 模式选择名称（IDLE时KEY2切换用）
MODE_NAMES = ["STATIC", "MOVING", "CIRCLE", "CALIBRATE"]


_fps_t = [0.0]
_fps_val = [0.0]
_fps_count = [0]


def render_hud(frame, state, det, info, cfg):
    """在画面上叠加HUD信息
    info: dict with keys like distance, offset_y, aligned, mode_idx
    """
    import time
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2

    # FPS计算
    _fps_count[0] += 1
    now = time.time()
    elapsed = now - _fps_t[0]
    if elapsed >= 1.0:
        _fps_val[0] = _fps_count[0] / elapsed
        _fps_count[0] = 0
        _fps_t[0] = now

    # 十字中心
    cv2.drawMarker(frame, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 15, 1)

    # 状态名 + FPS
    state_name = STATE_NAMES.get(state, "?")
    cv2.putText(frame, f"{state_name} {_fps_val[0]:.0f}fps", (5, 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

    # IDLE时显示待选模式
    if state == 0:
        mode_idx = info.get("mode_idx", 0)
        mode_text = f"MODE: {MODE_NAMES[mode_idx]} [KEY2=switch]"
        cv2.putText(frame, mode_text, (5, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1)
        cv2.putText(frame, "KEY1=START", (cx - 40, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
        # IDLE也画检测框，方便调试
        if det is not None:
            cv2.drawContours(frame, [det.approx], -1, (0, 255, 255), 2)
            cv2.circle(frame, (det.cx, det.cy), 4, (0, 0, 255), -1)
        return

    # 距离和偏移
    distance = info.get("distance", -1)
    offset_y = info.get("offset_y", 0)
    if distance > 0:
        cv2.putText(frame, f"D:{distance:.0f}mm OFS:{offset_y}px",
                    (5, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)

    # 检测框
    if det is not None:
        cv2.drawContours(frame, [det.approx], -1, (0, 255, 255), 2)
        # 滤波后的中心点
        fx = info.get("fx")
        fy = info.get("fy")
        if fx is not None:
            cv2.circle(frame, (int(fx), int(fy)), 5, (0, 0, 255), -1)

    # 激光对准指示
    if info.get("laser_on", False):
        cv2.circle(frame, (cx, cy), 20, (0, 255, 0), 3)
        cv2.putText(frame, "LASER", (cx - 25, cy - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # 搜索模式方向指示
    if state == 4:
        search_dir = info.get("search_dir", 0)
        arrow = "<< SEARCH" if search_dir == 0 else "SEARCH >>"
        cv2.putText(frame, arrow, (cx - 40, h - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 128, 0), 1)

    # 画圆进度
    if state == 5:
        progress = info.get("circle_progress", 0)
        cv2.putText(frame, f"CIRCLE: {progress*100:.0f}%",
                    (5, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 0, 255), 1)
