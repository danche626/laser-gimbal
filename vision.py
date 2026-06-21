"""
vision.py - 视觉检测模块
参照TI奖视觉代码: 反向二值化 + Canny边缘融合 + 矩形几何验证
"""
import cv2
import numpy as np
from collections import namedtuple

DetResult = namedtuple("DetResult", ["cx", "cy", "area", "approx", "distance_mm"])

_last_area = [0]


def detect_target(gray, cfg):
    """主检测流程: 高斯模糊 + 反向二值化 + 闭运算 + RETR_EXTERNAL + 矩形验证"""
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    _, binary = cv2.threshold(gray, cfg.BINARY_THRESHOLD, 255, cv2.THRESH_BINARY_INV)

    kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT, (cfg.MORPH_CLOSE_SIZE, cfg.MORPH_CLOSE_SIZE))
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    # RETR_EXTERNAL: 只取最外层轮廓，忽略内部红色圆环
    _, contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    h, w = gray.shape
    frame_area = h * w
    best = None
    best_area = 0

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < cfg.AREA_MIN_PIXELS:
            continue
        if area > frame_area * cfg.AREA_MAX_RATIO:
            continue

        peri = cv2.arcLength(contour, True)
        if peri < 20:
            continue
        approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
        if len(approx) != 4:
            continue

        corners = approx.reshape(4, 2)
        if not _check_rectangle(corners, cfg.ANGLE_TOLERANCE, cfg.SIDE_RATIO_TOLERANCE):
            continue

        rect = cv2.minAreaRect(contour)
        rw, rh = rect[1]
        if min(rw, rh) < 10:
            continue
        if max(rw, rh) / min(rw, rh) > cfg.RECT_ASPECT_MAX:
            continue

        if area > best_area:
            best_area = area
            M = cv2.moments(contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                dist = estimate_distance(area, cfg)
                best = DetResult(cx, cy, area, approx, dist)

    # 帧间面积一致性过滤
    if best and _last_area[0] > 0:
        ratio = best.area / _last_area[0]
        if ratio < cfg.AREA_JUMP_LOW or ratio > cfg.AREA_JUMP_HIGH:
            return None
    if best:
        _last_area[0] = best.area

    return best


def _check_rectangle(corners, angle_tol, side_ratio_tol):
    """验证四边形是否为矩形: 角度接近90° + 对边长度接近"""
    for i in range(4):
        p1 = corners[(i - 1) % 4].astype(float)
        p2 = corners[i].astype(float)
        p3 = corners[(i + 1) % 4].astype(float)
        v1 = p1 - p2
        v2 = p3 - p2
        len1 = np.linalg.norm(v1)
        len2 = np.linalg.norm(v2)
        if len1 == 0 or len2 == 0:
            return False
        cos_a = np.dot(v1, v2) / (len1 * len2)
        cos_a = np.clip(cos_a, -1.0, 1.0)
        angle = np.degrees(np.arccos(cos_a))
        if abs(angle - 90) > angle_tol:
            return False

    sides = []
    for i in range(4):
        sides.append(np.linalg.norm(corners[(i+1) % 4] - corners[i]))
    r1 = abs(sides[0] - sides[2]) / max(sides[0], sides[2]) if max(sides[0], sides[2]) > 0 else 1
    r2 = abs(sides[1] - sides[3]) / max(sides[1], sides[3]) if max(sides[1], sides[3]) > 0 else 1
    if r1 > side_ratio_tol or r2 > side_ratio_tol:
        return False
    return True


def perspective_center(gray, approx):
    """透视校正后精确定位矩形中心"""
    pts = approx.reshape(4, 2).astype(np.float32)
    s = pts.sum(axis=1)
    d = np.diff(pts, axis=1).flatten()
    tl = pts[np.argmin(s)]
    br = pts[np.argmax(s)]
    tr = pts[np.argmin(d)]
    bl = pts[np.argmax(d)]
    src = np.array([tl, tr, br, bl], dtype=np.float32)

    w1 = np.linalg.norm(tr - tl)
    w2 = np.linalg.norm(br - bl)
    h1 = np.linalg.norm(bl - tl)
    h2 = np.linalg.norm(br - tr)
    tw = int(max(w1, w2))
    th = int(max(h1, h2))
    if tw < 20 or th < 20:
        return None

    dst = np.array([[0, 0], [tw-1, 0], [tw-1, th-1], [0, th-1]],
                   dtype=np.float32)
    M_persp = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(gray, M_persp, (tw, th))

    _, wb = cv2.threshold(warped, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    _, w_contours, _ = cv2.findContours(wb, cv2.RETR_EXTERNAL,
                                         cv2.CHAIN_APPROX_SIMPLE)
    best_m = None
    best_a = 0
    for c in w_contours:
        a = cv2.contourArea(c)
        if a > best_a:
            best_a = a
            best_m = cv2.moments(c)

    if best_m and best_m["m00"] > 0:
        pcx = best_m["m10"] / best_m["m00"]
        pcy = best_m["m01"] / best_m["m00"]
    else:
        pcx, pcy = tw / 2.0, th / 2.0

    M_inv = cv2.getPerspectiveTransform(dst, src)
    pt = np.array([[[pcx, pcy]]], dtype=np.float32)
    orig_pt = cv2.perspectiveTransform(pt, M_inv)
    ox, oy = orig_pt[0][0]
    return (int(ox), int(oy))


def estimate_distance(area_pixels, cfg):
    """面积法测距: D = D_ref * sqrt(A_ref / A_cur)"""
    if area_pixels <= 0:
        return -1.0
    return cfg.CALIB_DISTANCE_MM * np.sqrt(cfg.CALIB_AREA_PIXELS / area_pixels)


def dynamic_laser_offset(distance_mm, cfg):
    """根据距离动态计算Y轴激光偏移(像素)"""
    if distance_mm <= 0:
        return -25
    offset = cfg.FOCAL_LENGTH_PX * cfg.LASER_BELOW_CAM_MM / distance_mm
    return -int(offset)
