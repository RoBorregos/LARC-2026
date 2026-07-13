#!/usr/bin/env python3
"""
separator_visao.py — Debug/tuning tool for ball color sorter
=============================================================
Raspberry Pi: serial disabled (for now)
Camera port : 0
Config file : separator_config.json

Groups :
  WARM  = Yellow / Orange / Red
  COOL  = Blue   / Black


=============================================================
Controls
=============================================================
  Left-drag   set ROI rectangle
  S           save config  =  separator_config.json
  M           toggle mask overlay
  Q / ESC     quit
  
"""

import cv2
import numpy as np
import json, os, sys, time

# ══════════════════════════════════════════════════════════════════════
#  CONSTANTS
# ══════════════════════════════════════════════════════════════════════
CONFIG_FILE = "separator_config.json"
CAM_PORT    = 0          # /dev/video2  — change to 0 for raspi_visao
FRAME_W     = 1920
FRAME_H     = 1080

MAIN_WIN    = "Separator Debug"
CTRL_WIN    = "Controls"
MASK_WIN    = "Masks"

_DISP_W, _DISP_H = 1280, 720

# ══════════════════════════════════════════════════════════════════════
#  CAMERA
# ══════════════════════════════════════════════════════════════════════
def open_camera():
    cap = cv2.VideoCapture(CAM_PORT, cv2.CAP_AVFOUNDATION)
    if not cap.isOpened():
        sys.exit(f"[ERROR] Cannot open /dev/video{CAM_PORT}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[Camera] /dev/video{CAM_PORT}  {w}x{h} @ {fps:.1f} fps")
    return cap

# ══════════════════════════════════════════════════════════════════════
#  DEFAULT CONFIG
# ══════════════════════════════════════════════════════════════════════
DEFAULT_CFG = dict(
    roi_x1=760, roi_y1=340, roi_x2=1160, roi_y2=740,

    warm_h_lo=0,   warm_h_hi=35,
    warm_s_min=80, warm_v_min=80,

    cool_h_lo=95,  cool_h_hi=135,
    cool_s_min=60, cool_v_min=40,

    black_v_max=50,

    warm_frac=5,
    cool_frac=5,

    morph_k=5,
    morph_iter=1,

    ghost_frames=10,

    area_min=800,
    area_max=80000,
)

def load_cfg() -> dict:
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE) as f:
            saved = json.load(f)
        merged = {**DEFAULT_CFG, **saved}
        print(f"[Config] loaded from {CONFIG_FILE}")
        return merged
    print(f"[Config] no {CONFIG_FILE} found — using defaults")
    return dict(DEFAULT_CFG)

def save_cfg(cfg: dict):
    with open(CONFIG_FILE, 'w') as f:
        json.dump(cfg, f, indent=2)
    print(f"[Config] saved → {CONFIG_FILE}")

# ══════════════════════════════════════════════════════════════════════
#  TRACKBARS
# ══════════════════════════════════════════════════════════════════════
def make_trackbars(cfg: dict):
    cv2.namedWindow(CTRL_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CTRL_WIN, 420, 600)

    def tb(name, val, mx):
        cv2.createTrackbar(name, CTRL_WIN, int(val), mx, lambda _: None)

    tb("warm_h_lo",    cfg['warm_h_lo'],    180)
    tb("warm_h_hi",    cfg['warm_h_hi'],    180)
    tb("warm_s_min",   cfg['warm_s_min'],   255)
    tb("warm_v_min",   cfg['warm_v_min'],   255)

    tb("cool_h_lo",    cfg['cool_h_lo'],    180)
    tb("cool_h_hi",    cfg['cool_h_hi'],    180)
    tb("cool_s_min",   cfg['cool_s_min'],   255)
    tb("cool_v_min",   cfg['cool_v_min'],   255)

    tb("black_v_max",  cfg['black_v_max'],  255)

    tb("warm_frac%",   cfg['warm_frac'],    100)
    tb("cool_frac%",   cfg['cool_frac'],    100)

    tb("morph_k",      cfg['morph_k'],      21)
    tb("morph_iter",   cfg['morph_iter'],   5)

    tb("ghost_frames", cfg['ghost_frames'], 60)

    tb("area_min/10",  cfg['area_min']//10, 5000)
    tb("area_max/10",  cfg['area_max']//10, 5000)

def read_trackbars(cfg: dict):
    def g(n): return cv2.getTrackbarPos(n, CTRL_WIN)
    cfg['warm_h_lo']    = g('warm_h_lo')
    cfg['warm_h_hi']    = g('warm_h_hi')
    cfg['warm_s_min']   = g('warm_s_min')
    cfg['warm_v_min']   = g('warm_v_min')
    cfg['cool_h_lo']    = g('cool_h_lo')
    cfg['cool_h_hi']    = g('cool_h_hi')
    cfg['cool_s_min']   = g('cool_s_min')
    cfg['cool_v_min']   = g('cool_v_min')
    cfg['black_v_max']  = g('black_v_max')
    cfg['warm_frac']    = g('warm_frac%')
    cfg['cool_frac']    = g('cool_frac%')
    cfg['morph_k']      = max(1, g('morph_k') | 1)
    cfg['morph_iter']   = max(1, g('morph_iter'))
    cfg['ghost_frames'] = max(1, g('ghost_frames'))
    cfg['area_min']     = g('area_min/10') * 10
    cfg['area_max']     = max(1, g('area_max/10')) * 10

# ══════════════════════════════════════════════════════════════════════
#  MOUSE → ROI drag
# ══════════════════════════════════════════════════════════════════════
_mouse = dict(dragging=False, x0=0, y0=0)

def _scale_to_full(x, y, full_w, full_h):
    return int(x * full_w / _DISP_W), int(y * full_h / _DISP_H)

def make_mouse_cb(cfg: dict, full_w: int, full_h: int):
    def cb(event, x, y, flags, param):
        fx, fy = _scale_to_full(x, y, full_w, full_h)
        if event == cv2.EVENT_LBUTTONDOWN:
            _mouse['dragging'] = True
            _mouse['x0'], _mouse['y0'] = fx, fy
        elif event == cv2.EVENT_MOUSEMOVE and _mouse['dragging']:
            cfg['roi_x1'] = min(_mouse['x0'], fx)
            cfg['roi_y1'] = min(_mouse['y0'], fy)
            cfg['roi_x2'] = max(_mouse['x0'], fx)
            cfg['roi_y2'] = max(_mouse['y0'], fy)
        elif event == cv2.EVENT_LBUTTONUP:
            _mouse['dragging'] = False
            cfg['roi_x1'] = min(_mouse['x0'], fx)
            cfg['roi_y1'] = min(_mouse['y0'], fy)
            cfg['roi_x2'] = max(_mouse['x0'], fx)
            cfg['roi_y2'] = max(_mouse['y0'], fy)
    return cb

# ══════════════════════════════════════════════════════════════════════
#  KALMAN GHOST TRACKER
# ══════════════════════════════════════════════════════════════════════
def _make_kalman():
    kf = cv2.KalmanFilter(4, 2)
    kf.measurementMatrix   = np.array([[1,0,0,0],[0,1,0,0]], np.float32)
    kf.transitionMatrix    = np.array([[1,0,1,0],[0,1,0,1],
                                        [0,0,1,0],[0,0,0,1]], np.float32)
    kf.processNoiseCov     = np.eye(4, dtype=np.float32) * 0.03
    kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1.0
    kf.errorCovPost        = np.eye(4, dtype=np.float32)
    return kf

_ghosts = {
    'warm': dict(kf=_make_kalman(), ttl=0, pos=(0, 0), r=0),
    'cool': dict(kf=_make_kalman(), ttl=0, pos=(0, 0), r=0),
}

def update_ghosts(detections: list, ghost_frames: int):
    seen = {k: None for k in _ghosts}
    for cx, cy, r, group in detections:
        if seen[group] is None or r > seen[group][2]:
            seen[group] = (cx, cy, r)

    for group, g in _ghosts.items():
        if seen[group]:
            cx, cy, r = seen[group]
            g['kf'].correct(np.array([[np.float32(cx)], [np.float32(cy)]]))
            g['ttl'] = ghost_frames
            g['r']   = r
        else:
            g['ttl'] = max(0, g['ttl'] - 1)
        pred     = g['kf'].predict()
        g['pos'] = (int(pred[0]), int(pred[1]))

# ══════════════════════════════════════════════════════════════════════
#  VISION PIPELINE
# ══════════════════════════════════════════════════════════════════════
def process_roi(roi_bgr: np.ndarray, cfg: dict):
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    h, w = roi_bgr.shape[:2]
    roi_pixels = h * w

    warm_lo1  = np.array([cfg['warm_h_lo'], cfg['warm_s_min'], cfg['warm_v_min']], np.uint8)
    warm_hi1  = np.array([cfg['warm_h_hi'], 255,               255],               np.uint8)
    warm_main = cv2.inRange(hsv, warm_lo1, warm_hi1)

    warm_mask = warm_main
    if cfg['warm_h_lo'] < 10:
        red_lo = np.array([160, cfg['warm_s_min'], cfg['warm_v_min']], np.uint8)
        red_hi = np.array([180, 255,               255],               np.uint8)
        warm_mask = cv2.bitwise_or(warm_main, cv2.inRange(hsv, red_lo, red_hi))

    cool_lo   = np.array([cfg['cool_h_lo'], cfg['cool_s_min'], cfg['cool_v_min']], np.uint8)
    cool_hi   = np.array([cfg['cool_h_hi'], 255,               255],               np.uint8)
    blue_mask = cv2.inRange(hsv, cool_lo, cool_hi)
    blk_mask  = (hsv[:, :, 2] < cfg['black_v_max']).astype(np.uint8) * 255
    cool_mask = cv2.bitwise_or(blue_mask, blk_mask)

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cfg['morph_k'], cfg['morph_k']))
    n = cfg['morph_iter']
    for m in (warm_mask, cool_mask):
        m[:] = cv2.morphologyEx(m, cv2.MORPH_OPEN,  k, iterations=n)
        m[:] = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k, iterations=n)

    warm_hit = (np.count_nonzero(warm_mask) / roi_pixels * 100) >= cfg['warm_frac']
    cool_hit = (np.count_nonzero(cool_mask) / roi_pixels * 100) >= cfg['cool_frac']

    detections = []
    for mask, group in ((warm_mask, 'warm'), (cool_mask, 'cool')):
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in cnts:
            area = cv2.contourArea(cnt)
            if cfg['area_min'] <= area <= cfg['area_max']:
                (cx, cy), r = cv2.minEnclosingCircle(cnt)
                detections.append((int(cx), int(cy), int(r), group))

    return warm_mask, cool_mask, detections, warm_hit, cool_hit

# ══════════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════════
def main():
    cfg = load_cfg()
    cap = open_camera()

    actual_w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[Camera] {actual_w}x{actual_h} @ {actual_fps:.1f} fps")
    print("[Serial] DISABLED")

    cv2.namedWindow(MAIN_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(MAIN_WIN, _DISP_W, _DISP_H)
    cv2.setMouseCallback(MAIN_WIN, make_mouse_cb(cfg, actual_w, actual_h))
    make_trackbars(cfg)

    show_masks  = False
    frame_count = 0
    t0          = time.time()
    fps_display = 0.0

    print("[Running]  S=save  M=masks  Q=quit")

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        read_trackbars(cfg)

        x1 = max(0, min(cfg['roi_x1'], actual_w - 2))
        y1 = max(0, min(cfg['roi_y1'], actual_h - 2))
        x2 = max(x1 + 2, min(cfg['roi_x2'], actual_w))
        y2 = max(y1 + 2, min(cfg['roi_y2'], actual_h))

        roi = frame[y1:y2, x1:x2]
        warm_mask, cool_mask, detections, pixel_warm_hit, pixel_cool_hit = \
            process_roi(roi, cfg)

        update_ghosts(detections, cfg['ghost_frames'])

        warm_hit = pixel_warm_hit or (_ghosts['warm']['ttl'] > 0)
        cool_hit = pixel_cool_hit or (_ghosts['cool']['ttl'] > 0)

        display  = frame.copy()
        roi_disp = display[y1:y2, x1:x2]
        tint = np.zeros_like(roi_disp)
        tint[warm_mask > 0] = (0, 130, 255)
        tint[cool_mask > 0] = (230, 70,  0)
        cv2.addWeighted(tint, 0.45, roi_disp, 1.0, 0, roi_disp)

        for cx, cy, r, group in detections:
            color = (0, 130, 255) if group == 'warm' else (230, 70, 0)
            cv2.circle(display, (x1 + cx, y1 + cy), r, color, 2)

        for group, g in _ghosts.items():
            if g['ttl'] > 0:
                gx, gy = g['pos']
                color = (0, 210, 255) if group == 'warm' else (200, 160, 255)
                cv2.circle(display, (x1 + gx, y1 + gy), max(g['r'], 8), color, 1)
                cv2.putText(display, f"{group[0].upper()} ghost:{g['ttl']}",
                            (x1 + gx + 10, y1 + gy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        if   warm_hit and cool_hit: roi_col = (0,   255, 255)
        elif warm_hit:              roi_col = (0,   130, 255)
        elif cool_hit:              roi_col = (230,  70,   0)
        else:                       roi_col = (80,  200,  80)
        cv2.rectangle(display, (x1, y1), (x2, y2), roi_col, 2)

        frame_count += 1
        if frame_count % 20 == 0:
            fps_display = frame_count / (time.time() - t0)

        warm_str = "■ WARM" if warm_hit else "  warm"
        cool_str = "■ COOL" if cool_hit else "  cool"
        cv2.putText(display,
                    f"FPS {fps_display:.1f}   {warm_str}   {cool_str}   [NO SERIAL]",
                    (12, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(display, "Drag=ROI  S=save  M=masks  Q=quit",
                    (12, actual_h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1)
        cv2.putText(display, f"ROI {x2-x1}x{y2-y1}",
                    (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, roi_col, 1)

        cv2.imshow(MAIN_WIN, cv2.resize(display, (_DISP_W, _DISP_H)))

        if show_masks and warm_mask is not None:
            combined = np.zeros((*warm_mask.shape, 3), dtype=np.uint8)
            combined[warm_mask > 0] = (0, 130, 255)
            combined[cool_mask > 0] = (230, 70, 0)
            cv2.imshow(MASK_WIN, cv2.resize(combined, (640, 360)))
        elif not show_masks:
            cv2.destroyWindow(MASK_WIN)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key == ord('s'):
            save_cfg(cfg)
        elif key == ord('m'):
            show_masks = not show_masks
            print(f"[Masks] {'ON' if show_masks else 'OFF'}")

    cap.release()
    cv2.destroyAllWindows()
    print("[Stopped]")


if __name__ == "__main__":
    main()