#!/usr/bin/env python3
"""
separator_debug.py — GUI tuning tool for separator_visao.py
============================================================
Camera  : port 0 (Mac — AVFoundation)
Config  : separator_config.json  (copy to Pi after tuning)
Groups  : WARM  = Yellow / Orange / Red
          COOL  = Blue   / Black
          WHITE = background (no ball)

Controls
--------
  Left-drag   set ROI rectangle
  S           save config
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
CAM_PORT    = 0
FRAME_W     = 640
FRAME_H     = 480

MAIN_WIN = "Separator Debug"
CTRL_WIN = "Controls"
MASK_WIN = "Masks"

# ══════════════════════════════════════════════════════════════════════
#  DEFAULT CONFIG
# ══════════════════════════════════════════════════════════════════════
DEFAULT_CFG = dict(
    roi_x=200, roi_y=100, roi_w=240, roi_h=280,

    # Warm = red / orange / yellow
    warm_h_lo=0,   warm_h_hi=35,
    warm_s_min=80, warm_v_min=80,

    # Cool = blue
    cool_h_lo=95,  cool_h_hi=135,
    cool_s_min=60, cool_v_min=40,

    # Black is folded into cool
    black_v_max=50,

    # White = background (no ball)
    white_s_max=40,    # low saturation
    white_v_min=180,   # high brightness

    # Detection thresholds (% of ROI pixels)
    warm_frac=5,
    cool_frac=5,
    white_frac=60, # if ROI is this much white, declare "no ball"

    morph_k=5,
    morph_iter=1,
)

def load_cfg() -> dict:
    if not os.path.exists(CONFIG_FILE):
        print(f"[Config] {CONFIG_FILE} not found — defaults")
        return dict(DEFAULT_CFG)
    with open(CONFIG_FILE) as f:
        saved = json.load(f)
    if "roi" in saved:
        r = saved["roi"]
        saved["roi_x"] = r["x"]; saved["roi_y"] = r["y"]
        saved["roi_w"] = r["w"]; saved["roi_h"] = r["h"]
    cfg = {**DEFAULT_CFG, **saved}
    print(f"[Config] loaded — ROI {cfg['roi_w']}x{cfg['roi_h']} @ ({cfg['roi_x']},{cfg['roi_y']})")
    return cfg

def save_cfg(cfg: dict):
    out = {k: v for k, v in cfg.items()}
    with open(CONFIG_FILE, 'w') as f:
        json.dump(out, f, indent=2)
    print(f"[Config] saved → {CONFIG_FILE}")

# ══════════════════════════════════════════════════════════════════════
#  TRACKBARS
# ══════════════════════════════════════════════════════════════════════
def make_trackbars(cfg: dict):
    cv2.namedWindow(CTRL_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CTRL_WIN, 420, 620)
    def tb(name, val, mx):
        cv2.createTrackbar(name, CTRL_WIN, int(val), mx, lambda _: None)

    tb("warm_h_lo",   cfg['warm_h_lo'],   180)
    tb("warm_h_hi",   cfg['warm_h_hi'],   180)
    tb("warm_s_min",  cfg['warm_s_min'],  255)
    tb("warm_v_min",  cfg['warm_v_min'],  255)

    tb("cool_h_lo",   cfg['cool_h_lo'],   180)
    tb("cool_h_hi",   cfg['cool_h_hi'],   180)
    tb("cool_s_min",  cfg['cool_s_min'],  255)
    tb("cool_v_min",  cfg['cool_v_min'],  255)

    tb("black_v_max", cfg['black_v_max'], 255)

    tb("white_s_max", cfg['white_s_max'], 255)
    tb("white_v_min", cfg['white_v_min'], 255)

    tb("warm_frac%",  cfg['warm_frac'],   100)
    tb("cool_frac%",  cfg['cool_frac'],   100)
    tb("white_frac%", cfg['white_frac'],  100)

    tb("morph_k",     cfg['morph_k'],     21)
    tb("morph_iter",  cfg['morph_iter'],  5)

def read_trackbars(cfg: dict):
    def g(n): return cv2.getTrackbarPos(n, CTRL_WIN)
    cfg['warm_h_lo']   = g('warm_h_lo')
    cfg['warm_h_hi']   = g('warm_h_hi')
    cfg['warm_s_min']  = g('warm_s_min')
    cfg['warm_v_min']  = g('warm_v_min')
    cfg['cool_h_lo']   = g('cool_h_lo')
    cfg['cool_h_hi']   = g('cool_h_hi')
    cfg['cool_s_min']  = g('cool_s_min')
    cfg['cool_v_min']  = g('cool_v_min')
    cfg['black_v_max'] = g('black_v_max')
    cfg['white_s_max'] = g('white_s_max')
    cfg['white_v_min'] = g('white_v_min')
    cfg['warm_frac']   = g('warm_frac%')
    cfg['cool_frac']   = g('cool_frac%')
    cfg['white_frac']  = g('white_frac%')
    cfg['morph_k']     = max(1, g('morph_k') | 1)
    cfg['morph_iter']  = max(1, g('morph_iter'))

# ══════════════════════════════════════════════════════════════════════
#  MOUSE → ROI drag
# ══════════════════════════════════════════════════════════════════════
_mouse = dict(dragging=False, x0=0, y0=0)

def make_mouse_cb(cfg: dict):
    def cb(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            _mouse['dragging'] = True
            _mouse['x0'], _mouse['y0'] = x, y
        elif event == cv2.EVENT_MOUSEMOVE and _mouse['dragging']:
            cfg['roi_x'] = min(_mouse['x0'], x)
            cfg['roi_y'] = min(_mouse['y0'], y)
            cfg['roi_w'] = abs(x - _mouse['x0'])
            cfg['roi_h'] = abs(y - _mouse['y0'])
        elif event == cv2.EVENT_LBUTTONUP:
            _mouse['dragging'] = False
            cfg['roi_x'] = min(_mouse['x0'], x)
            cfg['roi_y'] = min(_mouse['y0'], y)
            cfg['roi_w'] = abs(x - _mouse['x0'])
            cfg['roi_h'] = abs(y - _mouse['y0'])
    return cb

# ══════════════════════════════════════════════════════════════════════
#  VISION
# ══════════════════════════════════════════════════════════════════════
def process(roi_hsv, cfg, roi_pixels):
    # warm (red wraps around 0/180)
    warm = cv2.inRange(roi_hsv,
        np.array([cfg['warm_h_lo'], cfg['warm_s_min'], cfg['warm_v_min']], np.uint8),
        np.array([cfg['warm_h_hi'], 255, 255], np.uint8))
    if cfg['warm_h_lo'] < 10:
        warm = cv2.bitwise_or(warm, cv2.inRange(roi_hsv,
            np.array([160, cfg['warm_s_min'], cfg['warm_v_min']], np.uint8),
            np.array([180, 255, 255], np.uint8)))

    # cool (blue) + black folded in
    cool = cv2.inRange(roi_hsv,
        np.array([cfg['cool_h_lo'], cfg['cool_s_min'], cfg['cool_v_min']], np.uint8),
        np.array([cfg['cool_h_hi'], 255, 255], np.uint8))
    blk  = (roi_hsv[:, :, 2] < cfg['black_v_max']).astype(np.uint8) * 255
    cool = cv2.bitwise_or(cool, blk)

    # white = low saturation + high value (background)
    white = ((roi_hsv[:, :, 1] < cfg['white_s_max']) &
             (roi_hsv[:, :, 2] > cfg['white_v_min'])).astype(np.uint8) * 255

    # morphology
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cfg['morph_k'], cfg['morph_k']))
    n = cfg['morph_iter']
    warm  = cv2.morphologyEx(warm,  cv2.MORPH_OPEN,  k, iterations=n)
    warm  = cv2.morphologyEx(warm,  cv2.MORPH_CLOSE, k, iterations=n)
    cool  = cv2.morphologyEx(cool,  cv2.MORPH_OPEN,  k, iterations=n)
    cool  = cv2.morphologyEx(cool,  cv2.MORPH_CLOSE, k, iterations=n)
    white = cv2.morphologyEx(white, cv2.MORPH_OPEN,  k, iterations=n)
    white = cv2.morphologyEx(white, cv2.MORPH_CLOSE, k, iterations=n)

    warm_pct  = np.count_nonzero(warm)  / roi_pixels * 100
    cool_pct  = np.count_nonzero(cool)  / roi_pixels * 100
    white_pct = np.count_nonzero(white) / roi_pixels * 100

    # if mostly white → no ball present
    no_ball = white_pct >= cfg['white_frac']

    warm_hit = (not no_ball) and (warm_pct >= cfg['warm_frac'])
    cool_hit = (not no_ball) and (cool_pct >= cfg['cool_frac'])

    return warm, cool, white, warm_hit, cool_hit, no_ball, warm_pct, cool_pct, white_pct

# ══════════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════════
def main():
    cfg = load_cfg()

    cap = cv2.VideoCapture(CAM_PORT, cv2.CAP_AVFOUNDATION)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    if not cap.isOpened():
        sys.exit(f"[ERROR] Cannot open /dev/video{CAM_PORT}")

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[Camera] /dev/video{CAM_PORT}  {actual_w}x{actual_h}")

    cv2.namedWindow(MAIN_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(MAIN_WIN, actual_w, actual_h)
    cv2.setMouseCallback(MAIN_WIN, make_mouse_cb(cfg))
    make_trackbars(cfg)

    show_masks  = False
    frame_count = 0
    t0          = time.time()
    fps_disp    = 0.0

    print("[Running]  S=save  M=masks  Q=quit")

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        read_trackbars(cfg)

        rx = max(0, min(cfg['roi_x'], actual_w - 2))
        ry = max(0, min(cfg['roi_y'], actual_h - 2))
        rw = max(2, min(cfg['roi_w'], actual_w - rx))
        rh = max(2, min(cfg['roi_h'], actual_h - ry))
        roi_pixels = rw * rh

        roi_hsv = cv2.cvtColor(frame[ry:ry+rh, rx:rx+rw], cv2.COLOR_BGR2HSV)
        (warm_mask, cool_mask, white_mask,
         warm_hit, cool_hit, no_ball,
         warm_pct, cool_pct, white_pct) = process(roi_hsv, cfg, roi_pixels)

        # ── tinted overlay ──
        display  = frame.copy()
        roi_disp = display[ry:ry+rh, rx:rx+rw]
        tint = np.zeros_like(roi_disp)
        tint[warm_mask  > 0] = (0, 130, 255)      # orange
        tint[cool_mask  > 0] = (230, 70, 0)       # blue
        tint[white_mask > 0] = (200, 200, 200)    # gray
        cv2.addWeighted(tint, 0.40, roi_disp, 1.0, 0, roi_disp)

        # ── ROI border color by state ──
        if   no_ball:               roi_col, label = (200,200,200), "WHITE (no ball)"
        elif warm_hit and cool_hit: roi_col, label = (0,255,255),   "AMBIGUOUS"
        elif warm_hit:              roi_col, label = (0,130,255),   "WARM → mature"
        elif cool_hit:              roi_col, label = (230,70,0),    "COOL → immature"
        else:                       roi_col, label = (0,0,255),     "BLACK ball → holder"
        cv2.rectangle(display, (rx, ry), (rx+rw, ry+rh), roi_col, 2)

        # FPS
        frame_count += 1
        if frame_count % 15 == 0:
            fps_disp = frame_count / (time.time() - t0)

        # HUD
        hud1 = f"FPS {fps_disp:.1f}   W:{warm_pct:4.1f}%  C:{cool_pct:4.1f}%  Wh:{white_pct:4.1f}%"
        cv2.putText(display, hud1, (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 2, cv2.LINE_AA)
        cv2.putText(display, label, (8, 52),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, roi_col, 2, cv2.LINE_AA)
        cv2.putText(display, f"ROI {rw}x{rh} @ ({rx},{ry})",
                    (rx, ry - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, roi_col, 1)
        cv2.putText(display, "Drag=ROI  S=save  M=masks  Q=quit",
                    (8, actual_h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160,160,160), 1)

        cv2.imshow(MAIN_WIN, display)

        if show_masks:
            combined = np.zeros((*warm_mask.shape, 3), dtype=np.uint8)
            combined[warm_mask  > 0] = (0, 130, 255)
            combined[cool_mask  > 0] = (230, 70, 0)
            combined[white_mask > 0] = (200, 200, 200)
            cv2.imshow(MASK_WIN, combined)
        else:
            try: cv2.destroyWindow(MASK_WIN)
            except cv2.error: pass

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