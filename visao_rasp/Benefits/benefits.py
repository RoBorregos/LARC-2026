#!/usr/bin/env python3
"""
benefits.py — Box color detector (dispatcher mode)
====================================================
Camera  : /dev/video{CAM_PORT}  (V4L2)
Config  : benefits_config.json
Serial  : NONE — prints VISION:FE:XX lines for dispatcher to forward
Output  : detection results + VISION tags to stdout
Ctrl+C  : stop
"""

import cv2
import numpy as np
import sys, os, time, json

# ══════════════════════════════════════════════════════════════════════
#  CONFIG
# ══════════════════════════════════════════════════════════════════════
CONFIG_FILE = "benefits_config.json"
CAM_PORT    = 4
FRAME_W     = 640
FRAME_H     = 480

BOX_NONE = 0
BOX_RED  = 1
BOX_BLUE = 2
BOX_NAMES = {BOX_NONE: "NONE", BOX_RED: "RED", BOX_BLUE: "BLUE"}

DEFAULT_CFG = dict(
    roi_x1=200, roi_y1=150,
    roi_x2=440, roi_y2=330,
    red_h_lo1=0,    red_h_hi1=10,
    red_h_lo2=170,  red_h_hi2=180,
    red_s_min=80,   red_v_min=80,
    blue_h_lo=100,  blue_h_hi=130,
    blue_s_min=80,  blue_v_min=60,
    min_pct=5,
    morph_k=5,
    morph_iter=1,
)

def load_cfg() -> dict:
    if not os.path.exists(CONFIG_FILE):
        print(f"[Config] {CONFIG_FILE} not found -- using defaults")
        return dict(DEFAULT_CFG)
    with open(CONFIG_FILE) as f:
        saved = json.load(f)
    cfg = {**DEFAULT_CFG, **saved}
    print(f"[Config] loaded {CONFIG_FILE}")
    return cfg

# ══════════════════════════════════════════════════════════════════════
#  DETECTION
# ══════════════════════════════════════════════════════════════════════
def detect_box(roi_bgr, cfg):
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    total_pixels = roi_bgr.shape[0] * roi_bgr.shape[1]

    mk = max(1, cfg['morph_k'] | 1)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (mk, mk))
    n = max(1, cfg['morph_iter'])

    red1 = cv2.inRange(hsv,
        np.array([cfg['red_h_lo1'], cfg['red_s_min'], cfg['red_v_min']], np.uint8),
        np.array([cfg['red_h_hi1'], 255,              255],              np.uint8))
    red2 = cv2.inRange(hsv,
        np.array([cfg['red_h_lo2'], cfg['red_s_min'], cfg['red_v_min']], np.uint8),
        np.array([cfg['red_h_hi2'], 255,              255],              np.uint8))
    red_mask = cv2.bitwise_or(red1, red2)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN,  k, iterations=n)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, k, iterations=n)

    blue_mask = cv2.inRange(hsv,
        np.array([cfg['blue_h_lo'], cfg['blue_s_min'], cfg['blue_v_min']], np.uint8),
        np.array([cfg['blue_h_hi'], 255,               255],               np.uint8))
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN,  k, iterations=n)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, k, iterations=n)

    red_pct  = np.count_nonzero(red_mask)  / total_pixels * 100
    blue_pct = np.count_nonzero(blue_mask) / total_pixels * 100

    if red_pct >= cfg['min_pct'] and red_pct > blue_pct:
        return BOX_RED, red_pct, blue_pct
    elif blue_pct >= cfg['min_pct'] and blue_pct > red_pct:
        return BOX_BLUE, red_pct, blue_pct
    else:
        return BOX_NONE, red_pct, blue_pct

# ══════════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════════
def main():
    cfg = load_cfg()

    cap = cv2.VideoCapture("/dev/video_c920", cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    if not cap.isOpened():
        sys.exit("[ERROR] Cannot open /dev/video_c920")


    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[Camera] /dev/video_c920  {actual_w}x{actual_h}")
    print("[Mode] Dispatcher (VISION tags via stdout)")
    print("[Running] Ctrl+C to stop")

    frame_count = 0
    t0     = time.time()
    t_last = t0
    last_box = BOX_NONE

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                continue

            x1 = max(0, min(cfg['roi_x1'], actual_w - 2))
            y1 = max(0, min(cfg['roi_y1'], actual_h - 2))
            x2 = max(x1+2, min(cfg['roi_x2'], actual_w))
            y2 = max(y1+2, min(cfg['roi_y2'], actual_h))

            roi = frame[y1:y2, x1:x2]
            box_type, red_pct, blue_pct = detect_box(roi, cfg)

            # Output vision data for dispatcher to forward
            # Format: VISION:FE:XX (hex bytes)
            print(f"VISION:FE:{box_type:02X}", flush=True)

            frame_count += 1
            now = time.time()
            if now - t_last >= 1.0:
                fps = frame_count / (now - t0)
                name = BOX_NAMES[box_type]
                print(f"[FPS] {fps:5.1f}  |  Box: {name:4s}  |  red: {red_pct:5.1f}%  blue: {blue_pct:5.1f}%")
                t_last = now

            if box_type != last_box:
                print(f"[DETECT] Box changed: {BOX_NAMES[last_box]} -> {BOX_NAMES[box_type]}")
                last_box = box_type

    except KeyboardInterrupt:
        elapsed = time.time() - t0
        print(f"\n[Stopped]  {frame_count} frames in {elapsed:.1f}s  avg {frame_count/elapsed:.1f} fps")
    finally:
        cap.release()

if __name__ == "__main__":
    main()