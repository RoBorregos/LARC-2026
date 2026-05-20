#!/usr/bin/env python3
"""
benefits_debug.py — Box color detector with calibration UI
============================================================
Camera  : /dev/video4  (Logitech C920)
Controls: Trackbars for ROI, color thresholds, and min percentage
Keys    : S = save config, Q/ESC = quit

Windows:
  - Frame: live feed with ROI rectangle + detection result
  - Red Mask: thresholded red pixels inside ROI
  - Blue Mask: thresholded blue pixels inside ROI
  - Controls: trackbar sliders
"""

import cv2
import numpy as np
import json
import os
import sys

# ══════════════════════════════════════════════════════════════════════
#  CONFIG
# ══════════════════════════════════════════════════════════════════════
CONFIG_FILE = "benefits_config.json"
CAM_PORT    = 0
FRAME_W     = 640
FRAME_H     = 480

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
        print(f"[Config] Using defaults")
        return dict(DEFAULT_CFG)
    with open(CONFIG_FILE) as f:
        saved = json.load(f)
    cfg = {**DEFAULT_CFG, **saved}
    print(f"[Config] Loaded {CONFIG_FILE}")
    return cfg

def save_cfg(cfg):
    with open(CONFIG_FILE, 'w') as f:
        json.dump(cfg, f, indent=2)
    print(f"[Config] Saved to {CONFIG_FILE}")

# ══════════════════════════════════════════════════════════════════════
#  TRACKBAR SETUP
# ══════════════════════════════════════════════════════════════════════
CTRL_WIN = "Controls"

def nothing(x):
    pass

def setup_trackbars(cfg):
    cv2.namedWindow(CTRL_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CTRL_WIN, 500, 700)

    # ROI
    cv2.createTrackbar("ROI x1",     CTRL_WIN, cfg['roi_x1'], FRAME_W, nothing)
    cv2.createTrackbar("ROI y1",     CTRL_WIN, cfg['roi_y1'], FRAME_H, nothing)
    cv2.createTrackbar("ROI x2",     CTRL_WIN, cfg['roi_x2'], FRAME_W, nothing)
    cv2.createTrackbar("ROI y2",     CTRL_WIN, cfg['roi_y2'], FRAME_H, nothing)

    # Red thresholds
    cv2.createTrackbar("Red H lo1",  CTRL_WIN, cfg['red_h_lo1'], 180, nothing)
    cv2.createTrackbar("Red H hi1",  CTRL_WIN, cfg['red_h_hi1'], 180, nothing)
    cv2.createTrackbar("Red H lo2",  CTRL_WIN, cfg['red_h_lo2'], 180, nothing)
    cv2.createTrackbar("Red H hi2",  CTRL_WIN, cfg['red_h_hi2'], 180, nothing)
    cv2.createTrackbar("Red S min",  CTRL_WIN, cfg['red_s_min'],  255, nothing)
    cv2.createTrackbar("Red V min",  CTRL_WIN, cfg['red_v_min'],  255, nothing)

    # Blue thresholds
    cv2.createTrackbar("Blue H lo",  CTRL_WIN, cfg['blue_h_lo'], 180, nothing)
    cv2.createTrackbar("Blue H hi",  CTRL_WIN, cfg['blue_h_hi'], 180, nothing)
    cv2.createTrackbar("Blue S min", CTRL_WIN, cfg['blue_s_min'], 255, nothing)
    cv2.createTrackbar("Blue V min", CTRL_WIN, cfg['blue_v_min'], 255, nothing)

    # Detection threshold
    cv2.createTrackbar("Min %",      CTRL_WIN, cfg['min_pct'],   100, nothing)

    # Morphology
    cv2.createTrackbar("Morph K",    CTRL_WIN, cfg['morph_k'],    21, nothing)
    cv2.createTrackbar("Morph Iter", CTRL_WIN, cfg['morph_iter'],  10, nothing)

def read_trackbars() -> dict:
    return dict(
        roi_x1    = cv2.getTrackbarPos("ROI x1",     CTRL_WIN),
        roi_y1    = cv2.getTrackbarPos("ROI y1",     CTRL_WIN),
        roi_x2    = cv2.getTrackbarPos("ROI x2",     CTRL_WIN),
        roi_y2    = cv2.getTrackbarPos("ROI y2",     CTRL_WIN),

        red_h_lo1 = cv2.getTrackbarPos("Red H lo1",  CTRL_WIN),
        red_h_hi1 = cv2.getTrackbarPos("Red H hi1",  CTRL_WIN),
        red_h_lo2 = cv2.getTrackbarPos("Red H lo2",  CTRL_WIN),
        red_h_hi2 = cv2.getTrackbarPos("Red H hi2",  CTRL_WIN),
        red_s_min = cv2.getTrackbarPos("Red S min",  CTRL_WIN),
        red_v_min = cv2.getTrackbarPos("Red V min",  CTRL_WIN),

        blue_h_lo  = cv2.getTrackbarPos("Blue H lo",  CTRL_WIN),
        blue_h_hi  = cv2.getTrackbarPos("Blue H hi",  CTRL_WIN),
        blue_s_min = cv2.getTrackbarPos("Blue S min", CTRL_WIN),
        blue_v_min = cv2.getTrackbarPos("Blue V min", CTRL_WIN),

        min_pct    = cv2.getTrackbarPos("Min %",      CTRL_WIN),
        morph_k    = cv2.getTrackbarPos("Morph K",    CTRL_WIN),
        morph_iter = cv2.getTrackbarPos("Morph Iter", CTRL_WIN),
    )

# ══════════════════════════════════════════════════════════════════════
#  DETECTION
# ══════════════════════════════════════════════════════════════════════
BOX_NONE = 0
BOX_RED  = 1
BOX_BLUE = 2
BOX_NAMES = {BOX_NONE: "NONE", BOX_RED: "RED", BOX_BLUE: "BLUE"}
BOX_COLORS = {BOX_NONE: (180, 180, 180), BOX_RED: (0, 0, 255), BOX_BLUE: (255, 100, 0)}

def detect_box(roi_bgr, cfg):
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    total_pixels = roi_bgr.shape[0] * roi_bgr.shape[1]

    mk = max(1, cfg['morph_k'] | 1)  # must be odd
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (mk, mk))
    n = max(1, cfg['morph_iter'])

    # Red mask
    red1 = cv2.inRange(hsv,
        np.array([cfg['red_h_lo1'], cfg['red_s_min'], cfg['red_v_min']], np.uint8),
        np.array([cfg['red_h_hi1'], 255,              255],              np.uint8))
    red2 = cv2.inRange(hsv,
        np.array([cfg['red_h_lo2'], cfg['red_s_min'], cfg['red_v_min']], np.uint8),
        np.array([cfg['red_h_hi2'], 255,              255],              np.uint8))
    red_mask = cv2.bitwise_or(red1, red2)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN,  k, iterations=n)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, k, iterations=n)

    # Blue mask
    blue_mask = cv2.inRange(hsv,
        np.array([cfg['blue_h_lo'], cfg['blue_s_min'], cfg['blue_v_min']], np.uint8),
        np.array([cfg['blue_h_hi'], 255,               255],               np.uint8))
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN,  k, iterations=n)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, k, iterations=n)

    red_pct  = np.count_nonzero(red_mask)  / total_pixels * 100
    blue_pct = np.count_nonzero(blue_mask) / total_pixels * 100

    if red_pct >= cfg['min_pct'] and red_pct > blue_pct:
        box = BOX_RED
    elif blue_pct >= cfg['min_pct'] and blue_pct > red_pct:
        box = BOX_BLUE
    else:
        box = BOX_NONE

    return box, red_pct, blue_pct, red_mask, blue_mask

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
    print(f"[Keys] S=save config  Q/ESC=quit")

    setup_trackbars(cfg)

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        cfg = read_trackbars()

        # Clamp ROI
        x1 = max(0, min(cfg['roi_x1'], actual_w - 2))
        y1 = max(0, min(cfg['roi_y1'], actual_h - 2))
        x2 = max(x1 + 2, min(cfg['roi_x2'], actual_w))
        y2 = max(y1 + 2, min(cfg['roi_y2'], actual_h))

        roi = frame[y1:y2, x1:x2]
        box, red_pct, blue_pct, red_mask, blue_mask = detect_box(roi, cfg)

        # Draw ROI rectangle on frame
        color = BOX_COLORS[box]
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        # Draw label
        label = f"{BOX_NAMES[box]}  R:{red_pct:.1f}%  B:{blue_pct:.1f}%"
        cv2.putText(frame, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # Draw crosshair in ROI center
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        cv2.line(frame, (cx - 15, cy), (cx + 15, cy), (0, 255, 0), 1)
        cv2.line(frame, (cx, cy - 15), (cx, cy + 15), (0, 255, 0), 1)

        # Show windows
        cv2.imshow("Frame", frame)
        cv2.imshow("Red Mask", red_mask)
        cv2.imshow("Blue Mask", blue_mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:  # Q or ESC
            break
        elif key == ord('s') or key == ord('S'):
            save_cfg(cfg)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()