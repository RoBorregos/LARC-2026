#!/usr/bin/env python3
"""
separator_vision.py — Headless ball color classifier (vision to serial)
Purpose of this code : Watch a fixed ROI on the Orin, classify the ball sitting
                       there by color, and emit the result as VISION tags on
                       stdout. The dispatcher forwards those bytes to the Teensy,
                       which owns all servo / actuation logic. This file only
                       looks and reports — it drives no hardware.
Camera  : /dev/video_separator  (Global Shutter)
Config  : separator_config.json
Output  : VISION:FD:WW:CC lines on stdout  (WW = warm hit, CC = cool hit)
Ctrl+C  : stop

Classification
    WHITE (background)  to no ball, idle
    WARM (R/O/Y)        to mature
    COOL (blue + black) to immature
"""

import cv2
import numpy as np
import sys, json, os, time

CONFIG_FILE = "separator_config.json"
FRAME_W     = 640
FRAME_H     = 480

DEFAULT_CFG = dict(
    roi_x=551, roi_y=208, roi_w=197, roi_h=251,
    warm_h_lo=0,   warm_h_hi=35,
    warm_s_min=80, warm_v_min=80,
    cool_h_lo=95,  cool_h_hi=135,
    cool_s_min=60, cool_v_min=40,
    black_v_max=50,
    white_s_max=40,
    white_v_min=180,
    warm_frac=5,
    cool_frac=5,
    white_frac=60,
    morph_k=5,
    morph_iter=1,
)

# - Config
def load_cfg() -> dict:
    if not os.path.exists(CONFIG_FILE):
        print(f"[Config] {CONFIG_FILE} not found — using defaults", file=sys.stderr)
        return dict(DEFAULT_CFG)
    with open(CONFIG_FILE) as f:
        saved = json.load(f)
    if "roi" in saved:
        r = saved["roi"]
        saved["roi_x"] = r["x"]; saved["roi_y"] = r["y"]
        saved["roi_w"] = r["w"]; saved["roi_h"] = r["h"]
    cfg = {**DEFAULT_CFG, **saved}
    print(f"[Config] loaded — ROI {cfg['roi_w']}x{cfg['roi_h']} @ ({cfg['roi_x']},{cfg['roi_y']})",
          file=sys.stderr)
    return cfg

# - Classification
def process(roi_hsv, cfg, roi_pixels):
    """Return (warm_hit, cool_hit, no_ball) for the current ROI."""
    warm = cv2.inRange(roi_hsv,
        np.array([cfg['warm_h_lo'], cfg['warm_s_min'], cfg['warm_v_min']], np.uint8),
        np.array([cfg['warm_h_hi'], 255, 255], np.uint8))
    if cfg['warm_h_lo'] < 10:
        # Red wraps around the hue circle — also catch the high-hue red band
        warm = cv2.bitwise_or(warm, cv2.inRange(roi_hsv,
            np.array([160, cfg['warm_s_min'], cfg['warm_v_min']], np.uint8),
            np.array([180, 255, 255], np.uint8)))

    cool = cv2.inRange(roi_hsv,
        np.array([cfg['cool_h_lo'], cfg['cool_s_min'], cfg['cool_v_min']], np.uint8),
        np.array([cfg['cool_h_hi'], 255, 255], np.uint8))
    blk  = (roi_hsv[:, :, 2] < cfg['black_v_max']).astype(np.uint8) * 255
    cool = cv2.bitwise_or(cool, blk)

    white = ((roi_hsv[:, :, 1] < cfg['white_s_max']) &
             (roi_hsv[:, :, 2] > cfg['white_v_min'])).astype(np.uint8) * 255

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cfg['morph_k'], cfg['morph_k']))
    n = cfg['morph_iter']
    warm  = cv2.morphologyEx(warm,  cv2.MORPH_OPEN,  k, iterations=n)
    warm  = cv2.morphologyEx(warm,  cv2.MORPH_CLOSE, k, iterations=n)
    cool  = cv2.morphologyEx(cool,  cv2.MORPH_OPEN,  k, iterations=n)
    cool  = cv2.morphologyEx(cool,  cv2.MORPH_CLOSE, k, iterations=n)
    white = cv2.morphologyEx(white, cv2.MORPH_OPEN,  k, iterations=n)
    white = cv2.morphologyEx(white, cv2.MORPH_CLOSE, k, iterations=n)

    white_pct = np.count_nonzero(white) / roi_pixels * 100
    no_ball   = white_pct >= cfg['white_frac']

    warm_hit = (not no_ball) and (np.count_nonzero(warm) / roi_pixels * 100) >= cfg['warm_frac']
    cool_hit = (not no_ball) and (np.count_nonzero(cool) / roi_pixels * 100) >= cfg['cool_frac']
    return warm_hit, cool_hit, no_ball

# - Main
def main():
    cfg = load_cfg()

    cap = cv2.VideoCapture("/dev/video_separator", cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    if not cap.isOpened():
        sys.exit("[ERROR] Cannot open /dev/video_separator")

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[Camera] /dev/video_separator  {w}x{h}", file=sys.stderr)

    rx = max(0, min(cfg['roi_x'], w - 2))
    ry = max(0, min(cfg['roi_y'], h - 2))
    rw = min(cfg['roi_w'], w - rx)
    rh = min(cfg['roi_h'], h - ry)
    roi_pixels = rw * rh
    print(f"[ROI] {rw}x{rh} @ ({rx},{ry})  = {roi_pixels} px", file=sys.stderr)

    frames = 0
    hits_w = hits_c = idle = 0
    t0 = time.time()
    t_last = t0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            roi_hsv = cv2.cvtColor(frame[ry:ry+rh, rx:rx+rw], cv2.COLOR_BGR2HSV)
            warm_hit, cool_hit, no_ball = process(roi_hsv, cfg, roi_pixels)

            # Serial output: the dispatcher forwards this to the Teensy.
            print(f"VISION:FD:{int(warm_hit):02X}:{int(cool_hit):02X}", flush=True)

            if no_ball:  idle   += 1
            if warm_hit: hits_w += 1
            if cool_hit: hits_c += 1

            frames += 1
            now = time.time()
            if now - t_last >= 2.0:
                fps = frames / (now - t0)
                print(f"[SEP] {fps:5.1f} fps  W:{hits_w}  C:{hits_c}  idle:{idle}",
                      file=sys.stderr)
                t_last = now

    except KeyboardInterrupt:
        elapsed = time.time() - t0
        print(f"\n[Stopped] {frames} frames  {frames/max(elapsed,0.1):.1f} fps", file=sys.stderr)
    finally:
        cap.release()

if __name__ == "__main__":
    main()
