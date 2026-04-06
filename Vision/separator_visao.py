#!/usr/bin/env python3
"""
separator_visao.py — Headless ball sorter (dispatcher mode)
============================================================
Camera  : /dev/video3  (RealSense RGB via V4L2)
Config  : separator_config.json
Serial  : NONE — prints VISION:FD:WW:CC lines for dispatcher to forward
Output  : FPS + warm/cool hits to terminal, VISION tags to stdout
Ctrl+C  : stop
"""

import cv2
import numpy as np
import sys, json, os, time

# ══════════════════════════════════════════════════════════════════════
#  CONSTANTS
# ══════════════════════════════════════════════════════════════════════
CONFIG_FILE = "separator_config.json"
CAM_PORT    = 9
FRAME_W     = 1280
FRAME_H     = 720

DEFAULT_CFG = dict(
    roi_x1=760, roi_y1=340, roi_x2=1160, roi_y2=740,
    warm_h_lo=0,   warm_h_hi=35,
    warm_s_min=80, warm_v_min=80,
    cool_h_lo=95,  cool_h_hi=135,
    cool_s_min=60, cool_v_min=40,
    black_v_max=50,
    warm_frac=5,   cool_frac=5,
    morph_k=5,     morph_iter=1,
    ghost_frames=10,
    area_min=800,  area_max=80000,
)

# ══════════════════════════════════════════════════════════════════════
#  CONFIG
# ══════════════════════════════════════════════════════════════════════
def load_cfg() -> dict:
    if not os.path.exists(CONFIG_FILE):
        print(f"[Config] {CONFIG_FILE} not found — using defaults")
        return dict(DEFAULT_CFG)
    with open(CONFIG_FILE) as f:
        saved = json.load(f)
    cfg = {**DEFAULT_CFG, **saved}
    print(f"[Config] loaded {CONFIG_FILE}")
    print(f"[Config] ROI  x:{cfg['roi_x1']}-{cfg['roi_x2']}  y:{cfg['roi_y1']}-{cfg['roi_y2']}")
    return cfg

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
    'warm': dict(kf=_make_kalman(), ttl=0, pos=(0,0), r=0),
    'cool': dict(kf=_make_kalman(), ttl=0, pos=(0,0), r=0),
}

def update_ghosts(detections, ghost_frames):
    seen = {k: None for k in _ghosts}
    for cx, cy, r, group in detections:
        if seen[group] is None or r > seen[group][2]:
            seen[group] = (cx, cy, r)
    for group, g in _ghosts.items():
        if seen[group]:
            cx, cy, r = seen[group]
            g['kf'].correct(np.array([[np.float32(cx)],[np.float32(cy)]]))
            g['ttl'] = ghost_frames
            g['r']   = r
        else:
            g['ttl'] = max(0, g['ttl'] - 1)
        pred     = g['kf'].predict()
        g['pos'] = (int(pred[0]), int(pred[1]))

# ══════════════════════════════════════════════════════════════════════
#  VISION PIPELINE
# ══════════════════════════════════════════════════════════════════════
def process_roi(roi_bgr, cfg):
    hsv        = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    roi_pixels = roi_bgr.shape[0] * roi_bgr.shape[1]

    warm_main = cv2.inRange(hsv,
        np.array([cfg['warm_h_lo'], cfg['warm_s_min'], cfg['warm_v_min']], np.uint8),
        np.array([cfg['warm_h_hi'], 255,               255],               np.uint8))
    warm_mask = warm_main
    if cfg['warm_h_lo'] < 10:
        warm_mask = cv2.bitwise_or(warm_main, cv2.inRange(hsv,
            np.array([160, cfg['warm_s_min'], cfg['warm_v_min']], np.uint8),
            np.array([180, 255,               255],               np.uint8)))

    blue_mask = cv2.inRange(hsv,
        np.array([cfg['cool_h_lo'], cfg['cool_s_min'], cfg['cool_v_min']], np.uint8),
        np.array([cfg['cool_h_hi'], 255,               255],               np.uint8))
    blk_mask  = (hsv[:,:,2] < cfg['black_v_max']).astype(np.uint8) * 255
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
                (cx,cy), r = cv2.minEnclosingCircle(cnt)
                detections.append((int(cx), int(cy), int(r), group))

    return detections, warm_hit, cool_hit

# ══════════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════════
def main():
    cfg = load_cfg()

    cap = cv2.VideoCapture(CAM_PORT, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    if not cap.isOpened():
        sys.exit(f"[ERROR] Cannot open /dev/video{CAM_PORT}")

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[Camera] /dev/video{CAM_PORT}  {actual_w}x{actual_h}")
    print("[Mode] Dispatcher (VISION tags via stdout)")
    print("[Running] Ctrl+C to stop")

    frame_count = 0
    hit_counts  = {'warm': 0, 'cool': 0}
    t0          = time.time()
    t_last      = t0

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                continue

            x1 = max(0, min(cfg['roi_x1'], actual_w - 2))
            y1 = max(0, min(cfg['roi_y1'], actual_h - 2))
            x2 = max(x1+2, min(cfg['roi_x2'], actual_w))
            y2 = max(y1+2, min(cfg['roi_y2'], actual_h))

            detections, pixel_warm, pixel_cool = process_roi(frame[y1:y2, x1:x2], cfg)
            update_ghosts(detections, cfg['ghost_frames'])

            warm_hit = pixel_warm or (_ghosts['warm']['ttl'] > 0)
            cool_hit = pixel_cool or (_ghosts['cool']['ttl'] > 0)

            if warm_hit: hit_counts['warm'] += 1
            if cool_hit: hit_counts['cool'] += 1

            # Output vision data for dispatcher to forward
            # Format: VISION:FD:WW:CC (hex bytes)
            # 0xFD = separator sync byte, WW = warm, CC = cool
            print(f"VISION:FD:{int(warm_hit):02X}:{int(cool_hit):02X}", flush=True)

            frame_count += 1
            now = time.time()
            if now - t_last >= 1.0:
                fps = frame_count / (now - t0)
                w_str = "WARM" if warm_hit else "    "
                c_str = "COOL" if cool_hit else "    "
                print(f"[FPS] {fps:5.1f}  |  {w_str}  {c_str}  |  warm: {hit_counts['warm']:4d}  cool: {hit_counts['cool']:4d}")
                t_last = now

    except KeyboardInterrupt:
        elapsed = time.time() - t0
        print(f"\n[Stopped]  {frame_count} frames in {elapsed:.1f}s  avg {frame_count/elapsed:.1f} fps")
    finally:
        cap.release()

if __name__ == "__main__":
    main()