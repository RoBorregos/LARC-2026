#!/usr/bin/env python3
"""
orin_vision.py — Headless bean sorter (vision to serial)
Purpose of this code : Run the bean-detection vision loop on the Orin at full
                       camera resolution and emit what it sees as VISION tags on
                       stdout. The dispatcher forwards those bytes to the Teensy,
                       which owns all servo / actuation logic. This file only
                       looks and reports — it drives no hardware.
Camera  : /dev/video_zed  (V4L2, ZED 1344x376, processed at full resolution)
Config  : orin_config.json
Output  : VISION:XX bitfield lines on stdout  (bit 0 = right bean, bit 1 = left bean)
Ctrl+C  : stop

Deploy vs debug
    orin_vision.py       to this file: headless, emits VISION tags, no display.
    orin_vision_debug.py to preview version: opens windows to tune the config on a PC.
    Tune with the debug tool, then copy orin_config.json onto the Orin.
"""

import cv2
import numpy as np
import sys, json, os, time

# - Constants
CONFIG_FILE = "orin_config.json"
FRAME_W     = 1344
FRAME_H     = 376

# - Config
def load_cfg() -> dict:
    if not os.path.exists(CONFIG_FILE):
        sys.exit(f"[ERROR] {CONFIG_FILE} not found — run the debug tool first and press S.")
    with open(CONFIG_FILE) as f:
        raw = json.load(f)

    # Full resolution: config values are used directly (no downscale).
    # area keeps the legacy x10 tuning multiplier from the original config.
    return dict(
        s_min      = raw['s_min'],
        v_dark     = raw['v_dark'],
        bg_h_lo    = raw['bg_h_lo'],
        bg_h_hi    = raw['bg_h_hi'],
        bg_s_min   = raw['bg_s_min'],
        bg_v_min   = raw['bg_v_min'],
        area_min   = raw['area_min'] * 10,
        area_max   = raw['area_max'] * 10,
        rad_min    = max(1, raw['rad_min']),
        rad_max    = max(1, raw['rad_max']),
        circ_min   = raw['circ_min'] * 0.01,
        circ_max   = raw['circ_max'] * 0.01,
        morph_k    = max(1, raw['morph_k'] | 1),
        morph_iter = max(1, raw['morph_iter']),
        det_cy_l   = raw['det_cy_l'],
        det_cy_r   = raw['det_cy_r'],
        det_thick  = max(1, raw['det_thick']),
        trigger_x  = raw['trigger_x'],
        trig_y2    = raw['trig_y2'],
    )

# - Kalman ghost tracker
GHOST_FRAMES = 0

def _make_kalman():
    kf = cv2.KalmanFilter(4, 2)
    kf.measurementMatrix   = np.array([[1,0,0,0],[0,1,0,0]], np.float32)
    kf.transitionMatrix    = np.array([[1,0,1,0],[0,1,0,1],
                                       [0,0,1,0],[0,0,0,1]], np.float32)
    kf.processNoiseCov     = np.eye(4, dtype=np.float32) * 0.03
    kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1.0
    kf.errorCovPost        = np.eye(4, dtype=np.float32)
    return kf

_ghost = {
    'left':  dict(kf=_make_kalman(), ttl=0, pos=(0,0), r=0),
    'right': dict(kf=_make_kalman(), ttl=0, pos=(0,0), r=0),
}

def update_ghost(side, contour_data):
    g    = _ghost[side]
    best = next(((cx,cy,ri) for cx,cy,ri,passed in contour_data if passed), None)
    if best:
        cx, cy, ri = best
        g['kf'].correct(np.array([[np.float32(cx)],[np.float32(cy)]]))
        g['ttl'] = GHOST_FRAMES
        g['r']   = ri
    else:
        g['ttl'] = max(0, g['ttl'] - 1)
    pred     = g['kf'].predict()
    g['pos'] = (int(pred[0].item()), int(pred[1].item()))

def check_trigger(contour_data, cfg, side):
    y_lo = min(cfg['trigger_x'], cfg['trig_y2'])
    y_hi = max(cfg['trigger_x'], cfg['trig_y2'])
    for cx, cy, ri, passed in contour_data:
        if passed and (cy + ri >= y_lo) and (cy - ri <= y_hi):
            return True
    g = _ghost[side]
    if g['ttl'] > 0:
        gcy, gri = g['pos'][1], g['r']
        if (gcy + gri >= y_lo) and (gcy - gri <= y_hi):
            return True
    return False

# - Vision pipeline
def build_mask(half, cfg):
    hsv      = cv2.cvtColor(half, cv2.COLOR_BGR2HSV)
    raw_cand = ((hsv[:,:,1] >= cfg['s_min']) | (hsv[:,:,2] < cfg['v_dark'])).astype(np.uint8) * 255
    bg_lo    = np.array([cfg['bg_h_lo'], cfg['bg_s_min'], cfg['bg_v_min']], dtype=np.uint8)
    bg_hi    = np.array([cfg['bg_h_hi'], 255,             255],             dtype=np.uint8)
    clean    = cv2.bitwise_and(raw_cand, cv2.bitwise_not(cv2.inRange(hsv, bg_lo, bg_hi)))
    k        = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cfg['morph_k'], cfg['morph_k']))
    morph    = cv2.morphologyEx(clean, cv2.MORPH_OPEN,  k, iterations=cfg['morph_iter'])
    morph    = cv2.morphologyEx(morph, cv2.MORPH_CLOSE, k, iterations=cfg['morph_iter'])
    return morph

def detect(mask, cfg, side):
    det_cy = cfg['det_cy_l'] if side == 'left' else cfg['det_cy_r']
    det_th = cfg['det_thick']
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for cnt in cnts:
        area  = cv2.contourArea(cnt)
        perim = cv2.arcLength(cnt, True)
        if perim == 0:
            continue
        circ      = 4 * np.pi * area / (perim ** 2)
        (x, y), r = cv2.minEnclosingCircle(cnt)
        cx, cy, ri = int(x), int(y), int(r)
        passed = (abs(cx - det_cy) <= det_th and
                  cfg['rad_min']  <= ri   <= cfg['rad_max'] and
                  cfg['area_min'] <= area <= cfg['area_max'] and
                  cfg['circ_min'] <= circ <= cfg['circ_max'])
        out.append((cx, cy, ri, passed))
    return out

# - Main
def main():
    cfg = load_cfg()
    print(f"[Config] loaded {CONFIG_FILE} (full resolution)", file=sys.stderr)

    cap = cv2.VideoCapture("/dev/video_zed", cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        sys.exit("[ERROR] Cannot open /dev/video_zed")

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    half_w   = actual_w // 2
    print(f"[Camera] /dev/video_zed  {actual_w}x{actual_h}  half={half_w}x{actual_h}", file=sys.stderr)
    print("[Running] Ctrl+C to stop", file=sys.stderr)

    frame_count = 0
    hit_counts  = {'left': 0, 'right': 0}
    t0          = time.time()
    t_last      = t0

    try:
        while True:
            t_grab = time.monotonic()
            ret, frame = cap.read()
            if not ret or frame is None:
                continue
            t_read = time.monotonic()

            # Process each half at full resolution
            halves = [
                ('left',  frame[:, :half_w]),
                ('right', frame[:, half_w:]),
            ]

            hits = {}
            for side, half in halves:
                mask   = build_mask(half, cfg)
                cdata  = detect(mask, cfg, side)
                update_ghost(side, cdata)
                hits[side] = check_trigger(cdata, cfg, side)
                if hits[side]:
                    hit_counts[side] += 1

            t_proc = time.monotonic()

            # Serial output: single-byte bitfield the dispatcher forwards to the Teensy.
            bitfield = (int(hits['right']) & 1) | ((int(hits['left']) & 1) << 1)
            print(f"VISION:{bitfield:02X}", flush=True)

            # FPS logging on stderr so it never mixes with VISION output
            frame_count += 1
            now = time.time()
            if now - t_last >= 1.0:
                fps = frame_count / (now - t0)
                l_str = "L" if hits['left'] else " "
                r_str = "R" if hits['right'] else " "
                read_ms = (t_read - t_grab) * 1000
                proc_ms = (t_proc - t_read) * 1000
                total_ms = (t_proc - t_grab) * 1000
                print(f"[FPS] {fps:5.1f}  |  {l_str} {r_str}  |  "
                      f"L:{hit_counts['left']:4d}  R:{hit_counts['right']:4d}  |  "
                      f"read:{read_ms:.0f} proc:{proc_ms:.0f} total:{total_ms:.0f}ms",
                      file=sys.stderr)
                t_last = now

    except KeyboardInterrupt:
        elapsed = time.time() - t0
        print(f"\n[Stopped]  {frame_count} frames in {elapsed:.1f}s  "
              f"avg {frame_count/elapsed:.1f} fps", file=sys.stderr)
    finally:
        cap.release()

if __name__ == "__main__":
    main()
