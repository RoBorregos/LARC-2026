#!/usr/bin/env python3
"""
raspi_visao.py — Headless bean sorter (dispatcher mode)
=========================================================
Camera  : /dev/video{CAM_PORT}  (V4L2)
Config  : raspi_config.json
Serial  : NONE — prints VISION:FF:L:R lines for dispatcher to forward
Output  : FPS + hit counts to terminal, VISION tags to stdout
Ctrl+C  : stop
"""

import cv2
import numpy as np
import sys, json, os, time

# ══════════════════════════════════════════════════════════════════════
#  CONSTANTS
# ══════════════════════════════════════════════════════════════════════
CONFIG_FILE = "raspi_config.json"
CAM_PORT    = 0
FRAME_W     = 1344
FRAME_H     = 376

# ══════════════════════════════════════════════════════════════════════
#  CONFIG
# ══════════════════════════════════════════════════════════════════════
def load_cfg() -> dict:
    if not os.path.exists(CONFIG_FILE):
        sys.exit(f"[ERROR] {CONFIG_FILE} not found — run debug script first and press S.")
    with open(CONFIG_FILE) as f:
        raw = json.load(f)
    return dict(
        s_min      = raw['s_min'],
        v_dark     = raw['v_dark'],
        bg_h_lo    = raw['bg_h_lo'],
        bg_h_hi    = raw['bg_h_hi'],
        bg_s_min   = raw['bg_s_min'],
        bg_v_min   = raw['bg_v_min'],
        area_min   = raw['area_min'] * 10,
        area_max   = raw['area_max'] * 10,
        rad_min    = raw['rad_min'],
        rad_max    = raw['rad_max'],
        circ_min   = raw['circ_min'] * 0.01,
        circ_max   = raw['circ_max'] * 0.01,
        morph_k    = max(1, raw['morph_k'] | 1),
        morph_iter = max(1, raw['morph_iter']),
        det_cy_l   = raw['det_cy_l'],
        det_cy_r   = raw['det_cy_r'],
        det_thick  = max(1, raw['det_thick']),
        trigger_x  = raw['trigger_x'],
        trig_y2    = raw['trig_y2'],
        gh_lo      = raw['gh_lo'],
        gh_hi      = raw['gh_hi'],
        gs_min     = raw['gs_min'],
        gv_min     = raw['gv_min'],
        gfrac      = raw['gfrac'] / 100.0,
    )

# ══════════════════════════════════════════════════════════════════════
#  KNN CLASSIFIER
# ══════════════════════════════════════════════════════════════════════
KNN_SAMPLES = np.array([
    [ 20, 200, 220], [ 25, 180, 210], [ 15, 220, 230],
    [ 10, 220, 210], [ 12, 200, 200], [  8, 210, 195],
    [  5, 200, 180], [  3, 210, 190], [  2, 190, 170],
    [175, 200, 180], [172, 195, 185], [170, 185, 175],
    [110, 150,  80], [100, 140,  70], [120, 160,  90], [105, 130,  75],
    [  0,  20,  30], [  0,  10,  20], [ 15,  15,  25], [  0,   5,  15],
], dtype=np.float32)
KNN_LABELS = np.array([0]*12 + [1]*8, dtype=np.int32)
_knn = cv2.ml.KNearest_create()
_knn.train(KNN_SAMPLES, cv2.ml.ROW_SAMPLE, KNN_LABELS)

# ══════════════════════════════════════════════════════════════════════
#  KALMAN GHOST TRACKER
# ══════════════════════════════════════════════════════════════════════
GHOST_FRAMES = 10

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
    best = next(((cx,cy,ri) for cx,cy,ri,*_,passed,_ in contour_data if passed), None)
    if best:
        cx, cy, ri = best
        g['kf'].correct(np.array([[np.float32(cx)],[np.float32(cy)]]))
        g['ttl'] = GHOST_FRAMES
        g['r']   = ri
    else:
        g['ttl'] = max(0, g['ttl'] - 1)
    pred     = g['kf'].predict()
    g['pos'] = (int(pred[0]), int(pred[1]))

def check_trigger(contour_data, cfg, side):
    y_lo = min(cfg['trigger_x'], cfg['trig_y2'])
    y_hi = max(cfg['trigger_x'], cfg['trig_y2'])
    for cx, cy, ri, cnt, area, passed, circ in contour_data:
        if passed and (cy+ri >= y_lo) and (cy-ri <= y_hi):
            return True
    g = _ghost[side]
    if g['ttl'] > 0:
        gcy, gri = g['pos'][1], g['r']
        if (gcy+gri >= y_lo) and (gcy-gri <= y_hi):
            return True
    return False

# ══════════════════════════════════════════════════════════════════════
#  VISION PIPELINE
# ══════════════════════════════════════════════════════════════════════
def build_masks(half, cfg):
    hsv      = cv2.cvtColor(half, cv2.COLOR_BGR2HSV)
    raw_cand = ((hsv[:,:,1] >= cfg['s_min']) | (hsv[:,:,2] < cfg['v_dark'])).astype(np.uint8) * 255
    bg_lo    = np.array([cfg['bg_h_lo'], cfg['bg_s_min'], cfg['bg_v_min']], dtype=np.uint8)
    bg_hi    = np.array([cfg['bg_h_hi'], 255,             255],             dtype=np.uint8)
    clean    = cv2.bitwise_and(raw_cand, cv2.bitwise_not(cv2.inRange(hsv, bg_lo, bg_hi)))
    k        = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cfg['morph_k'], cfg['morph_k']))
    morph    = cv2.morphologyEx(clean, cv2.MORPH_OPEN,  k, iterations=cfg['morph_iter'])
    morph    = cv2.morphologyEx(morph, cv2.MORPH_CLOSE, k, iterations=cfg['morph_iter'])
    bean_lo  = np.array([cfg['gh_lo'], cfg['gs_min'], cfg['gv_min']], dtype=np.uint8)
    bean_hi  = np.array([cfg['gh_hi'], 255,           255],           dtype=np.uint8)
    return dict(hsv=hsv, cand_morph=morph, bean_green=cv2.inRange(hsv, bean_lo, bean_hi))

def detect(half, masks, cfg, side):
    det_cy = cfg['det_cy_l'] if side == 'left' else cfg['det_cy_r']
    det_th = cfg['det_thick']
    cnts, _ = cv2.findContours(masks['cand_morph'], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for cnt in cnts:
        area  = cv2.contourArea(cnt)
        perim = cv2.arcLength(cnt, True)
        if perim == 0: continue
        circ      = 4 * np.pi * area / (perim ** 2)
        (x,y), r  = cv2.minEnclosingCircle(cnt)
        cx,cy,ri  = int(x), int(y), int(r)
        passed    = (abs(cx - det_cy) <= det_th and
                     cfg['rad_min']  <= ri   <= cfg['rad_max'] and
                     cfg['area_min'] <= area <= cfg['area_max'] and
                     cfg['circ_min'] <= circ <= cfg['circ_max'])
        out.append((cx, cy, ri, cnt, area, passed, circ))
    return out

# ══════════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════════
def main():
    cfg = load_cfg()
    print(f"[Config] loaded {CONFIG_FILE}")

    cap = cv2.VideoCapture(CAM_PORT, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    if not cap.isOpened():
        sys.exit(f"[ERROR] Cannot open /dev/video{CAM_PORT}")

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    half_w   = actual_w // 2
    print(f"[Camera] /dev/video{CAM_PORT}  {actual_w}x{actual_h}  half={half_w}x{actual_h}")
    print("[Mode] Dispatcher (VISION tags via stdout)")
    print("[Running] Ctrl+C to stop")

    frame_count = 0
    hit_counts  = {'left': 0, 'right': 0}
    t0          = time.time()
    t_last      = t0

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                continue

            halves = [('left', frame[:, :half_w]), ('right', frame[:, half_w:])]
            hits   = {}
            for side, half in halves:
                masks       = build_masks(half, cfg)
                cdata       = detect(half, masks, cfg, side)
                update_ghost(side, cdata)
                hits[side]  = check_trigger(cdata, cfg, side)
                if hits[side]:
                    hit_counts[side] += 1

            # Output vision data for dispatcher to forward
            # Format: VISION:FF:LL:RR (hex bytes)
            print(f"VISION:FF:{int(hits['left']):02X}:{int(hits['right']):02X}", flush=True)

            frame_count += 1
            now = time.time()
            if now - t_last >= 1.0:
                fps = frame_count / (now - t0)
                l_str = "L" if hits['left'] else " "
                r_str = "R" if hits['right'] else " "
                print(f"[FPS] {fps:5.1f}  |  {l_str} {r_str}  |  L hits: {hit_counts['left']:4d}  R hits: {hit_counts['right']:4d}")
                t_last = now

    except KeyboardInterrupt:
        elapsed = time.time() - t0
        print(f"\n[Stopped]  {frame_count} frames in {elapsed:.1f}s  avg {frame_count/elapsed:.1f} fps")
    finally:
        cap.release()

if __name__ == "__main__":
    main()