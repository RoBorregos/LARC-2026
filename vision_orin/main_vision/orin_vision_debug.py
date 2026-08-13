#!/usr/bin/env python3
"""
orin_vision_debug.py — Bean sorter preview / calibration tool
Purpose of this code : Same vision pipeline as orin_vision.py, but instead of
                       emitting VISION tags it opens preview windows so you can
                       SEE what the Orin would detect. Use it to calibrate.
Camera  : /dev/video_zed  (V4L2, ZED 1344x376, processed at full resolution)
Config  : orin_config.json  (shared with orin_vision.py)
Serial  : NONE
Output  : two on-screen windows (FILTERS + RESULT) + FPS on stdout
Keys    : Q / ESC = quit

How this differs from orin_vision.py (READ THIS)
    This is the PC / debug build. It is meant to run on a computer (or on the
    Orin over X11 forwarding) because it opens a preview window showing exactly
    what the Orin would be detecting. You use it to calibrate: watch the FILTERS
    and RESULT windows, adjust orin_config.json until the beans are picked
    cleanly, then copy orin_config.json onto the Orin and run the headless
    orin_vision.py there. This file drives no hardware and sends no serial — it
    is purely for tuning and verification.

Windows
    FILTERS : candidate mask + background-green + morph result
    RESULT  : detection zone, trigger band, picked circles, hit state
"""

import cv2
import numpy as np
import sys, json, os, time

# - Constants (must match orin_vision.py)
CONFIG_FILE = "orin_config.json"
CAM_DEV     = "/dev/video_zed"
FRAME_W     = 1344
FRAME_H     = 376

# Display sizing — kept small so it stays smooth over X11 forwarding
PANEL_W = 420
PANEL_H = 235
FONT    = cv2.FONT_HERSHEY_SIMPLEX

# - Config loader (identical to orin_vision.py)
def load_cfg() -> dict:
    if not os.path.exists(CONFIG_FILE):
        sys.exit(f"[ERROR] {CONFIG_FILE} not found — tune on a PC and press S first.")
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

# - Vision pipeline (mirrors orin_vision.py, kept local so this file runs standalone)
def build_masks(half, cfg):
    """Return every intermediate mask so the preview can show each stage."""
    hsv      = cv2.cvtColor(half, cv2.COLOR_BGR2HSV)
    coloured = (hsv[:,:,1] >= cfg['s_min'])
    dark     = (hsv[:,:,2] <  cfg['v_dark'])
    raw_cand = (coloured | dark).astype(np.uint8) * 255
    bg_lo    = np.array([cfg['bg_h_lo'], cfg['bg_s_min'], cfg['bg_v_min']], np.uint8)
    bg_hi    = np.array([cfg['bg_h_hi'], 255, 255], np.uint8)
    bg_green = cv2.inRange(hsv, bg_lo, bg_hi)
    clean    = cv2.bitwise_and(raw_cand, cv2.bitwise_not(bg_green))
    k        = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cfg['morph_k'], cfg['morph_k']))
    morph    = cv2.morphologyEx(clean, cv2.MORPH_OPEN,  k, iterations=cfg['morph_iter'])
    morph    = cv2.morphologyEx(morph, cv2.MORPH_CLOSE, k, iterations=cfg['morph_iter'])
    return dict(coloured=coloured, dark=dark, bg_green=bg_green, morph=morph)

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
        pz = abs(cx - det_cy) <= det_th
        pr = cfg['rad_min']  <= ri   <= cfg['rad_max']
        pa = cfg['area_min'] <= area <= cfg['area_max']
        pc = cfg['circ_min'] <= circ <= cfg['circ_max']
        passed = pz and pr and pa and pc
        out.append((cx, cy, ri, area, circ, passed, pz, pr, pa, pc))
    return out

def check_trigger(cdata, cfg):
    y_lo = min(cfg['trigger_x'], cfg['trig_y2'])
    y_hi = max(cfg['trigger_x'], cfg['trig_y2'])
    for entry in cdata:
        cx, cy, ri = entry[0], entry[1], entry[2]
        passed = entry[5]
        if passed and (cy + ri >= y_lo) and (cy - ri <= y_hi):
            return True
    return False

# - Visualization
def panel_filters(half, masks):
    """Combined mask view: orange=saturated, blue=dark, white=both, red=bg-green, yellow=final edge."""
    vis = np.zeros((*half.shape[:2], 3), np.uint8)
    coloured = masks['coloured']; dark = masks['dark']; bg = masks['bg_green'] > 0
    vis[coloured & ~dark & ~bg] = (53, 130, 255)     # saturated to orange
    vis[~coloured & dark  & ~bg] = (200, 80, 30)     # dark      to blue
    vis[coloured & dark   & ~bg] = (255, 255, 255)   # both      to white
    vis[bg]                     = (0, 0, 200)        # bg green  to red
    edge = cv2.Canny(masks['morph'], 50, 150)
    vis[edge > 0] = (0, 255, 255)                    # final edge to yellow
    out = cv2.resize(vis, (PANEL_W, PANEL_H))
    tp = half.shape[0] * half.shape[1]
    cv2.putText(out, f"surv:{np.count_nonzero(masks['morph']) * 100 // tp}%",
                (6, PANEL_H - 8), FONT, 0.36, (180, 180, 180), 1)
    return cv2.rotate(out, cv2.ROTATE_90_COUNTERCLOCKWISE)

def panel_result(half, cdata, cfg, side, hit):
    """Live frame with detection zone, trigger band, picked circles and hit flag."""
    vis = cv2.resize(half, (PANEL_W, PANEL_H)).copy()
    sx = PANEL_W / half.shape[1]
    sy = PANEL_H / half.shape[0]

    # Detection zone (vertical band)
    det_cy = cfg['det_cy_l'] if side == 'left' else cfg['det_cy_r']
    det_th = cfg['det_thick']
    zl = int((det_cy - det_th) * sx)
    zr = int((det_cy + det_th) * sx)
    overlay = vis.copy()
    cv2.rectangle(overlay, (zl, 0), (zr, PANEL_H), (0, 60, 0), -1)
    cv2.addWeighted(overlay, 0.35, vis, 0.65, 0, vis)
    cv2.line(vis, (zl, 0), (zl, PANEL_H), (0, 220, 0), 1)
    cv2.line(vis, (zr, 0), (zr, PANEL_H), (0, 220, 0), 1)

    # Trigger band (horizontal)
    y1 = int(cfg['trigger_x'] * sy)
    y2 = int(cfg['trig_y2']   * sy)
    ylo, yhi = min(y1, y2), max(y1, y2)
    tc = (0, 0, 255) if hit else (60, 60, 220)
    tf = (0, 0, 120) if hit else (0, 0, 60)
    to = vis.copy()
    cv2.rectangle(to, (0, ylo), (PANEL_W, yhi), tf, -1)
    cv2.addWeighted(to, 0.35, vis, 0.65, 0, vis)
    cv2.line(vis, (0, ylo), (PANEL_W, ylo), tc, 1)
    cv2.line(vis, (0, yhi), (PANEL_W, yhi), tc, 1)

    # Contours
    for cx, cy, ri, area, circ, passed, pz, pr, pa, pc in cdata:
        dcx = int(cx * sx); dcy = int(cy * sy)
        dr  = max(1, int(ri * (sx + sy) / 2))
        if passed:
            cv2.circle(vis, (dcx, dcy), dr + 2, (0, 220, 0), 2)
            cv2.circle(vis, (dcx, dcy), 3, (255, 255, 255), -1)
            cv2.putText(vis, f"r:{ri} c:{circ:.2f}",
                        (max(0, dcx - dr), max(12, dcy - dr - 4)),
                        FONT, 0.32, (0, 220, 0), 1)
        else:
            # Show which check rejected this contour
            if   not pz: reason = "zone"
            elif not pr: reason = f"r:{ri}"
            elif not pa: reason = f"a:{int(area)}"
            else:        reason = f"c:{circ:.2f}"
            cv2.circle(vis, (dcx, dcy), dr, (60, 60, 180), 1)
            cv2.putText(vis, reason,
                        (max(0, dcx - dr), max(12, dcy - dr - 4)),
                        FONT, 0.30, (80, 80, 200), 1)

    # HUD
    cv2.putText(vis, side.upper(), (6, 16), FONT, 0.5, (220, 220, 220), 1)
    cv2.putText(vis, f"zone {det_cy}+/-{det_th}", (6, 32),
                FONT, 0.32, (0, 200, 0), 1)
    if hit:
        cv2.putText(vis, "HIT", (PANEL_W - 48, 20),
                    FONT, 0.6, (0, 0, 255), 2)
    return cv2.rotate(vis, cv2.ROTATE_90_COUNTERCLOCKWISE)

# - Main
def main():
    cfg = load_cfg()
    print(f"[Config] loaded {CONFIG_FILE} (full resolution)")

    cap = cv2.VideoCapture(CAM_DEV, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        sys.exit(f"[ERROR] Cannot open {CAM_DEV}")

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    half_w   = actual_w // 2
    print(f"[Camera] {actual_w}x{actual_h}  half={half_w}x{actual_h}")
    print("[Keys]   Q / ESC = quit")

    FILTERS_WIN = "FILTERS (L top, R bottom)"
    RESULT_WIN  = "RESULT  (L top, R bottom)"
    cv2.namedWindow(FILTERS_WIN, cv2.WINDOW_NORMAL)
    cv2.namedWindow(RESULT_WIN,  cv2.WINDOW_NORMAL)
    cv2.resizeWindow(FILTERS_WIN, PANEL_H * 2, PANEL_W)
    cv2.resizeWindow(RESULT_WIN,  PANEL_H * 2, PANEL_W)
    cv2.moveWindow(FILTERS_WIN, 0, 30)
    cv2.moveWindow(RESULT_WIN,  PANEL_W + 10, 30)

    frame_count = 0
    t0     = time.time()
    t_last = t0

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                continue

            # Process each half at full resolution
            halves = [
                ('left',  frame[:, :half_w]),
                ('right', frame[:, half_w:]),
            ]

            filter_panels = []
            result_panels = []
            for side, half in halves:
                masks = build_masks(half, cfg)
                cdata = detect(masks['morph'], cfg, side)
                hit   = check_trigger(cdata, cfg)
                filter_panels.append(panel_filters(half, masks))
                result_panels.append(panel_result(half, cdata, cfg, side, hit))

            cv2.imshow(FILTERS_WIN, np.hstack(filter_panels))
            cv2.imshow(RESULT_WIN,  np.hstack(result_panels))

            frame_count += 1
            now = time.time()
            if now - t_last >= 1.0:
                fps = frame_count / (now - t0)
                print(f"[FPS] {fps:5.1f}")
                t_last = now

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        elapsed = time.time() - t0
        print(f"[Stopped] {frame_count} frames in {elapsed:.1f}s "
              f"(avg {frame_count / max(elapsed, 0.001):.1f} fps)")

if __name__ == "__main__":
    main()
