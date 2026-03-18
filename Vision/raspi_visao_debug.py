#!/usr/bin/env python3
"""
Bean Sorter — Pipeline Debug Viewer   (beans_debug.py)
=======================================================
Opens Three windows:
  "Controls [S=save Q=quit]"  — all trackbars
  "LEFT  (lower intake)"      — 2 candidate mask  3 shape filter  4 classified
  "RIGHT (upper intake)"      — same for right camera half

Config loaded from beans_config.json (generate it once with beans_debug.py → S key).
Serial 

Config  : beans_config.json (save from this script with S key)

Shape filter (3) checks in order:
  1. Zone    (cx within det_center ± det_thick)   — vertical green band
  2. Radius  (rad_min <= r <= rad_max)
  3. Area    (area_min <= area <= area_max)
  4. Circ    (circ_min <= circ <= circ_max)

Red trigger band: horizontal, perpendicular to green zone.
Ball must be in green zone AND overlap the red band to trigger serial hit.

Keys:  Q / ESC = quit     S = save config
"""

import cv2
import numpy as np
import sys
import json
import os

CONFIG_FILE = "beans_config.json"

def load_cfg_file() -> dict:
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, 'r') as f:
            print(f"[Config] Loaded {CONFIG_FILE}")
            return json.load(f)
    return {}

def save_cfg(cfg: dict):
    raw = dict(
        s_min=cfg['s_min'],        v_dark=cfg['v_dark'],
        bg_h_lo=cfg['bg_h_lo'],    bg_h_hi=cfg['bg_h_hi'],
        bg_s_min=cfg['bg_s_min'],  bg_v_min=cfg['bg_v_min'],
        area_min=cfg['area_min'] // 10,
        area_max=cfg['area_max'] // 10,
        rad_min=cfg['rad_min'],    rad_max=cfg['rad_max'],
        circ_min=int(cfg['circ_min'] * 100),
        circ_max=int(cfg['circ_max'] * 100),
        morph_k=cfg['morph_k'],    morph_iter=cfg['morph_iter'],
        det_cy_l=cfg['det_cy_l'],  det_cy_r=cfg['det_cy_r'],
        det_thick=cfg['det_thick'],
        trigger_x=cfg['trigger_x'],trig_y2=cfg['trig_y2'],
        gh_lo=cfg['gh_lo'],        gh_hi=cfg['gh_hi'],
        gs_min=cfg['gs_min'],      gv_min=cfg['gv_min'],
        gfrac=int(cfg['gfrac'] * 100),
    )
    with open(CONFIG_FILE, 'w') as f:
        json.dump(raw, f, indent=2)
    print(f"[Config] Saved to {CONFIG_FILE}")

try:
    import serial
    _SERIAL_AVAILABLE = True
except ImportError:
    _SERIAL_AVAILABLE = False

# ══════════════════════════════════════════════════════════════════════
#  CAMERA
# ══════════════════════════════════════════════════════════════════════
CAM_PORT = 0
FRAME_W  = 1344
FRAME_H  = 376

# ══════════════════════════════════════════════════════════════════════
#  SERIAL
# ══════════════════════════════════════════════════════════════════════
SERIAL_PORT    = '/dev/tty.usbmodem184063501'
SERIAL_BAUD    = 115200
SERIAL_ENABLED = True

_ser = None

def _open_serial():
    global _ser
    if not _SERIAL_AVAILABLE:
        print("[Serial] pyserial not installed — skipping")
        return
    try:
        _ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0)
        print(f"[Serial] opened {SERIAL_PORT} @ {SERIAL_BAUD}")
    except Exception as e:
        print(f"[Serial] {e} — running without serial")

def _send(left_hit: bool, right_hit: bool):
    global _ser
    if not SERIAL_ENABLED:
        return
    if _ser is None:
        _open_serial()
        return
    try:
        data = bytes([0xFF, int(left_hit), int(right_hit)])
        _ser.write(data)
    except Exception as e:
        print(f"[Serial] ERROR: {e} — reconnecting...")
        _ser.close()
        _ser = None

# ══════════════════════════════════════════════════════════════════════
#  DISPLAY
# ══════════════════════════════════════════════════════════════════════
PANEL_W    = 560
PANEL_H    = 220
FONT       = cv2.FONT_HERSHEY_SIMPLEX
VIEW_SCALE = 2.0

# ══════════════════════════════════════════════════════════════════════
#  KNN
# ══════════════════════════════════════════════════════════════════════
KNN_SAMPLES = np.array([
    [ 20, 200, 220], [ 25, 180, 210], [ 15, 220, 230],
    [ 10, 220, 210], [ 12, 200, 200], [  8, 210, 195],
    [  5, 200, 180], [  3, 210, 190], [  2, 190, 170],
    [175, 200, 180], [172, 195, 185], [170, 185, 175],
    [110, 150,  80], [100, 140,  70], [120, 160,  90], [105, 130,  75],
    [  0,  20,  30], [  0,  10,  20], [ 15,  15,  25], [  0,   5,  15],
], dtype=np.float32)

KNN_LABELS = np.array([
    0,0,0, 0,0,0, 0,0,0, 0,0,0,
    1,1,1,1, 1,1,1,1,
], dtype=np.int32)

def build_knn():
    k = cv2.ml.KNearest_create()
    k.train(KNN_SAMPLES, cv2.ml.ROW_SAMPLE, KNN_LABELS)
    return k

KNN = build_knn()

LABEL_COLOR = {
    "PICK":     (  0, 200,   0),
    "NOT PICK": (  0,   0, 220),
}

# ══════════════════════════════════════════════════════════════════════
#  RADIUS DRAG STATE
# ══════════════════════════════════════════════════════════════════════
_GCX   = PANEL_W - 90
_GCY   = PANEL_H - 45
_drag  = dict(active=None)
_scale = dict(x=1.0)

# ── Ghost / Kalman tracker state ──────────────────────────────────
GHOST_FRAMES = 7   # how many frames to show ghost after ball disappears

def _make_kalman():
    kf = cv2.KalmanFilter(4, 2)   # state: x,y,vx,vy  meas: x,y
    kf.measurementMatrix  = np.array([[1,0,0,0],[0,1,0,0]], np.float32)
    kf.transitionMatrix   = np.array([[1,0,1,0],[0,1,0,1],
                                       [0,0,1,0],[0,0,0,1]], np.float32)
    kf.processNoiseCov    = np.eye(4, dtype=np.float32) * 0.03
    kf.measurementNoiseCov= np.eye(2, dtype=np.float32) * 1.0
    kf.errorCovPost       = np.eye(4, dtype=np.float32)
    return kf

_ghost = {
    'left':  dict(kf=_make_kalman(), ttl=0, pos=(0,0), r=0),
    'right': dict(kf=_make_kalman(), ttl=0, pos=(0,0), r=0),
}

def _mouse_cb(event, x, y, flags, param):
    py = y - PANEL_H
    if py < 0 or py >= PANEL_H:
        if event == cv2.EVENT_LBUTTONUP:
            _drag['active'] = None
        return

    cfg     = get_cfg()
    sx      = _scale['x']
    rmin_dp = max(4, int(cfg['rad_min'] * sx))
    rmax_dp = max(4, int(cfg['rad_max'] * sx))
    dx = x - _GCX;  dy = py - _GCY
    dist = (dx*dx + dy*dy) ** 0.5
    GRAB = 10

    if event == cv2.EVENT_LBUTTONDOWN:
        if abs(dist - rmin_dp) < GRAB:
            _drag['active'] = 'min'
        elif abs(dist - rmax_dp) < GRAB:
            _drag['active'] = 'max'
    elif event == cv2.EVENT_MOUSEMOVE and _drag['active']:
        new_r = max(1, min(200, int(dist / sx)))
        key   = 'rad_min' if _drag['active'] == 'min' else 'rad_max'
        cv2.setTrackbarPos(TB[key], CTRL_WIN, new_r)
    elif event == cv2.EVENT_LBUTTONUP:
        _drag['active'] = None


def draw_radius_guide(panel: np.ndarray, cfg: dict) -> np.ndarray:
    sx      = _scale['x']
    rmin_dp = max(4, int(cfg['rad_min'] * sx))
    rmax_dp = max(4, int(cfg['rad_max'] * sx))
    cx, cy  = _GCX, _GCY

    cv2.circle(panel, (cx, cy), rmax_dp, (  0, 220, 220), 2)
    cv2.circle(panel, (cx, cy), rmin_dp, (220,   0, 220), 2)

    for r, col in ((rmin_dp, (220, 0, 220)), (rmax_dp, (0, 220, 220))):
        hx = int(cx + r * 0.707);  hy = int(cy - r * 0.707)
        cv2.circle(panel, (hx, hy), 5, col, -1)
        cv2.circle(panel, (hx, hy), 5, (255, 255, 255), 1)

    lx = max(4, cx - rmax_dp - 2)
    cv2.putText(panel, f"min:{cfg['rad_min']}px",
                (lx, max(14, cy - rmax_dp - 14)), FONT, 0.30, (220,   0, 220), 1)
    cv2.putText(panel, f"max:{cfg['rad_max']}px",
                (lx, max(14, cy - rmax_dp -  4)), FONT, 0.30, (  0, 220, 220), 1)
    cv2.putText(panel, "drag edge",
                (lx, cy + rmax_dp + 12),          FONT, 0.26, (110, 110, 110), 1)
    return panel


# ══════════════════════════════════════════════════════════════════════
#  TRACKBAR NAMES
# ══════════════════════════════════════════════════════════════════════
CTRL_WIN = "Controls [S=save Q=quit]"

TB = dict(
    s_min      = "S1 sat_min",
    v_dark     = "S1 dark_max",
    bg_h_lo    = "BG H_lo",
    bg_h_hi    = "BG H_hi",
    bg_s_min   = "BG S_min",
    bg_v_min   = "BG V_min",
    area_min   = "S2 area_min x10",
    area_max   = "S2 area_max x10",
    rad_min    = "S2 rad_min px",
    rad_max    = "S2 rad_max px",
    circ_min   = "S2 circ_min x.01",
    circ_max   = "S2 circ_max x.01",
    morph_k    = "S2 morph_k",
    morph_iter = "S2 morph_iter",
    det_cy_l   = "S2 zone_center_L",
    det_cy_r   = "S2 zone_center_R",
    det_thick  = "S2 zone_thick px",
    trigger_x  = "S2 trig_y1 px",
    trig_y2    = "S2 trig_y2 px",
    gh_lo      = "S3 H_lo",
    gh_hi      = "S3 H_hi",
    gs_min     = "S3 S_min",
    gv_min     = "S3 V_min",
    gfrac      = "S3 frac %",
)


def make_trackbars(half_h: int, half_w: int):
    saved = load_cfg_file()
    def d(key, default): return saved.get(key, default)

    cv2.namedWindow(CTRL_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CTRL_WIN, 480, 1060)

    def mk(key, default, maxval):
        cv2.createTrackbar(TB[key], CTRL_WIN, min(d(key, default), maxval), maxval, lambda v: None)

    mk("s_min",      97,           255)
    mk("v_dark",     12,           255)
    mk("bg_h_lo",    61,           179)
    mk("bg_h_hi",   115,           179)
    mk("bg_s_min",    0,           255)
    mk("bg_v_min",    0,           255)
    mk("area_min",  435,          5000)
    mk("area_max", 3029,          8000)
    mk("rad_min",    39,           200)
    mk("rad_max",    79,           200)
    mk("circ_min",   54,           100)
    mk("circ_max",  100,           100)
    mk("morph_k",    17,            21)
    mk("morph_iter",  4,             5)
    mk("det_cy_l",  half_w // 2, half_w)
    mk("det_cy_r",  half_w // 2, half_w)
    mk("det_thick",  30,         half_w)
    mk("trigger_x", half_h // 3, half_h)
    mk("trig_y2",   half_h // 3 * 2, half_h)
    mk("gh_lo",     142,           179)
    mk("gh_hi",      98,           179)
    mk("gs_min",     24,           255)
    mk("gv_min",     20,           255)
    mk("gfrac",       8,           100)


def get_cfg() -> dict:
    def tb(key):
        return cv2.getTrackbarPos(TB[key], CTRL_WIN)
    return dict(
        s_min      = tb("s_min"),
        v_dark     = tb("v_dark"),
        bg_h_lo    = tb("bg_h_lo"),
        bg_h_hi    = tb("bg_h_hi"),
        bg_s_min   = tb("bg_s_min"),
        bg_v_min   = tb("bg_v_min"),
        area_min   = tb("area_min") * 10,
        area_max   = tb("area_max") * 10,
        rad_min    = tb("rad_min"),
        rad_max    = tb("rad_max"),
        circ_min   = tb("circ_min") * 0.01,
        circ_max   = tb("circ_max") * 0.01,
        morph_k    = max(1, tb("morph_k") | 1),
        morph_iter = max(1, tb("morph_iter")),
        det_cy_l   = tb("det_cy_l"),
        det_cy_r   = tb("det_cy_r"),
        det_thick  = max(1, tb("det_thick")),
        trigger_x  = tb("trigger_x"),
        trig_y2    = tb("trig_y2"),
        gh_lo      = tb("gh_lo"),
        gh_hi      = tb("gh_hi"),
        gs_min     = tb("gs_min"),
        gv_min     = tb("gv_min"),
        gfrac      = tb("gfrac") / 100.0,
    )


def print_cfg(cfg: dict):
    print("\n# ── paste into beans.py ─────────────────────────────────────")
    print(f"CAND_S_MIN       = {cfg['s_min']}")
    print(f"CAND_V_DARK      = {cfg['v_dark']}")
    print(f"BG_GREEN_LO      = np.array([{cfg['bg_h_lo']:3d}, {cfg['bg_s_min']:3d}, {cfg['bg_v_min']:3d}], dtype=np.uint8)")
    print(f"BG_GREEN_HI      = np.array([{cfg['bg_h_hi']:3d}, 255, 255], dtype=np.uint8)")
    print(f"AREA_MIN         = {cfg['area_min']}")
    print(f"AREA_MAX         = {cfg['area_max']}")
    print(f"RADIUS_MIN       = {cfg['rad_min']}")
    print(f"RADIUS_MAX       = {cfg['rad_max']}")
    print(f"CIRC_MIN         = {cfg['circ_min']:.3f}")
    print(f"CIRC_MAX         = {cfg['circ_max']:.3f}")
    print(f"MORPH_KERNEL     = {cfg['morph_k']}")
    print(f"MORPH_ITERATIONS = {cfg['morph_iter']}")
    print(f"DET_CENTER_L     = {cfg['det_cy_l']}")
    print(f"DET_CENTER_R     = {cfg['det_cy_r']}")
    print(f"DET_THICKNESS    = {cfg['det_thick']}")
    print(f"TRIG_Y1          = {cfg['trigger_x']}")
    print(f"TRIG_Y2          = {cfg['trig_y2']}")
    print(f"GREEN_LO         = np.array([{cfg['gh_lo']:3d}, {cfg['gs_min']:3d}, {cfg['gv_min']:3d}], dtype=np.uint8)")
    print(f"GREEN_HI         = np.array([{cfg['gh_hi']:3d}, 255, 255], dtype=np.uint8)")
    print(f"GREEN_FRAC_MIN   = {cfg['gfrac']:.2f}")
    print("# ─────────────────────────────────────────────────────────────\n")


# ══════════════════════════════════════════════════════════════════════
#  MASKS
# ══════════════════════════════════════════════════════════════════════
def build_masks(half: np.ndarray, cfg: dict) -> dict:
    hsv        = cv2.cvtColor(half, cv2.COLOR_BGR2HSV)
    coloured   = (hsv[:, :, 1] >= cfg['s_min'])
    dark       = (hsv[:, :, 2] <  cfg['v_dark'])
    raw_cand   = (coloured | dark).astype(np.uint8) * 255

    bg_lo      = np.array([cfg['bg_h_lo'], cfg['bg_s_min'], cfg['bg_v_min']], dtype=np.uint8)
    bg_hi      = np.array([cfg['bg_h_hi'], 255,             255],             dtype=np.uint8)
    bg_green   = cv2.inRange(hsv, bg_lo, bg_hi)
    cand_clean = cv2.bitwise_and(raw_cand, cv2.bitwise_not(bg_green))

    k          = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                           (cfg['morph_k'], cfg['morph_k']))
    cand_morph = cv2.morphologyEx(cand_clean, cv2.MORPH_OPEN,  k, iterations=cfg['morph_iter'])
    cand_morph = cv2.morphologyEx(cand_morph, cv2.MORPH_CLOSE, k, iterations=cfg['morph_iter'])

    bean_lo    = np.array([cfg['gh_lo'], cfg['gs_min'], cfg['gv_min']], dtype=np.uint8)
    bean_hi    = np.array([cfg['gh_hi'], 255,           255],           dtype=np.uint8)
    bean_green = cv2.inRange(hsv, bean_lo, bean_hi)

    return dict(hsv=hsv, coloured=coloured, dark=dark,
                bg_green=bg_green, cand_morph=cand_morph, bean_green=bean_green)


# ══════════════════════════════════════════════════════════════════════
#  PANEL 2  CANDIDATE MASK
# ══════════════════════════════════════════════════════════════════════
def panel_candidate(half: np.ndarray, masks: dict) -> np.ndarray:
    coloured = masks['coloured']
    dark     = masks['dark']
    bg       = masks['bg_green'] > 0

    vis = np.zeros((*half.shape[:2], 3), dtype=np.uint8)
    vis[ coloured & ~dark  & ~bg] = ( 53, 130, 255)
    vis[~coloured &  dark  & ~bg] = (200,  80,  30)
    vis[ coloured &  dark  & ~bg] = (255, 255, 255)
    vis[bg]                       = (  0,   0, 200)
    edge = cv2.Canny(masks['cand_morph'], 50, 150)
    vis[edge > 0] = (220, 220, 0)

    out = cv2.resize(vis, (PANEL_W, PANEL_H))
    total_px = half.shape[0] * half.shape[1]
    bg_pct   = np.count_nonzero(bg)                  * 100 // total_px
    surv_pct = np.count_nonzero(masks['cand_morph']) * 100 // total_px
    cv2.putText(out, f"bg-cut:{bg_pct}%  surviving:{surv_pct}%",
                (6, PANEL_H-20), FONT, 0.32, (160, 160, 160), 1)

    legend = [((0,0,200),"bg(cut)"), ((53,130,255),"colour"),
              ((200,80,30),"dark"), ((255,255,255),"both"), ((220,220,0),"edge")]
    lx = 4
    for col, txt in legend:
        cv2.rectangle(out, (lx, PANEL_H-12), (lx+8, PANEL_H-4), col, -1)
        cv2.putText(out, txt, (lx+10, PANEL_H-4), FONT, 0.27, (160,160,160), 1)
        lx += int(cv2.getTextSize(txt, FONT, 0.27, 1)[0][0]) + 16
    return out


# ══════════════════════════════════════════════════════════════════════
#  HELPERS — draw zone + trigger band onto any panel
# ══════════════════════════════════════════════════════════════════════
def _draw_zone_and_trigger(vis: np.ndarray, cfg: dict, side: str,
                           sx: float, hit: bool = False):
    det_cy = cfg['det_cy_l'] if side == 'left' else cfg['det_cy_r']
    det_th = cfg['det_thick']

    zone_left  = int((det_cy - det_th) * sx)
    zone_right = int((det_cy + det_th) * sx)
    sy         = PANEL_H / FRAME_H

    # Green zone fill (vertical band)
    overlay = vis.copy()
    cv2.rectangle(overlay, (zone_left, 0), (zone_right, PANEL_H), (0, 60, 0), -1)
    cv2.addWeighted(overlay, 0.35, vis, 0.65, 0, vis)
    cv2.line(vis, (zone_left,  0), (zone_left,  PANEL_H), (0, 220, 0), 2)
    cv2.line(vis, (zone_right, 0), (zone_right, PANEL_H), (0, 220, 0), 2)
    cv2.putText(vis, f"zone {det_cy}+/-{det_th}px",
                (max(4, zone_left + 2), 12), FONT, 0.28, (0, 200, 0), 1)

    # Red trigger band (horizontal)
    y1_d = int(cfg['trigger_x'] * sy)
    y2_d = int(cfg['trig_y2']   * sy)
    y_lo = min(y1_d, y2_d)
    y_hi = max(y1_d, y2_d)

    trig_col   = (0, 0, 255) if hit else (60, 60, 220)
    trig_fill  = (0, 0, 120) if hit else (0, 0, 60)

    trig_overlay = vis.copy()
    cv2.rectangle(trig_overlay, (0, y_lo), (PANEL_W, y_hi), trig_fill, -1)
    cv2.addWeighted(trig_overlay, 0.4, vis, 0.6, 0, vis)
    cv2.line(vis, (0, y_lo), (PANEL_W, y_lo), trig_col, 2)
    cv2.line(vis, (0, y_hi), (PANEL_W, y_hi), trig_col, 2)
    cv2.putText(vis, "TRIG", (4, max(10, y_lo - 4)), FONT, 0.28, trig_col, 1)


# ══════════════════════════════════════════════════════════════════════
#  TRIGGER HIT CHECK
# ══════════════════════════════════════════════════════════════════════
def check_trigger(contour_data: list, cfg: dict, side: str = 'left') -> bool:
    """True if any passed blob OR active ghost overlaps the horizontal trigger band."""
    y_lo = min(cfg['trigger_x'], cfg['trig_y2'])
    y_hi = max(cfg['trigger_x'], cfg['trig_y2'])

    # Real ball check
    for cx, cy, ri, cnt, area, passed, circ in contour_data:
        if passed and (cy + ri >= y_lo) and (cy - ri <= y_hi):
            return True

    # Ghost ball check — if tracker is still alive, use predicted position
    g = _ghost[side]
    if g['ttl'] > 0:
        gcy = g['pos'][1]
        gri = g['r']
        if (gcy + gri >= y_lo) and (gcy - gri <= y_hi):
            return True

    return False

def update_ghost(side: str, contour_data: list):
    """Feed best passed blob into Kalman; decrement ttl if no blob."""
    g = _ghost[side]
    best = None
    for cx, cy, ri, cnt, area, passed, circ in contour_data:
        if passed:
            best = (cx, cy, ri)
            break   # take first passing blob

    if best:
        cx, cy, ri = best
        meas = np.array([[np.float32(cx)], [np.float32(cy)]])
        g['kf'].correct(meas)
        g['ttl'] = GHOST_FRAMES
        g['r']   = ri
    else:
        g['ttl'] = max(0, g['ttl'] - 1)

    pred = g['kf'].predict()
    g['pos'] = (int(pred[0]), int(pred[1]))

# ══════════════════════════════════════════════════════════════════════
#  PANEL 3  SHAPE + RADIUS + ZONE FILTER
# ══════════════════════════════════════════════════════════════════════
def panel_shape(half: np.ndarray, masks: dict, cfg: dict, side: str):
    det_cy = cfg['det_cy_l'] if side == 'left' else cfg['det_cy_r']
    det_th = cfg['det_thick']

    contours, _ = cv2.findContours(masks['cand_morph'], cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    sx = PANEL_W / half.shape[1]
    sy = PANEL_H / half.shape[0]

    vis = (cv2.resize(half, (PANEL_W, PANEL_H)) * 0.4).astype(np.uint8)
    _draw_zone_and_trigger(vis, cfg, side, sx)

    contour_data = []
    for cnt in contours:
        area  = cv2.contourArea(cnt)
        perim = cv2.arcLength(cnt, True)
        if perim == 0:
            continue
        circ       = 4 * np.pi * area / (perim ** 2)
        (x, y), r  = cv2.minEnclosingCircle(cnt)
        cx, cy, ri = int(x), int(y), int(r)

        pass_zone = abs(cx - det_cy) <= det_th
        pass_rad  = cfg['rad_min']  <= ri   <= cfg['rad_max']
        pass_area = cfg['area_min'] <= area <= cfg['area_max']
        pass_circ = cfg['circ_min'] <= circ <= cfg['circ_max']
        passed    = pass_zone and pass_rad and pass_area and pass_circ

        dcx = int(cx * sx);  dcy = int(cy * sy)
        dr  = max(1, int(ri * (sx + sy) / 2))

        if passed:
            col = (0, 220, 255)
            cv2.circle(vis, (dcx, dcy), dr, col, 2)
            cv2.circle(vis, (dcx, dcy), 3, (255, 255, 255), -1)
            cv2.putText(vis, f"r:{ri}  c:{circ:.2f}",
                        (max(0, dcx-dr), max(14, dcy-dr-12)), FONT, 0.30, col, 1)
            cv2.putText(vis, f"a:{int(area)}",
                        (max(0, dcx-dr), max(14, dcy-dr-3)),  FONT, 0.30, col, 1)
        else:
            col = (40, 40, 180)
            for ang in range(0, 360, 18):
                a0 = np.radians(ang);  a1 = np.radians(ang + 9)
                p0 = (int(dcx + dr*np.cos(a0)), int(dcy + dr*np.sin(a0)))
                p1 = (int(dcx + dr*np.cos(a1)), int(dcy + dr*np.sin(a1)))
                cv2.line(vis, p0, p1, col, 1)
            if not pass_zone:
                reason = f"zone({cx})x"
            elif not pass_rad:
                reason = f"r:{ri}({'lo' if ri < cfg['rad_min'] else 'hi'})x"
            elif not pass_area:
                reason = f"a:{int(area)}({'lo' if area < cfg['area_min'] else 'hi'})x"
            else:
                reason = f"c:{circ:.2f}({'lo' if circ < cfg['circ_min'] else 'hi'})x"
            cv2.putText(vis, reason,
                        (max(0, dcx-dr), max(14, dcy-dr-3)), FONT, 0.28, col, 1)

        contour_data.append((cx, cy, ri, cnt, area, passed, circ))

    n_pass = sum(1 for d in contour_data if d[5])
    cv2.putText(vis, f"blobs:{len(contour_data)}  pass:{n_pass}",
                (6, PANEL_H-8), FONT, 0.34, (160, 160, 160), 1)
    draw_radius_guide(vis, cfg)
    return vis, contour_data


# ══════════════════════════════════════════════════════════════════════
#  PANEL 4  CLASSIFIED
# ══════════════════════════════════════════════════════════════════════
def panel_result(half: np.ndarray, masks: dict,
                 contour_data: list, cfg: dict, side: str,
                 hit: bool = False) -> np.ndarray:
    vis = cv2.resize(half, (PANEL_W, PANEL_H)).copy()
    sx  = PANEL_W / half.shape[1]
    sy  = PANEL_H / half.shape[0]

    _draw_zone_and_trigger(vis, cfg, side, sx, hit=hit)

    hsv        = masks['hsv']
    bean_green = masks['bean_green']

    for cx, cy, r, cnt, area, passed, circ in contour_data:
        if not passed:
            continue

        cnt_mask = np.zeros(half.shape[:2], dtype=np.uint8)
        cv2.drawContours(cnt_mask, [cnt], -1, 255, -1)
        total = np.count_nonzero(cnt_mask)
        green = np.count_nonzero(cv2.bitwise_and(bean_green, bean_green, mask=cnt_mask))
        gfrac = green / total if total > 0 else 0.0

        if gfrac >= cfg['gfrac']:
            label = "NOT PICK"
        else:
            pixels   = hsv[cnt_mask > 0]
            mean_hsv = np.mean(pixels, axis=0).reshape(1, 3).astype(np.float32)
            KNN.findNearest(mean_hsv, k=3)
            label = "PICK"

        col = LABEL_COLOR[label]
        dcx = int(cx * sx);  dcy = int(cy * sy)
        dr  = max(1, int(r * (sx + sy) / 2))

        cv2.circle(vis, (dcx, dcy), dr + 4, col, 3 if label == "PICK" else 2)
        cv2.circle(vis, (dcx, dcy), 4, (255, 255, 255), -1)
        mean_px = np.mean(hsv[cnt_mask > 0], axis=0).astype(int) if total > 0 else [0,0,0]
        cv2.putText(vis, label,
                    (max(0, dcx-dr), max(14, dcy-dr-14)), FONT, 0.5, col, 2)
        cv2.putText(vis, f"r:{r}  H{mean_px[0]} S{mean_px[1]} V{mean_px[2]}",
                    (max(0, dcx-dr), max(14, dcy-dr-4)), FONT, 0.28, (170,170,170), 1)
        cv2.putText(vis, f"g:{gfrac:.0%}",
                    (max(0, dcx-dr), max(14, dcy-dr+5)), FONT, 0.28, (170,170,170), 1)
        if label == "PICK":
            cv2.putText(vis, "v PICK", (dcx-16, dcy+dr+14), FONT, 0.36, col, 2)

    if hit:
        cv2.putText(vis, "HIT", (PANEL_W - 50, 20), FONT, 0.6, (0, 0, 255), 2)

    # Draw ghost if ball was lost but tracker still alive
    g   = _ghost[side]
    ttl = g['ttl']
    if ttl > 0:
        gx = int(g['pos'][0] * sx)
        gy = int(g['pos'][1] * sy)
        gr = max(1, int(g['r'] * (sx + sy) / 2))
        alpha = ttl / GHOST_FRAMES          # fades out
        ghost_col = (0, int(220 * alpha), int(220 * alpha))   # yellow→dim
        cv2.circle(vis, (gx, gy), gr + 6, ghost_col, 2)
        cv2.putText(vis, f"ghost {ttl}f", (max(0, gx-gr), max(14, gy-gr-4)),
                    FONT, 0.28, ghost_col, 1)
        
    return vis


# ══════════════════════════════════════════════════════════════════════
#  PANEL TITLE STAMP
# ══════════════════════════════════════════════════════════════════════
TITLE_BG = {
    "2 CANDIDATE MASK": (( 60, 35,  0), (255, 140,  53)),
    "3 SHAPE FILTER":   (( 50, 50,  0), (  0, 210, 255)),
    "4 CLASSIFIED":     ((  0, 50, 20), (  0, 210,   0)),
}

def stamp(panel: np.ndarray, title: str) -> np.ndarray:
    bg, fg = TITLE_BG.get(title, ((30,30,30),(200,200,200)))
    (tw, th), _ = cv2.getTextSize(title, FONT, 0.42, 1)
    cv2.rectangle(panel, (0, 0), (tw+16, th+10), bg, -1)
    cv2.putText(panel, title, (8, th+4), FONT, 0.42, fg, 1)
    return panel


# ══════════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════════
def main():
    cap = cv2.VideoCapture(CAM_PORT)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    if not cap.isOpened():
        sys.exit(f"[ERROR] Cannot open camera {CAM_PORT}")

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    half_w   = actual_w // 2
    print(f"[Camera] {actual_w}x{actual_h}  half={half_w}x{actual_h}")
    print("[Keys]  Q/ESC=quit   S=save config")

    if SERIAL_ENABLED:
        _open_serial()

    make_trackbars(actual_h, half_w)

    LEFT_WIN  = "LEFT  (lower intake)"
    RIGHT_WIN = "RIGHT (upper intake)"
    #VIEW_WIN  = "CLASSIFIED VIEW"

    #for win in (LEFT_WIN, RIGHT_WIN):
    #    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    #    cv2.resizeWindow(win, PANEL_W, PANEL_H * 3)
    #cv2.namedWindow(VIEW_WIN, cv2.WINDOW_NORMAL)

    cv2.moveWindow(LEFT_WIN,  0,                30)
    cv2.moveWindow(RIGHT_WIN, PANEL_W + 6,      30)
    cv2.moveWindow(CTRL_WIN,  PANEL_W * 2 + 12, 30)
    #cv2.moveWindow(VIEW_WIN,  0,                PANEL_H * 3 + 60)

    _scale['x'] = PANEL_W / half_w

    cv2.setMouseCallback(LEFT_WIN,  _mouse_cb)
    cv2.setMouseCallback(RIGHT_WIN, _mouse_cb)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        cfg = get_cfg()

        halves = [
            (LEFT_WIN,  'left',  frame[:, :half_w]),
            (RIGHT_WIN, 'right', frame[:, half_w:]),
        ]

        cdata_store = {}
        p4_panels   = {}

        for win, side, half in halves:
            masks          = build_masks(half, cfg)
            p2             = stamp(panel_candidate(half, masks), "2 CANDIDATE MASK")
            p3_img, cdata  = panel_shape(half, masks, cfg, side)
            p3             = stamp(p3_img,                       "3 SHAPE FILTER")
            cdata_store[win] = cdata
            update_ghost(side, cdata)
            hit            = check_trigger(cdata, cfg, side)
            p4             = panel_result(half, masks, cdata, cfg, side, hit=hit)
            p4_panels[win] = p4.copy()
            stacked  = np.vstack([p2, p3, stamp(p4, "4 CLASSIFIED")])
            rotated  = cv2.rotate(stacked, cv2.ROTATE_90_COUNTERCLOCKWISE)
            cv2.imshow(win, rotated)
            cv2.resizeWindow(win, PANEL_H * 3, PANEL_W)  # swap W and H since rotated

        left_hit  = check_trigger(cdata_store[LEFT_WIN],  cfg, 'left')
        right_hit = check_trigger(cdata_store[RIGHT_WIN], cfg, 'right')
        
        _send(left_hit, right_hit)

        if _ser and _ser.in_waiting:
            print("[Teensy]", _ser.readline().decode(errors="ignore").strip())

        if left_hit or right_hit:
            print(f"[Trigger] L={int(left_hit)}  R={int(right_hit)}", end='\r')

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        if key == ord('s'):
            save_cfg(cfg)
            print_cfg(cfg)

    cap.release()
    cv2.destroyAllWindows()
    if _ser:
        _ser.close()

if __name__ == "__main__":
    main()