#!/usr/bin/env python3
"""
Bean Sorter — Pipeline Debug Viewer   (beans_debug.py)
=======================================================
Camera reads 1344x376 (ZED stereo), splits into L/R halves,
then downscales each 2x to 336x188 — matching what raspi_visao.py
processes on the Pi.

Config saved in FULL-RES coordinates (672x376 space).
The Pi's raspi_visao.py auto-scales them down at load time.

Keys:  Q / ESC = quit     S = save config
"""

import cv2
import numpy as np
import sys, json, os

CONFIG_FILE = "beans_config.json"
DS = 2  # downscale factor — must match raspi_visao.py

def load_cfg_file() -> dict:
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, 'r') as f:
            print(f"[Config] Loaded {CONFIG_FILE}")
            return json.load(f)
    return {}

def save_cfg(cfg: dict):
    """Save in full-res coords (what trackbars show)."""
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
    )
    with open(CONFIG_FILE, 'w') as f:
        json.dump(raw, f, indent=2)
    print(f"[Config] Saved to {CONFIG_FILE}")

def scale_cfg(cfg: dict) -> dict:
    """Scale full-res spatial config values down by DS for processing."""
    s = DS
    sc = dict(cfg)
    sc['area_min']  = cfg['area_min'] // (s * s)
    sc['area_max']  = cfg['area_max'] // (s * s)
    sc['rad_min']   = max(1, cfg['rad_min'] // s)
    sc['rad_max']   = max(1, cfg['rad_max'] // s)
    sc['det_cy_l']  = cfg['det_cy_l'] // s
    sc['det_cy_r']  = cfg['det_cy_r'] // s
    sc['det_thick'] = max(1, cfg['det_thick'] // s)
    sc['trigger_x'] = cfg['trigger_x'] // s
    sc['trig_y2']   = cfg['trig_y2'] // s
    return sc

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
        print("[Serial] pyserial not installed — skipping"); return
    try:
        _ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0)
        print(f"[Serial] opened {SERIAL_PORT} @ {SERIAL_BAUD}")
    except Exception as e:
        print(f"[Serial] {e} — running without serial")

def _send(left_hit: bool, right_hit: bool):
    global _ser
    if not SERIAL_ENABLED: return
    if _ser is None: _open_serial(); return
    try:
        _ser.write(bytes([0xFF, int(left_hit), int(right_hit)]))
    except Exception as e:
        print(f"[Serial] ERROR: {e}"); _ser.close(); _ser = None

# ══════════════════════════════════════════════════════════════════════
#  DISPLAY
# ══════════════════════════════════════════════════════════════════════
PANEL_W    = 560
PANEL_H    = 220
FONT       = cv2.FONT_HERSHEY_SIMPLEX

# ══════════════════════════════════════════════════════════════════════
#  RADIUS DRAG STATE
# ══════════════════════════════════════════════════════════════════════
_GCX   = PANEL_W - 90
_GCY   = PANEL_H - 45
_drag  = dict(active=None)
_scale = dict(x=1.0)

GHOST_FRAMES = 4

def _make_kalman():
    kf = cv2.KalmanFilter(4, 2)
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
        if event == cv2.EVENT_LBUTTONUP: _drag['active'] = None
        return
    cfg     = get_cfg()
    sx      = _scale['x']
    rmin_dp = max(4, int(cfg['rad_min'] * sx))
    rmax_dp = max(4, int(cfg['rad_max'] * sx))
    dx = x - _GCX; dy = py - _GCY
    dist = (dx*dx + dy*dy) ** 0.5
    GRAB = 10
    if event == cv2.EVENT_LBUTTONDOWN:
        if abs(dist - rmin_dp) < GRAB: _drag['active'] = 'min'
        elif abs(dist - rmax_dp) < GRAB: _drag['active'] = 'max'
    elif event == cv2.EVENT_MOUSEMOVE and _drag['active']:
        new_r = max(1, min(200, int(dist / sx)))
        key = 'rad_min' if _drag['active'] == 'min' else 'rad_max'
        cv2.setTrackbarPos(TB[key], CTRL_WIN, new_r)
    elif event == cv2.EVENT_LBUTTONUP:
        _drag['active'] = None

def draw_radius_guide(panel, cfg):
    sx = _scale['x']
    rmin_dp = max(4, int(cfg['rad_min'] * sx))
    rmax_dp = max(4, int(cfg['rad_max'] * sx))
    cx, cy = _GCX, _GCY
    cv2.circle(panel, (cx,cy), rmax_dp, (0,220,220), 2)
    cv2.circle(panel, (cx,cy), rmin_dp, (220,0,220), 2)
    for r, col in ((rmin_dp,(220,0,220)), (rmax_dp,(0,220,220))):
        hx=int(cx+r*0.707); hy=int(cy-r*0.707)
        cv2.circle(panel,(hx,hy),5,col,-1); cv2.circle(panel,(hx,hy),5,(255,255,255),1)
    lx = max(4, cx - rmax_dp - 2)
    cv2.putText(panel,f"min:{cfg['rad_min']}px",(lx,max(14,cy-rmax_dp-14)),FONT,0.30,(220,0,220),1)
    cv2.putText(panel,f"max:{cfg['rad_max']}px",(lx,max(14,cy-rmax_dp-4)),FONT,0.30,(0,220,220),1)
    cv2.putText(panel,"drag edge",(lx,cy+rmax_dp+12),FONT,0.26,(110,110,110),1)
    return panel

# ══════════════════════════════════════════════════════════════════════
#  TRACKBAR NAMES  (full-res coords)
# ══════════════════════════════════════════════════════════════════════
CTRL_WIN = "Controls [S=save Q=quit]"
TB = dict(
    s_min="S1 sat_min", v_dark="S1 dark_max",
    bg_h_lo="BG H_lo", bg_h_hi="BG H_hi", bg_s_min="BG S_min", bg_v_min="BG V_min",
    area_min="S2 area_min x10", area_max="S2 area_max x10",
    rad_min="S2 rad_min px", rad_max="S2 rad_max px",
    circ_min="S2 circ_min x.01", circ_max="S2 circ_max x.01",
    morph_k="S2 morph_k", morph_iter="S2 morph_iter",
    det_cy_l="S2 zone_center_L", det_cy_r="S2 zone_center_R",
    det_thick="S2 zone_thick px",
    trigger_x="S2 trig_y1 px", trig_y2="S2 trig_y2 px",
)

def make_trackbars(half_h, half_w):
    """Trackbars in FULL-RES coordinates (672x376 half space)."""
    saved = load_cfg_file()
    def d(key, default): return saved.get(key, default)
    cv2.namedWindow(CTRL_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CTRL_WIN, 480, 860)
    def mk(key, default, maxval):
        cv2.createTrackbar(TB[key], CTRL_WIN, min(d(key, default), maxval), maxval, lambda v: None)
    mk("s_min",97,255); mk("v_dark",12,255)
    mk("bg_h_lo",61,179); mk("bg_h_hi",115,179); mk("bg_s_min",0,255); mk("bg_v_min",0,255)
    mk("area_min",435,5000); mk("area_max",3029,8000)
    mk("rad_min",39,200); mk("rad_max",79,200)
    mk("circ_min",54,100); mk("circ_max",100,100)
    mk("morph_k",17,21); mk("morph_iter",4,5)
    mk("det_cy_l",half_w//2,half_w); mk("det_cy_r",half_w//2,half_w)
    mk("det_thick",30,half_w)
    mk("trigger_x",half_h//3,half_h); mk("trig_y2",half_h//3*2,half_h)

def get_cfg() -> dict:
    """Read trackbars — returns FULL-RES cfg."""
    def tb(key): return cv2.getTrackbarPos(TB[key], CTRL_WIN)
    return dict(
        s_min=tb("s_min"), v_dark=tb("v_dark"),
        bg_h_lo=tb("bg_h_lo"), bg_h_hi=tb("bg_h_hi"),
        bg_s_min=tb("bg_s_min"), bg_v_min=tb("bg_v_min"),
        area_min=tb("area_min")*10, area_max=tb("area_max")*10,
        rad_min=tb("rad_min"), rad_max=tb("rad_max"),
        circ_min=tb("circ_min")*0.01, circ_max=tb("circ_max")*0.01,
        morph_k=max(1,tb("morph_k")|1), morph_iter=max(1,tb("morph_iter")),
        det_cy_l=tb("det_cy_l"), det_cy_r=tb("det_cy_r"),
        det_thick=max(1,tb("det_thick")),
        trigger_x=tb("trigger_x"), trig_y2=tb("trig_y2"),
    )

def print_cfg(cfg):
    print(f"\n# ── full-res values (672x376 half) ──")
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
    print(f"# ── DS={DS}x on Pi: coords / {DS}, area / {DS*DS} ──\n")

# ══════════════════════════════════════════════════════════════════════
#  MASKS  (operates on downscaled half with scaled cfg)
# ══════════════════════════════════════════════════════════════════════
def build_masks(half, cfg):
    hsv      = cv2.cvtColor(half, cv2.COLOR_BGR2HSV)
    coloured = (hsv[:,:,1] >= cfg['s_min'])
    dark     = (hsv[:,:,2] <  cfg['v_dark'])
    raw_cand = (coloured | dark).astype(np.uint8) * 255
    bg_lo    = np.array([cfg['bg_h_lo'], cfg['bg_s_min'], cfg['bg_v_min']], np.uint8)
    bg_hi    = np.array([cfg['bg_h_hi'], 255, 255], np.uint8)
    bg_green = cv2.inRange(hsv, bg_lo, bg_hi)
    cand_clean = cv2.bitwise_and(raw_cand, cv2.bitwise_not(bg_green))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cfg['morph_k'], cfg['morph_k']))
    cand_morph = cv2.morphologyEx(cand_clean, cv2.MORPH_OPEN,  k, iterations=cfg['morph_iter'])
    cand_morph = cv2.morphologyEx(cand_morph, cv2.MORPH_CLOSE, k, iterations=cfg['morph_iter'])
    return dict(hsv=hsv, coloured=coloured, dark=dark, bg_green=bg_green, cand_morph=cand_morph)

# ══════════════════════════════════════════════════════════════════════
#  PANELS
# ══════════════════════════════════════════════════════════════════════
def panel_candidate(half, masks):
    coloured=masks['coloured']; dark=masks['dark']; bg=masks['bg_green']>0
    vis = np.zeros((*half.shape[:2],3), np.uint8)
    vis[coloured & ~dark & ~bg]=(53,130,255)
    vis[~coloured & dark & ~bg]=(200,80,30)
    vis[coloured & dark & ~bg]=(255,255,255)
    vis[bg]=(0,0,200)
    edge=cv2.Canny(masks['cand_morph'],50,150); vis[edge>0]=(220,220,0)
    out=cv2.resize(vis,(PANEL_W,PANEL_H))
    tp=half.shape[0]*half.shape[1]
    cv2.putText(out,f"bg-cut:{np.count_nonzero(bg)*100//tp}%  surv:{np.count_nonzero(masks['cand_morph'])*100//tp}%",
                (6,PANEL_H-20),FONT,0.32,(160,160,160),1)
    return out

def _draw_zone_and_trigger(vis, cfg, side, sx, hit=False):
    """cfg here is SCALED (downscaled coords)."""
    det_cy = cfg['det_cy_l'] if side=='left' else cfg['det_cy_r']
    det_th = cfg['det_thick']
    zl=int((det_cy-det_th)*sx); zr=int((det_cy+det_th)*sx)
    sy=PANEL_H/( (FRAME_H//DS) )
    overlay=vis.copy()
    cv2.rectangle(overlay,(zl,0),(zr,PANEL_H),(0,60,0),-1)
    cv2.addWeighted(overlay,0.35,vis,0.65,0,vis)
    cv2.line(vis,(zl,0),(zl,PANEL_H),(0,220,0),2)
    cv2.line(vis,(zr,0),(zr,PANEL_H),(0,220,0),2)
    cv2.putText(vis,f"zone {det_cy}+/-{det_th}",(max(4,zl+2),12),FONT,0.28,(0,200,0),1)
    y1d=int(cfg['trigger_x']*sy); y2d=int(cfg['trig_y2']*sy)
    ylo=min(y1d,y2d); yhi=max(y1d,y2d)
    tc=(0,0,255) if hit else (60,60,220); tf=(0,0,120) if hit else (0,0,60)
    to=vis.copy(); cv2.rectangle(to,(0,ylo),(PANEL_W,yhi),tf,-1)
    cv2.addWeighted(to,0.4,vis,0.6,0,vis)
    cv2.line(vis,(0,ylo),(PANEL_W,ylo),tc,2); cv2.line(vis,(0,yhi),(PANEL_W,yhi),tc,2)
    cv2.putText(vis,"TRIG",(4,max(10,ylo-4)),FONT,0.28,tc,1)

def check_trigger(contour_data, cfg, side='left'):
    ylo=min(cfg['trigger_x'],cfg['trig_y2']); yhi=max(cfg['trigger_x'],cfg['trig_y2'])
    for cx,cy,ri,cnt,area,passed,circ in contour_data:
        if passed and (cy+ri>=ylo) and (cy-ri<=yhi): return True
    g=_ghost[side]
    if g['ttl']>0:
        gcy=g['pos'][1]; gri=g['r']
        if (gcy+gri>=ylo) and (gcy-gri<=yhi): return True
    return False

def update_ghost(side, contour_data):
    g=_ghost[side]; best=None
    for cx,cy,ri,cnt,area,passed,circ in contour_data:
        if passed: best=(cx,cy,ri); break
    if best:
        cx,cy,ri=best
        g['kf'].correct(np.array([[np.float32(cx)],[np.float32(cy)]]))
        g['ttl']=GHOST_FRAMES; g['r']=ri
    else: g['ttl']=max(0,g['ttl']-1)
    pred=g['kf'].predict(); g['pos']=(int(pred[0]),int(pred[1]))

def panel_shape(half, masks, cfg, side):
    """cfg here is SCALED."""
    det_cy = cfg['det_cy_l'] if side=='left' else cfg['det_cy_r']
    det_th = cfg['det_thick']
    contours,_=cv2.findContours(masks['cand_morph'],cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    sx=PANEL_W/half.shape[1]; sy=PANEL_H/half.shape[0]
    vis=(cv2.resize(half,(PANEL_W,PANEL_H))*0.4).astype(np.uint8)
    _draw_zone_and_trigger(vis,cfg,side,sx)
    contour_data=[]
    for cnt in contours:
        area=cv2.contourArea(cnt); perim=cv2.arcLength(cnt,True)
        if perim==0: continue
        circ=4*np.pi*area/(perim**2)
        (x,y),r=cv2.minEnclosingCircle(cnt)
        cx,cy,ri=int(x),int(y),int(r)
        pz=abs(cx-det_cy)<=det_th
        pr=cfg['rad_min']<=ri<=cfg['rad_max']
        pa=cfg['area_min']<=area<=cfg['area_max']
        pc=cfg['circ_min']<=circ<=cfg['circ_max']
        passed=pz and pr and pa and pc
        dcx=int(cx*sx); dcy=int(cy*sy); dr=max(1,int(ri*(sx+sy)/2))
        if passed:
            col=(0,220,255)
            cv2.circle(vis,(dcx,dcy),dr,col,2); cv2.circle(vis,(dcx,dcy),3,(255,255,255),-1)
            cv2.putText(vis,f"r:{ri} c:{circ:.2f}",(max(0,dcx-dr),max(14,dcy-dr-12)),FONT,0.30,col,1)
            cv2.putText(vis,f"a:{int(area)}",(max(0,dcx-dr),max(14,dcy-dr-3)),FONT,0.30,col,1)
        else:
            col=(40,40,180)
            for ang in range(0,360,18):
                a0=np.radians(ang); a1=np.radians(ang+9)
                p0=(int(dcx+dr*np.cos(a0)),int(dcy+dr*np.sin(a0)))
                p1=(int(dcx+dr*np.cos(a1)),int(dcy+dr*np.sin(a1)))
                cv2.line(vis,p0,p1,col,1)
            if not pz: reason=f"zone({cx})x"
            elif not pr: reason=f"r:{ri}({'lo' if ri<cfg['rad_min'] else 'hi'})x"
            elif not pa: reason=f"a:{int(area)}({'lo' if area<cfg['area_min'] else 'hi'})x"
            else: reason=f"c:{circ:.2f}({'lo' if circ<cfg['circ_min'] else 'hi'})x"
            cv2.putText(vis,reason,(max(0,dcx-dr),max(14,dcy-dr-3)),FONT,0.28,col,1)
        contour_data.append((cx,cy,ri,cnt,area,passed,circ))
    n_pass=sum(1 for d in contour_data if d[5])
    cv2.putText(vis,f"blobs:{len(contour_data)} pass:{n_pass}",(6,PANEL_H-8),FONT,0.34,(160,160,160),1)
    draw_radius_guide(vis,cfg)
    return vis, contour_data

def panel_result(half, masks, contour_data, cfg, side, hit=False):
    vis=cv2.resize(half,(PANEL_W,PANEL_H)).copy()
    sx=PANEL_W/half.shape[1]; sy=PANEL_H/half.shape[0]
    _draw_zone_and_trigger(vis,cfg,side,sx,hit=hit)
    hsv=masks['hsv']
    for cx,cy,r,cnt,area,passed,circ in contour_data:
        if not passed: continue
        col=(0,200,0); dcx=int(cx*sx); dcy=int(cy*sy); dr=max(1,int(r*(sx+sy)/2))
        cv2.circle(vis,(dcx,dcy),dr+4,col,3); cv2.circle(vis,(dcx,dcy),4,(255,255,255),-1)
        cnt_mask=np.zeros(half.shape[:2],np.uint8); cv2.drawContours(cnt_mask,[cnt],-1,255,-1)
        total=np.count_nonzero(cnt_mask)
        mean_px=np.mean(hsv[cnt_mask>0],axis=0).astype(int) if total>0 else [0,0,0]
        cv2.putText(vis,"PICK",(max(0,dcx-dr),max(14,dcy-dr-14)),FONT,0.5,col,2)
        cv2.putText(vis,f"r:{r} H{mean_px[0]} S{mean_px[1]} V{mean_px[2]}",(max(0,dcx-dr),max(14,dcy-dr-4)),FONT,0.28,(170,170,170),1)
    if hit: cv2.putText(vis,"HIT",(PANEL_W-50,20),FONT,0.6,(0,0,255),2)
    g=_ghost[side]
    if g['ttl']>0:
        gx=int(g['pos'][0]*sx); gy=int(g['pos'][1]*sy); gr=max(1,int(g['r']*(sx+sy)/2))
        alpha=g['ttl']/GHOST_FRAMES; gc=(0,int(220*alpha),int(220*alpha))
        cv2.circle(vis,(gx,gy),gr+6,gc,2)
        cv2.putText(vis,f"ghost {g['ttl']}f",(max(0,gx-gr),max(14,gy-gr-4)),FONT,0.28,gc,1)
    return vis

TITLE_BG={"2 CANDIDATE MASK":((60,35,0),(255,140,53)),"3 SHAPE FILTER":((50,50,0),(0,210,255)),"4 RESULT":((0,50,20),(0,210,0))}
def stamp(panel, title):
    bg,fg=TITLE_BG.get(title,((30,30,30),(200,200,200)))
    (tw,th),_=cv2.getTextSize(title,FONT,0.42,1)
    cv2.rectangle(panel,(0,0),(tw+16,th+10),bg,-1)
    cv2.putText(panel,title,(8,th+4),FONT,0.42,fg,1)
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
    half_w   = actual_w // 2    # 672  (full-res half)
    ds_w     = half_w // DS     # 336
    ds_h     = actual_h // DS   # 188
    print(f"[Camera] {actual_w}x{actual_h}  half={half_w}x{actual_h}  ds={ds_w}x{ds_h} (DS={DS}x)")
    print("[Keys]  Q/ESC=quit   S=save config")

    if SERIAL_ENABLED: _open_serial()

    # trackbars use FULL-RES half coords (672x376)
    make_trackbars(actual_h, half_w)

    LEFT_WIN  = "LEFT  (lower intake)"
    RIGHT_WIN = "RIGHT (upper intake)"
    cv2.moveWindow(LEFT_WIN,0,30)
    cv2.moveWindow(RIGHT_WIN,PANEL_W+6,30)
    cv2.moveWindow(CTRL_WIN,PANEL_W*2+12,30)

    # display scale uses DOWNSCALED half width for radius guide
    _scale['x'] = PANEL_W / ds_w

    cv2.setMouseCallback(LEFT_WIN, _mouse_cb)
    cv2.setMouseCallback(RIGHT_WIN, _mouse_cb)

    while True:
        ret, frame = cap.read()
        if not ret: continue

        cfg_full = get_cfg()                   # full-res from trackbars
        cfg      = scale_cfg(cfg_full)         # downscaled for processing

        halves = [
            (LEFT_WIN,  'left',  cv2.resize(frame[:, :half_w], (ds_w, ds_h), interpolation=cv2.INTER_NEAREST)),
            (RIGHT_WIN, 'right', cv2.resize(frame[:, half_w:], (ds_w, ds_h), interpolation=cv2.INTER_NEAREST)),
        ]

        cdata_store = {}
        for win, side, half in halves:
            masks         = build_masks(half, cfg)
            p2            = stamp(panel_candidate(half, masks), "2 CANDIDATE MASK")
            p3_img, cdata = panel_shape(half, masks, cfg, side)
            p3            = stamp(p3_img, "3 SHAPE FILTER")
            cdata_store[win] = cdata
            update_ghost(side, cdata)
            hit           = check_trigger(cdata, cfg, side)
            p4            = panel_result(half, masks, cdata, cfg, side, hit=hit)
            stacked = np.vstack([p2, p3, stamp(p4, "4 RESULT")])
            rotated = cv2.rotate(stacked, cv2.ROTATE_90_COUNTERCLOCKWISE)
            cv2.imshow(win, rotated)
            cv2.resizeWindow(win, PANEL_H*3, PANEL_W)

        left_hit  = check_trigger(cdata_store[LEFT_WIN],  cfg, 'left')
        right_hit = check_trigger(cdata_store[RIGHT_WIN], cfg, 'right')
        _send(left_hit, right_hit)

        if _ser and _ser.in_waiting:
            print("[Teensy]", _ser.readline().decode(errors="ignore").strip())
        if left_hit or right_hit:
            print(f"[Trigger] L={int(left_hit)} R={int(right_hit)}", end='\r')

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27): break
        if key == ord('s'):
            save_cfg(cfg_full)   # save FULL-RES values
            print_cfg(cfg_full)

    cap.release(); cv2.destroyAllWindows()
    if _ser: _ser.close()

if __name__ == "__main__":
    main()