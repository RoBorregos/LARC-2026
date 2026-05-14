#!/usr/bin/env python3
"""
camera_test.py Test all connected cameras
=============================================
Scans /dev/video* devices, opens each one, and reports:
  - Device name (from v4l2)
  - Actual resolution
  - Measured FPS (over 60 frames)
  - Whether it can actually capture frames

Run: python3 camera_test.py
  or: python3 camera_test.py 0        (test only /dev/video0)
  or: python3 camera_test.py 0 2 4    (test specific ports)
"""

import cv2
import sys
import os
import time
import subprocess

def get_v4l2_name(port):
    """Get camera name from v4l2-ctl."""
    try:
        result = subprocess.run(
            ['v4l2-ctl', '--device', f'/dev/video{port}', '--info'],
            capture_output=True, text=True, timeout=3
        )
        for line in result.stdout.splitlines():
            if 'Card type' in line:
                return line.split(':', 1)[1].strip()
    except Exception:
        pass
    return "unknown"

def test_camera(port, num_frames=60):
    """Test a single camera port."""
    dev = f"/dev/video{port}"
    if not os.path.exists(dev):
        print(f"  {dev}: does not exist")
        return None

    name = get_v4l2_name(port)
    print(f"\n{'='*60}")
    print(f"  /dev/video{port}  â€  {name}")
    print(f"{'='*60}")

    cap = cv2.VideoCapture(port, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"  FAILED to open")
        return None

    # Try different resolutions
    test_resolutions = [
        (1344, 376),   # ZED stereo
        (1920, 1080),  # Full HD
        (1280, 720),   # HD
        (640, 480),    # VGA
    ]

    # First, check default resolution
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"  Default resolution: {w}x{h}")

    # Try to read a frame at default
    ret, frame = cap.read()
    if not ret or frame is None:
        print(f"  FAILED to read frame at default resolution")
        cap.release()
        return None

    print(f"  Frame shape: {frame.shape}")
    print(f"  Frame dtype: {frame.dtype}")

    # Measure FPS
    print(f"  Measuring FPS over {num_frames} frames...")
    t0 = time.time()
    good = 0
    for _ in range(num_frames):
        ret, frame = cap.read()
        if ret and frame is not None:
            good += 1
    elapsed = time.time() - t0
    fps = good / elapsed if elapsed > 0 else 0

    print(f"  Captured: {good}/{num_frames} frames in {elapsed:.1f}s")
    print(f"  Measured FPS: {fps:.1f}")

    # Check supported resolutions
    print(f"\n  Testing resolutions:")
    for tw, th in test_resolutions:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, tw)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, th)
        aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        ret, _ = cap.read()
        status = "OK" if ret else "FAIL"
        match = "  <-- match" if (aw == tw and ah == th) else ""
        print(f"    Requested {tw}x{th} â Got {aw}x{ah}  [{status}]{match}")

    cap.release()
    return {'port': port, 'name': name, 'fps': fps, 'resolution': f'{w}x{h}'}

def main():
    print("Camera Test â€ scanning connected cameras\n")

    # List all video devices
    try:
        result = subprocess.run(['v4l2-ctl', '--list-devices'],
                                capture_output=True, text=True, timeout=5)
        print("v4l2-ctl --list-devices:")
        print(result.stdout)
    except Exception as e:
        print(f"v4l2-ctl not available: {e}")

    # Determine which ports to test
    if len(sys.argv) > 1:
        ports = [int(p) for p in sys.argv[1:]]
    else:
        # Scan /dev/video0 through /dev/video9
        ports = [i for i in range(10) if os.path.exists(f'/dev/video{i}')]

    print(f"Testing ports: {ports}")

    results = []
    for port in ports:
        r = test_camera(port)
        if r:
            results.append(r)

    # Summary
    print(f"\n{'='*60}")
    print(f"  SUMMARY")
    print(f"{'='*60}")
    if not results:
        print("  No working cameras found!")
    else:
        for r in results:
            print(f"  /dev/video{r['port']:2d}  {r['resolution']:>12s}  {r['fps']:5.1f} fps  {r['name']}")

    # USB device info
    print(f"\n  USB devices (lsusb):")
    try:
        result = subprocess.run(['lsusb'], capture_output=True, text=True, timeout=5)
        for line in result.stdout.splitlines():
            # Filter for likely cameras
            lower = line.lower()
            if any(k in lower for k in ['camera', 'video', 'stereo', 'zed', 'webcam', 'logitech', 'usb2.0']):
                print(f"    {line}")
        # Show all if no cameras found
        if not any(k in result.stdout.lower() for k in ['camera', 'video', 'stereo', 'zed']):
            print(result.stdout)
    except Exception:
        pass

if __name__ == "__main__":
    main()
