#!/usr/bin/env python3
"""
Two-phase camera calibration using a ChArUco board.

Phase 1 (capture):  Auto-capture frames for --duration seconds, save to disk.
Phase 2 (calibrate): Read saved images, detect corners, compute calibration.

If calibration fails, images are preserved — just re-run with 'calibrate' subcommand.

Usage:
    # Full pipeline: capture 60s then calibrate
    python calibration/calibrate_camera.py capture --camera 0 --duration 60

    # Re-run calibration on previously saved images
    python calibration/calibrate_camera.py calibrate --input calibration/captures/20260315_153000

    # Custom output path
    python calibration/calibrate_camera.py capture --camera 0 -o my_camera.yaml
"""
from __future__ import annotations

import argparse
import datetime
import platform
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import yaml


RESOLUTIONS = {"480p": (640, 480), "720p": (1280, 720), "1080p": (1920, 1080)}

MIN_CORNERS_PER_FRAME = 6
MIN_FRAMES_FOR_CALIBRATION = 15
CAPTURE_INTERVAL_SEC = 0.8
MOTION_THRESHOLD = 15.0


# ──────────────────────────────────────────────────────────────────────
# Camera helpers
# ──────────────────────────────────────────────────────────────────────

def open_camera(device: int, resolution: str, fps: int) -> Optional[cv2.VideoCapture]:
    width, height = RESOLUTIONS.get(resolution, (1280, 720))
    system = platform.system()
    if system == "Linux":
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    elif system == "Windows":
        cap = cv2.VideoCapture(device, cv2.CAP_DSHOW)
    else:
        cap = cv2.VideoCapture(device)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap if cap.isOpened() else None


# ──────────────────────────────────────────────────────────────────────
# ChArUco helpers
# ──────────────────────────────────────────────────────────────────────

def create_charuco(
    cols: int, rows: int, square_mm: float, marker_mm: float,
) -> tuple[cv2.aruco.CharucoBoard, cv2.aruco.CharucoDetector]:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    board = cv2.aruco.CharucoBoard(
        (cols, rows), square_mm / 1000.0, marker_mm / 1000.0, dictionary,
    )
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    charuco_params = cv2.aruco.CharucoParameters()
    detector = cv2.aruco.CharucoDetector(board, charuco_params, params)
    return board, detector


def detect_charuco(detector: cv2.aruco.CharucoDetector, frame: np.ndarray):
    """Returns (charuco_corners, charuco_ids, marker_corners, marker_ids)."""
    return detector.detectBoard(frame)


def compute_coverage(all_corners: list, image_size: tuple[int, int]) -> tuple[float, np.ndarray]:
    w, h = image_size
    grid = np.zeros((4, 4), dtype=bool)
    if not all_corners:
        return 0.0, grid
    for corners in all_corners:
        for pt in corners.reshape(-1, 2):
            gx = min(int(pt[0] / w * 4), 3)
            gy = min(int(pt[1] / h * 4), 3)
            grid[gy, gx] = True
    return float(grid.sum()) / 16.0, grid


def corners_moved(prev_corners: Optional[np.ndarray], cur_corners: np.ndarray) -> bool:
    if prev_corners is None:
        return True
    if prev_corners.shape != cur_corners.shape:
        return True
    diff = np.linalg.norm(prev_corners.reshape(-1, 2) - cur_corners.reshape(-1, 2), axis=1)
    return float(np.mean(diff)) > MOTION_THRESHOLD


# ──────────────────────────────────────────────────────────────────────
# HUD drawing
# ──────────────────────────────────────────────────────────────────────

def draw_hud(
    frame: np.ndarray,
    n_captured: int,
    n_corners: int,
    elapsed: float,
    duration: float,
    coverage: float,
    coverage_grid: np.ndarray,
) -> None:
    h, w = frame.shape[:2]
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w, 70), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

    remaining = max(0.0, duration - elapsed)
    time_color = (0, 255, 0) if remaining > 10 else (0, 165, 255)
    cv2.putText(frame, f"{remaining:.0f}s", (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, time_color, 2)

    corner_color = (0, 255, 0) if n_corners >= MIN_CORNERS_PER_FRAME else (0, 0, 255)
    cv2.putText(frame, f"Corners:{n_corners}", (100, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, corner_color, 2)

    cap_color = (0, 255, 0) if n_captured >= MIN_FRAMES_FOR_CALIBRATION else (0, 200, 255)
    cv2.putText(frame, f"Saved:{n_captured}", (10, 58),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, cap_color, 2)

    cov_color = (0, 255, 0) if coverage > 0.6 else (0, 200, 255)
    cv2.putText(frame, f"Cov:{coverage:.0%}", (180, 58),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, cov_color, 2)

    # Mini coverage grid
    cell = 12
    gx0, gy0 = w - 60, 10
    for gy in range(4):
        for gx in range(4):
            x1, y1 = gx0 + gx * cell, gy0 + gy * cell
            color = (0, 180, 0) if coverage_grid[gy, gx] else (60, 60, 60)
            cv2.rectangle(frame, (x1, y1), (x1 + cell, y1 + cell), color, -1)
            cv2.rectangle(frame, (x1, y1), (x1 + cell, y1 + cell), (100, 100, 100), 1)

    # Progress bar
    progress = min(elapsed / duration, 1.0)
    cv2.rectangle(frame, (0, h - 6), (w, h), (40, 40, 40), -1)
    cv2.rectangle(frame, (0, h - 6), (int(w * progress), h), (0, 200, 0), -1)


# ──────────────────────────────────────────────────────────────────────
# Phase 1: Capture
# ──────────────────────────────────────────────────────────────────────

def cmd_capture(args) -> int:
    output_yaml = _resolve_output(args.output)

    cap = open_camera(args.camera, args.resolution, args.fps)
    if cap is None:
        print("ERROR: Cannot open camera")
        return 1

    ret, test_frame = cap.read()
    if not ret:
        print("ERROR: Cannot read from camera")
        return 1
    actual_h, actual_w = test_frame.shape[:2]
    image_size = (actual_w, actual_h)
    print(f"Camera {args.camera}: {actual_w}x{actual_h}")

    # Create capture output dir
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    capture_dir = Path(__file__).parent / "captures" / ts
    capture_dir.mkdir(parents=True, exist_ok=True)

    # Save metadata for calibrate phase
    meta = {
        "cols": args.cols, "rows": args.rows,
        "square_mm": args.square, "marker_mm": args.marker,
        "image_size": list(image_size),
        "resolution": args.resolution,
    }
    meta_path = capture_dir / "meta.yaml"
    with open(meta_path, "w") as f:
        yaml.dump(meta, f, sort_keys=False)

    board, detector = create_charuco(args.cols, args.rows, args.square, args.marker)

    saved_count = 0
    all_detected_corners: list = []
    prev_corners: Optional[np.ndarray] = None
    last_capture_time = 0.0
    t_start = time.time()

    print(f"Saving to: {capture_dir}")
    print(f"Duration: {args.duration:.0f}s — move camera around the board. Press 'q' to stop.\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        elapsed = time.time() - t_start
        if elapsed >= args.duration:
            break

        charuco_corners, charuco_ids, marker_corners, marker_ids = detect_charuco(detector, frame)
        n_corners = len(charuco_corners) if charuco_corners is not None else 0

        now = time.time()
        should_capture = (
            n_corners >= MIN_CORNERS_PER_FRAME
            and now - last_capture_time >= CAPTURE_INTERVAL_SEC
            and charuco_corners is not None
            and corners_moved(prev_corners, charuco_corners)
        )

        if should_capture:
            fname = f"{saved_count:04d}.png"
            cv2.imwrite(str(capture_dir / fname), frame)
            all_detected_corners.append(charuco_corners)
            prev_corners = charuco_corners.copy()
            last_capture_time = now
            saved_count += 1
            coverage, _ = compute_coverage(all_detected_corners, image_size)
            print(f"  [{saved_count:3d}] {n_corners} corners, coverage={coverage:.0%}")

        # Lightweight preview
        vis = frame.copy()
        if marker_corners:
            cv2.aruco.drawDetectedMarkers(vis, marker_corners, marker_ids)
        if charuco_corners is not None and n_corners > 0:
            cv2.aruco.drawDetectedCornersCharuco(vis, charuco_corners, charuco_ids)

        coverage, coverage_grid = compute_coverage(all_detected_corners, image_size)
        draw_hud(vis, saved_count, n_corners, elapsed, args.duration, coverage, coverage_grid)
        cv2.imshow("Capture", vis)

        if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
            print("\nStopped early.")
            break

    cap.release()
    cv2.destroyAllWindows()

    print(f"\nCapture done: {saved_count} images saved to {capture_dir}")

    if saved_count < MIN_FRAMES_FOR_CALIBRATION:
        print(f"WARNING: Only {saved_count} frames, need {MIN_FRAMES_FOR_CALIBRATION}+. "
              "Re-run capture with the board more visible.")
        return 1

    print("\n--- Running calibration ---")
    return _calibrate_from_dir(capture_dir, output_yaml)


# ──────────────────────────────────────────────────────────────────────
# Phase 2: Calibrate from saved images
# ──────────────────────────────────────────────────────────────────────

def cmd_calibrate(args) -> int:
    capture_dir = Path(args.input)
    if not capture_dir.is_dir():
        print(f"ERROR: {capture_dir} is not a directory")
        return 1

    output_yaml = _resolve_output(args.output)
    return _calibrate_from_dir(capture_dir, output_yaml)


def _calibrate_from_dir(capture_dir: Path, output_yaml: Path) -> int:
    meta_path = capture_dir / "meta.yaml"
    if not meta_path.exists():
        print(f"ERROR: {meta_path} not found")
        return 1

    with open(meta_path) as f:
        meta = yaml.safe_load(f)

    cols, rows = meta["cols"], meta["rows"]
    square_mm, marker_mm = meta["square_mm"], meta["marker_mm"]
    image_size = tuple(meta["image_size"])

    board, detector = create_charuco(cols, rows, square_mm, marker_mm)

    image_files = sorted(capture_dir.glob("*.png"))
    if not image_files:
        print("ERROR: No .png files found")
        return 1

    print(f"Processing {len(image_files)} images from {capture_dir} ...")

    all_corners: list = []
    all_ids: list = []
    used = 0

    for img_path in image_files:
        frame = cv2.imread(str(img_path))
        if frame is None:
            continue
        charuco_corners, charuco_ids, _, _ = detect_charuco(detector, frame)
        n = len(charuco_corners) if charuco_corners is not None else 0
        if n >= MIN_CORNERS_PER_FRAME:
            all_corners.append(charuco_corners)
            all_ids.append(charuco_ids)
            used += 1

    print(f"Usable frames: {used}/{len(image_files)}")

    if used < MIN_FRAMES_FOR_CALIBRATION:
        print(f"ERROR: Need {MIN_FRAMES_FOR_CALIBRATION}+ frames, got {used}")
        return 1

    print("Calibrating ...")
    try:
        ret, K, D, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
            all_corners, all_ids, board, image_size, None, None,
        )
    except cv2.error as e:
        print(f"ERROR: OpenCV calibration failed: {e}")
        print(f"Images are preserved in {capture_dir} — try capturing more diverse angles.")
        return 1

    print(f"\n{'='*50}")
    print(f"RMS reprojection error: {ret:.4f}")
    print(f"{'='*50}")
    print(f"Camera matrix:\n{K}")
    print(f"Distortion:\n{D.ravel()}")

    if ret > 1.0:
        print(f"\nWARNING: RMS {ret:.4f} is high.")

    data = {
        "calibration_date": datetime.datetime.now().isoformat(),
        "camera_matrix": K.tolist(),
        "distortion_coefficients": D.tolist(),
        "image_size": list(image_size),
    }
    with open(output_yaml, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)
    print(f"\nSaved to: {output_yaml}")
    return 0


# ──────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────

def _resolve_output(output_arg: Optional[str]) -> Path:
    if output_arg:
        return Path(output_arg)
    return (Path(__file__).resolve().parent.parent
            / "src" / "micromvp" / "env" / "new_real_push_env" / "camera.yaml")


# ──────────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser(description="ChArUco camera calibration (two-phase)")
    sub = ap.add_subparsers(dest="command")

    # capture
    p_cap = sub.add_parser("capture", help="Capture frames then calibrate")
    p_cap.add_argument("--camera", type=int, default=0)
    p_cap.add_argument("--resolution", default="720p", choices=list(RESOLUTIONS.keys()))
    p_cap.add_argument("--fps", type=int, default=30)
    p_cap.add_argument("--cols", type=int, default=7)
    p_cap.add_argument("--rows", type=int, default=5)
    p_cap.add_argument("--square", type=float, default=30.0, help="square side mm")
    p_cap.add_argument("--marker", type=float, default=22.0, help="marker side mm")
    p_cap.add_argument("--duration", type=float, default=60.0, help="seconds")
    p_cap.add_argument("-o", "--output", default=None, help="output camera.yaml path")

    # calibrate
    p_cal = sub.add_parser("calibrate", help="Calibrate from saved images")
    p_cal.add_argument("--input", required=True, help="path to captures directory")
    p_cal.add_argument("-o", "--output", default=None, help="output camera.yaml path")

    args = ap.parse_args()

    if args.command == "capture":
        return cmd_capture(args)
    elif args.command == "calibrate":
        return cmd_calibrate(args)
    else:
        ap.print_help()
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
