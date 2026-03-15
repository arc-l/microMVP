#!/usr/bin/env python3
"""
Offline workspace analysis script.

Processes a video file or image sequence through the ArucoObserver workspace
estimation pipeline and saves structured results for human review.

Usage:
    python scripts/offline_workspace_analysis.py \
        --input video.mp4 \
        --calibration src/micromvp/env/new_real_push_env/camera.yaml \
        --output-dir workspace_analysis_output

    python scripts/offline_workspace_analysis.py \
        --input frames/ \
        --calibration camera.yaml \
        --output-dir workspace_analysis_output

Output directory structure:
    output-dir/
        summary.json          -- overall results: lock frame, final workspace params
        candidates.csv        -- per-frame workspace candidate data
        overlay/              -- annotated frames (png)
            frame_000000.png
            ...
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from micromvp.env.new_real_push_env.observer import (
    ArucoObserver,
    ObserverConfig,
    WorkspaceEstimate,
    _WorkspaceLockState,
    _MarkerInfo,
)


def load_calibration(path: str):
    with open(path, "r", encoding="utf-8") as f:
        calib = yaml.safe_load(f)
    K = np.array(calib["camera_matrix"], dtype=np.float32)
    D = np.array(calib["distortion_coefficients"], dtype=np.float32)
    size = tuple(calib["image_size"][:2])
    return K, D, size


def open_source(input_path: str) -> Tuple[cv2.VideoCapture, bool]:
    """Open a video file or image-sequence directory. Returns (cap, is_video)."""
    p = Path(input_path)
    if p.is_dir():
        exts = {".png", ".jpg", ".jpeg", ".bmp"}
        images = sorted(f for f in p.iterdir() if f.suffix.lower() in exts)
        if not images:
            raise FileNotFoundError(f"No images found in {input_path}")
        cap = cv2.VideoCapture(str(images[0]))
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open first image: {images[0]}")
        cap.release()
        return _ImageSeqCapture(images), False
    else:
        cap = cv2.VideoCapture(str(p))
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open video: {input_path}")
        return cap, True


class _ImageSeqCapture:
    """Minimal cv2.VideoCapture-like wrapper for an image sequence."""

    def __init__(self, paths: List[Path]):
        self._paths = paths
        self._idx = 0

    def read(self):
        if self._idx >= len(self._paths):
            return False, None
        frame = cv2.imread(str(self._paths[self._idx]))
        self._idx += 1
        if frame is None:
            return False, None
        return True, frame

    def release(self):
        pass

    def isOpened(self):
        return self._idx < len(self._paths)


def main():
    parser = argparse.ArgumentParser(description="Offline workspace analysis")
    parser.add_argument("--input", required=True, help="Video file or image directory")
    parser.add_argument("--calibration", required=True, help="camera.yaml calibration file")
    parser.add_argument("--output-dir", required=True, help="Output directory")
    parser.add_argument("--car-dict", default="DICT_4X4_50")
    parser.add_argument("--obstacle-dict", default="DICT_5X5_50")
    parser.add_argument("--car-marker-size-mm", type=float, default=36.0)
    parser.add_argument("--car-marker-height-cm", type=float, default=4.8)
    parser.add_argument("--obstacle-marker-size-mm", type=float, default=30.0)
    parser.add_argument("--obstacle-marker-height-cm", type=float, default=4.0)
    parser.add_argument("--lock-frames", type=int, default=30)
    parser.add_argument("--margin-cm", type=float, default=1.0)
    parser.add_argument("--min-side-cm", type=float, default=10.0)
    parser.add_argument("--max-frames", type=int, default=0, help="0 = process all")
    parser.add_argument("--save-every-nth-overlay", type=int, default=1,
                        help="Save overlay image every N frames (1 = every frame)")
    args = parser.parse_args()

    out_dir = Path(args.output_dir)
    overlay_dir = out_dir / "overlay"
    out_dir.mkdir(parents=True, exist_ok=True)
    overlay_dir.mkdir(parents=True, exist_ok=True)

    K, D, calib_size = load_calibration(args.calibration)
    calib_w, calib_h = calib_size

    cap, is_video = open_source(args.input)

    config = ObserverConfig(
        calibration_file=args.calibration,
        car_dict=args.car_dict,
        obstacle_dict=args.obstacle_dict,
        car_marker_size_mm=args.car_marker_size_mm,
        car_marker_height_cm=args.car_marker_height_cm,
        obstacle_marker_size_mm=args.obstacle_marker_size_mm,
        obstacle_marker_height_cm=args.obstacle_marker_height_cm,
        workspace_lock_frames=args.lock_frames,
        workspace_margin_cm=args.margin_cm,
        workspace_min_side_cm=args.min_side_cm,
        no_preview=True,
    )

    aruco = cv2.aruco
    dicts = {
        "DICT_4X4_50": aruco.DICT_4X4_50,
        "DICT_4X4_100": aruco.DICT_4X4_100,
        "DICT_5X5_50": aruco.DICT_5X5_50,
        "DICT_5X5_100": aruco.DICT_5X5_100,
        "DICT_6X6_50": aruco.DICT_6X6_50,
    }
    params = aruco.DetectorParameters()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

    car_detector = aruco.ArucoDetector(
        aruco.getPredefinedDictionary(dicts.get(args.car_dict, aruco.DICT_4X4_50)),
        params,
    )
    obs_detector = aruco.ArucoDetector(
        aruco.getPredefinedDictionary(dicts.get(args.obstacle_dict, aruco.DICT_5X5_50)),
        params,
    )

    lock_state = _WorkspaceLockState(args.lock_frames)

    obs = ArucoObserver.__new__(ArucoObserver)
    obs._config = config
    obs._K = K
    obs._D = D
    obs._frame_size = calib_size

    csv_path = out_dir / "candidates.csv"
    csv_file = open(csv_path, "w", newline="", encoding="utf-8")
    writer = csv.writer(csv_file)
    writer.writerow([
        "frame", "ready", "width_cm", "height_cm",
        "origin_x", "origin_y", "origin_z",
        "normal_x", "normal_y", "normal_z",
        "lock_state", "n_candidates", "n_required",
    ])

    lock_frame: Optional[int] = None
    final_ws: Optional[WorkspaceEstimate] = None
    frame_idx = 0

    print(f"Processing {args.input} ...")
    t0 = time.time()

    while True:
        if args.max_frames > 0 and frame_idx >= args.max_frames:
            break

        ret, frame = cap.read()
        if not ret:
            break

        actual_h, actual_w = frame.shape[:2]
        if (actual_w, actual_h) != (calib_w, calib_h):
            print(
                f"[WARN] Frame {frame_idx}: size {actual_w}x{actual_h} != "
                f"calibration {calib_w}x{calib_h}, skipping"
            )
            frame_idx += 1
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        def detect(detector, size_mm, height_cm):
            markers = {}
            corners_out, ids_out, _ = detector.detectMarkers(gray)
            if ids_out is None or len(ids_out) == 0:
                return markers, corners_out, ids_out
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners_out, size_mm / 1000.0, K, D
            )
            for i, mid in enumerate(ids_out.flatten().tolist()):
                rvec = rvecs[i, 0, :].astype(np.float32)
                tvec = tvecs[i, 0, :].astype(np.float32)
                R, _ = cv2.Rodrigues(rvec)
                markers[int(mid)] = _MarkerInfo(
                    marker_id=int(mid),
                    image_corners=corners_out[i].reshape(4, 2).astype(np.float32),
                    rvec=rvec, tvec=tvec,
                    normal_cam=R[:, 2].astype(np.float32),
                    height_cm=float(height_cm),
                )
            return markers, corners_out, ids_out

        car_m, car_c, car_ids = detect(car_detector, args.car_marker_size_mm, args.car_marker_height_cm)
        obs_m, obs_c, obs_ids = detect(obs_detector, args.obstacle_marker_size_mm, args.obstacle_marker_height_cm)

        all_markers = list(car_m.values()) + list(obs_m.values())
        candidate = obs._estimate_workspace(all_markers)
        lock_state.add_candidate(candidate)

        if lock_state.window_full and not lock_state.is_locked:
            if lock_state.check_stability(config):
                final_ws = lock_state.aggregate()
                lock_state.state = "locked"
                lock_frame = frame_idx
                print(
                    f"  LOCKED at frame {frame_idx}: "
                    f"{final_ws.width_cm:.1f} x {final_ws.height_cm:.1f} cm"
                )
            else:
                lock_state.state = "collecting"

        writer.writerow([
            frame_idx,
            candidate.ready,
            f"{candidate.width_cm:.2f}" if candidate.ready else "",
            f"{candidate.height_cm:.2f}" if candidate.ready else "",
            f"{candidate.origin_cam_m[0]:.6f}" if candidate.ready else "",
            f"{candidate.origin_cam_m[1]:.6f}" if candidate.ready else "",
            f"{candidate.origin_cam_m[2]:.6f}" if candidate.ready else "",
            f"{candidate.normal_cam[0]:.6f}" if candidate.ready else "",
            f"{candidate.normal_cam[1]:.6f}" if candidate.ready else "",
            f"{candidate.normal_cam[2]:.6f}" if candidate.ready else "",
            lock_state.state,
            len(lock_state.candidates),
            lock_state.required_frames,
        ])

        if frame_idx % args.save_every_nth_overlay == 0:
            vis = frame.copy()
            if car_ids is not None and len(car_ids) > 0:
                cv2.aruco.drawDetectedMarkers(vis, car_c, car_ids, borderColor=(0, 255, 0))
            if obs_ids is not None and len(obs_ids) > 0:
                cv2.aruco.drawDetectedMarkers(vis, obs_c, obs_ids, borderColor=(0, 165, 255))

            state_text = lock_state.state
            n_cand = len(lock_state.candidates)
            if candidate.ready:
                label = (
                    f"WS: {candidate.width_cm:.1f}x{candidate.height_cm:.1f}cm "
                    f"[{state_text} {n_cand}/{args.lock_frames}]"
                )
                color = (0, 255, 0) if lock_state.is_locked else (0, 200, 255)
            else:
                label = f"WS: not ready [{state_text} {n_cand}/{args.lock_frames}]"
                color = (0, 0, 255)
            cv2.putText(vis, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.putText(vis, f"Frame {frame_idx}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            overlay_path = overlay_dir / f"frame_{frame_idx:06d}.png"
            cv2.imwrite(str(overlay_path), vis)

        frame_idx += 1

    csv_file.close()
    cap.release()

    elapsed = time.time() - t0
    print(f"Processed {frame_idx} frames in {elapsed:.1f}s ({frame_idx / max(elapsed, 0.01):.1f} fps)")

    summary = {
        "input": args.input,
        "calibration": args.calibration,
        "total_frames": frame_idx,
        "lock_frame": lock_frame,
        "locked": lock_state.is_locked,
        "lock_frames_required": args.lock_frames,
        "final_workspace": None,
    }
    if final_ws is not None:
        summary["final_workspace"] = {
            "width_cm": final_ws.width_cm,
            "height_cm": final_ws.height_cm,
            "origin_cam_m": list(final_ws.origin_cam_m),
            "x_axis_cam": list(final_ws.x_axis_cam),
            "y_axis_cam": list(final_ws.y_axis_cam),
            "normal_cam": list(final_ws.normal_cam),
        }

    summary_path = out_dir / "summary.json"
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    print(f"\nResults saved to {out_dir}/")
    print(f"  summary.json    -- lock={'YES' if lock_state.is_locked else 'NO'}, frame={lock_frame}")
    print(f"  candidates.csv  -- {frame_idx} rows")
    print(f"  overlay/        -- annotated frames")


if __name__ == "__main__":
    main()
