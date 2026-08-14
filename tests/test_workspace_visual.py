#!/usr/bin/env python3
"""
Visual workspace inspection tool.

Starts NewRealPushEnv, waits for workspace lock, then stays running with
the camera preview open so you can visually verify the workspace boundary
against the physical setup.

Press 'q' or ESC to quit.

Usage:
    conda activate micromvp
    python tests/test_workspace_visual.py --camera 0 --port /dev/tty.usbmodem3101
"""
from __future__ import annotations

import argparse
import time

import cv2

from micromvp.env.new_real_push_env import NewRealPushEnv, NewRealPushConfig


def main() -> int:
    ap = argparse.ArgumentParser(description="Visual workspace inspection")
    ap.add_argument("--port", default="/dev/tty.usbmodem3101")
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--car-marker-size", type=float, default=27.0, help="car marker size mm")
    args = ap.parse_args()

    config = NewRealPushConfig(
        serial_port=args.port,
        camera_device=args.camera,
        car_marker_size_mm=args.car_marker_size,
        no_preview=False,
    )
    env = NewRealPushEnv(config)

    print("Starting environment, waiting for workspace lock ...")
    if not env.start(wait_for_ready=True, timeout=args.timeout):
        print("ERROR: Workspace did not lock. Check markers are visible.")
        env.close()
        return 1

    ws = env.workspace_config
    print(f"\nWorkspace locked: {ws.width:.1f} x {ws.height:.1f} cm")
    print(f"Detected cars: {ws.car_id_list}")
    print("\nPreview running. Press 'q' or ESC to quit.\n")

    try:
        while True:
            obs = env.observe()
            for car_id, o in obs.items():
                pass  # observations update internally for HUD display

            env.render()

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    except KeyboardInterrupt:
        pass

    env.close()
    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
