#!/usr/bin/env python3
"""
Integration test for NewRealPushEnv on real hardware.

Phases:
1. Startup & workspace lock (camera + ArUco detection)
2. Observation validation (detect cars, print positions)
3. Motor control test (forward, spin, stop)
4. Teardown

Usage:
    conda activate micromvp
    python tests/test_new_env_integration.py [--car-id 6] [--port /dev/tty.usbmodem3101]
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass
from typing import Dict, List, Optional

from micromvp.core.models import Action
from micromvp.env.new_real_push_env import NewRealPushEnv, NewRealPushConfig


@dataclass
class TestResult:
    phase: str
    passed: bool
    detail: str
    timestamp: float = 0.0

    def __post_init__(self):
        if self.timestamp == 0.0:
            self.timestamp = time.time()


def phase_startup(env: NewRealPushEnv, timeout: float) -> TestResult:
    """Phase 1: Start env and wait for workspace lock."""
    print("\n" + "=" * 60)
    print("Phase 1: Startup & Workspace Lock")
    print("=" * 60)

    t0 = time.time()
    ok = env.start(wait_for_ready=True, timeout=timeout)
    elapsed = time.time() - t0

    if not ok:
        return TestResult("startup", False, f"env.start() returned False after {elapsed:.1f}s")

    ws = env.workspace_config
    if ws.width <= 0 or ws.height <= 0:
        return TestResult(
            "startup", False,
            f"Workspace has invalid size: {ws.width:.1f} x {ws.height:.1f}",
        )

    detail = (
        f"Workspace locked in {elapsed:.1f}s: "
        f"{ws.width:.1f} x {ws.height:.1f} cm, "
        f"car_id_list={ws.car_id_list}"
    )
    print(f"  [OK] {detail}")
    return TestResult("startup", True, detail)


def phase_observation(env: NewRealPushEnv, expected_ids: List[int], duration: float) -> TestResult:
    """Phase 2: Observe for a few seconds and validate car detections."""
    print("\n" + "=" * 60)
    print(f"Phase 2: Observation Validation ({duration:.0f}s)")
    print("=" * 60)

    seen_ids: set[int] = set()
    obs_count = 0
    t0 = time.time()

    while time.time() - t0 < duration:
        observations = env.observe()
        if observations:
            obs_count += 1
            for car_id, obs in observations.items():
                seen_ids.add(car_id)
                if obs_count % 10 == 0:
                    print(
                        f"  Car {car_id}: x={obs.x:.1f}, y={obs.y:.1f}, "
                        f"theta={obs.theta:.1f}°"
                    )
        env.render()
        time.sleep(0.03)

    if not seen_ids:
        return TestResult("observation", False, "No cars detected during observation phase")

    missing = set(expected_ids) - seen_ids if expected_ids else set()
    extra = seen_ids - set(expected_ids) if expected_ids else set()

    detail = (
        f"Observed {obs_count} frames, detected cars: {sorted(seen_ids)}"
    )
    if missing:
        detail += f", MISSING expected: {sorted(missing)}"
    if extra:
        detail += f", extra detected: {sorted(extra)}"

    passed = len(missing) == 0
    print(f"  [{'OK' if passed else 'WARN'}] {detail}")
    return TestResult("observation", passed, detail)


def phase_motor_control(
    env: NewRealPushEnv, car_id: int, speed: float, duration_per_action: float
) -> TestResult:
    """Phase 3: Send motor commands and verify position changes."""
    print("\n" + "=" * 60)
    print(f"Phase 3: Motor Control (car={car_id}, speed={speed})")
    print("=" * 60)

    def get_pos() -> Optional[tuple]:
        obs = env.observe()
        if car_id in obs:
            o = obs[car_id]
            return (o.x, o.y, o.theta)
        return None

    pos_before = get_pos()
    if pos_before is None:
        return TestResult("motor_control", False, f"Car {car_id} not visible before motor test")

    print(f"  Start pos: x={pos_before[0]:.1f}, y={pos_before[1]:.1f}, θ={pos_before[2]:.1f}°")

    actions = [
        ("forward", Action(left_speed=speed, right_speed=speed)),
        ("spin_left", Action(left_speed=-speed, right_speed=speed)),
        ("stop", Action.stop()),
    ]

    position_log: List[dict] = []
    for name, action in actions:
        print(f"  Sending: {name} for {duration_per_action:.1f}s ...")
        t0 = time.time()
        while time.time() - t0 < duration_per_action:
            env.apply_actions({car_id: action})
            obs = env.observe()
            if car_id in obs:
                o = obs[car_id]
                position_log.append({
                    "t": time.time() - t0,
                    "action": name,
                    "x": o.x, "y": o.y, "theta": o.theta,
                })
            env.render()
            time.sleep(0.03)

    env.apply_actions({car_id: Action.stop()})
    time.sleep(0.3)

    pos_after = get_pos()
    if pos_after is None:
        return TestResult("motor_control", False, f"Car {car_id} not visible after motor test")

    dx = pos_after[0] - pos_before[0]
    dy = pos_after[1] - pos_before[1]
    dist = (dx**2 + dy**2) ** 0.5
    dtheta = abs(pos_after[2] - pos_before[2])

    detail = (
        f"Car {car_id}: moved {dist:.1f} cm (dx={dx:.1f}, dy={dy:.1f}), "
        f"rotated {dtheta:.1f}°, logged {len(position_log)} samples"
    )
    passed = dist > 1.0 or dtheta > 5.0
    if not passed:
        detail += " — WARNING: minimal movement detected, motors may not be responding"

    print(f"  End pos: x={pos_after[0]:.1f}, y={pos_after[1]:.1f}, θ={pos_after[2]:.1f}°")
    print(f"  [{'OK' if passed else 'FAIL'}] {detail}")
    return TestResult("motor_control", passed, detail)


def main() -> int:
    ap = argparse.ArgumentParser(description="NewRealPushEnv integration test")
    ap.add_argument("--port", default="/dev/tty.usbmodem3101")
    ap.add_argument("--car-id", type=int, default=6, help="primary car ID to test")
    ap.add_argument("--speed", type=float, default=0.3, help="motor test speed [0-1]")
    ap.add_argument("--timeout", type=float, default=30.0, help="workspace lock timeout")
    ap.add_argument("--observe-sec", type=float, default=5.0, help="observation phase duration")
    ap.add_argument("--action-sec", type=float, default=2.0, help="per-action duration")
    ap.add_argument("--no-motor", action="store_true", help="skip motor control phase")
    ap.add_argument("--no-preview", action="store_true", help="disable camera preview")
    ap.add_argument("--camera", type=int, default=0, help="camera device index")
    args = ap.parse_args()

    print("=" * 60)
    print("NewRealPushEnv Integration Test")
    print("=" * 60)
    print(f"  Serial port: {args.port}")
    print(f"  Camera:      device {args.camera}")
    print(f"  Target car:  {args.car_id}")
    print(f"  Speed:       {args.speed}")

    config = NewRealPushConfig(
        serial_port=args.port,
        camera_device=args.camera,
        robot_ids=[args.car_id],
        no_preview=args.no_preview,
        car_marker_size_mm=27.0,
    )
    env = NewRealPushEnv(config)

    results: List[TestResult] = []

    try:
        r1 = phase_startup(env, args.timeout)
        results.append(r1)
        if not r1.passed:
            print("\n[ABORT] Startup failed, skipping remaining phases.")
            return 1

        r2 = phase_observation(env, [args.car_id], args.observe_sec)
        results.append(r2)

        if not args.no_motor and r2.passed:
            r3 = phase_motor_control(env, args.car_id, args.speed, args.action_sec)
            results.append(r3)
        elif args.no_motor:
            print("\n  [SKIP] Motor phase skipped (--no-motor)")

    except KeyboardInterrupt:
        print("\n\n[INTERRUPTED] Ctrl+C received, stopping motors...")
        env.stop_all()
    finally:
        env.close()

    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    all_passed = True
    for r in results:
        status = "PASS" if r.passed else "FAIL"
        print(f"  [{status}] {r.phase}: {r.detail}")
        if not r.passed:
            all_passed = False

    print("=" * 60)
    if all_passed:
        print("Result: ALL PHASES PASSED")
    else:
        print("Result: SOME PHASES FAILED")
    print("=" * 60)

    return 0 if all_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
