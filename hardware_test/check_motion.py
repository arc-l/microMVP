#!/usr/bin/env python3
"""Drive each named car through four motions, then stop.

Forward, backward, counter-clockwise, clockwise — one car at a time, so you
can watch each one and see whether it moves the way it is told. No camera
involved; this only exercises the serial path down to the wheels.

    python -m hardware_test.check_motion --cars 3
    python -m hardware_test.check_motion --cars 1,2,5 --speed 0.25

Put the car on the floor with room around it before running. If a car
moves backwards when told to go forward, or spins the wrong way, flip
`actuation.invert_left_wheel` / `invert_right_wheel` in your config.
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import List

from micromvp.config import ConfigError, load_config
from micromvp.core.models import Action
from micromvp.env.real_env.serial_action import SerialActionConfig, SerialActionSender

# (label, left, right) as multiples of --speed.
# Counter-clockwise means the left wheel drives backwards, matching the
# workspace convention where +Y is up and angles grow counter-clockwise.
MOTIONS = [
    ("forward          ", 1.0, 1.0),
    ("backward         ", -1.0, -1.0),
    ("counter-clockwise", -1.0, 1.0),
    ("clockwise        ", 1.0, -1.0),
]


def parse_cars(raw: str) -> List[int]:
    cars: List[int] = []
    for part in raw.split(","):
        part = part.strip()
        if not part:
            continue
        try:
            car_id = int(part)
        except ValueError:
            raise SystemExit(f"'{part}' is not a car id; expected e.g. --cars 1,2,3")
        if car_id < 1:
            raise SystemExit(f"car id must be >= 1, got {car_id}")
        cars.append(car_id)
    if not cars:
        raise SystemExit("no car ids given; use --cars 3")
    return cars


def main() -> int:
    parser = argparse.ArgumentParser(description="Motion check for one or more cars")
    parser.add_argument(
        "--cars", type=str, required=True,
        help="Car ids to test, comma separated (e.g. 3 or 1,2,5)",
    )
    parser.add_argument(
        "--config", type=str, default="config/car_v4.yaml",
        help="Deployment config (default: config/car_v4.yaml)",
    )
    parser.add_argument(
        "--speed", type=float, default=0.2,
        help="Wheel thrust for the test, 0..1 (default: 0.2)",
    )
    parser.add_argument(
        "--duration", type=float, default=0.8,
        help="Seconds per motion (default: 0.8)",
    )
    parser.add_argument(
        "--pause", type=float, default=0.7,
        help="Seconds stopped between motions (default: 0.7)",
    )
    args = parser.parse_args()

    cars = parse_cars(args.cars)
    speed = min(1.0, max(0.0, args.speed))

    try:
        cfg = load_config(args.config)
        sender = SerialActionSender(SerialActionConfig.from_config(cfg, robot_ids=cars))
    except ConfigError as exc:
        print(exc)
        return 1

    if not sender.start():
        print("Could not open the serial port.")
        print("  Run `python -m hardware_test.find_ap` to find the AP.")
        return 1

    # The AP reboots when the port opens; let it come up first.
    time.sleep(2.0)

    print(f"Testing cars {cars} at thrust {speed}, {args.duration}s per motion.")
    print("Give them room. Ctrl-C stops everything.\n")

    try:
        for car_id in cars:
            print(f"car {car_id}")
            for label, left, right in MOTIONS:
                print(f"  {label}  L={left * speed:+.2f} R={right * speed:+.2f}")
                sender.set_action(car_id, Action(left * speed, right * speed))
                time.sleep(args.duration)
                sender.set_action(car_id, Action.stop())
                time.sleep(args.pause)
            print()
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        sender.stop_all()
        sender.stop()

    print("Done. If a car sat still, check that its firmware CAR_ID matches")
    print("the id you tested (xiao/xiao_1_8_ESP_NOW.ino).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
