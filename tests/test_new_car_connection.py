#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time

import serial


HDR0 = 0xAA
HDR1 = 0x55
PAYLOAD_LEN = 32

CAR_ID = 6
CARS_PER_LEVEL = 10
FULL_THRUST = 127
RUN_THRUST = 0.3*FULL_THRUST


def encode_thrust(thrust: int) -> int:
    """Encode thrust to sign-magnitude byte used by Xiao car firmware."""
    mag = min(abs(int(thrust)), 127)
    if thrust < 0:
        return 0x80 | mag
    return mag


def level_and_slots(car_id: int) -> tuple[int, int, int]:
    """
    Return (level, left_pos, right_pos) for the 32-byte payload.
    Matches xiao_1_8_ESP_NOW.ino slot mapping.
    """
    if car_id <= 0:
        raise ValueError("car_id must be >= 1")

    level = (car_id - 1) // CARS_PER_LEVEL
    car_index = car_id - level * CARS_PER_LEVEL  # 1..10

    right_pos = 3 * car_index
    left_pos = right_pos - 1
    if not (0 <= left_pos < PAYLOAD_LEN and 0 <= right_pos < PAYLOAD_LEN):
        raise ValueError(f"car_id={car_id} maps out of payload range")
    return level, left_pos, right_pos


def build_frame(level: int, payload: bytes) -> bytes:
    if len(payload) != PAYLOAD_LEN:
        raise ValueError("payload length must be 32 bytes")

    frame = bytearray()
    frame.append(HDR0)
    frame.append(HDR1)
    frame.append(level & 0xFF)
    frame.append(PAYLOAD_LEN)
    frame.extend(payload)

    cksum = 0
    cksum ^= level & 0xFF
    cksum ^= PAYLOAD_LEN
    for b in payload:
        cksum ^= b
    frame.append(cksum & 0xFF)
    return bytes(frame)


def build_cmd_frame(car_id: int, left_thrust: int, right_thrust: int) -> bytes:
    level, left_pos, right_pos = level_and_slots(car_id)
    payload = bytearray(PAYLOAD_LEN)
    payload[left_pos] = encode_thrust(left_thrust)
    payload[right_pos] = encode_thrust(right_thrust)
    return build_frame(level, payload)


def hold_command(
    ser: serial.Serial,
    car_id: int,
    left_thrust: int,
    right_thrust: int,
    duration_s: float,
    hz: float,
) -> None:
    dt = 1.0 / hz
    t_end = time.time() + duration_s
    frame = build_cmd_frame(car_id, left_thrust, right_thrust)

    while time.time() < t_end:
        ser.write(frame)
        time.sleep(dt)


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Serial packet test for Xiao AP: control only car #6"
    )
    ap.add_argument("--port", default="/dev/tty.usbmodem31301", help="serial port path")
    ap.add_argument("--baud", type=int, default=115200, help="serial baud rate")
    ap.add_argument("--hz", type=float, default=30.0, help="send rate while holding command")
    ap.add_argument("--car-id", type=int, default=CAR_ID, help="target car id")
    ap.add_argument("--left-seconds", type=float, default=3.0, help="left spin duration per cycle")
    ap.add_argument("--right-seconds", type=float, default=3.0, help="right spin duration per cycle")
    args = ap.parse_args()

    print(
        f"[test] open serial {args.port} @ {args.baud}, car_id={args.car_id}, hz={args.hz}"
    )
    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        time.sleep(0.2)

        # Clear stale bytes, then send a short stop before test.
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        hold_command(ser, args.car_id, 0, 0, duration_s=0.2, hz=args.hz)

        print("[test] running infinite cycle: LEFT then RIGHT (Ctrl+C to stop)")
        cycle = 0
        try:
            while True:
                cycle += 1
                print(f"[test] cycle={cycle} LEFT {args.left_seconds:.2f}s")
                # Spin left: left wheel backward, right wheel forward.
                hold_command(
                    ser,
                    args.car_id,
                    left_thrust=-RUN_THRUST,
                    right_thrust=+RUN_THRUST,
                    duration_s=args.left_seconds,
                    hz=args.hz,
                )

                print(f"[test] cycle={cycle} RIGHT {args.right_seconds:.2f}s")
                # Spin right: left wheel forward, right wheel backward.
                hold_command(
                    ser,
                    args.car_id,
                    left_thrust=+RUN_THRUST,
                    right_thrust=-RUN_THRUST,
                    duration_s=args.right_seconds,
                    hz=args.hz,
                )
                print(f"[test] cycle={cycle} STOP 1s")
                hold_command(ser, args.car_id, 0, 0, duration_s=1, hz=args.hz)
        except KeyboardInterrupt:
            print("\n[test] Ctrl+C received, sending STOP...")
        finally:
            hold_command(ser, args.car_id, 0, 0, duration_s=0.4, hz=args.hz)

    print("[test] done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
