#!/usr/bin/env python3
"""Find which serial port the AP is on.

Every candidate port is opened and probed: we send well-formed zero-thrust
frames and watch the AP's own 1 Hz [STAT] line. If frame_ok climbs
while we are sending, that port is the AP. Nothing moves — the frames
carry zero thrust.

    python -m hardware_test.find_ap
    python -m hardware_test.find_ap --port /dev/cu.usbmodem101   # check one

Put the reported path into the `actuation.serial_port` field of your
deployment config, or leave it as "auto" if only one AP is ever
plugged in.
"""
from __future__ import annotations

import argparse
import re
import sys
import time
from typing import List, Optional, Tuple

import serial

from micromvp.env.real_env.serial_action import (
    PAYLOAD_LEN,
    SerialActionSender,
    candidate_serial_ports,
)

# The AP prints this once a second; frame_ok counts frames it accepted.
STAT_RE = re.compile(r"frame_ok=(\d+).*?bad_ck=(\d+)")

# ESP32 boards reset when the port opens. Give the firmware time to boot
# before expecting it to answer.
BOOT_SECONDS = 2.0
PROBE_SECONDS = 3.0
SEND_HZ = 30.0


def _zero_frame() -> bytes:
    """A valid frame that commands every car on level 0 to stop."""
    return SerialActionSender._build_frame(0, bytearray(PAYLOAD_LEN))


def probe(port: str, verbose: bool = False) -> Tuple[bool, str]:
    """Return (is_ap, human-readable detail) for one port."""
    frame = _zero_frame()
    try:
        conn = serial.Serial(port, 115200, timeout=0.1, write_timeout=0.5)
    except serial.SerialException as exc:
        return False, f"could not open ({exc.strerror or exc})"

    try:
        time.sleep(BOOT_SECONDS)
        conn.reset_input_buffer()

        deadline = time.time() + PROBE_SECONDS
        buffer = b""
        sent = 0
        while time.time() < deadline:
            conn.write(frame)
            sent += 1
            buffer += conn.read(256)
            time.sleep(1.0 / SEND_HZ)
        buffer += conn.read(conn.in_waiting or 0)
    except serial.SerialException as exc:
        return False, f"write failed ({exc})"
    finally:
        conn.close()

    text = buffer.decode(errors="replace")
    if verbose and text.strip():
        for line in text.strip().splitlines():
            print(f"      | {line}")

    matches = STAT_RE.findall(text)
    if not matches:
        if text.strip():
            return False, f"answered, but no [STAT] line (sent {sent} frames)"
        return False, f"silent (sent {sent} frames)"

    accepted = sum(int(ok) for ok, _ in matches)
    rejected = sum(int(bad) for _, bad in matches)
    if accepted == 0:
        return False, f"[STAT] seen but frame_ok stayed 0, bad_ck={rejected}"
    return True, f"frame_ok={accepted} bad_ck={rejected} over {PROBE_SECONDS:.0f}s"


def main() -> int:
    parser = argparse.ArgumentParser(description="Locate the Xiao AP")
    parser.add_argument(
        "--port", type=str, default=None,
        help="Probe only this port instead of scanning",
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true",
        help="Print the raw serial output from each port",
    )
    args = parser.parse_args()

    ports: List[str] = [args.port] if args.port else candidate_serial_ports()
    if not ports:
        print("No serial devices found.")
        print("  Plug the Xiao AP in over USB and try again.")
        print("  On macOS the device shows up as /dev/cu.usbmodem*,")
        print("  on Linux as /dev/ttyACM* — check with `ls /dev/tty*`.")
        return 1

    print(f"Probing {len(ports)} port(s); this takes ~{BOOT_SECONDS + PROBE_SECONDS:.0f}s each.")
    print("Sending zero-thrust frames only — nothing will move.\n")

    found: Optional[str] = None
    for port in ports:
        print(f"  {port} ... ", end="", flush=True)
        ok, detail = probe(port, verbose=args.verbose)
        print(("AP  " if ok else "no  ") + detail)
        if ok and found is None:
            found = port

    print()
    if found is None:
        print("No AP responded.")
        print("  - Is the AP firmware (xiao/xiao_ap_ESP_NOW.ino) flashed?")
        print("  - Does anything else hold the port open (Arduino IDE monitor)?")
        print("  - Try -v to see what the port actually sent back.")
        return 1

    print(f"AP found on {found}")
    print(f"  Set this in your config:  actuation.serial_port: {found}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
