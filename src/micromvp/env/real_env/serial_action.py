"""
Serial action sender for the Xiao AP protocol.

Protocol on USB serial (PC -> Xiao AP):
    0xAA 0x55 | level(1) | len(1=32) | payload(32) | checksum_xor(1)

Protocol over ESP-NOW broadcast (handled by the AP firmware):
    level(1) | payload(32)

Payload layout (matches xiao/xiao_1_8_ESP_NOW.ino):
- Each level holds up to CARS_PER_LEVEL (default 10) cars
- Each car uses a 3-byte slot at positions [3*car_index-1 .. 3*car_index]
  - Byte at (3*index - 1): left wheel thrust  (sign-magnitude)
  - Byte at (3*index):     right wheel thrust (sign-magnitude)
- Sign-magnitude encoding: bit7=sign, bits[6:0]=magnitude 0..127
"""
from __future__ import annotations

import glob
import platform
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional

import serial

from micromvp.config import Config
from micromvp.core.models import Action


HDR0 = 0xAA
HDR1 = 0x55
PAYLOAD_LEN = 32
FULL_THRUST = 127


@dataclass
class SerialActionConfig:
    port: str = "/dev/tty.usbmodem1101"
    baudrate: int = 115200
    send_hz: float = 30.0
    cars_per_level: int = 10
    invert_left_wheel: bool = False
    invert_right_wheel: bool = False
    initial_robot_ids: List[int] = field(default_factory=list)

    @classmethod
    def from_config(cls, cfg: "Config", robot_ids: Optional[List[int]] = None) -> "SerialActionConfig":
        """Build from the deployment YAML. Every field is required there."""
        who = "SerialActionSender"
        port = cfg.require("actuation.serial_port", str, who=who)
        return cls(
            port=resolve_serial_port(port),
            baudrate=cfg.require("actuation.baudrate", int, who=who),
            send_hz=cfg.require("actuation.send_hz", float, who=who),
            cars_per_level=cfg.require("actuation.cars_per_level", int, who=who),
            invert_left_wheel=cfg.require("actuation.invert_left_wheel", bool, who=who),
            invert_right_wheel=cfg.require("actuation.invert_right_wheel", bool, who=who),
            initial_robot_ids=list(robot_ids or []),
        )


def candidate_serial_ports() -> List[str]:
    """Device paths that could plausibly be the AP, best first."""
    patterns = [
        "/dev/cu.usbmodem*",   # macOS, callout device (does not block on DCD)
        "/dev/ttyACM*",        # Linux, USB CDC
        "/dev/ttyUSB*",        # Linux, USB serial bridges
    ]
    found: List[str] = []
    for pattern in patterns:
        found.extend(sorted(glob.glob(pattern)))
    if platform.system() == "Windows":
        found.extend(f"COM{i}" for i in range(1, 33))
    return found


def resolve_serial_port(port: str) -> str:
    """Turn the configured port into a concrete device path.

    "auto" picks the first plausible device. That is a convenience for a
    desk with one AP plugged in; pin the path in the config once you
    know it, and run `python -m hardware_test.find_ap` to confirm which
    device actually answers.
    """
    if port != "auto":
        return port

    candidates = candidate_serial_ports()
    if not candidates:
        raise RuntimeError(
            "actuation.serial_port is 'auto' but no serial device was found.\n"
            "  Plug in the Xiao AP, or set an explicit path in the config.\n"
            "  Run `python -m hardware_test.find_ap` to list what is connected."
        )
    chosen = candidates[0]
    if len(candidates) > 1:
        print(
            f"[SerialSender] serial_port=auto matched {len(candidates)} devices "
            f"{candidates}, using {chosen}. Pin it in the config to be sure."
        )
    else:
        print(f"[SerialSender] serial_port=auto resolved to {chosen}")
    return chosen


class SerialActionSender:
    """
    Multi-robot action sender for the Xiao ESP-NOW serial protocol.

    Owns only the actuation side. Observation / discovery are intentionally
    kept outside so the localization stack can change independently.
    """

    def __init__(self, config: SerialActionConfig) -> None:
        self._config = config
        self._running = False

        self._ser: Optional[serial.Serial] = None
        self._serial_lock = threading.Lock()

        self._actions: Dict[int, Action] = {
            rid: Action.stop() for rid in config.initial_robot_ids
        }
        self._active_robot_ids: set = set(config.initial_robot_ids)
        self._action_lock = threading.Lock()

        self._thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> bool:
        if self._running:
            return True

        try:
            self._ser = serial.Serial(
                port=self._config.port,
                baudrate=self._config.baudrate,
                timeout=0.1,
                write_timeout=0.1,
            )
            print(f"[SerialSender] Port {self._config.port} opened at {self._config.baudrate}")
        except serial.SerialException as exc:
            print(f"[SerialSender] Error opening port: {exc}")
            return False

        self._running = True

        self._thread = threading.Thread(target=self._send_loop, daemon=True)
        self._thread.start()

        print(f"[SerialSender] Started at {self._config.send_hz} Hz")
        return True

    def stop(self) -> None:
        if not self._running:
            return

        self.stop_all()
        self._running = False

        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

        if self._ser is not None and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

        print("[SerialSender] Stopped")

    def set_action(self, robot_id: int, action: Action) -> None:
        with self._action_lock:
            self._active_robot_ids.add(robot_id)
            self._actions[robot_id] = action

    def set_actions(self, actions: Dict[int, Action]) -> None:
        with self._action_lock:
            for robot_id, action in actions.items():
                self._active_robot_ids.add(robot_id)
                self._actions[robot_id] = action

    def stop_all(self) -> None:
        with self._action_lock:
            stop_snapshot: Dict[int, Action] = {}
            for rid in self._active_robot_ids:
                stop_action = Action.stop()
                self._actions[rid] = stop_action
                stop_snapshot[rid] = stop_action

        self._send_actions_snapshot(stop_snapshot)

    def add_robot(self, robot_id: int) -> None:
        with self._action_lock:
            self._active_robot_ids.add(robot_id)
            self._actions.setdefault(robot_id, Action.stop())

    def remove_robot(self, robot_id: int) -> None:
        with self._action_lock:
            self._active_robot_ids.discard(robot_id)
            self._actions.pop(robot_id, None)

    # ------------------------------------------------------------------
    # Background: send loop
    # ------------------------------------------------------------------

    def _send_loop(self) -> None:
        period = 1.0 / max(1.0, self._config.send_hz)
        while self._running:
            loop_start = time.perf_counter()

            with self._action_lock:
                snapshot = {
                    rid: act
                    for rid, act in self._actions.items()
                    if rid in self._active_robot_ids
                }

            if snapshot:
                self._send_actions_snapshot(snapshot)

            elapsed = time.perf_counter() - loop_start
            remaining = period - elapsed
            if remaining > 0:
                time.sleep(remaining)

    def _send_actions_snapshot(self, actions: Dict[int, Action]) -> None:
        frames = self._build_level_frames(actions.items())
        for frame in frames.values():
            self._write_frame(frame)

    # ------------------------------------------------------------------
    # Protocol: ESP-NOW frame building
    # ------------------------------------------------------------------

    def _build_level_frames(self, actions: Iterable[tuple[int, Action]]) -> Dict[int, bytes]:
        payloads: Dict[int, bytearray] = {}

        for robot_id, action in actions:
            level, left_pos, right_pos = self._level_and_slots(robot_id)
            payload = payloads.setdefault(level, bytearray(PAYLOAD_LEN))
            payload[left_pos] = self._encode_action_value(
                -action.left_speed if self._config.invert_left_wheel else action.left_speed
            )
            payload[right_pos] = self._encode_action_value(
                -action.right_speed if self._config.invert_right_wheel else action.right_speed
            )

        return {
            level: self._build_frame(level, payload)
            for level, payload in payloads.items()
        }

    def _level_and_slots(self, robot_id: int) -> tuple[int, int, int]:
        if robot_id <= 0:
            raise ValueError("robot_id must be >= 1")

        level = (robot_id - 1) // self._config.cars_per_level
        car_index = robot_id - level * self._config.cars_per_level  # 1..10

        right_pos = 3 * car_index
        left_pos = right_pos - 1

        if not (0 <= left_pos < PAYLOAD_LEN and 0 <= right_pos < PAYLOAD_LEN):
            raise ValueError(f"robot_id={robot_id} maps out of payload range")

        return level, left_pos, right_pos

    @staticmethod
    def _encode_action_value(speed: float) -> int:
        speed = max(-1.0, min(1.0, float(speed)))
        magnitude = min(int(round(abs(speed) * FULL_THRUST)), FULL_THRUST)
        if speed < 0:
            return 0x80 | magnitude
        return magnitude

    @staticmethod
    def _build_frame(level: int, payload: bytes | bytearray) -> bytes:
        if len(payload) != PAYLOAD_LEN:
            raise ValueError(f"payload must be exactly {PAYLOAD_LEN} bytes")

        frame = bytearray()
        frame.append(HDR0)
        frame.append(HDR1)
        frame.append(level & 0xFF)
        frame.append(PAYLOAD_LEN)
        frame.extend(payload)

        checksum = 0
        checksum ^= level & 0xFF
        checksum ^= PAYLOAD_LEN
        for b in payload:
            checksum ^= b
        frame.append(checksum & 0xFF)
        return bytes(frame)

    def _write_frame(self, frame: bytes) -> bool:
        if self._ser is None or not self._ser.is_open:
            return False

        with self._serial_lock:
            try:
                self._ser.write(frame)
                return True
            except serial.SerialException as exc:
                print(f"[SerialSender] Serial write failed: {exc}")
                return False
