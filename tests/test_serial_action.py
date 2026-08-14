"""
Unit tests for the Xiao ESP-NOW serial action sender.

Tests cover:
- robot_id -> level/slot mapping
- frame header / payload / checksum construction
- sign-magnitude encoding
- wheel invert behaviour
- multi-robot / multi-level packing
"""
import pytest

from micromvp.core.models import Action
from micromvp.env.real_env.serial_action import (
    FULL_THRUST,
    HDR0,
    HDR1,
    PAYLOAD_LEN,
    SerialActionConfig,
    SerialActionSender,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_sender(**overrides) -> SerialActionSender:
    defaults = dict(port="/dev/null", baudrate=115200)
    defaults.update(overrides)
    return SerialActionSender(SerialActionConfig(**defaults))


def _xor_checksum(level: int, payload: bytes) -> int:
    ck = level & 0xFF
    ck ^= PAYLOAD_LEN
    for b in payload:
        ck ^= b
    return ck & 0xFF


# ---------------------------------------------------------------------------
# level_and_slots mapping
# ---------------------------------------------------------------------------

class TestLevelAndSlots:
    def test_car_1_level_0(self):
        s = _make_sender()
        level, left, right = s._level_and_slots(1)
        assert level == 0
        assert right == 3
        assert left == 2

    def test_car_10_level_0(self):
        s = _make_sender(cars_per_level=10)
        level, left, right = s._level_and_slots(10)
        assert level == 0
        assert right == 30
        assert left == 29

    def test_car_11_level_1(self):
        s = _make_sender(cars_per_level=10)
        level, left, right = s._level_and_slots(11)
        assert level == 1
        assert right == 3
        assert left == 2

    def test_car_6_default(self):
        s = _make_sender()
        level, left, right = s._level_and_slots(6)
        assert level == 0
        assert right == 18
        assert left == 17

    def test_invalid_robot_id_zero(self):
        s = _make_sender()
        with pytest.raises(ValueError):
            s._level_and_slots(0)

    def test_invalid_robot_id_negative(self):
        s = _make_sender()
        with pytest.raises(ValueError):
            s._level_and_slots(-1)

    def test_overflow_robot_id(self):
        """car_index=11 within a level -> right_pos=33 > PAYLOAD_LEN=32."""
        s = _make_sender(cars_per_level=20)
        with pytest.raises(ValueError):
            s._level_and_slots(11)


# ---------------------------------------------------------------------------
# Sign-magnitude encoding
# ---------------------------------------------------------------------------

class TestEncodeActionValue:
    def test_zero(self):
        assert SerialActionSender._encode_action_value(0.0) == 0

    def test_full_positive(self):
        assert SerialActionSender._encode_action_value(1.0) == FULL_THRUST

    def test_full_negative(self):
        assert SerialActionSender._encode_action_value(-1.0) == 0x80 | FULL_THRUST

    def test_half_positive(self):
        val = SerialActionSender._encode_action_value(0.5)
        assert 0 < val < FULL_THRUST
        assert val & 0x80 == 0

    def test_half_negative(self):
        val = SerialActionSender._encode_action_value(-0.5)
        assert val & 0x80 == 0x80
        magnitude = val & 0x7F
        assert 0 < magnitude < FULL_THRUST

    def test_clamp_above_one(self):
        assert SerialActionSender._encode_action_value(2.0) == FULL_THRUST

    def test_clamp_below_neg_one(self):
        assert SerialActionSender._encode_action_value(-2.0) == 0x80 | FULL_THRUST


# ---------------------------------------------------------------------------
# Frame building
# ---------------------------------------------------------------------------

class TestBuildFrame:
    def test_header_and_length(self):
        payload = bytes(PAYLOAD_LEN)
        frame = SerialActionSender._build_frame(0, payload)
        assert frame[0] == HDR0
        assert frame[1] == HDR1
        assert frame[2] == 0  # level
        assert frame[3] == PAYLOAD_LEN
        assert len(frame) == 4 + PAYLOAD_LEN + 1  # hdr(4) + payload + checksum(1)

    def test_checksum_all_zeros(self):
        payload = bytes(PAYLOAD_LEN)
        frame = SerialActionSender._build_frame(0, payload)
        expected_ck = _xor_checksum(0, payload)
        assert frame[-1] == expected_ck

    def test_checksum_nonzero(self):
        payload = bytearray(PAYLOAD_LEN)
        payload[2] = 0x7F
        payload[3] = 0x80 | 0x3F
        frame = SerialActionSender._build_frame(1, payload)
        expected_ck = _xor_checksum(1, payload)
        assert frame[-1] == expected_ck

    def test_payload_length_mismatch(self):
        with pytest.raises(ValueError):
            SerialActionSender._build_frame(0, bytes(16))

    def test_level_byte_masking(self):
        payload = bytes(PAYLOAD_LEN)
        frame = SerialActionSender._build_frame(256, payload)
        assert frame[2] == 0  # 256 & 0xFF == 0


# ---------------------------------------------------------------------------
# Wheel invert
# ---------------------------------------------------------------------------

class TestWheelInvert:
    def test_no_invert(self):
        s = _make_sender(invert_left_wheel=False, invert_right_wheel=False)
        action = Action(left_speed=1.0, right_speed=1.0)
        frames = s._build_level_frames([(1, action)])
        frame = frames[0]
        payload = frame[4:4 + PAYLOAD_LEN]
        left_byte = payload[2]
        right_byte = payload[3]
        assert left_byte == FULL_THRUST
        assert right_byte == FULL_THRUST

    def test_invert_left(self):
        s = _make_sender(invert_left_wheel=True, invert_right_wheel=False)
        action = Action(left_speed=1.0, right_speed=1.0)
        frames = s._build_level_frames([(1, action)])
        frame = frames[0]
        payload = frame[4:4 + PAYLOAD_LEN]
        left_byte = payload[2]
        right_byte = payload[3]
        assert left_byte == (0x80 | FULL_THRUST)
        assert right_byte == FULL_THRUST

    def test_invert_right(self):
        s = _make_sender(invert_left_wheel=False, invert_right_wheel=True)
        action = Action(left_speed=1.0, right_speed=1.0)
        frames = s._build_level_frames([(1, action)])
        frame = frames[0]
        payload = frame[4:4 + PAYLOAD_LEN]
        left_byte = payload[2]
        right_byte = payload[3]
        assert left_byte == FULL_THRUST
        assert right_byte == (0x80 | FULL_THRUST)

    def test_invert_both(self):
        s = _make_sender(invert_left_wheel=True, invert_right_wheel=True)
        action = Action(left_speed=1.0, right_speed=1.0)
        frames = s._build_level_frames([(1, action)])
        frame = frames[0]
        payload = frame[4:4 + PAYLOAD_LEN]
        assert payload[2] == (0x80 | FULL_THRUST)
        assert payload[3] == (0x80 | FULL_THRUST)


# ---------------------------------------------------------------------------
# Multi-robot / multi-level packing
# ---------------------------------------------------------------------------

class TestMultiRobot:
    def test_two_robots_same_level(self):
        s = _make_sender(cars_per_level=10)
        actions = [
            (1, Action(left_speed=1.0, right_speed=0.5)),
            (2, Action(left_speed=-0.5, right_speed=-1.0)),
        ]
        frames = s._build_level_frames(actions)
        assert len(frames) == 1
        assert 0 in frames

    def test_two_robots_different_levels(self):
        s = _make_sender(cars_per_level=10)
        actions = [
            (1, Action(left_speed=1.0, right_speed=0.5)),
            (11, Action(left_speed=-0.5, right_speed=-1.0)),
        ]
        frames = s._build_level_frames(actions)
        assert len(frames) == 2
        assert 0 in frames
        assert 1 in frames

    def test_stop_action_yields_zero_bytes(self):
        s = _make_sender()
        action = Action.stop()
        frames = s._build_level_frames([(1, action)])
        frame = frames[0]
        payload = frame[4:4 + PAYLOAD_LEN]
        assert payload[2] == 0
        assert payload[3] == 0

    def test_all_slots_independent(self):
        """Robot 1 and robot 3 should not interfere with each other's slots."""
        s = _make_sender()
        actions = [
            (1, Action(left_speed=1.0, right_speed=1.0)),
            (3, Action(left_speed=-1.0, right_speed=-1.0)),
        ]
        frames = s._build_level_frames(actions)
        frame = frames[0]
        payload = frame[4:4 + PAYLOAD_LEN]

        _, l1, r1 = s._level_and_slots(1)
        _, l3, r3 = s._level_and_slots(3)

        assert payload[l1] == FULL_THRUST
        assert payload[r1] == FULL_THRUST
        assert payload[l3] == (0x80 | FULL_THRUST)
        assert payload[r3] == (0x80 | FULL_THRUST)

        untouched_indices = set(range(PAYLOAD_LEN)) - {l1, r1, l3, r3}
        for i in untouched_indices:
            assert payload[i] == 0, f"payload[{i}] should be 0 but got {payload[i]}"
