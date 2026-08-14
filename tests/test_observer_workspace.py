"""
Unit tests for the workspace lock state machine and aggregation logic.

All tests are pure algorithm tests – no camera or hardware required.
"""
import time

import numpy as np
import pytest

from micromvp.env.real_env.observer import (
    ArucoObserver,
    ObserverConfig,
    WorkspaceEstimate,
    _WorkspaceLockState,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_ws(
    width: float = 40.0,
    height: float = 60.0,
    origin: tuple = (0.0, 0.0, 0.5),
    normal: tuple = (0.0, 0.0, 1.0),
    x_axis: tuple = (1.0, 0.0, 0.0),
    y_axis: tuple = (0.0, 1.0, 0.0),
) -> WorkspaceEstimate:
    return WorkspaceEstimate(
        ready=True,
        width_cm=width,
        height_cm=height,
        origin_cam_m=origin,
        normal_cam=normal,
        x_axis_cam=x_axis,
        y_axis_cam=y_axis,
        timestamp=time.time(),
    )


def _default_config(**overrides) -> ObserverConfig:
    defaults = dict(
        workspace_lock_frames=5,
        workspace_width_tolerance_cm=2.0,
        workspace_height_tolerance_cm=2.0,
        workspace_origin_tolerance_m=0.01,
        workspace_normal_angle_tolerance_deg=3.0,
    )
    defaults.update(overrides)
    return ObserverConfig(**defaults)


# ---------------------------------------------------------------------------
# Window not full -> cannot lock
# ---------------------------------------------------------------------------

class TestWindowNotFull:
    def test_single_candidate_not_lockable(self):
        state = _WorkspaceLockState(required_frames=5)
        state.add_candidate(_make_ws())
        assert not state.window_full
        assert state.state == "collecting"

    def test_four_of_five_not_lockable(self):
        state = _WorkspaceLockState(required_frames=5)
        for _ in range(4):
            state.add_candidate(_make_ws())
        assert not state.window_full
        assert not state.check_stability(_default_config())


# ---------------------------------------------------------------------------
# Stable window -> lock
# ---------------------------------------------------------------------------

class TestStableLock:
    def test_lock_after_n_stable_frames(self):
        n = 5
        state = _WorkspaceLockState(required_frames=n)
        cfg = _default_config(workspace_lock_frames=n)
        for _ in range(n):
            state.add_candidate(_make_ws())
        assert state.window_full
        assert state.check_stability(cfg)

        result = state.aggregate()
        state.state = "locked"

        assert state.is_locked
        assert result.ready
        assert abs(result.width_cm - 40.0) < 0.01
        assert abs(result.height_cm - 60.0) < 0.01

    def test_not_ready_candidate_resets_window(self):
        state = _WorkspaceLockState(required_frames=5)
        for _ in range(3):
            state.add_candidate(_make_ws())
        assert len(state.candidates) == 3

        state.add_candidate(WorkspaceEstimate())  # not ready
        assert len(state.candidates) == 0
        assert state.state == "collecting"

    def test_locked_state_ignores_new_candidates(self):
        state = _WorkspaceLockState(required_frames=2)
        for _ in range(2):
            state.add_candidate(_make_ws())
        state.state = "locked"

        state.add_candidate(_make_ws(width=99.0))
        assert len(state.candidates) == 2  # unchanged


# ---------------------------------------------------------------------------
# Unstable window -> no lock
# ---------------------------------------------------------------------------

class TestUnstableNoLock:
    def test_width_variation_too_large(self):
        state = _WorkspaceLockState(required_frames=5)
        cfg = _default_config(workspace_width_tolerance_cm=1.0)
        for i in range(5):
            state.add_candidate(_make_ws(width=40.0 + i * 0.5))
        assert state.window_full
        assert not state.check_stability(cfg)

    def test_height_variation_too_large(self):
        state = _WorkspaceLockState(required_frames=5)
        cfg = _default_config(workspace_height_tolerance_cm=1.0)
        for i in range(5):
            state.add_candidate(_make_ws(height=60.0 + i * 0.5))
        assert state.window_full
        assert not state.check_stability(cfg)

    def test_origin_drift_too_large(self):
        state = _WorkspaceLockState(required_frames=3)
        cfg = _default_config(
            workspace_lock_frames=3, workspace_origin_tolerance_m=0.005
        )
        origins = [(0.0, 0.0, 0.5), (0.01, 0.0, 0.5), (0.0, 0.01, 0.5)]
        for o in origins:
            state.add_candidate(_make_ws(origin=o))
        assert state.window_full
        assert not state.check_stability(cfg)

    def test_normal_angle_too_large(self):
        state = _WorkspaceLockState(required_frames=3)
        cfg = _default_config(
            workspace_lock_frames=3, workspace_normal_angle_tolerance_deg=1.0
        )
        angle_rad = np.radians(5.0)
        normals = [
            (0.0, 0.0, 1.0),
            (np.sin(angle_rad), 0.0, np.cos(angle_rad)),
            (0.0, np.sin(angle_rad), np.cos(angle_rad)),
        ]
        for n in normals:
            state.add_candidate(_make_ws(normal=n))
        assert state.window_full
        assert not state.check_stability(cfg)


# ---------------------------------------------------------------------------
# Aggregation correctness
# ---------------------------------------------------------------------------

class TestAggregation:
    def test_median_scalars(self):
        state = _WorkspaceLockState(required_frames=5)
        widths = [39.0, 40.0, 40.5, 41.0, 38.5]
        for w in widths:
            state.add_candidate(_make_ws(width=w))

        result = state.aggregate()
        assert abs(result.width_cm - np.median(widths)) < 0.01

    def test_mean_origin(self):
        state = _WorkspaceLockState(required_frames=3)
        origins = [(0.01, 0.02, 0.50), (0.02, 0.01, 0.51), (0.015, 0.015, 0.505)]
        for o in origins:
            state.add_candidate(_make_ws(origin=o))

        result = state.aggregate()
        expected = np.mean(np.array(origins), axis=0)
        actual = np.array(result.origin_cam_m)
        np.testing.assert_allclose(actual, expected, atol=1e-6)

    def test_normalized_normal(self):
        state = _WorkspaceLockState(required_frames=3)
        normals = [
            (0.01, 0.0, 1.0),
            (-0.01, 0.0, 1.0),
            (0.0, 0.01, 1.0),
        ]
        for n in normals:
            state.add_candidate(_make_ws(normal=n))

        result = state.aggregate()
        norm = np.linalg.norm(result.normal_cam)
        assert abs(norm - 1.0) < 1e-6


class TestMarkerOffsetTransform:
    def test_marker_local_forward_stays_aligned_with_yaw(self):
        offset = np.array([0.0, 1.8], dtype=np.float64)

        out_x = ArucoObserver._marker_xy_to_workspace_xy(offset, 0.0)
        out_y = ArucoObserver._marker_xy_to_workspace_xy(offset, 90.0)
        out_left = ArucoObserver._marker_xy_to_workspace_xy(offset, 180.0)

        np.testing.assert_allclose(out_x, np.array([1.8, 0.0]), atol=1e-6)
        np.testing.assert_allclose(out_y, np.array([0.0, 1.8]), atol=1e-6)
        np.testing.assert_allclose(out_left, np.array([-1.8, 0.0]), atol=1e-6)

    def test_marker_local_right_tracks_car_right_side(self):
        offset = np.array([1.0, 0.0], dtype=np.float64)

        out_x = ArucoObserver._marker_xy_to_workspace_xy(offset, 0.0)
        out_y = ArucoObserver._marker_xy_to_workspace_xy(offset, 90.0)

        np.testing.assert_allclose(out_x, np.array([0.0, -1.0]), atol=1e-6)
        np.testing.assert_allclose(out_y, np.array([1.0, 0.0]), atol=1e-6)
