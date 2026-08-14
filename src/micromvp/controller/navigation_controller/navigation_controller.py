"""
Navigation Controller (v2) - Pure Pursuit + CTE-PD path following
with in-place rotation support.

What’s new vs your old NavigationController:
- Path following upgraded to PurePursuit + Cross-Track-Error PD controller
- Path resampling (max gap <= max_point_gap_ratio * car_size)
- Prefix arc-length + no-skip forward window gating (prevents jumping at self-intersections)
- Optional rotate-in-place capture when far from path + facing away (during ACQUIRE)
- Keeps your original public API shape:
    - set_path(path)
    - clear_path()
    - rotate_to(theta, on_done=None)
    - cancel_rotation()
    - step(observation) -> Action
    - reset()
    - set_speed(speed)

Rotation behavior stays the same:
- Rotates in-place until |error| <= ROTATION_TOLERANCE_DEG
- Must remain stable for ROTATION_STABLE_TIME seconds

Axis convention:
- X: right
- Y: up
- 0 deg: +X
- deg increases CCW (+Y is 90 deg)
"""
from __future__ import annotations

import math
import time
from typing import List, Optional, Tuple
from enum import Enum, auto

from micromvp.config import Config
from micromvp.controller.base import Controller
from micromvp.core.models import (
    Action,
    RobotObservation,
    WorkspaceConfig,
)

Point = Tuple[float, float]


# ---------------------------
# Utils
# ---------------------------

def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def _wrap_to_pi(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad <= -math.pi:
        rad += 2.0 * math.pi
    return rad


def _project_point_to_segment(p: Point, a: Point, b: Point) -> Tuple[Point, float]:
    ax, ay = a
    bx, by = b
    px, py = p

    vx = bx - ax
    vy = by - ay
    denom = vx * vx + vy * vy
    if denom <= 1e-12:
        return a, 0.0

    t = ((px - ax) * vx + (py - ay) * vy) / denom
    t = _clamp(t, 0.0, 1.0)
    return (ax + t * vx, ay + t * vy), t


# ---------------------------
# Navigation state machine
# ---------------------------

class NavigationState(Enum):
    IDLE = auto()
    FOLLOWING = auto()
    FINISHED_PATH = auto()
    ROTATING = auto()
    ROTATION_STABLE = auto()
    ROTATION_DONE = auto()


class NavigationController(Controller):
    """
    Navigation controller with:
    1) Pure Pursuit + CTE-PD follow path (no-skip arc-length gated)
    2) In-place rotation to target orientation (stable-hold completion)

    Status labels:
    - "IDLE"
    - "FOLLOWING"
    - "FINISHED"
    - "ROTATING"
    - "ROTATION_STABLE"
    - "ROTATION_DONE"
    """

    # -------- Rotation parameters --------
    # Defaults for direct construction; from_config() overrides them per
    # instance from the deployment YAML.
    ROTATION_TOLERANCE_DEG = 2.5    # within +-2.5 deg
    ROTATION_STABLE_TIME = 0.5      # seconds in tolerance to finish
    ROTATION_SPEED_MAX = 0.25
    ROTATION_SPEED_MIN = 0.15

    @classmethod
    def from_config(
        cls,
        robot_id: int,
        ws_config: WorkspaceConfig,
        cfg: "Config",
    ) -> "NavigationController":
        """Build from the deployment YAML (the `control` section)."""
        who = "NavigationController"
        controller = cls(
            robot_id,
            ws_config,
            lookahead_distance=cfg.require("control.lookahead_cm", who=who),
            max_speed=cfg.require("control.max_speed", float, who=who),
            goal_tolerance=cfg.require("control.goal_tolerance_cm", who=who),
            max_point_gap_ratio=cfg.require("control.max_point_gap_ratio", float, who=who),
            no_skip_ratio=cfg.require("control.no_skip_ratio", float, who=who),
        )
        controller.ROTATION_TOLERANCE_DEG = cfg.require(
            "control.rotation.tolerance_deg", float, who=who
        )
        controller.ROTATION_STABLE_TIME = cfg.require(
            "control.rotation.stable_time_sec", float, who=who
        )
        controller.ROTATION_SPEED_MAX = cfg.require(
            "control.rotation.speed_max", float, who=who
        )
        controller.ROTATION_SPEED_MIN = cfg.require(
            "control.rotation.speed_min", float, who=who
        )
        return controller

    def __init__(
        self,
        robot_id: int,
        ws_config: WorkspaceConfig,
        lookahead_distance: Optional[float] = None,
        max_speed: float = 0.3,
        goal_tolerance: Optional[float] = None,
        # ---- PP+PD knobs (car_size-based, unit-insensitive) ----
        max_point_gap_ratio: float = 0.01,
        no_skip_ratio: float = 0.5,
    ) -> None:
        super().__init__(robot_id, ws_config)

        self.car_size = max(ws_config.car_width, ws_config.car_height)

        # Path-following params
        self._max_point_gap_ratio = max(1e-4, float(max_point_gap_ratio))
        self._no_skip_ratio = max(0.1, float(no_skip_ratio))

        self._lookahead_distance = lookahead_distance or (0.5 * self.car_size)
        self._max_speed = min(1.0, max(0.0, max_speed))
        self._goal_tolerance = goal_tolerance or (0.2 * self.car_size)

        # CTE-PD knobs
        self._cte_dot_alpha = 0.25
        self._min_turn_factor = 0.25

        # Path state
        self._path_raw: List[Point] = []
        self._path: List[Point] = []
        self._path_s: List[float] = []   # prefix arc-length
        self._path_index: int = 0

        # Timestamp/velocity bookkeeping
        self._prev_timestamp: Optional[float] = None

        # PD state
        self._prev_cte: Optional[float] = None
        self._prev_cte_time: Optional[float] = None
        self._cte_dot_filt: float = 0.0

        # Rotation state
        self._target_theta: Optional[float] = None
        self._rotation_stable_start: Optional[float] = None
        self._on_rotation_done_callback: Optional[callable] = None


        # Nav state
        self._nav_state = NavigationState.IDLE
        self._car_state.status_label = "IDLE"

        # ---- ALIGN behavior ----
        self._align_enter_deg = 60.0
        self._align_exit_deg = 45.0   # 滞回：进入60，退出45（更稳），你也可以改成同为60
        self._align_active = False

        # ---- deadband ----
        self._wheel_deadband = 0.05

    # ---------------------------
    # Properties
    # ---------------------------

    @property
    def path(self) -> List[Point]:
        return self._path.copy()

    @property
    def path_index(self) -> int:
        return self._path_index

    @property
    def lookahead_distance(self) -> float:
        return self._lookahead_distance

    @property
    def max_speed(self) -> float:
        return self._max_speed

    @property
    def is_following_path(self) -> bool:
        return self._nav_state == NavigationState.FOLLOWING

    @property
    def is_rotating(self) -> bool:
        return self._nav_state in (NavigationState.ROTATING, NavigationState.ROTATION_STABLE)

    @property
    def is_busy(self) -> bool:
        return self._nav_state not in (
            NavigationState.IDLE,
            NavigationState.FINISHED_PATH,
            NavigationState.ROTATION_DONE,
        )

    # ---------------------------
    # Path preprocessing
    # ---------------------------

    def _resample_path(self, path: List[Point], max_step: float) -> List[Point]:
        if len(path) <= 1:
            return list(path)

        out: List[Point] = []
        for i in range(len(path) - 1):
            x0, y0 = path[i]
            x1, y1 = path[i + 1]
            dx = x1 - x0
            dy = y1 - y0
            d = math.hypot(dx, dy)

            if d < 1e-12:
                continue

            n = int(math.ceil(d / max_step))

            if not out:
                out.append((x0, y0))

            for k in range(1, n + 1):
                t = k / n
                out.append((x0 + t * dx, y0 + t * dy))

        return out

    def _build_prefix_s(self) -> None:
        self._path_s = [0.0]
        for i in range(len(self._path) - 1):
            x0, y0 = self._path[i]
            x1, y1 = self._path[i + 1]
            self._path_s.append(self._path_s[-1] + math.hypot(x1 - x0, y1 - y0))

    def _max_reachable_index(self, start_idx: int) -> int:
        if not self._path_s:
            return start_idx

        start_idx = int(_clamp(float(start_idx), 0.0, float(len(self._path_s) - 1)))
        s0 = self._path_s[start_idx]
        s_max = s0 + self._no_skip_ratio * self.car_size

        j = start_idx
        n = len(self._path_s)
        while j + 1 < n and self._path_s[j + 1] <= s_max:
            j += 1
        return j

    # ---------------------------
    # Public API
    # ---------------------------

    def set_path(self, path: List[Point]) -> None:
        # cancel rotation if any
        self._target_theta = None
        self._rotation_stable_start = None
        self._on_rotation_done_callback = None
        self._car_state.metadata.pop("target_theta", None)

        self.car_size = max(self._ws_config.car_width, self._ws_config.car_height)

        self._path_raw = list(path)
        max_step = self._max_point_gap_ratio * self.car_size
        self._path = self._resample_path(self._path_raw, max_step)
        self._build_prefix_s()

        self._path_index = 0
        self._prev_cte = None
        self._prev_cte_time = None
        self._cte_dot_filt = 0.0

        if self._path:
            self._nav_state = NavigationState.FOLLOWING
            self._car_state.status_label = "FOLLOWING"
            self._car_state.metadata["path"] = self._path
            self._car_state.metadata["path_index"] = self._path_index
        else:
            self._nav_state = NavigationState.IDLE
            self._car_state.status_label = "IDLE"
            self._car_state.metadata.pop("path", None)
            self._car_state.metadata.pop("path_index", None)

        self._car_state.metadata.pop("target_point", None)

    def clear_path(self) -> None:
        self._path_raw = []
        self._path = []
        self._path_s = []
        self._path_index = 0

        self._prev_cte = None
        self._prev_cte_time = None
        self._cte_dot_filt = 0.0

        if self._nav_state not in (NavigationState.ROTATING, NavigationState.ROTATION_STABLE):
            self._nav_state = NavigationState.IDLE
            self._car_state.status_label = "IDLE"

        self._car_state.metadata.pop("path", None)
        self._car_state.metadata.pop("path_index", None)
        self._car_state.metadata.pop("target_point", None)

    def rotate_to(self, target_theta: float, on_done: Optional[callable] = None) -> None:
        # Normalize [0, 360)
        self._target_theta = target_theta % 360.0
        self._rotation_stable_start = None
        self._on_rotation_done_callback = on_done

        # clear path task
        self._path_raw = []
        self._path = []
        self._path_s = []
        self._path_index = 0
        self._car_state.metadata.pop("path", None)
        self._car_state.metadata.pop("path_index", None)
        self._car_state.metadata.pop("target_point", None)

        # reset PD state
        self._prev_cte = None
        self._prev_cte_time = None
        self._cte_dot_filt = 0.0

        self._nav_state = NavigationState.ROTATING
        self._car_state.status_label = "ROTATING"
        self._car_state.metadata["target_theta"] = self._target_theta

    def cancel_rotation(self) -> None:
        self._target_theta = None
        self._rotation_stable_start = None
        self._on_rotation_done_callback = None

        if self._path:
            self._nav_state = NavigationState.FOLLOWING
            self._car_state.status_label = "FOLLOWING"
        else:
            self._nav_state = NavigationState.IDLE
            self._car_state.status_label = "IDLE"

        self._car_state.metadata.pop("target_theta", None)

    def step(self, observation: RobotObservation) -> Action:
        self.update(observation)
        return self.calculate_action()

    def update(self, observation: RobotObservation) -> None:
        prev_x = self._car_state.x
        prev_y = self._car_state.y
        prev_theta = self._car_state.theta
        prev_time = self._prev_timestamp

        self._car_state.x = observation.x
        self._car_state.y = observation.y
        self._car_state.theta = observation.theta  # degrees

        if prev_time is not None and observation.timestamp > prev_time:
            dt = observation.timestamp - prev_time
            if dt > 0:
                dx = observation.x - prev_x
                dy = observation.y - prev_y
                distance = math.hypot(dx, dy)
                self._car_state.linear_velocity = distance / dt

                dtheta = observation.theta - prev_theta
                if dtheta > 180:
                    dtheta -= 360
                elif dtheta < -180:
                    dtheta += 360
                self._car_state.angular_velocity = dtheta / dt

        self._prev_timestamp = observation.timestamp
        self._last_observation = observation

    # ---------------------------
    # Main dispatch
    # ---------------------------

    def calculate_action(self) -> Action:
        if self._nav_state in (NavigationState.ROTATING, NavigationState.ROTATION_STABLE):
            return self._calculate_rotation_action()

        if self._nav_state == NavigationState.FOLLOWING:
            return self._calculate_path_action()

        return Action.stop()

    # ---------------------------
    # Rotation control
    # ---------------------------

    def _calculate_rotation_action(self) -> Action:
        if self._target_theta is None:
            self._nav_state = NavigationState.IDLE
            self._car_state.status_label = "IDLE"
            self._car_state.metadata.pop("target_theta", None)
            return Action.stop()

        current_theta = self._car_state.theta
        error = self._target_theta - current_theta

        while error > 180:
            error -= 360
        while error < -180:
            error += 360

        if abs(error) <= self.ROTATION_TOLERANCE_DEG:
            now = time.time()

            if self._rotation_stable_start is None:
                self._rotation_stable_start = now
                self._nav_state = NavigationState.ROTATION_STABLE
                self._car_state.status_label = "ROTATION_STABLE"

            elif now - self._rotation_stable_start >= self.ROTATION_STABLE_TIME:
                self._nav_state = NavigationState.ROTATION_DONE
                self._car_state.status_label = "ROTATION_DONE"
                self._car_state.metadata.pop("target_theta", None)

                if self._on_rotation_done_callback:
                    cb = self._on_rotation_done_callback
                    self._on_rotation_done_callback = None
                    cb()

                return Action.stop()

            return Action.stop()

        self._rotation_stable_start = None
        self._nav_state = NavigationState.ROTATING
        self._car_state.status_label = "ROTATING"

        abs_error = abs(error)
        if abs_error > 45.0:
            speed = self.ROTATION_SPEED_MAX
        elif abs_error > 15.0:
            t = (abs_error - 15.0) / 30.0
            speed = self.ROTATION_SPEED_MIN + t * (self.ROTATION_SPEED_MAX - self.ROTATION_SPEED_MIN)
        else:
            speed = self.ROTATION_SPEED_MIN

        if error > 0:
            return Action(left_speed=-speed, right_speed=speed)
        return Action(left_speed=speed, right_speed=-speed)

    def _enforce_deadband_scale(self, vl: float, vr: float) -> Tuple[float, float]:
        db = self._wheel_deadband

        max_abs = max(abs(vl), abs(vr))

        # 都非常小：认为就是停
        if max_abs < 1e-6:
            return 0.0, 0.0

        # 如果已经有一个轮子 >= db，直接返回
        if max_abs >= db:
            return vl, vr

        # 需要动但都没过deadband：整体缩放
        scale = db / max_abs
        vl2 = vl * scale
        vr2 = vr * scale

        # 仍然保证在 [-1,1]
        m2 = max(abs(vl2), abs(vr2))
        if m2 > 1.0:
            s2 = 1.0 / m2
            vl2 *= s2
            vr2 *= s2

        return vl2, vr2


    # ---------------------------
    # Path control (Pure Pursuit + CTE-PD)
    # ---------------------------

    def _calculate_path_action(self) -> Action:
        if not self._path:
            self._nav_state = NavigationState.IDLE
            self._car_state.status_label = "IDLE"
            return Action.stop()

        robot_x = self._car_state.x
        robot_y = self._car_state.y
        robot_theta = self._car_state.theta  # degrees

        self.car_size = max(self._ws_config.car_width, self._ws_config.car_height)
        wheel_base = self._ws_config.wheel_base

        final_point = self._path[-1]
        dist_to_goal = math.hypot(robot_x - final_point[0], robot_y - final_point[1])
        if dist_to_goal < self._goal_tolerance:
            self._nav_state = NavigationState.FINISHED_PATH
            self._car_state.status_label = "FINISHED"
            self._car_state.metadata.pop("target_point", None)
            return Action.stop()

        target_point = self._find_target_point(robot_x, robot_y)
        if target_point is None:
            target_point = final_point

        ref_point, theta_ref_rad, cte_raw, seg_idx = self._find_closest_reference((robot_x, robot_y))
        self._path_index = max(self._path_index, seg_idx)

        curvature_pp, heading_error_rad, dist_to_target = self._pure_pursuit_curvature(
            robot_x, robot_y, robot_theta, target_point
        )

        heading_deg = abs(math.degrees(heading_error_rad))

        # ALIGN hysteresis, when angle difference is too much, rotate in place
        if self._align_active:
            if heading_deg <= self._align_exit_deg:
                self._align_active = False
        else:
            if heading_deg >= self._align_enter_deg:
                self._align_active = True

        if self._align_active:
            # 原地转：用 heading_error 的符号决定方向
            # 用 max_speed 来缩放一个角速度（你也可以改成常量）
            w = 0.6 * self._max_speed
            left_speed = -w if heading_error_rad > 0 else w
            right_speed = w if heading_error_rad > 0 else -w

            # deadband缩放（你的规则）
            left_speed, right_speed = self._enforce_deadband_scale(left_speed, right_speed)

            self._car_state.metadata["mode"] = "ALIGN"
            self._car_state.metadata["heading_error_deg"] = math.degrees(heading_error_rad)
            self._car_state.metadata["target_point"] = target_point
            self._car_state.status_label = "FOLLOWING"
            self._nav_state = NavigationState.FOLLOWING
            return Action(
                left_speed=_clamp(left_speed, -1.0, 1.0),
                right_speed=_clamp(right_speed, -1.0, 1.0),
            )

        # normal PD following
        # PD gating and scaling (car_size-based)
        dist_to_path = abs(cte_raw)

        cte_pd_enable_dist = 2.5 * self.car_size
        cte_clip = 1.0 * self.car_size
        cte_deadband = 0.10 * self.car_size

        curv_pd_max = 2.0 / max(self.car_size, 1e-9)
        curvature_max = 3.5 / max(self.car_size, 1e-9)

        rotate_in_place_rad = math.radians(110.0)

        e01 = abs(heading_error_rad) / math.pi  # 0..1
        turn_factor = 1.0 - 0.8 * e01           # 你可以调0.8这个斜率
        turn_factor = _clamp(turn_factor, self._min_turn_factor, 1.0)
        base_speed = self._max_speed * turn_factor

        use_pd = dist_to_path <= cte_pd_enable_dist

        cte_used = cte_raw
        if abs(cte_used) < cte_deadband:
            cte_used = 0.0
        cte_used = _clamp(cte_used, -cte_clip, cte_clip)

        now_t = self._prev_timestamp
        if (not use_pd) or (now_t is None):
            self._prev_cte = None
            self._prev_cte_time = None
            self._cte_dot_filt = 0.0
            cte_dot_filt = 0.0
            cte_pd_curv = 0.0
        else:
            if self._prev_cte is None or self._prev_cte_time is None:
                cte_dot = 0.0
            else:
                dt = now_t - self._prev_cte_time
                cte_dot = (cte_used - self._prev_cte) / dt if dt > 1e-6 else 0.0

            self._cte_dot_filt = (1.0 - self._cte_dot_alpha) * self._cte_dot_filt + self._cte_dot_alpha * cte_dot
            cte_dot_filt = self._cte_dot_filt

            self._prev_cte = cte_used
            self._prev_cte_time = now_t

            kp = 1.2
            kd = 0.35

            cte_n = cte_used / max(self.car_size, 1e-9)
            cte_dot_n = cte_dot_filt / max(self.car_size, 1e-9)

            cte_pd_curv = (kp * cte_n + kd * cte_dot_n) / max(self.car_size, 1e-9)
            cte_pd_curv = _clamp(cte_pd_curv, -curv_pd_max, curv_pd_max)

        curvature_cmd = curvature_pp + cte_pd_curv
        curvature_cmd = _clamp(curvature_cmd, -curvature_max, curvature_max)

        curv_slow = _clamp(1.0 - 0.65 * (abs(curvature_cmd) / max(curvature_max, 1e-9)), 0.35, 1.0)
        base_speed *= curv_slow

        # Acquire mode: far from path and facing away => rotate in place a bit
        if (not use_pd) and (abs(heading_error_rad) > rotate_in_place_rad):
            w = 0.6 * self._max_speed
            left_speed = -w if heading_error_rad > 0 else w
            right_speed = w if heading_error_rad > 0 else -w

            self._car_state.metadata["mode"] = "ROTATE_IN_PLACE"
            self._car_state.metadata["pd_enabled"] = False
            self._car_state.metadata["cte"] = cte_raw
            self._car_state.metadata["cte_used"] = cte_used
            self._car_state.metadata["cte_dot_filt"] = 0.0
            self._car_state.metadata["curvature_pp"] = curvature_pp
            self._car_state.metadata["curvature_cmd"] = 0.0
            self._car_state.metadata["heading_error_deg"] = math.degrees(heading_error_rad)
            self._car_state.metadata["theta_ref_deg"] = math.degrees(theta_ref_rad)
            self._car_state.metadata["ref_point"] = ref_point
            self._car_state.metadata["dist_to_target"] = dist_to_target
            self._car_state.metadata["goal_tol"] = self._goal_tolerance
            self._car_state.metadata["no_skip_ratio"] = self._no_skip_ratio
            self._car_state.metadata["max_point_gap_ratio"] = self._max_point_gap_ratio
            self._car_state.metadata["cte_pd_enable_dist"] = cte_pd_enable_dist

            self._car_state.metadata["target_point"] = target_point
            self._car_state.metadata["path_index"] = self._path_index
            self._car_state.status_label = "FOLLOWING"
            self._nav_state = NavigationState.FOLLOWING

            return Action(
                left_speed=_clamp(left_speed, -1.0, 1.0),
                right_speed=_clamp(right_speed, -1.0, 1.0),
            )

        diff = curvature_cmd * wheel_base / 2.0
        left_speed = base_speed * (1.0 - diff)
        right_speed = base_speed * (1.0 + diff)

        max_wheel = max(abs(left_speed), abs(right_speed))
        if max_wheel > self._max_speed:
            scale = self._max_speed / max_wheel
            left_speed *= scale
            right_speed *= scale

        left_speed, right_speed = self._enforce_deadband_scale(left_speed, right_speed)

        left_speed = _clamp(left_speed, -1.0, 1.0)
        right_speed = _clamp(right_speed, -1.0, 1.0)

        # Metadata
        self._car_state.metadata["mode"] = "TRACK" if use_pd else "ACQUIRE"
        self._car_state.metadata["pd_enabled"] = use_pd
        self._car_state.metadata["cte"] = cte_raw
        self._car_state.metadata["cte_used"] = cte_used
        self._car_state.metadata["cte_dot_filt"] = cte_dot_filt if use_pd else 0.0
        self._car_state.metadata["curvature_pp"] = curvature_pp
        self._car_state.metadata["curvature_cmd"] = curvature_cmd
        self._car_state.metadata["heading_error_deg"] = math.degrees(heading_error_rad)
        self._car_state.metadata["theta_ref_deg"] = math.degrees(theta_ref_rad)
        self._car_state.metadata["ref_point"] = ref_point
        self._car_state.metadata["dist_to_target"] = dist_to_target
        self._car_state.metadata["goal_tol"] = self._goal_tolerance
        self._car_state.metadata["cte_pd_enable_dist"] = cte_pd_enable_dist
        self._car_state.metadata["curvature_max"] = curvature_max
        self._car_state.metadata["curv_pd_max"] = curv_pd_max
        self._car_state.metadata["no_skip_ratio"] = self._no_skip_ratio
        self._car_state.metadata["max_point_gap_ratio"] = self._max_point_gap_ratio

        self._car_state.metadata["target_point"] = target_point
        self._car_state.metadata["path_index"] = self._path_index

        self._car_state.status_label = "FOLLOWING"
        self._nav_state = NavigationState.FOLLOWING

        return Action(left_speed=left_speed, right_speed=right_speed)

    # ---------------------------
    # Gated target finding (no-skip)
    # ---------------------------

    def _find_target_point(self, robot_x: float, robot_y: float) -> Optional[Point]:
        if not self._path:
            return None

        start = self._path_index
        imax = self._max_reachable_index(start)

        best_idx = start
        best_dist = float("inf")

        for i in range(start, imax + 1):
            px, py = self._path[i]
            dist = math.hypot(robot_x - px, robot_y - py)
            if dist < best_dist:
                best_dist = dist
                best_idx = i

        self._path_index = max(self._path_index, best_idx)

        start = self._path_index
        imax = self._max_reachable_index(start)

        for i in range(start, imax + 1):
            px, py = self._path[i]
            dist = math.hypot(robot_x - px, robot_y - py)
            if dist >= self._lookahead_distance:
                self._path_index = i
                return (px, py)

        return self._path[min(imax, len(self._path) - 1)]

    # ---------------------------
    # Pure Pursuit curvature
    # ---------------------------

    def _pure_pursuit_curvature(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta_deg: float,
        target: Point,
    ) -> Tuple[float, float, float]:
        dx = target[0] - robot_x
        dy = target[1] - robot_y
        distance = math.hypot(dx, dy)
        if distance < 1e-9:
            return 0.0, 0.0, 0.0

        angle_to_target = math.atan2(dy, dx)
        robot_theta_rad = math.radians(robot_theta_deg)
        heading_error_rad = _wrap_to_pi(angle_to_target - robot_theta_rad)

        L = max(distance, self._lookahead_distance)
        curvature_pp = 2.0 * math.sin(heading_error_rad) / L
        return curvature_pp, heading_error_rad, distance

    # ---------------------------
    # Gated closest reference (no-skip) for CTE
    # ---------------------------

    def _find_closest_reference(self, pos: Point) -> Tuple[Point, float, float, int]:
        x, y = pos
        n = len(self._path)
        if n == 1:
            px, py = self._path[0]
            theta_ref = math.radians(self._car_state.theta)
            cte = math.hypot(x - px, y - py)
            return (px, py), theta_ref, cte, 0

        start_i = min(self._path_index, n - 2)
        end_i = min(self._max_reachable_index(start_i), n - 1)
        seg_end = max(start_i, min(end_i - 1, n - 2))

        best_dist2 = float("inf")
        best_proj: Point = self._path[start_i]
        best_theta = 0.0
        best_cte = 0.0
        best_i = start_i

        for i in range(start_i, seg_end + 1):
            a = self._path[i]
            b = self._path[i + 1]
            proj, _t = _project_point_to_segment((x, y), a, b)

            dxp = x - proj[0]
            dyp = y - proj[1]
            dist2 = dxp * dxp + dyp * dyp
            if dist2 < best_dist2:
                best_dist2 = dist2
                best_proj = proj
                best_i = i

                seg_dx = b[0] - a[0]
                seg_dy = b[1] - a[1]
                theta_ref = math.atan2(seg_dy, seg_dx)
                best_theta = theta_ref

                nx = -math.sin(theta_ref)
                ny = math.cos(theta_ref)
                best_cte = nx * (x - proj[0]) + ny * (y - proj[1])

        return best_proj, best_theta, best_cte, best_i

    # ---------------------------
    # Reset / speed
    # ---------------------------

    def reset(self) -> None:
        super().reset()

        self._path_raw = []
        self._path = []
        self._path_s = []
        self._path_index = 0
        self._prev_timestamp = None

        self._prev_cte = None
        self._prev_cte_time = None
        self._cte_dot_filt = 0.0

        self._target_theta = None
        self._rotation_stable_start = None
        self._on_rotation_done_callback = None

        self._nav_state = NavigationState.IDLE
        self._car_state.status_label = "IDLE"

        self._car_state.metadata.pop("path", None)
        self._car_state.metadata.pop("path_index", None)
        self._car_state.metadata.pop("target_point", None)
        self._car_state.metadata.pop("target_theta", None)

    def set_speed(self, speed: float) -> None:
        self._max_speed = max(0.0, min(1.0, speed))
