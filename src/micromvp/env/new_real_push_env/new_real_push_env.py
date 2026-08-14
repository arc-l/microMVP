"""
NewRealPushEnv – real hardware environment using:
- Adaptive workspace estimation (no fixed ground markers)
- Xiao ESP-NOW serial protocol for motor commands
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from typing import Dict, List

from micromvp.core.models import Action, RobotObservation, WorkspaceConfig
from micromvp.env.base import Environment

from .observer import ArucoObserver, ObserverConfig
from .serial_action import SerialActionConfig, SerialActionSender


_DEFAULT_CALIB = os.path.join(os.path.dirname(__file__), "camera.yaml")
_DEFAULT_OBSTACLE = os.path.join(os.path.dirname(__file__), "obstacle.json")


@dataclass
class NewRealPushConfig:
    # Workspace geometry (initial; will be overwritten by observer estimate)
    width: float = 0.0
    height: float = 0.0

    # Car body dimensions (cm) – used by GUI / collision
    car_width: float = 4.2
    car_height: float = 4.8
    # 当小车朝向 Y 轴正方向时，两轮轴心点相对于车身左下角的坐标， 用于运动学计算
    offset_w: float = 2.1
    offset_h: float = 4.1
    # 当小车朝向 Y 轴正方向时， 两轮轴心点相对于marker坐标系中心点的坐标，用于detection和rendering.
    marker_center_to_wheel_center_offset_cm: tuple[float, float] = (0.0, 0.0)

    # Differential-drive parameters
    wheel_base: float = 4.2
    max_wheel_speed: float = 10.0
    frequency: float = 30.0

    # Known robot IDs (can start empty; observer auto-discovers)
    robot_ids: List[int] = field(default_factory=list)

    # Camera
    camera_device: int = 0
    camera_resolution: str = "720p"
    camera_fps: int = 60
    undistort: bool = False
    calibration_file: str = _DEFAULT_CALIB

    # ArUco – car markers
    car_dict: str = "DICT_4X4_50"
    car_marker_size_mm: float = 36.0
    car_marker_height_cm: float = 4.8

    # ArUco – obstacle markers
    obstacle_dict: str = "DICT_5X5_50"
    obstacle_marker_size_mm: float = 30.0
    obstacle_marker_height_cm: float = 4.0
    obstacle_marker_config_file: str = _DEFAULT_OBSTACLE

    # Workspace estimation
    workspace_margin_cm: float = 1.0
    workspace_min_side_cm: float = 10.0
    warmup_frames: int = 30
    no_preview: bool = False

    # Serial / ESP-NOW action sender
    serial_port: str = "/dev/tty.usbmodem3101"
    serial_baudrate: int = 115200
    send_hz: float = 30.0
    cars_per_level: int = 10
    invert_left_wheel: bool = False
    invert_right_wheel: bool = False

    obstacle_detection_interval_sec: float = 1.0


v3_config = NewRealPushConfig(
    car_width=4.8,
    car_height=5.2,
    offset_w=2.4,
    offset_h=2.6,
    wheel_base=4.2,
    marker_center_to_wheel_center_offset_cm=(0.0, 0.0),
    car_marker_size_mm=36.0,
)

v4_config = NewRealPushConfig(
    car_width=4.2,
    car_height=4.8,
    offset_w=2.1,
    offset_h=4.5,
    wheel_base=4.2,
    marker_center_to_wheel_center_offset_cm=(0.0, 1.8),
    car_marker_size_mm=36.0,
)


class NewRealPushEnv(Environment):
    """
    Real hardware environment combining:
    - ArucoObserver with adaptive ground-plane workspace
    - SerialActionSender with Xiao ESP-NOW protocol
    """

    def __init__(self, config: NewRealPushConfig) -> None:
        self._config = config
        self._started = False
        self._speed_scale = 1.0

        self._workspace_config = WorkspaceConfig(
            width=config.width,
            height=config.height,
            car_width=config.car_width,
            car_height=config.car_height,
            offset_w=config.offset_w,
            offset_h=config.offset_h,
            wheel_base=config.wheel_base,
            max_wheel_speed=config.max_wheel_speed,
            frequency=config.frequency,
            car_id_list=list(config.robot_ids),
        )

        obs_config = ObserverConfig(
            camera_device=config.camera_device,
            resolution=config.camera_resolution,
            fps=config.camera_fps,
            undistort=config.undistort,
            calibration_file=config.calibration_file,
            car_dict=config.car_dict,
            obstacle_dict=config.obstacle_dict,
            car_marker_size_mm=config.car_marker_size_mm,
            car_marker_height_cm=config.car_marker_height_cm,
            obstacle_marker_size_mm=config.obstacle_marker_size_mm,
            obstacle_marker_height_cm=config.obstacle_marker_height_cm,
            marker_center_to_wheel_center_offset_cm=config.marker_center_to_wheel_center_offset_cm,
            obstacle_marker_config_file=config.obstacle_marker_config_file,
            workspace_margin_cm=config.workspace_margin_cm,
            workspace_min_side_cm=config.workspace_min_side_cm,
            warmup_frames=config.warmup_frames,
            no_preview=config.no_preview,
            obstacle_detection_interval_sec=config.obstacle_detection_interval_sec,
        )
        self._observer = ArucoObserver(obs_config)

        action_config = SerialActionConfig(
            port=config.serial_port,
            baudrate=config.serial_baudrate,
            send_hz=config.send_hz,
            cars_per_level=config.cars_per_level,
            invert_left_wheel=config.invert_left_wheel,
            invert_right_wheel=config.invert_right_wheel,
            initial_robot_ids=list(config.robot_ids),
        )
        self._action_sender = SerialActionSender(action_config)

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def workspace_config(self) -> WorkspaceConfig:
        return self._workspace_config

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self, wait_for_ready: bool = True, timeout: float = 5.0) -> bool:
        if self._started:
            return True

        print("[NewRealPushEnv] Starting subsystems...")

        if not self._observer.start():
            print("[NewRealPushEnv] Error: Failed to start observer.")
            return False

        if not self._action_sender.start():
            print("[NewRealPushEnv] Error: Failed to start action sender.")
            self._observer.stop()
            return False

        self._started = True

        if wait_for_ready:
            print(f"[NewRealPushEnv] Waiting for workspace (timeout={timeout}s)...")
            t0 = time.time()
            while time.time() - t0 < timeout:
                if self._observer.is_workspace_ready():
                    self._sync_workspace_from_observer()
                    self._sync_robot_ids_from_observer()
                    print("[NewRealPushEnv] Workspace ready!")
                    return True
                time.sleep(0.05)
            print("[NewRealPushEnv] Warning: workspace not ready within timeout.")

        return True

    def close(self) -> None:
        if not self._started:
            return
        print("[NewRealPushEnv] Closing...")
        self._action_sender.stop()
        self._observer.stop()
        self._started = False
        print("[NewRealPushEnv] Closed.")

    # ------------------------------------------------------------------
    # Environment interface
    # ------------------------------------------------------------------

    def observe(self) -> Dict[int, RobotObservation]:
        self._sync_workspace_from_observer()
        self._sync_robot_ids_from_observer()

        car_obs = self._observer.get_observations()
        observations: Dict[int, RobotObservation] = {}
        for car_id, obs in car_obs.items():
            observations[car_id] = RobotObservation(
                robot_id=car_id,
                x=obs.x_cm,
                y=obs.y_cm,
                theta=obs.yaw_deg,
                timestamp=obs.timestamp,
            )
        return observations

    def apply_actions(self, actions: Dict[int, Action]) -> None:
        if self._speed_scale != 1.0:
            scaled = {}
            for rid, act in actions.items():
                scaled[rid] = Action(
                    left_speed=act.left_speed * self._speed_scale,
                    right_speed=act.right_speed * self._speed_scale,
                )
            self._action_sender.set_actions(scaled)
        else:
            self._action_sender.set_actions(actions)

    def set_speed_scale(self, scale: float) -> None:
        self._speed_scale = max(0.0, min(1.0, float(scale)))
        if self._speed_scale == 0.0:
            self._action_sender.stop_all()

    # ------------------------------------------------------------------
    # Extended API
    # ------------------------------------------------------------------

    def get_obstacles(self) -> list:
        return self._observer.get_obstacles()

    def render(self) -> None:
        self._observer.render()

    def stop_all(self) -> None:
        self._action_sender.stop_all()

    def add_robot(self, robot_id: int) -> None:
        self._action_sender.add_robot(robot_id)
        if robot_id not in self._workspace_config.car_id_list:
            self._workspace_config.car_id_list.append(robot_id)

    def remove_robot(self, robot_id: int) -> None:
        self._action_sender.remove_robot(robot_id)
        if robot_id in self._workspace_config.car_id_list:
            self._workspace_config.car_id_list.remove(robot_id)

    # ------------------------------------------------------------------
    # Internal sync
    # ------------------------------------------------------------------

    def _sync_workspace_from_observer(self) -> None:
        ws = self._observer.get_workspace_estimate()
        if not ws.ready:
            return
        self._workspace_config.width = ws.width_cm
        self._workspace_config.height = ws.height_cm

    def _sync_robot_ids_from_observer(self) -> None:
        """Promote observed car IDs to controllable robots.

        Design choice: any car detected by the camera is automatically
        registered as a controllable robot in the action sender.  This is
        intentional convenience for setups where all visible cars should be
        controllable.  If you need explicit opt-in, gate this behind a
        config flag (e.g. ``auto_register_observed_cars``).
        """
        for robot_id in self._observer.get_detected_car_ids():
            if robot_id not in self._workspace_config.car_id_list:
                self._workspace_config.car_id_list.append(robot_id)
                self._action_sender.add_robot(robot_id)
