"""
NewRealPushEnv – real hardware environment using:
- Adaptive workspace estimation (no fixed ground markers)
- Xiao ESP-NOW serial protocol for motor commands
"""
from __future__ import annotations

import time
from typing import Dict

from micromvp.config import Config
from micromvp.core.models import Action, RobotObservation, WorkspaceConfig
from micromvp.env.base import Environment

from .observer import ArucoObserver, ObserverConfig
from .serial_action import SerialActionConfig, SerialActionSender


class NewRealPushEnv(Environment):
    """
    Real hardware environment combining:
    - ArucoObserver with adaptive ground-plane workspace
    - SerialActionSender with Xiao ESP-NOW protocol
    """

    def __init__(self, cfg: Config) -> None:
        self._cfg = cfg
        self._started = False
        self._speed_scale = 1.0

        who = "NewRealPushEnv"
        axle = cfg.require_pair("car.axle_offset_cm", who=who)

        # Width and height start at zero: the observer measures the real
        # workspace from the camera and fills them in once it locks.
        self._workspace_config = WorkspaceConfig(
            width=0.0,
            height=0.0,
            car_width=cfg.require("car.body_width_cm", float, who=who),
            car_height=cfg.require("car.body_height_cm", float, who=who),
            offset_w=axle[0],
            offset_h=axle[1],
            wheel_base=cfg.require("car.wheel_base_cm", float, who=who),
            max_wheel_speed=cfg.require("car.max_wheel_speed_cm_s", float, who=who),
            frequency=cfg.require("runtime.frequency", float, who=who),
            car_id_list=[],
        )

        self._observer = ArucoObserver(ObserverConfig.from_config(cfg))
        self._action_sender = SerialActionSender(SerialActionConfig.from_config(cfg))

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
