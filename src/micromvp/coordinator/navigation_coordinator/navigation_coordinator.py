"""
Navigation Coordinator - Single robot navigation with obstacle avoidance.

This coordinator provides:
1. Web server API for external control:
   - setup_obstacle(obstacles): Set obstacle geometry
   - goto(x, y, theta): Navigate to position with final orientation
   - follow_path(path): Follow a sequence of waypoints

2. GUI integration:
   - Speed slider to adjust robot movement speed
   - Rotation input to manually rotate robot to target angle
   - Hand-drawn path following from canvas
   - Path visualization until completion

3. Single robot control with selectable ID

Priority: GUI commands override external web server commands.
"""
from __future__ import annotations

import json
import math
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from typing import Dict, List, Any, Optional, Tuple
from functools import partial

from micromvp.controller.base import Controller
from micromvp.controller.navigation_controller import NavigationController, NavigationState
from micromvp.coordinator.base import Coordinator
from micromvp.core.models import (
    Action,
    CarState,
    RobotObservation,
    WorkspaceConfig,
)


Point = Tuple[float, float]


class NavigationCoordinator(Coordinator):
    """
    Coordinator for single robot navigation with obstacle avoidance.

    Features:
    - Web server with setup_obstacle, goto, follow_path APIs
    - GUI speed slider and rotation input
    - Hand-drawn path following
    - RVG-based path planning for obstacle avoidance
    - GUI commands take priority over web server commands

    Usage:
        coordinator = NavigationCoordinator(
            ws_config,
            controllers={robot_id: NavigationController(...)},
            active_robot_id=robot_id,
            webserver_port=8080
        )

        # Register GUI callbacks
        gui.register_callback("set_robot_speed", coordinator.on_speed_change)
        gui.register_callback("set_rotation_target", coordinator.on_rotation_input)
        gui.register_callback("on_curve_drawn", coordinator.on_curve_drawn)
    """

    def __init__(
        self,
        ws_config: WorkspaceConfig,
        controllers: Dict[int, Controller],
        active_robot_id: Optional[int] = None,
        webserver_port: int = 8080,
        robot_geometry: Optional[List[Tuple[float, float]]] = None,
        robot_geometry_scale: float = 1.2,
    ) -> None:
        """
        Initialize navigation coordinator.

        Args:
            ws_config: Workspace configuration from environment
            controllers: Dict mapping robot_id to Controller instance
                         Should be NavigationController for full functionality
            active_robot_id: The robot ID to control (default: first in list)
            webserver_port: Port for the web server API (default: 8080)
            robot_geometry: Robot shape as list of (x,y) vertices for RVG
                           Default is a rectangle based on car dimensions
            robot_geometry_scale: Scale factor for robot geometry (default: 1.2)
                                 Use >1.0 for more conservative paths that keep
                                 larger distance from obstacles
        """
        super().__init__(ws_config, controllers)

        # Active robot selection
        if active_robot_id is not None:
            self._active_robot_id = active_robot_id
        elif ws_config.car_id_list:
            self._active_robot_id = ws_config.car_id_list[0]
        else:
            self._active_robot_id = None

        # Obstacle storage for path planning (web API obstacles)
        self._obstacles: List[List[Tuple[float, float]]] = []
        self._obstacles_lock = threading.Lock()

        # ArUco-detected obstacles (passed from environment)
        self._aruco_obstacles: List[List[Tuple[float, float]]] = []
        self._aruco_obstacles_lock = threading.Lock()

        # External command tracking (for priority management)
        self._external_command_active = False
        self._external_path: Optional[List[Point]] = None

        # Robot geometry for RVG solver (with optional scaling)
        self._robot_geometry_scale = robot_geometry_scale
        base_geometry = robot_geometry or self._default_robot_geometry()
        # Apply scale to robot geometry for more conservative path planning
        if robot_geometry_scale != 1.0:
            self._robot_geometry = [
                (x * robot_geometry_scale, y * robot_geometry_scale)
                for x, y in base_geometry
            ]
        else:
            self._robot_geometry = base_geometry

        # Web server
        self._webserver_port = webserver_port
        self._webserver: Optional[HTTPServer] = None
        self._webserver_thread: Optional[threading.Thread] = None

        # Current path being displayed (for visualization)
        self._display_path: Optional[List[Point]] = None
        self._display_path_lock = threading.Lock()

        # Pending path for pre-rotation before path following
        # When a path is received, we first rotate toward the start point,
        # then start following the path after rotation completes
        self._pending_path: Optional[List[Point]] = None
        self._pending_goto_theta: Optional[float] = None  # Final theta for goto command
        self._pending_path_lock = threading.Lock()

        # Minimum distance to trigger pre-rotation (skip if already very close to path start)
        self._pre_rotation_min_distance = max(ws_config.car_width, ws_config.car_height) * 0.5

        # Path interpolation settings
        # Maximum distance between consecutive path points (smaller = denser path)
        self._max_point_spacing = max(ws_config.car_width, ws_config.car_height) * 0.1
        # Encode RVG "rotate in place" states as short heading markers so the
        # downstream point-following controller keeps the planned orientation.
        self._heading_marker_length = max(ws_config.car_width, ws_config.car_height) * 0.25

        # Start web server
        self._start_webserver()

    def _default_robot_geometry(self) -> List[Tuple[float, float]]:
        """Create default robot geometry for RVG path planning.

        The polygon is centred on the wheel-axle (rotation centre) and
        oriented so that theta=0 corresponds to the car facing X+,
        matching the workspace heading convention.

        WorkspaceConfig defines offset_w / offset_h as the axle position
        relative to the body bottom-left corner *when the car faces Y+*.
        We rotate the body polygon by -90° (CW) so that at theta=0
        the polygon faces X+.
        """
        ow = self._ws_config.offset_w
        oh = self._ws_config.offset_h
        cw = self._ws_config.car_width
        ch = self._ws_config.car_height
        left_extent = ow
        right_extent = cw - ow
        rear_extent = oh
        front_extent = ch - oh

        # The physical V4 geometry can be very nose-short relative to the axle.
        # RVG then accepts paths that are theoretically collision-free but
        # operationally too aggressive for a forward-only car. Use a more
        # conservative planner footprint whose front reach is at least the
        # lateral half-width.
        planner_front_extent = max(front_extent, left_extent, right_extent)

        return [
            (-rear_extent, left_extent),
            (-rear_extent, -right_extent),
            (planner_front_extent, -right_extent),
            (planner_front_extent, left_extent),
        ]

    # -------------------------------------------------------------------------
    # Web Server Implementation
    # -------------------------------------------------------------------------

    def _start_webserver(self) -> None:
        """Start the HTTP server in a background thread."""
        handler = partial(_NavigationRequestHandler, coordinator=self)
        try:
            self._webserver = HTTPServer(("0.0.0.0", self._webserver_port), handler)
            self._webserver_thread = threading.Thread(
                target=self._webserver.serve_forever,
                daemon=True
            )
            self._webserver_thread.start()
            print(f"[NavigationCoordinator] Web server started on port {self._webserver_port}")
        except OSError as e:
            print(f"[NavigationCoordinator] Failed to start web server: {e}")
            self._webserver = None

    def _stop_webserver(self) -> None:
        """Stop the web server."""
        if self._webserver:
            self._webserver.shutdown()
            self._webserver = None

    # -------------------------------------------------------------------------
    # Web API Handlers
    # -------------------------------------------------------------------------

    def api_setup_obstacle(self, obstacles: List[List[List[float]]]) -> Dict[str, Any]:
        """
        Set obstacles for path planning.

        Args:
            obstacles: List of obstacle polygons. Each polygon is a list of
                      [x, y] points in counter-clockwise order.

        Returns:
            API response dict
        """
        with self._obstacles_lock:
            self._obstacles = [
                [(pt[0], pt[1]) for pt in polygon]
                for polygon in obstacles
            ]
        return {"status": "ok", "obstacle_count": len(self._obstacles)}

    def api_goto(self, x: float, y: float, theta: float) -> Dict[str, Any]:
        """
        Navigate robot to position with final orientation.

        Uses RVG solver to find obstacle-avoiding path. First rotates toward
        the path start, then follows the path, and finally rotates to the
        target orientation.

        Args:
            x: Target x position
            y: Target y position
            theta: Target orientation in degrees [0, 360)

        Returns:
            API response dict
        """
        controller = self._get_active_controller()
        if controller is None:
            return {"status": "error", "message": "No active robot"}

        # Get current position
        current_x = controller.car_state.x
        current_y = controller.car_state.y
        current_theta = controller.car_state.theta

        # Plan path using RVG with all obstacles (web API + ArUco)
        all_obstacles = self._get_all_obstacles()

        path = self._plan_path(
            (current_x, current_y, current_theta),
            (x, y, theta),
            all_obstacles
        )

        if path is None:
            return {"status": "error", "message": "Path planning failed"}

        # Set external command
        self._external_command_active = True
        self._external_path = path
        self._set_display_path(path)

        # Start path with pre-rotation toward path start
        self._start_path_with_pre_rotation(path, goto_theta=theta)

        return {
            "status": "ok",
            "path_length": len(path),
            "target": {"x": x, "y": y, "theta": theta}
        }

    def api_follow_path(self, path: List[List[float]]) -> Dict[str, Any]:
        """
        Follow a sequence of waypoints.

        First rotates toward the path start, then follows the path.

        Args:
            path: List of [x, y] waypoints

        Returns:
            API response dict
        """
        controller = self._get_active_controller()
        if controller is None:
            return {"status": "error", "message": "No active robot"}

        # Convert to tuples
        path_points = [(pt[0], pt[1]) for pt in path]

        # Set external command
        self._external_command_active = True
        self._external_path = path_points
        self._set_display_path(path_points)

        # Start path with pre-rotation toward path start
        self._start_path_with_pre_rotation(path_points)

        return {"status": "ok", "path_length": len(path_points)}

    # -------------------------------------------------------------------------
    # Path Planning with RVG
    # -------------------------------------------------------------------------

    def _plan_path(
        self,
        start: Tuple[float, float, float],
        goal: Tuple[float, float, float],
        obstacles: List[List[Tuple[float, float]]]
    ) -> Optional[List[Point]]:
        """
        Plan a path using RVG solver.

        Args:
            start: Start pose (x, y, theta)
            goal: Goal pose (x, y, theta)
            obstacles: List of obstacle polygons

        Returns:
            Path as list of (x, y) points, or None if planning fails
        """
        try:
            from rvg import vertex, polygon, rvg
        except ImportError:
            print("[NavigationCoordinator] RVG not available, using direct path")
            return [start[:2], goal[:2]]

        TWO_PI = 2.0 * math.pi

        try:
            # --- border (CCW rectangle) ---
            w = self._ws_config.width
            h = self._ws_config.height
            border_verts = [
                vertex(0, 0), vertex(w, 0), vertex(w, h), vertex(0, h)
            ]
            border = polygon(border_verts, False)

            # --- robot polygon ---
            # _robot_geometry vertices are centred on the wheel-axle and
            # oriented for theta=0 = facing X+  (see _default_robot_geometry)
            robot_verts = [vertex(pt[0], pt[1]) for pt in self._robot_geometry]
            robot_poly = polygon(robot_verts, vertex(0, 0), False)

            # --- obstacle polygons ---
            obstacle_polys = []
            for obs in obstacles:
                if len(obs) >= 3:
                    obs_verts = [vertex(pt[0], pt[1]) for pt in obs]
                    obstacle_polys.append(polygon(obs_verts, False))

            # --- RVG solver ---
            solver = rvg(
                robot=robot_poly,
                border=border,
                obstacles=obstacle_polys,
                resolution=36,
                numThreads=1,
                verbose=False,
                fineApprox=True,
            )
            solver.setWeight(euclideanWeight=1.0, rotationalWeight=0.1)

            # --- start / goal vertices with actual heading ---
            start_theta = math.radians(start[2]) % TWO_PI
            goal_theta = math.radians(goal[2]) % TWO_PI
            start_v = vertex(start[0], start[1], 0.0, TWO_PI, start_theta)
            goal_v = vertex(goal[0], goal[1], 0.0, TWO_PI, goal_theta)

            # Pre-check legality to avoid C++ runtime_error
            layers = solver.getLayers()
            if layers:
                layer = layers[0]
                if not layer.legalConfig(start_v):
                    print(f"[NavigationCoordinator] Start ({start[0]:.1f}, {start[1]:.1f}) "
                          "illegal (too close to obstacle/border), using direct path")
                    return [start[:2], goal[:2]]
                if not layer.legalConfig(goal_v):
                    print(f"[NavigationCoordinator] Goal ({goal[0]:.1f}, {goal[1]:.1f}) "
                          "illegal (too close to obstacle/border), using direct path")
                    return [start[:2], goal[:2]]

            # --- shortest path ---
            path_result = solver.shortestPath(start_v, goal_v)

            if path_result is None or len(path_result) == 0:
                return [start[:2], goal[:2]]

            path = self._convert_rvg_path(path_result)

            return path if len(path) >= 2 else [start[:2], goal[:2]]

        except Exception as e:
            print(f"[NavigationCoordinator] Path planning error: {e}")
            return [start[:2], goal[:2]]

    def _convert_rvg_path(self, path_result: List[Any]) -> List[Point]:
        """
        Convert RVG configuration states into a point path.

        RVG can emit repeated (x, y) positions with different theta values to
        represent in-place rotations. The downstream controller only follows
        points, so dropping those states loses critical orientation
        information. We encode them as short heading markers instead.
        """
        path: List[Point] = []
        prev_xy: Optional[Point] = None
        prev_theta: Optional[float] = None
        count = len(path_result)

        for idx, v in enumerate(path_result):
            xy = (v.getX(), v.getY())
            theta = v.getTheta()

            if prev_xy is None:
                path.append(xy)
                prev_xy = xy
                prev_theta = theta
                continue

            same_xy = (
                abs(xy[0] - prev_xy[0]) <= 1e-6
                and abs(xy[1] - prev_xy[1]) <= 1e-6
            )

            if same_xy:
                dtheta = (theta - (prev_theta or 0.0) + math.pi) % (2.0 * math.pi) - math.pi
                is_terminal_rotation = idx == (count - 1)
                if abs(dtheta) > math.radians(1.0) and not is_terminal_rotation:
                    marker = (
                        xy[0] + self._heading_marker_length * math.cos(theta),
                        xy[1] + self._heading_marker_length * math.sin(theta),
                    )
                    if (
                        abs(marker[0] - path[-1][0]) > 1e-6
                        or abs(marker[1] - path[-1][1]) > 1e-6
                    ):
                        path.append(marker)
                prev_theta = theta
                continue

            if abs(xy[0] - path[-1][0]) > 1e-6 or abs(xy[1] - path[-1][1]) > 1e-6:
                path.append(xy)
            prev_xy = xy
            prev_theta = theta

        return path

    def _interpolate_path(
        self,
        path: List[Point],
        max_spacing: Optional[float] = None
    ) -> List[Point]:
        """
        Interpolate a sparse path to create a dense path with evenly spaced points.

        Takes critical waypoints from RVG and adds intermediate points so that
        consecutive points are no more than max_spacing apart. This is needed
        for pure pursuit algorithm which requires dense points.

        Args:
            path: Sparse path as list of (x, y) critical points
            max_spacing: Maximum distance between consecutive points.
                        If None, uses self._max_point_spacing

        Returns:
            Dense path with interpolated points
        """
        if not path or len(path) < 2:
            return path

        if max_spacing is None:
            max_spacing = self._max_point_spacing

        dense_path: List[Point] = [path[0]]

        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i + 1]

            # Calculate distance between consecutive points
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            segment_length = math.sqrt(dx * dx + dy * dy)

            if segment_length <= max_spacing:
                # Segment is short enough, just add the endpoint
                dense_path.append(p2)
            else:
                # Need to interpolate: calculate number of segments needed
                num_segments = math.ceil(segment_length / max_spacing)

                # Add intermediate points (excluding start point which is already added)
                for j in range(1, num_segments + 1):
                    t = j / num_segments
                    interp_x = p1[0] + t * dx
                    interp_y = p1[1] + t * dy
                    dense_path.append((interp_x, interp_y))

        return dense_path

    def _get_all_obstacles(self) -> List[List[Tuple[float, float]]]:
        """Get combined list of web API obstacles and ArUco-detected obstacles."""
        with self._obstacles_lock:
            web_obstacles = list(self._obstacles)
        with self._aruco_obstacles_lock:
            aruco_obstacles = list(self._aruco_obstacles)
        return web_obstacles + aruco_obstacles

    # -------------------------------------------------------------------------
    # GUI Callbacks
    # -------------------------------------------------------------------------

    def on_speed_change(self, speed: float) -> None:
        """
        Handle speed slider change from GUI.

        Args:
            speed: New speed value [0, 1]
        """
        controller = self._get_active_controller()
        if controller is not None and hasattr(controller, "set_speed"):
            controller.set_speed(speed)

    def on_rotation_input(self, input_text: str) -> None:
        """
        Handle rotation target input from GUI.

        Args:
            input_text: Text input (should be angle 0-359.99)
        """
        try:
            angle = float(input_text)
            if 0 <= angle < 360:
                self._gui_rotate_to(angle)
        except ValueError:
            pass  # Ignore invalid input

    def on_curve_drawn(self, points: List[Tuple[float, float]]) -> None:
        """
        Handle curve drawn event from GUI.

        GUI commands take priority, so this clears external commands.
        First rotates toward the path start, then follows the path.

        Args:
            points: List of (x, y) points defining the drawn curve
        """
        if not points:
            return

        # GUI takes priority - clear external command and pending path
        self._external_command_active = False
        self._external_path = None
        self._clear_pending_path()

        controller = self._get_active_controller()
        if controller is None:
            return

        # Set display path
        self._set_display_path(points)

        # Start path with pre-rotation toward path start
        self._start_path_with_pre_rotation(points)

    def on_canvas_click(self, x: float, y: float) -> None:
        """
        Handle canvas click event from GUI.

        Plans a path from current position to clicked point using RVG solver,
        interpolates it for pure pursuit, and sends to the controller.
        No final rotation is applied (unlike api_goto).

        GUI commands take priority over external web server commands.

        Args:
            x: Target x position (workspace coordinates)
            y: Target y position (workspace coordinates)
        """
        # GUI takes priority - clear external command and pending path
        self._external_command_active = False
        self._external_path = None
        self._clear_pending_path()

        controller = self._get_active_controller()
        if controller is None:
            return

        # Get current position
        current_x = controller.car_state.x
        current_y = controller.car_state.y
        current_theta = controller.car_state.theta

        # Plan path using RVG with all obstacles (web API + ArUco)
        all_obstacles = self._get_all_obstacles()

        path = self._plan_path(
            (current_x, current_y, current_theta),
            (x, y, 0),  # theta=0 for goal (not used since no final rotation)
            all_obstacles
        )

        if path is None or len(path) < 2:
            # Planning failed or trivial path, go direct
            path = [(current_x, current_y), (x, y)]

        # Set display path (show the original critical points path)
        self._set_display_path(path)

        # Start path with pre-rotation toward path start
        # Note: no goto_theta passed, so no final rotation will be applied
        self._start_path_with_pre_rotation(path)

    def on_key_press(self, key: str) -> None:
        """
        Handle key press event from GUI.

        Args:
            key: Key name (lowercase)
        """
        key = key.lower()

        # C to clear path
        if key == "c":
            self._clear_current_task()

        # Space for emergency stop
        elif key == "space":
            self._emergency_stop()

    def on_key_release(self, key: str) -> None:
        """Handle key release event from GUI."""
        pass  # No action needed

    def on_car_click(self, robot_id: int) -> None:
        """
        Handle car click event from GUI.

        Args:
            robot_id: ID of the clicked robot
        """
        if robot_id in self._controllers:
            self._active_robot_id = robot_id

    # -------------------------------------------------------------------------
    # Internal Methods
    # -------------------------------------------------------------------------

    def _get_active_controller(self) -> Optional[NavigationController]:
        """Get the currently active controller."""
        if self._active_robot_id is None:
            return None
        controller = self._controllers.get(self._active_robot_id)
        if isinstance(controller, NavigationController):
            return controller
        return None

    def _gui_rotate_to(self, angle: float) -> None:
        """
        Start rotation to target angle (GUI command).

        GUI commands take priority over external commands.

        Args:
            angle: Target angle in degrees [0, 360)
        """
        # GUI takes priority
        self._external_command_active = False
        self._external_path = None
        self._set_display_path(None)

        controller = self._get_active_controller()
        if controller is not None:
            controller.rotate_to(angle)

    def _clear_current_task(self) -> None:
        """Clear current task and stop robot."""
        self._external_command_active = False
        self._external_path = None
        self._set_display_path(None)
        self._clear_pending_path()

        controller = self._get_active_controller()
        if controller is not None:
            controller.clear_path()
            controller.cancel_rotation()

    def _emergency_stop(self) -> None:
        """Emergency stop all robots."""
        self._external_command_active = False
        self._external_path = None
        self._set_display_path(None)
        self._clear_pending_path()

        for controller in self._controllers.values():
            if isinstance(controller, NavigationController):
                controller.clear_path()
                controller.cancel_rotation()
            controller.stop()

    def _set_display_path(self, path: Optional[List[Point]]) -> None:
        """Set the path to display in GUI."""
        with self._display_path_lock:
            self._display_path = path

    def _start_path_with_pre_rotation(
        self,
        path: List[Point],
        goto_theta: Optional[float] = None
    ) -> bool:
        """
        Start path following with pre-rotation toward the first waypoint.

        First rotates the robot to face the first target waypoint of the path,
        then starts following the path after rotation completes.

        Handles two cases:
        1. RVG/click-to-go paths: path[0] is current position, path[1] is first waypoint
        2. Hand-drawn curves: path[0] is already the first waypoint to go to

        Args:
            path: List of (x, y) waypoints
            goto_theta: Optional final orientation for goto command

        Returns:
            True if pre-rotation started, False if path following started directly
        """
        controller = self._get_active_controller()
        if controller is None or not path:
            return False

        # Get current position
        current_x = controller.car_state.x
        current_y = controller.car_state.y
        current_theta = controller.car_state.theta

        # Determine the first actual waypoint to navigate to
        # If path[0] is very close to current position, it's the start point (RVG path)
        # and we should look at path[1] for the first target waypoint
        first_point = path[0]
        dx = first_point[0] - current_x
        dy = first_point[1] - current_y
        dist_to_first = math.sqrt(dx * dx + dy * dy)

        if dist_to_first < self._pre_rotation_min_distance and len(path) > 1:
            # path[0] is current position, use path[1] as target waypoint
            target_point = path[1]
        else:
            # path[0] is already the first waypoint (hand-drawn curve)
            target_point = first_point

        # Calculate distance and angle to target waypoint
        dx = target_point[0] - current_x
        dy = target_point[1] - current_y
        distance = math.sqrt(dx * dx + dy * dy)

        # If very close to target, skip pre-rotation and start path following
        if distance < self._pre_rotation_min_distance:
            dense_path = self._interpolate_path(path)
            controller.set_path(dense_path)
            if goto_theta is not None:
                controller._car_state.metadata["goto_target_theta"] = goto_theta
            return False

        # Calculate angle to target waypoint
        target_angle = math.degrees(math.atan2(dy, dx))
        if target_angle < 0:
            target_angle += 360

        # Check if we need to rotate (heading differs significantly from target direction)
        heading_error = target_angle - current_theta
        while heading_error > 180:
            heading_error -= 360
        while heading_error < -180:
            heading_error += 360

        # If heading is already close enough (within 45 degrees), skip pre-rotation
        if abs(heading_error) < 45:
            dense_path = self._interpolate_path(path)
            controller.set_path(dense_path)
            if goto_theta is not None:
                controller._car_state.metadata["goto_target_theta"] = goto_theta
            return False

        # Store pending path for after rotation completes
        with self._pending_path_lock:
            self._pending_path = path
            self._pending_goto_theta = goto_theta

        # Start rotation toward first waypoint
        controller.rotate_to(target_angle)
        return True

    def _check_pending_path_transition(self, controller: NavigationController) -> None:
        """
        Check if pre-rotation completed and start pending path if so.

        Called from process() to handle the transition from pre-rotation
        to path following.

        Args:
            controller: The navigation controller to check
        """
        with self._pending_path_lock:
            if self._pending_path is None:
                return

            # Check if rotation completed (or nearly complete)
            if controller._nav_state in (NavigationState.ROTATION_DONE,
                                          NavigationState.ROTATION_STABLE):
                # Rotation complete - start path following
                path = self._pending_path
                goto_theta = self._pending_goto_theta
                self._pending_path = None
                self._pending_goto_theta = None

                # Cancel rotation state and start path with interpolated dense path
                controller.cancel_rotation()
                dense_path = self._interpolate_path(path)
                controller.set_path(dense_path)
                if goto_theta is not None:
                    controller._car_state.metadata["goto_target_theta"] = goto_theta

    def _clear_pending_path(self) -> None:
        """Clear any pending path waiting for pre-rotation."""
        with self._pending_path_lock:
            self._pending_path = None
            self._pending_goto_theta = None

    # -------------------------------------------------------------------------
    # Coordinator Interface
    # -------------------------------------------------------------------------

    def process(
        self,
        observations: Dict[int, RobotObservation],
        aruco_obstacles: Optional[List[List[Tuple[float, float]]]] = None,
    ) -> Dict[int, Action]:
        """
        Process observations and return actions.

        Handles:
        - Pre-rotation to path start transition
        - Path following to final rotation transition (for goto)

        Args:
            observations: Dict mapping robot_id to RobotObservation
            aruco_obstacles: Optional list of ArUco-detected obstacle polygons

        Returns:
            Dict mapping robot_id to Action
        """
        # Update ArUco obstacles if provided
        if aruco_obstacles is not None:
            with self._aruco_obstacles_lock:
                self._aruco_obstacles = aruco_obstacles

        actions: Dict[int, Action] = {}

        for robot_id, controller in self._controllers.items():
            if robot_id not in observations:
                actions[robot_id] = Action.stop()
                continue

            if isinstance(controller, NavigationController):
                # Check for pending path transition (pre-rotation complete)
                if robot_id == self._active_robot_id:
                    self._check_pending_path_transition(controller)

                # Handle goto final rotation transition
                if controller._nav_state == NavigationState.FINISHED_PATH:
                    # Check if this was a goto command
                    target_theta = controller._car_state.metadata.get("goto_target_theta")
                    if target_theta is not None:
                        # Start rotation to final orientation
                        controller._car_state.metadata.pop("goto_target_theta", None)
                        controller.rotate_to(target_theta)

                # Clear display path when task completes
                if (controller._nav_state in (NavigationState.IDLE,
                                               NavigationState.FINISHED_PATH,
                                               NavigationState.ROTATION_DONE)):
                    if self._external_command_active and robot_id == self._active_robot_id:
                        self._external_command_active = False
                        self._set_display_path(None)

            actions[robot_id] = controller.step(observations[robot_id])

        return actions

    def gather_car_state(self) -> List[CarState]:
        """
        Gather car states with active robot metadata.

        Returns:
            List of CarState objects
        """
        states = []
        for controller in self._controllers.values():
            state = controller.car_state
            state.metadata["is_active"] = (
                controller.robot_id == self._active_robot_id
            )
            states.append(state)
        return states

    def get_additional_drawings(self) -> List[Dict[str, Any]]:
        """
        Get additional drawings for GUI.

        Draws:
        - Selection indicator around active robot
        - Current path being followed
        - Target point
        - Obstacles

        Returns:
            List of drawing commands
        """
        drawings = []

        # Draw selection indicator for active robot
        if self._active_robot_id is not None:
            controller = self._controllers.get(self._active_robot_id)
            if controller is not None:
                state = controller.car_state
                drawings.append({
                    "uuid": f"selection_indicator_{self._active_robot_id}",
                    "type": "circle",
                    "center": (state.x, state.y),
                    "radius": max(self._ws_config.car_width, self._ws_config.car_height) * 0.8,
                    "color": "green",
                    "width": 2,
                })

        # Draw current path
        with self._display_path_lock:
            display_path = self._display_path

        if display_path and len(display_path) >= 2:
            drawings.append({
                "uuid": "navigation_path",
                "type": "path",
                "points": display_path,
                "color": "#00FF00",
                "width": 2,
            })

        # Draw target point from controller
        if self._active_robot_id is not None:
            controller = self._controllers.get(self._active_robot_id)
            if controller is not None and isinstance(controller, NavigationController):
                state = controller.car_state

                # Draw path from controller metadata
                path = state.metadata.get("path")
                if path and len(path) >= 2 and display_path is None:
                    drawings.append({
                        "uuid": f"path_{self._active_robot_id}",
                        "type": "path",
                        "points": path,
                        "color": "#00FF00",
                        "width": 2,
                    })

                # Draw target point
                target = state.metadata.get("target_point")
                if target and state.status_label == "FOLLOWING":
                    drawings.append({
                        "uuid": f"target_{self._active_robot_id}",
                        "type": "point",
                        "position": target,
                        "radius": 5,
                        "color": "#FF0000",
                    })

                    # Draw line from robot to target
                    drawings.append({
                        "uuid": f"target_line_{self._active_robot_id}",
                        "type": "line",
                        "start": (state.x, state.y),
                        "end": target,
                        "color": "#FF000080",
                        "width": 1,
                    })

                # Draw rotation target indicator
                target_theta = state.metadata.get("target_theta")
                if target_theta is not None:
                    # Draw a line showing target direction
                    length = max(self._ws_config.car_width, self._ws_config.car_height) * 1.5
                    rad = math.radians(target_theta)
                    end_x = state.x + length * math.cos(rad)
                    end_y = state.y + length * math.sin(rad)
                    drawings.append({
                        "uuid": f"rotation_target_{self._active_robot_id}",
                        "type": "line",
                        "start": (state.x, state.y),
                        "end": (end_x, end_y),
                        "color": "#0088FF",
                        "width": 2,
                    })

        # Draw web API obstacles (orange)
        with self._obstacles_lock:
            web_obstacles = list(self._obstacles)

        for i, obstacle in enumerate(web_obstacles):
            if len(obstacle) >= 3:
                # Close the polygon
                points = list(obstacle) + [obstacle[0]]
                drawings.append({
                    "uuid": f"obstacle_web_{i}",
                    "type": "path",
                    "points": points,
                    "color": "#FF6600",
                    "width": 2,
                })

        # Draw ArUco-detected obstacles (magenta)
        with self._aruco_obstacles_lock:
            aruco_obstacles = list(self._aruco_obstacles)

        for i, obstacle in enumerate(aruco_obstacles):
            if len(obstacle) >= 3:
                # Close the polygon
                points = list(obstacle) + [obstacle[0]]
                drawings.append({
                    "uuid": f"obstacle_aruco_{i}",
                    "type": "path",
                    "points": points,
                    "color": "#FF00FF",
                    "width": 2,
                })

        return drawings

    def select_robot(self, robot_id: int) -> bool:
        """
        Select a robot as active.

        Args:
            robot_id: ID of robot to select

        Returns:
            True if robot was found and selected
        """
        if robot_id in self._controllers:
            self._active_robot_id = robot_id
            return True
        return False

    def reset(self) -> None:
        """Reset coordinator and all controllers."""
        self._external_command_active = False
        self._external_path = None
        self._set_display_path(None)
        self._clear_pending_path()
        with self._obstacles_lock:
            self._obstacles = []
        with self._aruco_obstacles_lock:
            self._aruco_obstacles = []
        super().reset()

    def close(self) -> None:
        """Clean up resources."""
        self._stop_webserver()
        super().close()


class _NavigationRequestHandler(BaseHTTPRequestHandler):
    """HTTP request handler for navigation API."""

    def __init__(self, *args, coordinator: NavigationCoordinator, **kwargs):
        self.coordinator = coordinator
        super().__init__(*args, **kwargs)

    def log_message(self, format, *args):
        """Suppress default logging."""
        pass

    def _send_json_response(self, data: Dict[str, Any], status: int = 200) -> None:
        """Send a JSON response."""
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def _read_json_body(self) -> Optional[Dict[str, Any]]:
        """Read and parse JSON body."""
        try:
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length)
            return json.loads(body.decode())
        except (ValueError, json.JSONDecodeError):
            return None

    def do_OPTIONS(self) -> None:
        """Handle CORS preflight."""
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self) -> None:
        """Handle GET requests."""
        if self.path == "/status":
            controller = self.coordinator._get_active_controller()
            if controller:
                state = controller.car_state
                self._send_json_response({
                    "status": "ok",
                    "robot_id": self.coordinator._active_robot_id,
                    "position": {"x": state.x, "y": state.y, "theta": state.theta},
                    "state": state.status_label,
                })
            else:
                self._send_json_response({
                    "status": "ok",
                    "robot_id": None,
                    "position": None,
                    "state": "NO_ROBOT"
                })
        else:
            self._send_json_response({"status": "error", "message": "Not found"}, 404)

    def do_POST(self) -> None:
        """Handle POST requests."""
        data = self._read_json_body()

        if self.path == "/setup_obstacle":
            if data is None or "obstacles" not in data:
                self._send_json_response(
                    {"status": "error", "message": "Invalid request body"}, 400
                )
                return
            result = self.coordinator.api_setup_obstacle(data["obstacles"])
            self._send_json_response(result)

        elif self.path == "/goto":
            if data is None:
                self._send_json_response(
                    {"status": "error", "message": "Invalid request body"}, 400
                )
                return
            x = data.get("x")
            y = data.get("y")
            theta = data.get("theta", 0)
            if x is None or y is None:
                self._send_json_response(
                    {"status": "error", "message": "Missing x or y"}, 400
                )
                return
            result = self.coordinator.api_goto(x, y, theta)
            self._send_json_response(result)

        elif self.path == "/follow_path":
            if data is None or "path" not in data:
                self._send_json_response(
                    {"status": "error", "message": "Invalid request body"}, 400
                )
                return
            result = self.coordinator.api_follow_path(data["path"])
            self._send_json_response(result)

        else:
            self._send_json_response({"status": "error", "message": "Not found"}, 404)
