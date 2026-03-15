"""
ArUco-based camera observer with adaptive workspace estimation.

Instead of fixed workspace markers on the ground, the coordinate system is
bootstrapped from car markers (4x4, 36mm, 4.8cm height) and obstacle markers
(5x5, 30mm, 4.0cm height).  Their known physical heights let us estimate the
ground plane; we then project the camera's field of view onto that plane and
pick the largest axis-aligned inscribed rectangle as the workspace.

Workspace lock lifecycle:
    collecting  ->  stable  ->  locked
Candidates are accumulated for workspace_lock_frames consecutive ready frames.
Only when the window is full AND all stability thresholds are met does the
state transition to "locked".

Threading contract (same as real_push_env.observer):
- start(): safe from GUI main thread
- _run_loop(): background thread, compute only, NO cv2 HighGUI
- render(): MUST be called from GUI main thread
- stop(): ideally from GUI main thread
"""
from __future__ import annotations

import json
import platform
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml


Point = Tuple[float, float]


@dataclass
class ObserverConfig:
    # Camera
    camera_device: int = 0
    resolution: str = "720p"
    fps: int = 60
    undistort: bool = False
    calibration_file: str = ""

    # ArUco dictionaries
    car_dict: str = "DICT_4X4_50"
    obstacle_dict: str = "DICT_5X5_50"

    # Marker physical parameters
    car_marker_size_mm: float = 36.0
    car_marker_height_cm: float = 4.8
    obstacle_marker_size_mm: float = 30.0
    obstacle_marker_height_cm: float = 4.0

    # Wheel-center offset applied after pose estimation
    marker_center_to_wheel_center_offset_cm: tuple[float, float] = (0.0, 0.0)

    # Obstacle config: JSON file with per-marker polygon_local (same as real_push_env)
    obstacle_marker_config_file: str = ""

    # Workspace estimation
    workspace_margin_cm: float = 1.0
    workspace_min_side_cm: float = 10.0

    # Workspace lock: multi-frame aggregation
    workspace_lock_frames: int = 30
    workspace_width_tolerance_cm: float = 2.0
    workspace_height_tolerance_cm: float = 2.0
    workspace_origin_tolerance_m: float = 0.01
    workspace_normal_angle_tolerance_deg: float = 3.0

    # Warmup (discard initial camera frames for auto-exposure settling)
    warmup_frames: int = 30

    # Preview
    no_preview: bool = False
    preview_window_name: str = "NewRealPushObserver"
    draw_axis: bool = True
    axis_length_m: float = 0.03

    # Obstacle detection throttle
    obstacle_detection_interval_sec: float = 1.0


@dataclass
class CarObservation:
    car_id: int
    x_cm: float
    y_cm: float
    yaw_deg: float
    timestamp: float


@dataclass
class WorkspaceEstimate:
    ready: bool = False
    width_cm: float = 0.0
    height_cm: float = 0.0
    origin_cam_m: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    x_axis_cam: Tuple[float, float, float] = (1.0, 0.0, 0.0)
    y_axis_cam: Tuple[float, float, float] = (0.0, 1.0, 0.0)
    normal_cam: Tuple[float, float, float] = (0.0, 0.0, 1.0)
    timestamp: float = 0.0


@dataclass
class _MarkerInfo:
    """Internal per-marker pose data produced during detection."""
    marker_id: int
    image_corners: np.ndarray   # (4,2) float32
    rvec: np.ndarray            # (3,) float32
    tvec: np.ndarray            # (3,) float32
    normal_cam: np.ndarray      # (3,) float32  – marker Z-axis in camera frame
    height_cm: float            # known physical height above ground


class _WorkspaceLockState:
    """
    State machine for workspace lock lifecycle.

    States:
        collecting  – accumulating ready candidates, window not yet full
        stable      – window full and all stability thresholds met (-> lock)
        locked      – final aggregated workspace committed
    """

    def __init__(self, required_frames: int) -> None:
        self.required_frames = max(1, required_frames)
        self.candidates: deque[WorkspaceEstimate] = deque(maxlen=self.required_frames)
        self.state: str = "collecting"

    @property
    def is_locked(self) -> bool:
        return self.state == "locked"

    def add_candidate(self, ws: WorkspaceEstimate) -> None:
        if self.is_locked:
            return
        if ws.ready:
            self.candidates.append(ws)
        else:
            self.candidates.clear()
            self.state = "collecting"

    @property
    def window_full(self) -> bool:
        return len(self.candidates) == self.required_frames

    def check_stability(self, config: ObserverConfig) -> bool:
        if not self.window_full:
            return False

        widths = [c.width_cm for c in self.candidates]
        heights = [c.height_cm for c in self.candidates]
        if max(widths) - min(widths) > config.workspace_width_tolerance_cm:
            return False
        if max(heights) - min(heights) > config.workspace_height_tolerance_cm:
            return False

        origins = np.array([c.origin_cam_m for c in self.candidates], dtype=np.float64)
        origin_spread = np.max(np.linalg.norm(origins - origins.mean(axis=0), axis=1))
        if origin_spread > config.workspace_origin_tolerance_m:
            return False

        normals = np.array([c.normal_cam for c in self.candidates], dtype=np.float64)
        norms = np.linalg.norm(normals, axis=1, keepdims=True)
        norms = np.where(norms < 1e-8, 1.0, norms)
        normals = normals / norms
        mean_normal = normals.mean(axis=0)
        mean_normal = mean_normal / max(np.linalg.norm(mean_normal), 1e-8)
        dots = np.clip(normals @ mean_normal, -1.0, 1.0)
        max_angle_deg = float(np.degrees(np.arccos(dots.min())))
        if max_angle_deg > config.workspace_normal_angle_tolerance_deg:
            return False

        return True

    def aggregate(self) -> WorkspaceEstimate:
        """Aggregate candidates: median for scalars, normalized-mean for vectors."""
        widths = np.array([c.width_cm for c in self.candidates])
        heights = np.array([c.height_cm for c in self.candidates])
        origins = np.array([c.origin_cam_m for c in self.candidates], dtype=np.float64)
        x_axes = np.array([c.x_axis_cam for c in self.candidates], dtype=np.float64)
        y_axes = np.array([c.y_axis_cam for c in self.candidates], dtype=np.float64)
        normals = np.array([c.normal_cam for c in self.candidates], dtype=np.float64)

        def _normalize(v: np.ndarray) -> np.ndarray:
            n = np.linalg.norm(v)
            return v / n if n > 1e-8 else v

        agg_normal = _normalize(normals.mean(axis=0))
        agg_x = _normalize(x_axes.mean(axis=0))
        agg_y = _normalize(y_axes.mean(axis=0))

        return WorkspaceEstimate(
            ready=True,
            width_cm=float(np.median(widths)),
            height_cm=float(np.median(heights)),
            origin_cam_m=tuple(origins.mean(axis=0).tolist()),
            x_axis_cam=tuple(agg_x.tolist()),
            y_axis_cam=tuple(agg_y.tolist()),
            normal_cam=tuple(agg_normal.tolist()),
            timestamp=time.time(),
        )


class ArucoObserver:
    """Camera observer with adaptive ground-plane workspace estimation."""

    def __init__(self, config: ObserverConfig) -> None:
        self._config = config
        self._running = False

        self._cap: Optional[cv2.VideoCapture] = None
        self._K: Optional[np.ndarray] = None
        self._D: Optional[np.ndarray] = None
        self._frame_size: Optional[Tuple[int, int]] = None  # (width, height)

        self._car_detector: Optional[cv2.aruco.ArucoDetector] = None
        self._obstacle_detector: Optional[cv2.aruco.ArucoDetector] = None
        self._obstacle_marker_len_m: float = 0.0
        self._obstacle_marker_config: Dict[int, List[Point]] = {}

        self._workspace = WorkspaceEstimate()
        self._workspace_lock = threading.Lock()
        self._ws_lock_state = _WorkspaceLockState(config.workspace_lock_frames)

        self._observations: Dict[int, CarObservation] = {}
        self._obs_lock = threading.Lock()

        self._obstacle_polygons: List[List[Point]] = []
        self._obstacle_lock = threading.Lock()
        self._last_obstacle_time: float = 0.0

        self._known_car_ids: set[int] = set()
        self._car_ids_lock = threading.Lock()

        self._thread: Optional[threading.Thread] = None
        self._vis_frame: Optional[np.ndarray] = None
        self._vis_lock = threading.Lock()
        self._window_inited = False
        self._frame_callback: Optional[Callable[[np.ndarray], None]] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> bool:
        if self._running:
            return True

        if not self._config.calibration_file:
            print("[Observer] Error: calibration_file is required.")
            return False

        try:
            self._K, self._D, self._frame_size = self._load_calibration(
                self._config.calibration_file
            )
        except Exception as exc:
            print(f"[Observer] Failed to load calibration: {exc}")
            return False

        self._cap = self._open_camera()
        if self._cap is None or not self._cap.isOpened():
            print("[Observer] Failed to open camera")
            return False

        for _ in range(max(0, int(self._config.warmup_frames))):
            ok, _ = self._cap.read()
            if not ok:
                break

        # --- Resolution validation ---
        ret, first_frame = self._cap.read()
        if not ret or first_frame is None:
            print("[Observer] Failed to read first frame after warmup")
            self._cap.release()
            return False
        actual_h, actual_w = first_frame.shape[:2]
        calib_w, calib_h = self._frame_size
        if (actual_w, actual_h) != (calib_w, calib_h):
            print(
                f"[Observer] FATAL: Resolution mismatch! "
                f"calibration={calib_w}x{calib_h}, "
                f"actual={actual_w}x{actual_h}, "
                f"requested={self._config.resolution}"
            )
            self._cap.release()
            return False

        self._setup_detectors()

        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        print(f"[Observer] Started (no_preview={self._config.no_preview})")
        return True

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

        if not self._config.no_preview and self._window_inited:
            try:
                cv2.destroyWindow(self._config.preview_window_name)
            except Exception:
                pass
            self._window_inited = False

    def set_frame_callback(self, callback: Callable[[np.ndarray], None]) -> None:
        self._frame_callback = callback

    def is_workspace_ready(self) -> bool:
        with self._workspace_lock:
            return self._workspace.ready

    def get_workspace_estimate(self) -> WorkspaceEstimate:
        with self._workspace_lock:
            ws = self._workspace
            return WorkspaceEstimate(
                ready=ws.ready,
                width_cm=ws.width_cm,
                height_cm=ws.height_cm,
                origin_cam_m=ws.origin_cam_m,
                x_axis_cam=ws.x_axis_cam,
                y_axis_cam=ws.y_axis_cam,
                normal_cam=ws.normal_cam,
                timestamp=ws.timestamp,
            )

    def get_workspace_lock_state(self) -> str:
        """Return the current workspace lock state: 'collecting', 'stable', or 'locked'."""
        return self._ws_lock_state.state

    def get_workspace_lock_progress(self) -> Tuple[int, int]:
        """Return (current_candidates, required_frames)."""
        return len(self._ws_lock_state.candidates), self._ws_lock_state.required_frames

    def get_observations(self) -> Dict[int, CarObservation]:
        with self._obs_lock:
            return dict(self._observations)

    def get_obstacles(self) -> List[List[Point]]:
        with self._obstacle_lock:
            return [list(poly) for poly in self._obstacle_polygons]

    def get_detected_car_ids(self) -> List[int]:
        with self._car_ids_lock:
            return sorted(self._known_car_ids)

    def render(self) -> None:
        if self._config.no_preview:
            return
        if not self._window_inited:
            try:
                cv2.namedWindow(self._config.preview_window_name, cv2.WINDOW_NORMAL)
            except Exception:
                print("[Observer] Warning: cv2.namedWindow failed, disabling preview.")
                self._config.no_preview = True
                return
            self._window_inited = True

        frame = None
        with self._vis_lock:
            if self._vis_frame is not None:
                frame = self._vis_frame.copy()
        if frame is None:
            return
        cv2.imshow(self._config.preview_window_name, frame)
        cv2.waitKey(1)

    # ------------------------------------------------------------------
    # Background loop – decomposed into small steps
    # ------------------------------------------------------------------

    def _run_loop(self) -> None:
        while self._running:
            frame = self._read_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            gray = self._preprocess(frame)
            timestamp = time.time()

            car_markers, car_corners, car_ids = self._detect_markers(
                gray,
                self._car_detector,
                self._config.car_marker_size_mm,
                self._config.car_marker_height_cm,
            )
            obstacle_markers, obs_corners, obs_ids = self._detect_markers(
                gray,
                self._obstacle_detector,
                self._config.obstacle_marker_size_mm,
                self._config.obstacle_marker_height_cm,
            )

            workspace = self._try_lock_workspace(car_markers, obstacle_markers)
            car_obs = self._update_observations(car_markers, workspace, timestamp)
            self._update_obstacles(obstacle_markers, workspace)
            self._update_preview(
                frame, car_corners, car_ids, obs_corners, obs_ids,
                car_markers, workspace, car_obs,
            )

            if self._frame_callback is not None:
                try:
                    self._frame_callback(frame)
                except Exception:
                    pass

    def _read_frame(self) -> Optional[np.ndarray]:
        ret, frame = self._cap.read()
        return frame if ret else None

    def _preprocess(self, frame: np.ndarray) -> np.ndarray:
        if self._config.undistort and self._K is not None and self._D is not None:
            frame = cv2.undistort(frame, self._K, self._D)
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def _try_lock_workspace(
        self,
        car_markers: Dict[int, _MarkerInfo],
        obstacle_markers: Dict[int, _MarkerInfo],
    ) -> WorkspaceEstimate:
        """Drive the collecting -> stable -> locked state machine."""
        with self._workspace_lock:
            if self._ws_lock_state.is_locked:
                return self._workspace

        all_markers = list(car_markers.values()) + list(obstacle_markers.values())
        candidate = self._estimate_workspace(all_markers)

        self._ws_lock_state.add_candidate(candidate)

        if self._ws_lock_state.window_full and not self._ws_lock_state.is_locked:
            if self._ws_lock_state.check_stability(self._config):
                aggregated = self._ws_lock_state.aggregate()
                self._ws_lock_state.state = "locked"
                with self._workspace_lock:
                    self._workspace = aggregated
                n, total = len(self._ws_lock_state.candidates), self._ws_lock_state.required_frames
                print(
                    f"[Observer] Workspace locked ({total} frames): "
                    f"{aggregated.width_cm:.1f} x {aggregated.height_cm:.1f} cm"
                )
                return aggregated
            else:
                self._ws_lock_state.state = "collecting"

        with self._workspace_lock:
            if candidate.ready:
                self._workspace = candidate
            return self._workspace

    def _update_observations(
        self,
        car_markers: Dict[int, _MarkerInfo],
        workspace: WorkspaceEstimate,
        timestamp: float,
    ) -> Dict[int, CarObservation]:
        car_obs: Dict[int, CarObservation] = {}
        if workspace.ready:
            car_obs = self._build_car_observations(car_markers, workspace, timestamp)
            with self._obs_lock:
                self._observations = car_obs
            with self._car_ids_lock:
                self._known_car_ids = set(car_obs.keys())
        else:
            with self._obs_lock:
                self._observations = {}
        return car_obs

    def _update_obstacles(
        self,
        obstacle_markers: Dict[int, _MarkerInfo],
        workspace: WorkspaceEstimate,
    ) -> None:
        if not workspace.ready:
            with self._obstacle_lock:
                self._obstacle_polygons = []
            return

        now = time.time()
        if now - self._last_obstacle_time >= self._config.obstacle_detection_interval_sec:
            self._last_obstacle_time = now
            polys = self._build_obstacle_polygons(obstacle_markers, workspace)
            with self._obstacle_lock:
                self._obstacle_polygons = polys

    def _update_preview(
        self,
        frame: np.ndarray,
        car_corners: list,
        car_ids: Optional[np.ndarray],
        obs_corners: list,
        obs_ids: Optional[np.ndarray],
        car_markers: Dict[int, _MarkerInfo],
        workspace: WorkspaceEstimate,
        car_obs: Dict[int, CarObservation],
    ) -> None:
        if self._config.no_preview:
            return
        vis = frame.copy()
        self._draw_detected_markers(vis, car_corners, car_ids, (0, 255, 0))
        self._draw_detected_markers(vis, obs_corners, obs_ids, (0, 165, 255))
        if self._config.draw_axis and self._K is not None:
            self._draw_car_axes(vis, car_markers)
        self._draw_workspace_overlay(vis, workspace, car_obs)
        with self._vis_lock:
            self._vis_frame = vis

    # ------------------------------------------------------------------
    # Marker detection
    # ------------------------------------------------------------------

    def _detect_markers(
        self,
        gray: np.ndarray,
        detector: Optional[cv2.aruco.ArucoDetector],
        marker_size_mm: float,
        marker_height_cm: float,
    ) -> Tuple[Dict[int, _MarkerInfo], list, Optional[np.ndarray]]:
        markers: Dict[int, _MarkerInfo] = {}
        if detector is None or marker_size_mm <= 0.0:
            return markers, [], None

        corners, ids, _ = detector.detectMarkers(gray)
        if ids is None or len(ids) == 0:
            return markers, corners, ids

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_size_mm / 1000.0, self._K, self._D
        )

        for i, marker_id in enumerate(ids.flatten().tolist()):
            rvec = rvecs[i, 0, :].astype(np.float32)
            tvec = tvecs[i, 0, :].astype(np.float32)
            R, _ = cv2.Rodrigues(rvec)
            markers[int(marker_id)] = _MarkerInfo(
                marker_id=int(marker_id),
                image_corners=corners[i].reshape(4, 2).astype(np.float32),
                rvec=rvec,
                tvec=tvec,
                normal_cam=R[:, 2].astype(np.float32),
                height_cm=float(marker_height_cm),
            )

        return markers, corners, ids

    # ------------------------------------------------------------------
    # Workspace estimation from ground plane
    # ------------------------------------------------------------------

    def _estimate_workspace(self, markers: List[_MarkerInfo]) -> WorkspaceEstimate:
        if not markers:
            return WorkspaceEstimate()

        normal_cam, plane_point = self._estimate_floor_plane(markers)
        if normal_cam is None or plane_point is None:
            return WorkspaceEstimate()

        x_axis_cam, y_axis_cam = self._build_plane_basis(normal_cam)

        visible_quad = self._project_image_corners_to_plane(
            plane_point, normal_cam, x_axis_cam, y_axis_cam
        )
        if visible_quad is None:
            return WorkspaceEstimate()

        rect = self._inscribed_rect(visible_quad, self._config.workspace_margin_cm)
        if rect is None:
            return WorkspaceEstimate()

        width_cm = rect[1][0] - rect[0][0]
        height_cm = rect[3][1] - rect[0][1]
        if width_cm < self._config.workspace_min_side_cm or height_cm < self._config.workspace_min_side_cm:
            return WorkspaceEstimate()

        origin_cam = (
            plane_point
            + (rect[0][0] / 100.0) * x_axis_cam
            + (rect[0][1] / 100.0) * y_axis_cam
        )

        return WorkspaceEstimate(
            ready=True,
            width_cm=float(width_cm),
            height_cm=float(height_cm),
            origin_cam_m=tuple(origin_cam.tolist()),
            x_axis_cam=tuple(x_axis_cam.tolist()),
            y_axis_cam=tuple(y_axis_cam.tolist()),
            normal_cam=tuple(normal_cam.tolist()),
            timestamp=time.time(),
        )

    def _estimate_floor_plane(
        self, markers: List[_MarkerInfo]
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        From each detected marker, compute a ground-plane sample:
        - Normal: marker's Z-axis (oriented so that it faces the camera)
        - Point: marker center projected downward by its known physical height
        Average all samples to get a robust plane estimate.
        """
        normals: List[np.ndarray] = []
        points: List[np.ndarray] = []

        for m in markers:
            n = m.normal_cam.astype(np.float64)
            n_len = float(np.linalg.norm(n))
            if n_len < 1e-6:
                continue
            n = n / n_len
            if n[2] < 0.0:
                n = -n

            center = m.tvec.reshape(3).astype(np.float64)
            height_m = m.height_cm / 100.0
            ground = center + n * height_m

            normals.append(n)
            points.append(ground)

        if not normals:
            return None, None

        mean_normal = np.mean(np.stack(normals), axis=0)
        n_len = float(np.linalg.norm(mean_normal))
        if n_len < 1e-6:
            return None, None
        mean_normal = (mean_normal / n_len).astype(np.float32)
        mean_point = np.mean(np.stack(points), axis=0).astype(np.float32)
        return mean_normal, mean_point

    @staticmethod
    def _build_plane_basis(normal: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Build an orthonormal (x, y) basis on the plane with the given normal."""
        ref = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        x_axis = ref - float(np.dot(ref, normal)) * normal
        if np.linalg.norm(x_axis) < 1e-6:
            ref = np.array([0.0, 1.0, 0.0], dtype=np.float32)
            x_axis = ref - float(np.dot(ref, normal)) * normal
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(normal, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        return x_axis.astype(np.float32), y_axis.astype(np.float32)

    def _project_image_corners_to_plane(
        self,
        plane_point: np.ndarray,
        plane_normal: np.ndarray,
        x_axis: np.ndarray,
        y_axis: np.ndarray,
    ) -> Optional[List[Point]]:
        """
        Cast rays from each image corner through the camera model onto
        the ground plane.  Return the 4 intersection points in plane-local
        coordinates (cm).
        """
        w, h = self._frame_size
        corners_px = np.array(
            [[0.0, 0.0], [w - 1.0, 0.0], [w - 1.0, h - 1.0], [0.0, h - 1.0]],
            dtype=np.float64,
        )
        K_inv = np.linalg.inv(self._K.astype(np.float64))
        plane_point_64 = plane_point.astype(np.float64)
        plane_normal_64 = plane_normal.astype(np.float64)
        x_axis_64 = x_axis.astype(np.float64)
        y_axis_64 = y_axis.astype(np.float64)

        quad: List[Point] = []
        for u, v in corners_px:
            ray = K_inv @ np.array([u, v, 1.0], dtype=np.float64)
            denom = float(np.dot(plane_normal_64, ray))
            if abs(denom) < 1e-8:
                return None
            t = float(np.dot(plane_normal_64, plane_point_64)) / denom
            if t <= 0.0:
                return None
            hit = ray * t
            delta = hit - plane_point_64
            quad.append((
                float(np.dot(delta, x_axis_64)) * 100.0,
                float(np.dot(delta, y_axis_64)) * 100.0,
            ))
        return quad

    @staticmethod
    def _inscribed_rect(
        quad: List[Point], margin_cm: float
    ) -> Optional[List[Point]]:
        """
        Compute the largest axis-aligned rectangle inscribed in a convex
        quadrilateral (given in order: TL, TR, BR, BL in plane coords).
        """
        if len(quad) != 4:
            return None
        tl, tr, br, bl = quad

        left = max(tl[0], bl[0]) + margin_cm
        right = min(tr[0], br[0]) - margin_cm
        top = max(tl[1], tr[1]) + margin_cm
        bottom = min(bl[1], br[1]) - margin_cm

        if right <= left or bottom <= top:
            return None
        return [(left, top), (right, top), (right, bottom), (left, bottom)]

    # ------------------------------------------------------------------
    # Coordinate transforms
    # ------------------------------------------------------------------

    @staticmethod
    def _camera_to_workspace_xy(
        point_cam: np.ndarray, workspace: WorkspaceEstimate
    ) -> Point:
        origin = np.array(workspace.origin_cam_m, dtype=np.float64)
        x_axis = np.array(workspace.x_axis_cam, dtype=np.float64)
        y_axis = np.array(workspace.y_axis_cam, dtype=np.float64)
        delta = point_cam.astype(np.float64) - origin
        return (
            float(np.dot(delta, x_axis)) * 100.0,
            float(np.dot(delta, y_axis)) * 100.0,
        )

    @staticmethod
    def _project_marker_center_to_floor(marker: _MarkerInfo) -> np.ndarray:
        """Project the marker center down to ground along its normal."""
        n = marker.normal_cam.astype(np.float64)
        n_len = float(np.linalg.norm(n))
        if n_len < 1e-6:
            return marker.tvec.reshape(3).astype(np.float64)
        n = n / n_len
        if n[2] < 0.0:
            n = -n
        center = marker.tvec.reshape(3).astype(np.float64)
        return center + n * (marker.height_cm / 100.0)

    def _project_pixel_to_floor(
        self, pixel_xy: np.ndarray, workspace: WorkspaceEstimate
    ) -> Optional[np.ndarray]:
        """Cast a ray from a pixel through the camera to the ground plane."""
        plane_origin = np.array(workspace.origin_cam_m, dtype=np.float64)
        plane_normal = np.array(workspace.normal_cam, dtype=np.float64)
        ray = np.linalg.inv(self._K.astype(np.float64)) @ np.array(
            [pixel_xy[0], pixel_xy[1], 1.0], dtype=np.float64
        )
        denom = float(np.dot(plane_normal, ray))
        if abs(denom) < 1e-8:
            return None
        t = float(np.dot(plane_normal, plane_origin)) / denom
        if t <= 0.0:
            return None
        return (ray * t).astype(np.float32)

    # ------------------------------------------------------------------
    # Car observations
    # ------------------------------------------------------------------

    def _build_car_observations(
        self,
        car_markers: Dict[int, _MarkerInfo],
        workspace: WorkspaceEstimate,
        timestamp: float,
    ) -> Dict[int, CarObservation]:
        observations: Dict[int, CarObservation] = {}
        offset = np.array(
            self._config.marker_center_to_wheel_center_offset_cm, dtype=np.float64
        )

        for car_id, marker in car_markers.items():
            center_cam = self._project_marker_center_to_floor(marker)
            center_xy = np.array(
                self._camera_to_workspace_xy(center_cam, workspace), dtype=np.float64
            )

            top_mid = 0.5 * (marker.image_corners[0] + marker.image_corners[1])
            bottom_mid = 0.5 * (marker.image_corners[2] + marker.image_corners[3])
            top_floor = self._project_pixel_to_floor(top_mid, workspace)
            bottom_floor = self._project_pixel_to_floor(bottom_mid, workspace)
            if top_floor is None or bottom_floor is None:
                continue

            top_xy = np.array(
                self._camera_to_workspace_xy(top_floor, workspace), dtype=np.float64
            )
            bottom_xy = np.array(
                self._camera_to_workspace_xy(bottom_floor, workspace), dtype=np.float64
            )
            heading = top_xy - bottom_xy
            yaw_deg = float(np.degrees(np.arctan2(heading[1], heading[0]))) % 360.0

            theta_rad = np.radians(yaw_deg)
            rot = np.array(
                [[np.cos(theta_rad), -np.sin(theta_rad)],
                 [np.sin(theta_rad),  np.cos(theta_rad)]],
                dtype=np.float64,
            )
            wheel_center = center_xy + rot @ offset

            observations[car_id] = CarObservation(
                car_id=car_id,
                x_cm=float(wheel_center[0]),
                y_cm=float(wheel_center[1]),
                yaw_deg=yaw_deg,
                timestamp=timestamp,
            )

        return observations

    # ------------------------------------------------------------------
    # Obstacle polygons
    # ------------------------------------------------------------------

    def _build_obstacle_polygons(
        self,
        obstacle_markers: Dict[int, _MarkerInfo],
        workspace: WorkspaceEstimate,
    ) -> List[List[Point]]:
        if not self._obstacle_marker_config:
            return []

        polygons: List[List[Point]] = []
        for marker in obstacle_markers.values():
            if marker.marker_id not in self._obstacle_marker_config:
                continue

            polygon_local = self._obstacle_marker_config[marker.marker_id]

            center_cam = self._project_marker_center_to_floor(marker)
            center_xy = np.array(
                self._camera_to_workspace_xy(center_cam, workspace), dtype=np.float64
            )

            top_mid = 0.5 * (marker.image_corners[0] + marker.image_corners[1])
            bottom_mid = 0.5 * (marker.image_corners[2] + marker.image_corners[3])
            top_floor = self._project_pixel_to_floor(top_mid, workspace)
            bottom_floor = self._project_pixel_to_floor(bottom_mid, workspace)
            if top_floor is None or bottom_floor is None:
                continue

            top_xy = np.array(
                self._camera_to_workspace_xy(top_floor, workspace), dtype=np.float64
            )
            bottom_xy = np.array(
                self._camera_to_workspace_xy(bottom_floor, workspace), dtype=np.float64
            )
            heading = top_xy - bottom_xy
            yaw = float(np.arctan2(heading[1], heading[0]))
            rot = np.array(
                [[np.cos(yaw), -np.sin(yaw)],
                 [np.sin(yaw),  np.cos(yaw)]],
                dtype=np.float64,
            )

            polygon_ws: List[Point] = []
            for pt_local_cm in polygon_local:
                pt = np.array([pt_local_cm[0], pt_local_cm[1]], dtype=np.float64)
                polygon_ws.append(tuple((center_xy + rot @ pt).tolist()))
            polygons.append(polygon_ws)

        return polygons

    # ------------------------------------------------------------------
    # Drawing helpers (NO HighGUI calls)
    # ------------------------------------------------------------------

    def _draw_car_axes(
        self,
        frame: np.ndarray,
        car_markers: Dict[int, "_MarkerInfo"],
    ) -> None:
        for marker in car_markers.values():
            cv2.drawFrameAxes(
                frame,
                self._K,
                self._D if self._D is not None else np.zeros(5),
                marker.rvec,
                marker.tvec,
                self._config.axis_length_m,
            )

    @staticmethod
    def _put_text(
        frame: np.ndarray,
        text: str,
        pos: Tuple[int, int],
        color: Tuple[int, int, int] = (0, 255, 0),
        scale: float = 0.5,
        thickness: int = 1,
    ) -> None:
        cv2.putText(
            frame, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness
        )

    @staticmethod
    def _draw_detected_markers(
        frame: np.ndarray,
        corners: list,
        ids: Optional[np.ndarray],
        color: Tuple[int, int, int],
    ) -> None:
        if ids is None or len(ids) == 0:
            return
        cv2.aruco.drawDetectedMarkers(frame, corners, ids, borderColor=color)

    def _draw_workspace_overlay(
        self,
        frame: np.ndarray,
        workspace: WorkspaceEstimate,
        car_observations: Dict[int, CarObservation],
    ) -> None:
        lock_state = self._ws_lock_state.state
        n_cand, n_req = len(self._ws_lock_state.candidates), self._ws_lock_state.required_frames

        if lock_state == "locked":
            text = f"WS: {workspace.width_cm:.1f} x {workspace.height_cm:.1f} cm [locked]"
            color = (0, 255, 0)
        elif workspace.ready:
            text = f"WS: {workspace.width_cm:.1f} x {workspace.height_cm:.1f} cm [{lock_state} {n_cand}/{n_req}]"
            color = (0, 200, 255)
        else:
            text = f"WS: not ready [{lock_state} {n_cand}/{n_req}]"
            color = (0, 0, 255)
        self._put_text(frame, text, (10, 30), color, 0.7, 2)

        n_cars = len(car_observations)
        self._put_text(
            frame, f"Cars: {n_cars}", (10, 60), (0, 255, 0), 0.7, 2
        )

        for car_obs in car_observations.values():
            label = f"{car_obs.car_id}: {car_obs.x_cm:.1f},{car_obs.y_cm:.1f}"
            if self._K is not None:
                pt_cam = self._workspace_xy_to_camera(
                    car_obs.x_cm, car_obs.y_cm, workspace
                )
                if pt_cam is not None:
                    px, _ = cv2.projectPoints(
                        pt_cam.reshape(1, 1, 3),
                        np.zeros(3), np.zeros(3),
                        self._K, self._D if self._D is not None else np.zeros(5),
                    )
                    px = px.reshape(2)
                    ix, iy = int(px[0]), int(px[1])
                    self._put_text(frame, label, (ix + 10, iy - 10), (255, 255, 255), 0.45, 1)

        if workspace.ready and self._K is not None:
            self._draw_workspace_boundary(frame, workspace)

    def _workspace_xy_to_camera(
        self, x_cm: float, y_cm: float, workspace: WorkspaceEstimate
    ) -> Optional[np.ndarray]:
        """Convert workspace (x_cm, y_cm) back to 3D camera coordinates."""
        if not workspace.ready:
            return None
        origin = np.array(workspace.origin_cam_m, dtype=np.float64)
        u = np.array(workspace.x_axis_cam, dtype=np.float64)
        v = np.array(workspace.y_axis_cam, dtype=np.float64)
        return (origin + u * (x_cm / 100.0) + v * (y_cm / 100.0)).astype(np.float32)

    def _draw_workspace_boundary(
        self, frame: np.ndarray, workspace: WorkspaceEstimate
    ) -> None:
        """Draw the workspace rectangle as a yellow polygon."""
        w, h = workspace.width_cm, workspace.height_cm
        corners_cm = [(0.0, 0.0), (w, 0.0), (w, h), (0.0, h)]
        pts_2d = []
        for cx, cy in corners_cm:
            pt3 = self._workspace_xy_to_camera(cx, cy, workspace)
            if pt3 is None:
                return
            px, _ = cv2.projectPoints(
                pt3.reshape(1, 1, 3),
                np.zeros(3), np.zeros(3),
                self._K, self._D if self._D is not None else np.zeros(5),
            )
            pts_2d.append(px.reshape(2).astype(int))
        pts_arr = np.array(pts_2d, dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(frame, [pts_arr], isClosed=True, color=(0, 255, 255), thickness=2)

    # ------------------------------------------------------------------
    # Camera / calibration / detector setup
    # ------------------------------------------------------------------

    def _setup_detectors(self) -> None:
        aruco = cv2.aruco
        dicts = {
            "DICT_4X4_50": aruco.DICT_4X4_50,
            "DICT_4X4_100": aruco.DICT_4X4_100,
            "DICT_5X5_50": aruco.DICT_5X5_50,
            "DICT_5X5_100": aruco.DICT_5X5_100,
            "DICT_6X6_50": aruco.DICT_6X6_50,
        }
        params = aruco.DetectorParameters()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

        car_dict_enum = dicts.get(self._config.car_dict, aruco.DICT_4X4_50)
        self._car_detector = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(car_dict_enum), params
        )

        obs_dict_enum = dicts.get(self._config.obstacle_dict, aruco.DICT_5X5_50)
        self._obstacle_detector = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(obs_dict_enum), params
        )

        if self._config.obstacle_marker_config_file:
            try:
                self._obstacle_marker_len_m, self._obstacle_marker_config = (
                    self._load_obstacle_config(self._config.obstacle_marker_config_file)
                )
                print(
                    f"[Observer] Loaded obstacle config with "
                    f"{len(self._obstacle_marker_config)} markers"
                )
            except Exception as exc:
                print(f"[Observer] Failed to load obstacle config: {exc}")
                self._obstacle_marker_config = {}

    @staticmethod
    def _load_obstacle_config(
        file_path: str,
    ) -> Tuple[float, Dict[int, List[Point]]]:
        with open(file_path, "r", encoding="utf-8") as f:
            config = json.load(f)

        marker_size_m = config.get("marker_size_mm", 40.0) / 1000.0
        markers = config.get("markers", {})

        marker_config: Dict[int, List[Point]] = {}
        for marker_id_str, marker_def in markers.items():
            try:
                marker_id = int(marker_id_str)
                polygon_local = marker_def.get("polygon_local", [])
                if polygon_local:
                    marker_config[marker_id] = [
                        (pt[0], pt[1]) for pt in polygon_local
                    ]
            except (ValueError, TypeError):
                continue

        return marker_size_m, marker_config

    def _open_camera(self) -> Optional[cv2.VideoCapture]:
        resolutions = {"480p": (640, 480), "720p": (1280, 720), "1080p": (1920, 1080)}
        width, height = resolutions.get(self._config.resolution, (1280, 720))
        system = platform.system()

        if system == "Linux":
            cap = cv2.VideoCapture(self._config.camera_device, cv2.CAP_V4L2)
        elif system == "Windows":
            cap = cv2.VideoCapture(self._config.camera_device, cv2.CAP_DSHOW)
        else:
            cap = cv2.VideoCapture(self._config.camera_device)

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, self._config.fps)
        cap.set(cv2.CAP_PROP_EXPOSURE, -6)
        return cap

    @staticmethod
    def _load_calibration(
        file_path: str,
    ) -> Tuple[np.ndarray, np.ndarray, Tuple[int, int]]:
        with open(file_path, "r", encoding="utf-8") as f:
            calib = yaml.safe_load(f)
        K = np.array(calib["camera_matrix"], dtype=np.float32)
        D = np.array(calib["distortion_coefficients"], dtype=np.float32)
        size = tuple(calib["image_size"][:2])
        return K, D, size
