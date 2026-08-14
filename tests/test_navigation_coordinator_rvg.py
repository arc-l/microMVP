"""Path-planning tests that need the external RVG planner.

Excluded from the default run: `import rvg` succeeds against the bare rvg/
directory as a namespace package, so importorskip cannot tell whether the
compiled extension is really usable. Run explicitly with:

    pytest -m rvg
"""
import math

import pytest

pytestmark = pytest.mark.rvg

from micromvp.controller.base import Controller
from micromvp.coordinator.navigation_coordinator.navigation_coordinator import (
    NavigationCoordinator,
)
from micromvp.core.models import CarState, WorkspaceConfig


class _DummyController(Controller):
    def __init__(self, car_id: int, x: float, y: float, theta: float) -> None:
        self._car_state = CarState(car_id=car_id, x=x, y=y, theta=theta)

    def step(self, observation):
        raise NotImplementedError

    @property
    def car_state(self) -> CarState:
        return self._car_state

    def update(self, observation):
        raise NotImplementedError

    def calculate_action(self):
        raise NotImplementedError


def _make_coordinator(monkeypatch: pytest.MonkeyPatch) -> NavigationCoordinator:
    monkeypatch.setattr(NavigationCoordinator, "_start_webserver", lambda self: None)
    ws = WorkspaceConfig(
        width=50.0,
        height=30.0,
        car_width=4.2,
        car_height=4.8,
        offset_w=2.1,
        offset_h=4.5,
        wheel_base=4.2,
        max_wheel_speed=10.0,
        frequency=30.0,
        car_id_list=[1],
    )
    controllers = {1: _DummyController(1, 10.0, 15.0, 0.0)}
    return NavigationCoordinator(
        ws,
        controllers,
        active_robot_id=1,
        webserver_port=0,
        robot_geometry_scale=1.0,
    )


def test_plan_path_preserves_initial_rvg_rotation(monkeypatch: pytest.MonkeyPatch):
    coord = _make_coordinator(monkeypatch)
    obstacles = [
        [(20.0, 0.0), (22.0, 0.0), (22.0, 12.0), (20.0, 12.0)],
        [(20.0, 18.0), (22.0, 18.0), (22.0, 30.0), (20.0, 30.0)],
    ]

    path = coord._plan_path((10.0, 15.0, 0.0), (40.0, 15.0, 0.0), obstacles)

    assert path is not None
    assert len(path) >= 3
    assert path[0] == (10.0, 15.0)
    assert path[-1] == (40.0, 15.0)

    first_heading_deg = math.degrees(
        math.atan2(path[1][1] - path[0][1], path[1][0] - path[0][0])
    )
    assert 3.0 <= first_heading_deg <= 7.0


def test_default_robot_geometry_uses_conservative_front_extent(
    monkeypatch: pytest.MonkeyPatch,
):
    coord = _make_coordinator(monkeypatch)

    front_extent = max(x for x, _ in coord._robot_geometry)
    upper_half_width = max(y for _, y in coord._robot_geometry)
    lower_half_width = abs(min(y for _, y in coord._robot_geometry))

    assert front_extent >= 2.1
    assert front_extent >= upper_half_width
    assert front_extent >= lower_half_width
