"""
MicroMVP Environment Module.

An environment owns sensing and actuation. It reports where the robots are
and applies wheel commands, without knowing anything about control logic.

- Environment: Abstract base class
- SimEnv: In-memory simulation, no hardware required
- RealEnv: Real hardware — ArUco tracking with an adaptive workspace,
  plus motor commands over the Xiao ESP-NOW serial gateway
"""

from .base import Environment
from .sim_env import SimConfig, SimEnv
from .real_env import RealEnv

__all__ = [
    "Environment",
    "SimConfig",
    "SimEnv",
    "RealEnv",
]
