"""
MicroMVP Environment Module.

An environment owns sensing and actuation. It reports where the robots are
and applies wheel commands, without knowing anything about control logic.

- Environment: Abstract base class
- SimEnv: In-memory simulation, no hardware required
- NewRealPushEnv: Real hardware — ArUco tracking with an adaptive workspace,
  plus motor commands over the Xiao ESP-NOW serial gateway
"""

from .base import Environment
from .sim_env import SimConfig, SimEnv
from .new_real_push_env import NewRealPushEnv, NewRealPushConfig, v3_config, v4_config

__all__ = [
    "Environment",
    "SimConfig",
    "SimEnv",
    "NewRealPushEnv",
    "NewRealPushConfig",
    "v3_config",
    "v4_config",
]
