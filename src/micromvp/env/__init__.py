"""
MicroMVP Environment Module.

Provides environment interfaces for robot control:
- Environment: Abstract base class
- SimEnv: In-memory simulation environment
- RealPushEnv: Real hardware environment (1st Spring St setup)
- NewRealPushEnv: Real hardware with adaptive workspace + ESP-NOW protocol
"""

from .base import Environment
from .sim_env import SimConfig, SimEnv
from .real_push_env import RealPushEnv, RealPushConfig
from .new_real_push_env import NewRealPushEnv, NewRealPushConfig, v3_config, v4_config

__all__ = [
    "Environment",
    "SimConfig",
    "SimEnv",
    "RealPushEnv",
    "RealPushConfig",
    "NewRealPushEnv",
    "NewRealPushConfig",
    "v3_config",
    "v4_config",
]
