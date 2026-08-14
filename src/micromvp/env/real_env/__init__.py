"""
real_env – Real hardware environment with adaptive workspace and ESP-NOW protocol.

Configured entirely from the deployment YAML; see config/car_v4.yaml.
"""
from .real_env import RealEnv

__all__ = ["RealEnv"]
