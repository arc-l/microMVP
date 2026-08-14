"""
MicroMVP 机器人控制系统核心数据模型

本模块定义了跨组件通信的标准数据结构，采用“环境定义约束，控制器驱动逻辑”的解耦设计：

1. 环境配置 (WorkspaceConfig): 
   由 Env 提供，定义物理空间的尺寸、小车几何参数、动力学极限及系统步频。
   所有度量单位（如像素、厘米）均以此为准。

2. 交互接口 (Observation/Action): 
   Env 与 Controller 之间的标准 I/O。Observation 反馈真实物理位置，Action 输出归一化的控制指令。

3. 控制器配置与目标 (CarControllerConfig/TargetPose): 
   定义控制算法的超参数（如前瞻距离比例）及运动目标。参数设计采用比例化（Ratio），
   使其能自动适配不同尺寸的物理环境。

4. 状态快照与元数据 (CarState/WorldState): 
   CarState 是控制器内部逻辑的“黑盒”快照，包含状态标签及灵活的 metadata 字典，
   支持不同算法自定义扩展字段。
   CarState can be used by Coordinator for high-level decision making and by GUI for rendering.

设计原则：
- 统一单位：所有位置和速度计算均遵循 WorkspaceConfig 指定的环境度量（Env Metrics）。
- 状态内聚：控制器self-defined 内部状态通过 metadata 传递，保持全局模型简洁。
- 观察即事实：RobotObservation 仅包含 Env 探测到的真实信息，缺失即不报，无伪观测。
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple, Any


Point = Tuple[float, float]
Pose = Tuple[float, float, float]  # (x, y, theta)


@dataclass(slots=True)
class WorkspaceConfig:
    """
    工作空间配置信息，由 Env 提供。
    所有度量单位均使用该环境的统一单位（如：像素 Pixel）。
    """
    # 1. 尺寸定义 (以左下角为原点 0,0)
    width: float
    height: float

    # 2. 小车外廓尺寸 (用于渲染及碰撞参考)
    car_width: float
    car_height: float

    # 3. 小车中心点偏移 (w_offset, h_offset)
    # 当小车朝向 Y 轴正方向时，两轮轴心点相对于车身左下角的坐标
    offset_w: float
    offset_h: float

    # 4. 动力学物理参数
    wheel_base: float      # 两轮间距 (同 Workspace 单位)
    max_wheel_speed: float # 单轮最大线速度 (单位/秒)

    # 5. 系统频率
    frequency: float       # Env.step() 的每秒执行次数

    # 6. Number of robots/cars in this workspace
    car_id_list: list

    # 7. marker 中心 -> 两轮轴心的向量，车头朝 Y 轴正方向时测量。
    # 位姿上报的是轮轴中心，而人眼在画面里对照的是 marker，
    # 所以渲染需要这个偏移把车贴图摆回 marker 的位置。
    marker_to_axle_offset: Tuple[float, float] = (0.0, 0.0)

    @property
    def dt(self) -> float:
        # return 每次时间步长 (秒)
        return 1.0 / self.frequency if self.frequency > 0 else 0.0

@dataclass(slots=True)
class RobotObservation:
    """
    Observation data for a single robot from the environment.

    This is the output of Env.observe() for each robot.
    Env produces this; Controller consumes this.
    """
    robot_id: int # given by env
    x: float # same unit as workspace
    y: float
    theta: float  # (0,359.99), use X's positive direction as 0 degree
    timestamp: float = field(default_factory=time.time)

    @property
    def pose(self) -> Pose:
        return (self.x, self.y, self.theta)

    @property
    def position(self) -> Point:
        return (self.x, self.y)


@dataclass(slots=True)
class Action:
    """
    Action command for a single robot.

    This is the input to Env.apply_actions() for each robot.
    Car produces this; Env consumes this.
    """
    left_speed: float   # Left wheel speed, normalized [-1, 1]
    right_speed: float  # Right wheel speed, normalized [-1, 1]

    @classmethod
    def stop(cls) -> "Action":
        """Create a stop action."""
        return cls(left_speed=0.0, right_speed=0.0)

    def as_tuple(self) -> Tuple[float, float]:
        return (self.left_speed, self.right_speed)


@dataclass(slots=True)
class CarControllerConfig:
    """
    Every car have its own controller.
    Here is the configuration for the controller of a single car. 
    These properties do not change during runtime.


    NOTE: The definition of the car size/speed/etc is provided by the environment, 
          with the assumption that every car is the same, with only ID differs.
    """

    # Lookahead parameters for path following, ratio based on car size.
    lookahead_base_ratio: float = 1.0   # Base lookahead distance
    lookahead_min_ratio: float = 0.5    # Minimum lookahead
    lookahead_max_ratio: float = 2.0    # Maximum lookahead
    lookahead_k_v_ratio: float = 0.3     # Velocity-dependent lookahead factor

    # Soft turn parameters
    omega_soft_dist: float = 27.0  # Distance threshold for soft turning
    omega_min_scale: float = 0.25  # Minimum angular velocity scale



@dataclass(slots=True)
class TargetPose:
    """
    One important function of the controller is to drive the car to a target pose.
    """
    x: float
    y: float
    tolerance_pos_ratio: float          # Position tolerance in percentage of car size
    tolerance_theta: float              # Orientation tolerance in radians
    theta: Optional[float] = None       # Sometimes we do not care about theta, but if provided, we will try to reach it.

    @property
    def position(self) -> Point:
        return (self.x, self.y)

    @property
    def pose(self) -> Pose:
        return (self.x, self.y, self.theta if self.theta is not None else 0.0)


@dataclass(slots=True)
class CarState:
    """
    小车运行时的完整状态快照。
    由 Controller 维护并更新，用于 Coordinator 决策和 GUI 渲染。
    """
    car_id: int
    # 基础物理状态 (基于最新观测和内部估计)
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    
    # 当前estimated 速度状态
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    
    # 核心状态机标签 (使用字符串或 Metadata 以支持不同控制器)
    # 让控制器自己定义字符串标签
    status_label: str = "IDLE" 
    
    # 控制器特有的元数据 (e.g. target point, target path, etc)
    metadata: Dict[str, Any] = field(default_factory=dict)