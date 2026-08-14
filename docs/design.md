# 多机器人系统重构设计文档（Env / Car / Coordinator）

## 1. 设计目标（Design Goals）

本系统旨在构建一个**可扩展、可复用、可同时支持仿真与真实系统（Sim / Real）**的多机器人控制框架，满足以下目标：

1. **单车独立性**

   * 每辆小车是一个独立的控制实体（agent）
   * 拥有自己的状态、控制器和高层行为接口（goto / follow_path）
   * 不依赖全局控制器即可独立运行

2. **环境解耦（Env ↔ Car 解耦）**

   * 环境不关心小车如何决策
   * 小车不关心环境如何执行动作（仿真 / ZMQ / 网络 / 硬件）
   * Env 与 Car 只通过 **Observation / Action 数据结构**交互

3. **统一 Sim / Real 执行模型**

   * 同一套控制代码可以在 SimEnv 和 RealEnv 上运行
   * 区别仅体现在 Env 的内部实现

4. **高层协同可选**

   * 提供一个可选的 Coordinator，用于多车任务编排
   * 用户可以完全绕过 Coordinator，自行编写多车控制逻辑

---

## 2. 总体架构概览

系统由三个核心组件构成：

```
┌─────────────┐
│   Env       │   ← 世界 / 通信 / 执行
│ (Sim / Real)│
└─────▲───────┘
      │ Observation
      │
┌─────┴───────┐
│ Coordinator │   ← 可选，高层调度 / 协同
│  (while)    │
└─────▲───────┘
      │ 调用高层API
      │
┌─────┴───────┐
│   Car       │   ← 独立 agent（状态 + controller）
│ (N agents)  │
└─────────────┘
```

**关键原则**：

* Env **永远不 import / 不调用 Car**
* Car **永远不直接操作 Env 的执行接口**
* Coordinator 只是一个 *while loop 的封装*

---

## 3. Env（环境组件）

### 3.1 核心职责

Env 是**世界与执行的适配层（world adapter）**，只负责两件事：

1. **获取环境反馈（Observation）**
2. **执行动作（Action）**

Env 不包含任何控制逻辑。

---

### 3.2 Env 对外接口语义

#### 3.2.1 获取观测

```python
obs = env.step()
```

或（等价拆分）：

```python
obs = env.observe()
```

返回内容：

```text
obs: Dict[robot_id, RobotObservation]
```

每个 `RobotObservation` 至少包含：

* robot_id
* pose (x, y, theta)
* timestamp
* valid / confidence（RealEnv 中尤其重要）

说明：

* **SimEnv**：从模拟器读取 GT 或加噪声后的状态
* **RealEnv**：从相机 / ArUco / 视觉系统中估计状态
* Env 不保证每一帧所有 robot 都有有效观测（Real 中可能丢帧）

---

#### 3.2.2 执行动作

```python
env.apply_actions(actions)
```

其中：

```text
actions: Dict[robot_id, Action]
```

说明：

* Action 是纯数据（例如左右轮速度 / v,w）
* **SimEnv**：直接推进动力学
* **RealEnv**：编码、打包、通过 ZMQ/网络广播发送给各车

Env **不关心 action 从哪里来**，只负责执行。

---

### 3.3 Env 不应该做的事（非常重要）

Env **不应该**：

* ❌ import Car / Controller
* ❌ 调用 car.get_action()
* ❌ 管理 goto / follow / 状态机
* ❌ 决定避障策略或控制律

Env 是 **被动执行者 + 状态提供者**。

---

## 4. Car（小车组件 / Agent）

Car 是系统中**最核心的独立实体**。

---

### 4.1 Car 的三类职责

#### 4.1.1 静态属性（Identity & Capability）

这些是**运行期不变的常量**：

* robot_id（系统内部唯一 ID）
* tag_id（视觉系统使用的 ArUco / 标记 ID）
* 物理参数：

  * wheel_base
  * wheel_radius
  * 最大速度 / 角速度
* action_space 类型（diff-drive / v,w / raw）

这些信息 **不属于 Env**。

---

#### 4.1.2 Runtime State（运行时状态）

由 Car 自己维护、完全由 Observation 更新：

* 当前 pose（来自 obs）
* 速度估计（可选）
* controller 内部状态：

  * 当前模式（IDLE / GOTO / FOLLOW）
  * 当前任务是否完成
  * 内部计时 / 失败标志等

**重要**：
Car 的 state **不依赖 Env 的内部实现**，只依赖 Observation 数据。

---

#### 4.1.3 Controller / Policy（核心）

Car 内部包含一个 controller（或 policy），提供统一接口：

```python
action = car.get_action(obs)
```

controller 的行为由高层 API 改变：

* `car.goto(pose, tolerance=...)`
* `car.follow_path(points, loop=...)`
* `car.stop()`

这些 API **不会直接执行动作**，只会：

* 修改 controller 的目标
* 修改内部状态机
* 影响后续 `get_action()` 的输出

---

### 4.2 Path 的设计约定

* 用户提供的初始形式：**点列**（List[(x,y)] 或 Waypoint）
* Car 内部可以将其封装为 Path 对象：

  * 预计算弧长
  * 提供 `project()` / `point_at_s()`
* 用户无需感知 Path 对象的存在

---

## 5. Coordinator（调度器，可选）

Coordinator 是一个**高层控制逻辑的封装**，本质是一个 while loop。

---

### 5.1 Coordinator 的职责

* 调用 Env 获取观测
* 将观测分发给各 Car
* 调用 Car 的高层 API（goto / follow）
* 根据 Car 的任务状态决定下一步行为
* 收集所有 Car 的 action 并交给 Env 执行

---

### 5.2 Coordinator 的典型使用场景

示例：多车绕圈

1. 用户定义一个圆形轨迹（密集点列）
2. Coordinator 均匀采样起始点
3. 对每辆车：

   * 调用 `car.goto(start_pose)`
4. 等待所有 car.task_state == DONE
5. 对每辆车：

   * 调用 `car.follow_path(circle_path_from_its_start)`
6. 循环运行

---

### 5.3 Coordinator 的可选性

* 用户 **可以不使用 Coordinator**
* 用户可以直接：

  * 自己写 while loop
  * 自己调用 car.get_action()
  * 自己实现多车协同算法

Coordinator 只是：

* 官方示例
* 高层逻辑模板
* 便于复用的调度工具

---

## 6. 时间推进与 Tick 的统一原则

* **Env 拥有节拍（tick）**
* Car / Controller **不自行驱动时间**
* 每个 tick：

  1. `obs = env.step()`
  2. `action_i = car_i.get_action(obs_i)`
  3. `env.apply_actions(actions)`

RealEnv 中：

* `step()` 可能只是等待新一帧视觉结果

SimEnv 中：

* `step()` 明确推进仿真时间

**对上层代码而言，两者一致。**

---

## 7. 关键边界检查（用于防止架构退化）

如果出现以下情况，说明设计被破坏：

* Env 调用 car.get_action()
* Car 直接发送 ZMQ / 网络包
* Controller 内依赖 SimEnv 的内部对象
* 每辆 Car 各自跑一个控制线程并直接执行
* 协同逻辑混入单车 controller

---



## 9. GUI 组件（观测与控制）
GUI 应当作为一个外部观察者和全局调度器的控制台存在。

9.1 GUI 的双重角色
Passive Visualizer（被动显示器）：

输入来源：直接订阅或读取 env.observe() 返回的 obs。

职责：将 obs 里的位姿信息实时绘制在屏幕上。

原则：不修改任何底层状态，仅做视觉映射（Mapping）。

Active Controller（主动控制器）：

操作对象：通过 Coordinator 暴露的接口修改参数。

职责：

设置全局 car_speed。

切换 pattern（如 Circle / 8-shape）。

控制步进（Pause / Step / Resume）。

## 8. 总结

本重构目标是将系统明确拆分为：

* **Env**：世界与执行
* **Car**：独立智能体
* **Coordinator**：高层协同（可选）

该结构：

* 支持 Sim / Real 统一
* 支持多车独立运行
* 支持未来引入学习策略 / 分布式控制 / 更复杂协同
