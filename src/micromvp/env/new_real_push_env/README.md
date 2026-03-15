# new_real_push_env

基于自适应地面平面估计 + Xiao ESP-NOW 串口协议的真实硬件环境模块。

## 与 real_push_env 的区别

| | real_push_env | new_real_push_env |
|---|---|---|
| 坐标系定义 | 固定地面 ArUco marker (DICT_5X5) | 自适应：从车+障碍物 marker 反推地面平面 |
| 串口协议 | 聚合协议 | Xiao ESP-NOW 帧格式 `0xAA 0x55 \| level \| len=32 \| payload(32) \| xor` |
| Workspace 初始化 | 已知固定尺寸 | 多帧聚合锁定（默认 30 帧稳定后 lock） |
| 障碍物 | JSON polygon_local 配置 | 同上，沿用 JSON polygon_local |

## 文件结构

```
new_real_push_env/
├── __init__.py              # 公共 API：NewRealPushEnv, NewRealPushConfig, v3_config, v4_config
├── new_real_push_env.py     # 主环境类，组合 observer + action sender
├── observer.py              # ArUco 相机观测 + 自适应 workspace 估计
├── serial_action.py         # ESP-NOW 串口动作发送
├── camera.yaml              # 相机标定文件（默认）
└── README.md
```

## 公共 API

只通过包级 import 暴露以下 4 个名字：

```python
from micromvp.env.new_real_push_env import (
    NewRealPushEnv,
    NewRealPushConfig,
    v3_config,
    v4_config,
)
```

内部类（`ArucoObserver`、`ObserverConfig`、`SerialActionSender` 等）不在公共 API 内，需要时请从具体模块路径显式 import。

## 快速使用

```python
from micromvp.env.new_real_push_env import NewRealPushEnv, NewRealPushConfig

config = NewRealPushConfig(
    serial_port="/dev/tty.usbmodem3101",
    calibration_file="path/to/camera.yaml",
    robot_ids=[1, 6],
)
env = NewRealPushEnv(config)

if env.start(wait_for_ready=True, timeout=10.0):
    obs = env.observe()
    env.apply_actions({1: Action(left_speed=0.5, right_speed=0.5)})
    env.render()  # 必须在 GUI 主线程调用

env.close()
```

## Workspace 锁定机制

Observer 启动后进入三阶段状态机：

```
collecting  →  stable  →  locked
```

- **collecting**：累积 `ready=True` 的候选帧。任何一帧 `ready=False` 会清空窗口重新开始。
- **stable**：窗口满（默认 30 帧）且通过稳定性检查（width/height 波动、origin 漂移、normal 夹角均在阈值内）。
- **locked**：对窗口内候选做聚合（标量取 median，向量归一化后取 mean），得到最终 workspace，此后不再更新。

相关配置字段（`ObserverConfig`）：

| 字段 | 默认值 | 说明 |
|---|---|---|
| `workspace_lock_frames` | 30 | 需要多少连续 ready 帧 |
| `workspace_width_tolerance_cm` | 2.0 | width 波动阈值 |
| `workspace_height_tolerance_cm` | 2.0 | height 波动阈值 |
| `workspace_origin_tolerance_m` | 0.01 | origin 漂移阈值 |
| `workspace_normal_angle_tolerance_deg` | 3.0 | normal 最大夹角 |

## 分辨率校验

`start()` 在 warmup 后读取首帧，将实际分辨率与 `camera.yaml` 中 `image_size` 对比。不一致时打印 FATAL 信息并返回 `False`，不会静默继续运行。

## 串口协议

与 `xiao/xiao_ap_ESP_NOW.ino` 配合。每帧结构：

```
[0xAA] [0x55] [level] [0x20] [payload x 32] [xor_checksum]
```

- 每个 level 容纳 `cars_per_level`（默认 10）辆车
- 每辆车占 3 字节：`[左轮, 右轮]` 位于 `payload[3*index-1]` 和 `payload[3*index]`
- 编码：sign-magnitude，bit7=符号位，bits[6:0]=幅值 0..127

## 自动发现车辆

`_sync_robot_ids_from_observer()` 会将 observer 检测到的车辆 ID 自动注册为可控制对象。这是设计选择（方便"所有可见车辆都可控"的场景）。如需显式注册，后续可添加 `auto_register_observed_cars` 开关。

## 测试

```bash
# 在 micromvp conda 环境中运行
conda activate micromvp
python -m pytest tests/test_serial_action.py tests/test_observer_workspace.py tests/test_new_real_push_env_api.py -v
```

测试覆盖：
- `test_serial_action.py` — level/slot 映射、帧构建、checksum、wheel invert、多 level 打包（27 项）
- `test_observer_workspace.py` — 多帧聚合状态机：窗口未满不 lock、稳定后 lock、波动不 lock、聚合正确性（12 项）
- `test_new_real_push_env_api.py` — 包级导出检查、APStatusInfo 已删除确认（4 项）

## 离线视觉分析

```bash
python scripts/offline_workspace_analysis.py \
    --input video.mp4 \
    --calibration src/micromvp/env/new_real_push_env/camera.yaml \
    --output-dir workspace_analysis_output
```

输出：
- `summary.json` — 是否 lock、lock 帧号、最终 workspace 参数
- `candidates.csv` — 每帧候选 workspace 数据
- `overlay/*.png` — 标注后的帧图片，供人工复核
