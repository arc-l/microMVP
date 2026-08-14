# real_env — 真机环境

ArUco 视觉定位 + Xiao ESP-NOW 串口执行。实现 `Environment` 接口，只负责
「机器人在哪」和「把轮速发出去」，不含任何控制逻辑。

配置全部来自部署 YAML（见 `config/car_v4.yaml`），本模块不带自己的默认值。

## 文件

```
real_env/
├── __init__.py         # 公共 API：RealEnv
├── real_env.py         # 主类，组合 observer + action sender
├── observer.py         # 相机、ArUco 检测、地平面拟合、工作区锁定
└── serial_action.py    # ESP-NOW 串口协议编码与发送线程
```

## 用法

```python
from micromvp.config import load_config
from micromvp.env import RealEnv

env = RealEnv(load_config("config/car_v4.yaml"))
env.start(wait_for_ready=True, timeout=10.0)

observations = env.observe()              # {car_id: RobotObservation}
env.apply_actions({3: Action(0.2, 0.2)})  # 左右轮 [-1, 1]
obstacles = env.get_obstacles()           # [[(x, y), ...], ...]
env.render()                              # 预览窗口，必须在 GUI 主线程调用
env.close()
```

## 工作区是怎么定出来的

地面上**不需要**任何标定物。流程是：

1. 每个可见 marker 贡献一个地平面样本 —— 法向量取该 marker 的 Z 轴，
   地面点由 marker 中心沿法向量下推它的已知高度（`car.marker_height_cm`
   / `obstacle.marker_height_cm`）得到。所有样本平均成一个平面。
2. 相机视野的四个角经内参反投影，与该平面求交，得到地面上一个四边形。
3. 取四边形内最大的轴对齐矩形，四边各缩 `workspace.margin_cm`。
4. 原点定在该矩形左下角，X 向右、Y 向上。

坐标轴朝向由**相机姿态**决定，与白板、marker 位置无关。所以估计值必须
先「锁定」：连续 `workspace.lock_frames` 帧都稳定在
`workspace.tolerance` 之内才提交，之后不再改变，整个会话的坐标才可比。

`is_workspace_ready()` 返回的就是「是否已锁定」。

### 锁不上怎么办

最常见的原因不是参数，是**相机在反复对焦**。对比度检测式自动对焦在纯色
或低对比度背景（地毯、白墙）上找不到锐度峰值，会周期性地来回搜索。虚焦
帧检不到 marker，而任何一帧没有 marker 都会清空累积窗口，于是永远凑不满
`lock_frames`。

症状是锁定计数器反复归零。解决办法见 README 的 Troubleshooting 一节。

## 串口协议

PC → 网关（USB 串口，帧格式）：

```
0xAA 0x55 | level(1) | len(1=32) | payload(32) | checksum_xor(1)
```

网关 → 小车（ESP-NOW 广播）：`level(1) + payload(32)`

payload 中每辆车占 3 字节槽位，`level = (car_id - 1) // cars_per_level`，
槽内左右轮各一字节，采用符号-幅值编码（bit7 符号，bits[6:0] 幅值 0..127）。
与 `xiao/xiao_1_8_ESP_NOW.ino` 的解析一一对应。

命令以 `actuation.send_hz` 持续重发，不是事件触发；发送流一旦中断，
小车会自行停下。

## 自动发现车辆

相机看到的任何车 marker 都会被自动注册为可控机器人，无需预先声明 id。
若需要显式白名单，改 `_sync_robot_ids_from_observer()`。
