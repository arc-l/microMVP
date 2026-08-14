# Camera Calibration

使用 ChArUco 标定板进行相机内参标定。

## 步骤

### 1. 生成标定板

```bash
conda activate micromvp
python calibration/generate_board.py
```

输出 `calibration/charuco_board.png` 和 `charuco_board.pdf`。

**打印时注意**：选择 100% 缩放（不要 fit-to-page），打印后用尺量一下方格边长是否为 30mm。

### 2. 标定

将打印的标定板固定在平面上（如贴在桌面或墙上），然后：

```bash
python calibration/calibrate_camera.py --camera 0 --resolution 720p
```

操作方式：
- 手持摄像头对准标定板，从不同角度和距离晃动
- 按 **SPACE** 捕获一帧（或加 `--auto` 自动捕获）
- 采集 20-40 帧，尽量覆盖画面各区域和多种倾斜角度
- 按 **c** 开始标定计算

标定完成后自动保存到 `src/micromvp/env/real_env/camera.yaml`。

### 3. 验证

```bash
python tests/test_new_env_integration.py --camera 0 --port /dev/tty.usbmodem3101 --car-id 6
```

观察 workspace lock 是否稳定，坐标是否准确。

## 参数说明

### generate_board.py

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--cols` | 7 | 棋盘列数 |
| `--rows` | 5 | 棋盘行数 |
| `--square` | 30 | 方格边长 (mm) |
| `--marker` | 22 | ArUco marker 边长 (mm) |
| `--dpi` | 300 | 输出分辨率 |

### calibrate_camera.py

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--camera` | 0 | 摄像头设备号 |
| `--resolution` | 720p | 标定分辨率（必须与运行时一致） |
| `--auto` | off | 自动捕获模式 |
| `-o` | env 目录下 camera.yaml | 输出路径 |

## 注意事项

- 标定分辨率必须与 env 运行时分辨率一致（默认 720p = 1280×720）
- ChArUco 使用 DICT_6X6_250，不会与小车 marker (4x4) 和障碍物 marker (5x5) 冲突
- RMS 重投影误差 < 0.5 为优秀，< 1.0 为可用
