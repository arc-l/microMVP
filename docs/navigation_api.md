# Navigation System Example

This example demonstrates the Navigation system with web server API and obstacle avoidance capabilities.

## Overview

The Navigation system provides:
- **NavigationController**: Path following with in-place rotation support
- **NavigationCoordinator**: Web server API, GUI integration, and RVG-based path planning

## Quick Start

```bash
# Basic usage (robot ID 1, camera 0, port 8080)
python examples/test_real_env_navigation.py

# Specify robot ID and port
python examples/test_real_env_navigation.py --robot 2 --port 9000

# Full options
python examples/test_real_env_navigation.py \
    --robot 1 \
    --camera 0 \
    --warmup 30 \
    --max-speed 0.3 \
    --port 8080 \
    --calib /path/to/camera.yaml
```

## Command Line Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--robot` | 1 | Robot ID to control |
| `--camera` | 0 | Camera device index |
| `--warmup` | 30 | Warmup frames (0=dynamic mode) |
| `--no-preview` | False | Disable camera preview window |
| `--calib` | (default path) | Camera calibration file path |
| `--max-speed` | 0.3 | Maximum robot speed [0-1] |
| `--lookahead` | auto | Lookahead distance for path following |
| `--port` | 8080 | Web server port |

## GUI Controls

### Canvas Interaction
- **Draw curve**: Click and drag to draw a path. The robot will follow it.

### Control Panel
- **Speed slider**: Adjust the robot's movement speed (0-1)
- **Rotation input**: Enter an angle (0-359.99) and press Enter to rotate the robot

### Keyboard Shortcuts
| Key | Action |
|-----|--------|
| C | Clear current task |
| Space | Emergency stop |
| Escape | Quit application |

## Web Server API

The web server runs on the specified port (default: 8080) and provides RESTful endpoints.

### GET /status
Get current robot status.

**Response:**
```json
{
    "status": "ok",
    "robot_id": 1,
    "position": {"x": 150.5, "y": 200.3, "theta": 45.0},
    "state": "IDLE"
}
```

**Example:**
```bash
curl http://localhost:8080/status
```

### POST /setup_obstacle
Set obstacles for path planning. Each obstacle is a polygon defined by vertices in counter-clockwise order.

**Request body:**
```json
{
    "obstacles": [
        [[100, 100], [200, 100], [200, 200], [100, 200]],
        [[300, 150], [350, 150], [350, 250], [300, 250]]
    ]
}
```

**Response:**
```json
{
    "status": "ok",
    "obstacle_count": 2
}
```

**Example:**
```bash
curl -X POST http://localhost:8080/setup_obstacle \
    -H "Content-Type: application/json" \
    -d '{"obstacles": [[[100,100], [200,100], [200,200], [100,200]]]}'
```

### POST /goto
Navigate the robot to a target position with final orientation. Uses RVG path planning to avoid obstacles.

**Request body:**
```json
{
    "x": 300,
    "y": 200,
    "theta": 45
}
```

**Response:**
```json
{
    "status": "ok",
    "path_length": 5,
    "target": {"x": 300, "y": 200, "theta": 45}
}
```

**Example:**
```bash
curl -X POST http://localhost:8080/goto \
    -H "Content-Type: application/json" \
    -d '{"x": 300, "y": 200, "theta": 45}'
```

### POST /follow_path
Follow a sequence of waypoints without final rotation.

**Request body:**
```json
{
    "path": [[100, 100], [200, 150], [300, 100], [400, 200]]
}
```

**Response:**
```json
{
    "status": "ok",
    "path_length": 4
}
```

**Example:**
```bash
curl -X POST http://localhost:8080/follow_path \
    -H "Content-Type: application/json" \
    -d '{"path": [[100,100], [200,150], [300,100]]}'
```

## Priority Rules

**GUI commands take priority over web server commands.**

When a GUI action occurs (drawing a path, entering rotation angle), any pending web server command is cancelled and the GUI command is executed instead.

## Robot States

The robot reports the following states:

| State | Description |
|-------|-------------|
| `IDLE` | No task assigned |
| `FOLLOWING` | Following a path |
| `FINISHED` | Reached end of path |
| `ROTATING` | Rotating in-place to target angle |
| `ROTATION_STABLE` | Aligned with target, holding stable |
| `ROTATION_DONE` | Rotation completed |

## Rotation Behavior

When using `goto` or the rotation input:
1. Robot rotates slowly in-place toward the target angle
2. Once within ±5 degrees of target, robot holds position
3. After 0.5 seconds of stable alignment, rotation is complete

## Path Planning

If the RVG library is available, the `goto` command uses visibility graph path planning to find obstacle-avoiding paths. If RVG is not installed, direct paths are used.

**Installing RVG:**
```bash
# RVG should already be available in the project
# If not, refer to the rvg directory in the repository
```

## Example Python Client

```python
import requests

base_url = "http://localhost:8080"

# Set obstacles
obstacles = [
    [[100, 100], [200, 100], [200, 200], [100, 200]]
]
requests.post(f"{base_url}/setup_obstacle", json={"obstacles": obstacles})

# Navigate to position
requests.post(f"{base_url}/goto", json={"x": 300, "y": 200, "theta": 90})

# Check status
response = requests.get(f"{base_url}/status")
print(response.json())

# Follow a path
path = [[50, 50], [100, 100], [150, 50], [200, 100]]
requests.post(f"{base_url}/follow_path", json={"path": path})
```

## Prerequisites

1. **Camera Setup**: ArUco-based tracking camera calibrated
2. **Workspace Markers**: 5x5 ArUco markers for workspace boundary
3. **Robot Markers**: 4x4 ArUco markers on robots
4. **Dependencies**: PyQt6, numpy, opencv-python

## Troubleshooting

### Web server fails to start
- Check if the port is already in use: `lsof -i :8080`
- Try a different port with `--port`

### Robot not detected
- Ensure ArUco markers are visible to camera
- Check camera calibration file path
- Wait for warmup to complete

### Path planning returns direct path
- RVG library may not be installed
- Check console for path planning errors

### Robot doesn't rotate accurately
- The ±5 degree tolerance is by design for stability
- Ensure robot has clear space for rotation
