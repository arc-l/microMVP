# MicroMVP

A multi-robot control framework for small differential-drive cars tracked by
an overhead camera. Robots carry ArUco markers, an overhead camera works out
where they are, and wheel commands go back over an ESP-NOW link.

The same controller code runs against simulation or real hardware, because
control logic never talks to hardware directly — it goes through three
layers you can replace one at a time.

```
┌───────────────────────────────────────────────────────────────────┐
│                               GUI                                 │
│        draws the workspace, takes clicks and drawn paths          │
└───────────────────────────────────────────────────────────────────┘
                                 ↑↓
┌───────────────────────────────────────────────────────────────────┐
│                           Coordinator                             │
│    distributes observations, collects actions, plans paths,       │
│    assigns tasks, bridges the GUI                                 │
└───────────────────────────────────────────────────────────────────┘
                                 ↑↓
┌─────────────────┐   ┌─────────────────┐   ┌─────────────────┐
│  Controller 1   │   │  Controller 2   │   │  Controller N   │
│  one per robot  │   │  one per robot  │   │  one per robot  │
│  pose → wheels  │   │  pose → wheels  │   │  pose → wheels  │
└─────────────────┘   └─────────────────┘   └─────────────────┘
                                 ↑↓
┌───────────────────────────────────────────────────────────────────┐
│                           Environment                             │
│    SimEnv or RealEnv — observe() reports poses,                   │
│    apply_actions() drives the wheels                              │
└───────────────────────────────────────────────────────────────────┘
```

The Coordinator is the only layer that touches the Environment: it passes
each robot's observation down to that robot's Controller, and sends the
collected actions back.

---

## Install

```bash
git clone <this repo>
cd micromvp_v4
pip install -e .
```

Python 3.12+. Pulls in PyQt6, NumPy, OpenCV (contrib, for ArUco), pyserial
and PyYAML.

Optional extras:

```bash
pip install -e ".[calibration]"   # matplotlib, for rendering a calibration board
pip install -e ".[dev]"           # pytest
```

The RVG path planner is optional and installed separately — see
[Path planning](#path-planning). Without it the system still runs; it plans
straight lines instead.

---

## Quick start

### 1. Connect the Xiao AP

Plug the AP board into USB. `actuation.serial_port` is `auto` by
default, so there is nothing to configure — the port is found at startup.

To check which port it landed on:

```bash
python -m hardware_test.find_ap
```

```
  /dev/cu.usbmodem101 ... AP  frame_ok=19 bad_ck=0 over 3s
```

### 2. Check the wheels turn

Put a car on the floor with room around it, and give it the id you flashed
into its firmware:

```bash
python -m hardware_test.check_motion --cars 3
```

It drives forward, backward, counter-clockwise, then clockwise, pausing
between each so you can watch.

### 3. Connect the camera

Plug the camera in and point it at the area you want to drive in. On a Mac,
Photo Booth is the quickest way to see what it sees.

There is no strict requirement on angle or height. The workspace is derived
from the view itself, so all that matters is that the camera covers the
area you want to work in.

### 4. Run the demo

```bash
python examples/navigation.py --config config/car_v4.yaml
```

On startup it prints the workspace it measured and the cars it found:

```
[main] Workspace ready: 90.2 × 51.4 cm
[main] Detected cars: [3]
```

### 5. Drive it

In the window: draw a curve on the canvas and the robot follows it, or
click a point and it plans a path there. Click a car to select it. The
sidebar has a speed slider and a "rotate to" box. `C` clears the current
task, `Space` stops, `Escape` quits.

There is also an HTTP API on port 8080 for driving it from your own code:

```bash
curl -X POST http://localhost:8080/goto \
     -H "Content-Type: application/json" \
     -d '{"x": 30, "y": 20, "theta": 45}'
```

Full API in [docs/navigation_api.md](docs/navigation_api.md).

---

## Concepts

MicroMVP is built from three components.

**Controller** does the low-level motion control of a single robot: follow
this path, rotate to that heading. One controller per robot.

**Environment** is the world the robots act in. It changes as they move,
and it gives you observations — where each robot is and which way it faces.
`SimEnv` simulates it; `RealEnv` is the real one, seen through a camera.

**Coordinator** is in charge overall. It takes the observations from the
environment and hands each controller what it needs, and it handles
everything that involves more than one robot — collision avoidance, path
planning, deciding who goes where.

### Environment — takes actions, returns observations

It applies the actions it is given, which changes the state of the world,
and it reports observations of that state.

```python
observations = env.observe()               # {car_id: RobotObservation(x, y, theta, t)}
env.apply_actions({3: Action(0.5, 0.5)})   # left/right wheel thrust, -1..1
```

That is the entire interface. How the state changes and where the
observations come from is up to the implementation: `SimEnv` steps a model
in memory, `RealEnv` reads poses from the overhead camera and sends wheel
commands to the AP. Anything implementing `observe` / `apply_actions`
works.

`RealEnv` also derives the workspace itself — there are no calibration
markers on the floor. It fits the ground plane from the markers it can see,
projects the camera's field of view onto that plane, and takes the largest
rectangle inside it. Details in
[src/micromvp/env/real_env/README.md](src/micromvp/env/real_env/README.md).

### Controller — low-level motion control for one robot

One instance per robot. You give it a task — a path to follow, a heading to
rotate to — and it keeps its own state while carrying that task out, one
`step()` per observation.

```python
controller = NavigationController.from_config(robot_id, ws_config, cfg)
controller.set_path([(10, 10), (20, 15), (30, 10)])
action = controller.step(observation)
```

Shipped controllers:

| Controller | What it does |
|---|---|
| `NavigationController` | Pure pursuit + cross-track-error PD, with in-place rotation. The one the example uses. |
| `PurePursuitFollowPathController` | Plain pure pursuit. |
| `PurePursuit_PD_FollowPathController` | Pure pursuit with a PD correction term. |
| `StanleyFollowPathController` | Stanley steering. |
| `TargetFollowController` | Chase a moving point. |
| `WASDController` | Keyboard driving. |

To write your own, subclass `Controller` and implement three methods:

```python
class MyController(Controller):
    def update(self, observation):      # absorb the new pose
        self._car_state.x = observation.x
        self._car_state.y = observation.y
        self._car_state.theta = observation.theta

    def calculate_action(self):         # decide what to do about it
        return Action(left_speed=0.0, right_speed=0.0)

    def step(self, observation):
        self.update(observation)
        return self.calculate_action()
```

### Coordinator — coordination across robots

Processes the observations from the environment, hands each controller what
it needs, and collects the actions back. Anything spanning more than one
robot belongs here: collision avoidance, path planning, formations, task
assignment. It is also what the GUI talks to.

```python
actions = coordinator.process(observations)
env.apply_actions(actions)

car_states = coordinator.gather_car_state()          # for rendering
drawings = coordinator.get_additional_drawings()     # overlays
```

Shipped coordinators: `NavigationCoordinator` (single robot, obstacle
avoidance, HTTP API), `KeyboardCoordinator`, `FollowPathCoordinator`,
`FormationCoordinator`.

### Putting it together

The whole main loop, with nothing left out:

```python
from micromvp.config import load_config
from micromvp.controller import NavigationController
from micromvp.coordinator import NavigationCoordinator
from micromvp.env import RealEnv

cfg = load_config("config/car_v4.yaml")

env = RealEnv(cfg)
env.start(wait_for_ready=True, timeout=10.0)
ws_config = env.workspace_config

controllers = {
    rid: NavigationController.from_config(rid, ws_config, cfg)
    for rid in ws_config.car_id_list
}
coordinator = NavigationCoordinator.from_config(ws_config, controllers, cfg)

while True:
    observations = env.observe()
    actions = coordinator.process(observations)
    env.apply_actions(actions)
    time.sleep(1 / ws_config.frequency)
```

`examples/navigation.py` is this plus a GUI and a render timer.

### Which layer do I change?

| I want to… | Change |
|---|---|
| use different hardware or a different tracker | Environment |
| change how a robot follows a path | Controller |
| coordinate several robots, assign tasks | Coordinator |
| change the planner | `planner.name` in the config |
| change a physical dimension or a gain | the config, never code |

---

## Configuration

One YAML file describes one physical setup, completely. To reproduce a
deployment, copy the file.

```bash
cp config/car_v4.yaml config/my_table.yaml
python examples/navigation.py --config config/my_table.yaml
```

Every module reads its fields out of that file, so there are no per-module
config classes to learn. `config/car_v4.yaml` is commented throughout and
is the reference.

| Section | Covers |
|---|---|
| `car` | ArUco dictionary, marker edge length and height, body size, axle offset, wheel base |
| `obstacle` | ArUco dictionary, marker size and height, and each marker id's polygon |
| `camera` | device, resolution, fps, calibration file, preview |
| `workspace` | margins and how steady the estimate must be before it locks |
| `tracking` | per-car outlier rejection |
| `actuation` | serial port, baud, send rate, wheel inversion |
| `runtime` | main loop frequency |
| `control` | speed ceiling, lookahead, goal tolerance, rotation behaviour |
| `navigation` | web API port, active robot, planner safety margin |
| `planner` | which planner, and its parameters |

Fields are **required**, not optional. A missing one stops startup and says
exactly what to do:

```
missing required field 'car.marker_size_mm'
  needed by : ArucoObserver
  config    : config/my_table.yaml
  add it to the 'car' section.
  that section currently has: aruco_dict, axle_offset_cm, body_height_cm, ...
```

A field the file contains but nothing reads — usually a typo — is reported
on startup too.

### Measuring the physical fields

- **`car.marker_size_mm`** — the side of the marker's *black border*, not
  including the white quiet zone. ArUco infers distance from apparent size,
  so this scales every distance the system reports.
- **`car.marker_height_cm`** — how far the marker plane sits above the
  floor, i.e. the height of the car.
- **`obstacle.marker_height_cm`** — same for obstacle blocks. Use `0.0` if
  the markers lie flat on the ground.

Cars and obstacles must use different ArUco dictionaries (4x4 and 5x5 by
default), so that a car and an obstacle sharing an id stay distinguishable.

---

## Path planning

`NavigationCoordinator` plans around obstacles when a planner is available.

- **`planner.name: straight`** — straight line from start to goal. Ignores
  obstacles. Always available.
- **`planner.name: rvg`** — a rotational visibility graph, which plans in
  SE(2) and so accounts for the robot's shape *and* heading. Needs the RVG
  extension built and importable.

RVG is not vendored in this repo; it is a separate C++ project with Python
bindings. Build it and make sure `import rvg` works, then set
`planner.name: rvg`. If it is selected but not importable, the system warns
once and falls back to straight lines.

The RVG-dependent tests are excluded from the default run and need the
plugin present:

```bash
pytest -m rvg
```

---

## Camera calibration

The vision system needs your camera's intrinsics. `config/camera.yaml`
ships with a working set, but it describes *our* camera — recalibrate for
yours.

```bash
python calibration/generate_board.py     # produces a ChArUco board to print
python calibration/calibrate_camera.py   # capture views, solve, write camera.yaml
```

See [calibration/README.md](calibration/README.md).

Recalibrate whenever the lens focus changes: focus changes focal length,
and the intrinsics stop matching.

---

## Troubleshooting

### The AP is not found

`python -m hardware_test.find_ap` probes every candidate port by sending
real frames and watching the AP's own `[STAT]` counter climb, so a
device that merely accepts bytes cannot be mistaken for it. Nothing moves
during the probe — the frames carry zero thrust.

If it reports nothing:

- The AP shows up as `/dev/cu.usbmodem*` on macOS, `/dev/ttyACM*` on
  Linux, `COM*` on Windows. Compare `ls /dev/tty*` before and after
  plugging it in.
- Close anything holding the port open; an Arduino IDE serial monitor will
  block it.
- Confirm `xiao/xiao_ap_ESP_NOW.ino` is flashed to the board.

With more than one serial device attached, `auto` takes the first match and
says so. Pin the right one in the config to remove the guesswork:

```yaml
actuation:
  serial_port: /dev/cu.usbmodem101
```

### The workspace never locks

Symptom: the lock counter climbs, then drops to zero, over and over. It
never reaches `workspace.lock_frames`, and startup times out with
`workspace not ready`.

The usual cause is **the camera hunting for focus**, and it is not obvious
from watching the video.

Contrast-detection autofocus moves the lens and compares sharpness to find
a peak. Point it at plain carpet, a bare table, or a white wall and there
is no peak to find, so it searches forever: sharp for half a second, blurred
for half a second, repeating. Blurred frames detect no markers, and a frame
with no markers clears the accumulated window, so the count can never reach
30.

Confirm it by watching the preview for a second or two — you will see it
breathe in and out on a regular cycle.

Fixes, in order of how well they hold:

1. **Lock focus in the camera itself.** OpenCV cannot do this on macOS —
   `CAP_PROP_AUTOFOCUS` and friends all fail to set. Use the vendor's
   utility, or `uvc-util -I 0 -s auto-focus=0` on macOS, `v4l2-ctl
   --set-ctrl=focus_automatic_continuous=0` on Linux. Then recalibrate,
   since the focal length is now different.
2. **Give the autofocus something to hold onto.** A sheet of white paper or
   a board under the workspace is usually enough — the markers plus the
   paper's edges provide the contrast the carpet did not. This is a
   workaround: move the paper or change the lighting and the hunting can
   come back.
3. **Loosen `workspace.tolerance`** — only if the image is genuinely sharp
   and the estimate is merely noisy. It will not help against hunting.

### Markers are detected but distances are wrong

Everything looks self-consistent on screen but the numbers are off. Check
`car.marker_size_mm` and `obstacle.marker_size_mm` against the actual
printed markers, measuring the black border only. A marker declared 30 mm
that is really 40 mm puts the whole workspace 25% closer than it is.

A good cross-check: the system should agree with itself. If cars and
obstacles disagree about where the floor is, one of the two marker
descriptions is wrong.

### A car ignores commands

`python -m hardware_test.check_motion --cars <id>` isolates this from the
vision system entirely. If the wheels still do not turn, the `CAR_ID` in
the car's firmware does not match the id you are sending to.

Note that ESP-NOW broadcasts are not acknowledged: the AP reporting
`send_ok` means the AP sent the packet, not that any car heard it.

### The web API will not start

Port 8080 is in use. Change `navigation.webserver_port`.

---

## Repository layout

```
config/            one YAML per deployment, plus camera intrinsics
examples/          navigation.py — the worked example
hardware_test/     bring-up checks you run by hand against real hardware
calibration/       ChArUco board generation and camera calibration
src/micromvp/
  config.py        the config loader every module reads through
  core/            data models, differential-drive kinematics, path patterns
  env/             Environment interface, SimEnv, RealEnv
  controller/      per-robot control algorithms
  coordinator/     multi-robot orchestration and the GUI bridge
  gui/             PyQt6 window, canvas, sidebar
xiao/              ESP32 firmware — car and AP
tests/             pytest suite, no hardware needed
docs/              web API, GUI spec, original design notes
```

---

## Coordinate system

- Origin at the bottom-left of the workspace, `(0, 0)`
- **X** right, **Y** up
- **Theta** 0° along +X, increasing counter-clockwise
- Distances in centimetres; wheel commands normalised to `-1..1`

The workspace origin is tied to the camera's field of view, not to anything
on the floor. That is why the estimate is locked once and then frozen — move
the camera and every coordinate from before the move becomes meaningless.

---

## Tests

```bash
pytest              # 63 tests, no hardware required
pytest -m rvg       # additionally requires the RVG plugin
```
