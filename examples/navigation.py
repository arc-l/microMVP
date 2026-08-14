"""
Test navigation control with NewRealPushEnv (adaptive workspace + ESP-NOW).

This example demonstrates:
- NewRealPushEnv with adaptive workspace estimation (no fixed ground markers)
- NavigationController with pure pursuit + CTE-PD path following
- NavigationCoordinator with web server API and RVG obstacle avoidance
- GUI controls for speed, rotation, and hand-drawn paths

Web API (default port 8080):
    POST /setup_obstacle  - Set obstacle geometry
    POST /goto            - Navigate to position with orientation
    POST /follow_path     - Follow a waypoint sequence
    GET  /status          - Get current robot status

GUI Controls:
    Draw curve on canvas  - Robot follows the drawn path
    Click on canvas       - Robot navigates to clicked point (with RVG)
    Speed slider          - Adjust robot movement speed
    Rotation input        - Enter angle (0-359.99) and press Enter
    Click on car          - Select active robot
    C                     - Clear current task
    Space                 - Emergency stop
    Escape                - Quit

Web API Examples:
    curl -X POST http://localhost:8080/setup_obstacle \
        -H "Content-Type: application/json" \
        -d '{"obstacles": [[[10,10], [20,10], [20,20], [10,20]]]}'

    curl -X POST http://localhost:8080/goto \
        -H "Content-Type: application/json" \
        -d '{"x": 30, "y": 20, "theta": 45}'

    curl http://localhost:8080/status
"""
import argparse
import threading
import time

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication

from micromvp.config import ConfigError, load_config
from micromvp.controller import NavigationController
from micromvp.coordinator import NavigationCoordinator
from micromvp.env import NewRealPushEnv
from micromvp.gui import MVPWindow


def main():
    parser = argparse.ArgumentParser(description="MicroMVP navigation")
    parser.add_argument(
        "--config", type=str, default="config/car_v4.yaml",
        help="Deployment config describing your hardware (default: config/car_v4.yaml)",
    )
    parser.add_argument(
        "--timeout", type=float, default=10.0,
        help="Seconds to wait for the workspace to lock (default: 10)",
    )
    args = parser.parse_args()

    # ---- 1. Load the one config that describes this setup ----
    try:
        cfg = load_config(args.config)
    except ConfigError as exc:
        print(f"[main] {exc}")
        return

    env = NewRealPushEnv(cfg)

    # ---- 2. Start env and wait for workspace + car discovery ----
    print("[main] Starting environment …")
    if not env.start(wait_for_ready=True, timeout=args.timeout):
        print("[main] Failed to start environment.")
        return

    ws_config = env.workspace_config
    print(f"[main] Workspace ready: {ws_config.width:.1f} × {ws_config.height:.1f} cm")
    print(f"[main] Detected cars: {ws_config.car_id_list}")

    if not ws_config.car_id_list:
        print("[main] No cars detected – exiting.")
        env.close()
        return

    # ---- 3. Create controllers (after workspace is ready) ----
    controllers = {
        rid: NavigationController.from_config(rid, ws_config, cfg)
        for rid in ws_config.car_id_list
    }

    # ---- 4. Create coordinator ----
    coordinator = NavigationCoordinator.from_config(ws_config, controllers, cfg)
    active_id = coordinator.active_robot_id

    # Values the GUI needs to display, read from the same config.
    max_speed = cfg.require("control.max_speed", float, who="navigation example")
    web_port = cfg.require("navigation.webserver_port", int, who="navigation example")

    # Anything in the config that nothing read is almost always a typo.
    for field in cfg.unused_fields():
        print(f"[main] warning: config field '{field}' was not used by anything")

    # ---- 5. Setup GUI ----
    gui_config = {
        "canvas": {
            "click_canvas_callback": True,
            "draw_curve_callback": True,
        },
        "control_panel": [
            {"type": "label", "text": "=== Navigation ==="},
            {"type": "label", "text": "Draw path / click to go"},
            {"type": "label", "text": ""},
            {
                "type": "continuous_slider",
                "label": "Robot Speed",
                "range": [0.0, 1.0],
                "default": max_speed,
                "callback_name": "set_robot_speed",
            },
            {"type": "label", "text": ""},
            {
                "type": "input",
                "label": "Rotate to:",
                "placeholder": "0-359.99",
                "callback_name": "set_rotation_target",
            },
            {"type": "label", "text": ""},
            {"type": "label", "text": "=== Web API ==="},
            {"type": "label", "text": f"Port: {web_port}"},
            {"type": "label", "text": "POST /setup_obstacle"},
            {"type": "label", "text": "POST /goto"},
            {"type": "label", "text": "POST /follow_path"},
            {"type": "label", "text": "GET /status"},
            {"type": "label", "text": ""},
            {"type": "label", "text": "=== Keyboard ==="},
            {"type": "label", "text": "  C: Clear task"},
            {"type": "label", "text": "  Space: Stop"},
            {"type": "label", "text": "  Escape: Quit"},
        ],
    }
    gui = MVPWindow(gui_config, ws_config)

    running = True

    def on_key_press(key: str):
        nonlocal running
        if key == "escape":
            running = False
            gui.close_window()
        else:
            coordinator.on_key_press(key)

    gui.register_callback("on_key_press", on_key_press)
    gui.register_callback("on_key_release", coordinator.on_key_release)
    gui.register_callback("on_car_click", coordinator.on_car_click)
    gui.register_callback("on_curve_drawn", coordinator.on_curve_drawn)
    gui.register_callback("on_canvas_click", coordinator.on_canvas_click)
    gui.register_callback("set_robot_speed", coordinator.on_speed_change)
    gui.register_callback("set_rotation_target", coordinator.on_rotation_input)

    # ---- 6. Logic loop (background thread) ----
    latest_observations = {}
    obs_lock = threading.Lock()

    def logic_loop():
        while running:
            t0 = time.time()

            observations = env.observe()
            obstacles = env.get_obstacles()

            if observations:
                actions = coordinator.process(observations, aruco_obstacles=obstacles)
                env.apply_actions(actions)

                with obs_lock:
                    latest_observations.update(observations)

            elapsed = time.time() - t0
            time.sleep(max(0, (1.0 / ws_config.frequency) - elapsed))

    logic_thread = threading.Thread(target=logic_loop, daemon=True)
    logic_thread.start()

    # ---- 7. Render timer (Qt main thread) ----
    def main_thread_update():
        if not running:
            return

        env.render()

        with obs_lock:
            obs = latest_observations.copy() if latest_observations else {}

        if obs:
            car_states = {s.car_id: s for s in coordinator.gather_car_state()}
            drawings = coordinator.get_additional_drawings()
            gui.update(car_states, drawings)

    render_timer = QTimer()
    render_timer.timeout.connect(main_thread_update)
    render_timer.start(33)  # ~30 FPS

    # ---- 8. Print instructions and run ----
    print()
    print("=" * 60)
    print("MicroMVP Navigation")
    print("=" * 60)
    print(f"  Workspace : {ws_config.width:.1f} × {ws_config.height:.1f} cm")
    print(f"  Cars      : {ws_config.car_id_list}")
    print(f"  Active    : {active_id}")
    print(f"  Web API   : http://localhost:{web_port}")
    print(f"  Max speed : {max_speed}")
    print(f"  Config    : {cfg.source}")
    print("=" * 60)

    gui.run()  # blocks until window closed

    # ---- 9. Cleanup ----
    running = False
    render_timer.stop()
    print("\n[main] Shutting down …")
    coordinator.close()
    env.close()
    print("[main] Done.")


if __name__ == "__main__":
    main()
