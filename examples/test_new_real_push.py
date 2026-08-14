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
import dataclasses
import threading
import time

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication

from micromvp.controller import NavigationController
from micromvp.coordinator import NavigationCoordinator
from micromvp.env import NewRealPushEnv, NewRealPushConfig, v3_config, v4_config
from micromvp.gui import MVPWindow


def main():
    parser = argparse.ArgumentParser(description="NewRealPushEnv navigation test")
    parser.add_argument(
        "--car-version", type=int, default=4, choices=[3, 4],
        help="Car hardware version (default: 4)",
    )
    parser.add_argument(
        "--camera", type=int, default=0,
        help="Camera device index (default: 0)",
    )
    parser.add_argument(
        "--serial-port", type=str, default="/dev/tty.usbmodem101",
        help="Serial port for ESP-NOW bridge (default: /dev/tty.usbmodem31301)",
    )
    parser.add_argument(
        "--warmup", type=int, default=30,
        help="Warmup frames for workspace estimation (default: 30)",
    )
    parser.add_argument(
        "--no-preview", action="store_true", default=False,
        help="Disable camera preview window",
    )
    parser.add_argument(
        "--max-speed", type=float, default=0.3,
        help="Maximum robot speed [0-1] (default: 0.3)",
    )
    parser.add_argument(
        "--port", type=int, default=8080,
        help="Web server port (default: 8080)",
    )
    parser.add_argument(
        "--timeout", type=float, default=10.0,
        help="Workspace ready timeout in seconds (default: 10)",
    )
    args = parser.parse_args()

    # ---- 1. Build env config from preset + CLI overrides ----
    base_config = v4_config if args.car_version == 4 else v3_config
    config = dataclasses.replace(
        base_config,
        camera_device=args.camera,
        serial_port=args.serial_port,
        warmup_frames=args.warmup,
        no_preview=args.no_preview,
    )

    env = NewRealPushEnv(config)

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
        rid: NavigationController(
            rid,
            ws_config,
            max_speed=args.max_speed,
        )
        for rid in ws_config.car_id_list
    }

    active_id = ws_config.car_id_list[0]

    # ---- 4. Create coordinator ----
    coordinator = NavigationCoordinator(
        ws_config,
        controllers,
        active_robot_id=active_id,
        webserver_port=args.port,
    )

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
                "default": args.max_speed,
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
            {"type": "label", "text": f"Port: {args.port}"},
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
    print("NewRealPushEnv Navigation")
    print("=" * 60)
    print(f"  Workspace : {ws_config.width:.1f} × {ws_config.height:.1f} cm")
    print(f"  Cars      : {ws_config.car_id_list}")
    print(f"  Active    : {active_id}")
    print(f"  Web API   : http://localhost:{args.port}")
    print(f"  Max speed : {args.max_speed}")
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
