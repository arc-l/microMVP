"""
Test keyboard control with NewRealPushEnv (adaptive workspace + ESP-NOW).

Logs every frame's key input + observation to a CSV for post-analysis.
On exit (Escape or window close), saves:
  - keyboard_log.csv  (t, keys, action_l, action_r, x, y, theta)
  - keyboard_log_plot.png (if matplotlib available)

Controls:
    1-9        Select robot by number
    Tab        Cycle to next robot
    W/A/S/D    Move selected robot
    Space      Emergency stop all robots
    Click car  Select robot
    Escape     Quit
"""
import csv
import threading
import time
import argparse

from micromvp.gui import MVPWindow
from micromvp.env import NewRealPushEnv, NewRealPushConfig
from micromvp.controller import WASDController
from micromvp.coordinator import KeyboardCoordinator
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication


def handle_canvas_click(x, y):
    print(f"[GUI Event] Canvas clicked at Workspace coordinates: ({x:.1f}, {y:.1f})")


def main():
    parser = argparse.ArgumentParser(description="NewRealPushEnv keyboard control test")
    parser.add_argument(
        "--robots", type=str, default="",
        help="Robot IDs as 'id,id,...' (e.g., '1,2,3'). Empty = auto-discover"
    )
    parser.add_argument(
        "--camera", type=int, default=0,
        help="Camera device index (default: 0)"
    )
    parser.add_argument(
        "--warmup", type=int, default=30,
        help="Warmup frames for workspace estimation (default: 30)"
    )
    parser.add_argument(
        "--no-preview", action="store_true", default=False,
        help="Disable camera preview window"
    )
    parser.add_argument(
        "--calib", type=str, default="",
        help="Camera calibration file path (default: built-in camera.yaml)"
    )
    parser.add_argument(
        "--max-speed", type=float, default=0.5,
        help="Initial robot speed [0-1] (default: 0.5)"
    )
    parser.add_argument(
        "--serial-port", type=str, default="/dev/tty.usbmodem3101",
        help="Serial port for ESP-NOW bridge (default: /dev/tty.usbmodem3101)"
    )
    parser.add_argument(
        "--car-marker-size", type=float, default=36.0,
        help="Car marker size in mm (default: 36.0)"
    )
    parser.add_argument(
        "--output", type=str, default="keyboard_log.csv",
        help="Output CSV path (default: keyboard_log.csv)"
    )
    args = parser.parse_args()

    # Parse robot IDs (empty = auto-discover)
    robot_ids = []
    if args.robots:
        for entry in args.robots.split(","):
            entry = entry.strip()
            if entry:
                robot_ids.append(int(entry))

    if robot_ids:
        print(f"Configuring {len(robot_ids)} robot(s): {robot_ids}")
    else:
        print("No robot IDs specified — will auto-discover from camera")

    # Setup environment
    config = NewRealPushConfig(
        robot_ids=robot_ids,
        camera_device=args.camera,
        warmup_frames=args.warmup,
        no_preview=args.no_preview,
        serial_port=args.serial_port,
        car_marker_size_mm=args.car_marker_size,
    )
    if args.calib:
        config.calibration_file = args.calib

    env = NewRealPushEnv(config)

    # Start environment
    print("\nStarting environment, waiting for workspace to settle...")

    if not env.start(wait_for_ready=True, timeout=10.0):
        print("Failed to start environment (camera or workspace issue)")
        return

    ws_config = env.workspace_config
    print(f"Environment ready! Workspace: {ws_config.width:.1f} x {ws_config.height:.1f} cm")
    print(f"Detected cars: {ws_config.car_id_list}")

    if not ws_config.car_id_list:
        print("No cars detected — exiting. Try placing a car marker in camera view.")
        env.close()
        return

    # Create controllers for each robot
    controllers = {
        robot_id: WASDController(robot_id, ws_config, max_speed=args.max_speed)
        for robot_id in ws_config.car_id_list
    }

    # Create keyboard coordinator
    coordinator = KeyboardCoordinator(ws_config, controllers)

    # ---- Logging state ----
    log_rows = []
    log_lock = threading.Lock()
    t0 = time.time()

    # Setup GUI
    gui_config = {
        "canvas": {
            "click_canvas_callback": True,
            "draw_curve_callback": True,
        },
        "control_panel": [
            {"type": "label", "text": "=== Real Robot Control ==="},
            {"type": "label", "text": "1-9: Select robot"},
            {"type": "label", "text": "WASD: Move robot"},
            {"type": "label", "text": "Tab: Next robot"},
            {"type": "label", "text": "Space: Stop all"},
            {"type": "label", "text": ""},
            {
                "type": "continuous_slider",
                "label": "Robot Speed",
                "range": [0.0, 1.0],
                "default": args.max_speed,
                "callback_name": "set_robot_speed",
            },
            {"type": "label", "text": ""},
            {"type": "label", "text": f"Cars: {ws_config.car_id_list}"},
            {"type": "label", "text": f"Logging to: {args.output}"},
        ],
    }
    gui = MVPWindow(gui_config, ws_config)

    # Track running state
    running = True

    # Register keyboard callbacks
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
    gui.register_callback("on_canvas_click", handle_canvas_click)
    gui.register_callback("set_robot_speed", coordinator.set_speed)

    # Shared state between threads
    latest_observations = {}
    obs_lock = threading.Lock()

    # Logic loop
    def logic_loop():
        while running:
            start_time = time.time()

            observations = env.observe()

            if observations:
                actions = coordinator.process(observations)
                env.apply_actions(actions)

                with obs_lock:
                    latest_observations.update(observations)

                # ---- Log every frame ----
                keys_snapshot = sorted(coordinator.pressed_keys)
                active_id = coordinator.active_robot_id
                for car_id, obs in observations.items():
                    act = actions.get(car_id)
                    row = {
                        "t": round(time.time() - t0, 4),
                        "car_id": car_id,
                        "active": 1 if car_id == active_id else 0,
                        "keys": "+".join(keys_snapshot) if keys_snapshot else "",
                        "action_l": round(act.left_speed, 4) if act else 0,
                        "action_r": round(act.right_speed, 4) if act else 0,
                        "x": round(obs.x, 3),
                        "y": round(obs.y, 3),
                        "theta": round(obs.theta, 2),
                        "obs_ts": round(obs.timestamp, 4),
                    }
                    with log_lock:
                        log_rows.append(row)

            elapsed = time.time() - start_time
            sleep_time = max(0, (1 / ws_config.frequency) - elapsed)
            time.sleep(sleep_time)

    # Start logic thread
    logic_thread = threading.Thread(target=logic_loop, daemon=True)
    logic_thread.start()

    # Print instructions
    print("\n" + "=" * 50)
    print("NewRealPushEnv — Keyboard Control (with logging)")
    print("=" * 50)
    print("Controls:")
    print("  1-9    : Select robot")
    print("  Tab    : Cycle to next robot")
    print("  W/A/S/D: Move selected robot")
    print("  Space  : Emergency stop")
    print("  Escape : Quit & save log")
    print("  Click  : Select robot")
    print(f"  Output : {args.output}")
    print("=" * 50)

    # QTimer callback runs in Qt main thread
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

    # Run GUI (blocks until closed)
    gui.run()

    # ---- Cleanup & save ----
    running = False
    render_timer.stop()
    env.close()

    with log_lock:
        rows = list(log_rows)

    if not rows:
        print("\nNo data logged.")
        return

    fieldnames = ["t", "car_id", "active", "keys", "action_l", "action_r",
                  "x", "y", "theta", "obs_ts"]
    with open(args.output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nSaved {len(rows)} rows to {args.output}")

    # ---- Quick plot ----
    try:
        import numpy as np
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        car_id = rows[0]["car_id"]
        car_rows = [r for r in rows if r["car_id"] == car_id]

        ts = np.array([r["t"] for r in car_rows])
        xs = np.array([r["x"] for r in car_rows])
        ys = np.array([r["y"] for r in car_rows])
        thetas = np.array([r["theta"] for r in car_rows])
        als = np.array([r["action_l"] for r in car_rows])
        ars = np.array([r["action_r"] for r in car_rows])

        fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)

        axes[0].plot(ts, als, "r-", linewidth=0.8, label="left")
        axes[0].plot(ts, ars, "b-", linewidth=0.8, label="right")
        axes[0].set_ylabel("Action")
        axes[0].legend(loc="upper right")
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(ts, xs, "b-", linewidth=0.5)
        axes[1].set_ylabel("X (cm)")
        axes[1].grid(True, alpha=0.3)

        axes[2].plot(ts, ys, "g-", linewidth=0.5)
        axes[2].set_ylabel("Y (cm)")
        axes[2].grid(True, alpha=0.3)

        axes[3].plot(ts, thetas, "m-", linewidth=0.5)
        axes[3].set_ylabel("Theta (deg)")
        axes[3].set_xlabel("Time (s)")
        axes[3].grid(True, alpha=0.3)

        fig.suptitle(f"Keyboard Control Log (car {car_id})")
        fig.tight_layout()

        plot_path = args.output.replace(".csv", "_plot.png")
        fig.savefig(plot_path, dpi=150)
        print(f"Plot saved to {plot_path}")
    except ImportError:
        print("matplotlib not installed, skipping plot.")

    print("Done.")


if __name__ == "__main__":
    main()
