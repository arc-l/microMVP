"""
Diagnostic: spin a car in place and log tracking data.

Usage:
    python examples/diag_spin_tracking.py --duration 10 --no-preview

Outputs:
    - diag_spin_data.csv  (raw observations)
    - diag_spin_plot.png  (x/y/theta over time)
    - Console summary with outlier statistics
"""
import argparse
import csv
import time

import numpy as np

from micromvp.core.models import Action
from micromvp.env import NewRealPushEnv, NewRealPushConfig


def main():
    parser = argparse.ArgumentParser(description="Spin tracking diagnostic")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--serial-port", type=str, default="/dev/tty.usbmodem3101")
    parser.add_argument("--calib", type=str, default="")
    parser.add_argument("--car-marker-size", type=float, default=27.0)
    parser.add_argument("--no-preview", action="store_true", default=False)
    parser.add_argument("--warmup", type=int, default=30)
    parser.add_argument("--duration", type=float, default=10.0,
                        help="Seconds to spin and record (default: 10)")
    parser.add_argument("--spin-speed", type=float, default=0.25,
                        help="Wheel speed for spinning (default: 0.25)")
    parser.add_argument("--output", type=str, default="diag_spin_data.csv")
    args = parser.parse_args()

    config = NewRealPushConfig(
        camera_device=args.camera,
        serial_port=args.serial_port,
        warmup_frames=args.warmup,
        no_preview=args.no_preview,
        car_marker_size_mm=args.car_marker_size,
    )
    if args.calib:
        config.calibration_file = args.calib

    env = NewRealPushEnv(config)

    print("[diag] Starting environment ...")
    if not env.start(wait_for_ready=True, timeout=15.0):
        print("[diag] Failed to start environment")
        return

    ws = env.workspace_config
    print(f"[diag] Workspace: {ws.width:.1f} x {ws.height:.1f} cm")
    print(f"[diag] Cars: {ws.car_id_list}")

    if not ws.car_id_list:
        print("[diag] No cars detected.")
        env.close()
        return

    car_id = ws.car_id_list[0]
    spin_action = {car_id: Action(left_speed=-args.spin_speed, right_speed=args.spin_speed)}

    # Collect observations
    rows = []
    t0 = time.time()
    frame_count = 0
    miss_count = 0

    print(f"[diag] Spinning car {car_id} for {args.duration}s (speed={args.spin_speed}) ...")

    while time.time() - t0 < args.duration:
        obs = env.observe()
        env.apply_actions(spin_action)

        if not args.no_preview:
            env.render()

        frame_count += 1
        if car_id in obs:
            o = obs[car_id]
            rows.append({
                "t": o.timestamp - t0,
                "x": o.x,
                "y": o.y,
                "theta": o.theta,
            })
        else:
            miss_count += 1

        elapsed = time.time() - (t0 + frame_count / ws.frequency)
        if elapsed < 0:
            time.sleep(-elapsed)

    # Stop
    env.apply_actions({car_id: Action.stop()})
    env.close()
    print(f"[diag] Done. Frames={frame_count}, observations={len(rows)}, missed={miss_count}")

    if len(rows) < 10:
        print("[diag] Too few observations to analyze.")
        return

    # Save CSV
    with open(args.output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["t", "x", "y", "theta"])
        writer.writeheader()
        writer.writerows(rows)
    print(f"[diag] Saved {len(rows)} rows to {args.output}")

    # Analyze
    ts = np.array([r["t"] for r in rows])
    xs = np.array([r["x"] for r in rows])
    ys = np.array([r["y"] for r in rows])
    thetas = np.array([r["theta"] for r in rows])

    # Position drift: for in-place spin, x/y should stay constant
    x_mean, x_std = xs.mean(), xs.std()
    y_mean, y_std = ys.mean(), ys.std()

    # Detect position outliers (> 3 std from mean)
    x_outliers = np.abs(xs - x_mean) > 3 * max(x_std, 0.1)
    y_outliers = np.abs(ys - y_mean) > 3 * max(y_std, 0.1)
    pos_outliers = x_outliers | y_outliers

    # Angle jumps: consecutive angle change should be smooth
    dtheta = np.diff(thetas)
    # Unwrap 360-degree jumps
    dtheta = np.where(dtheta > 180, dtheta - 360, dtheta)
    dtheta = np.where(dtheta < -180, dtheta + 360, dtheta)
    dt = np.diff(ts)
    dt = np.where(dt < 1e-6, 1e-6, dt)
    omega = dtheta / dt  # deg/s

    omega_median = np.median(omega)
    omega_std = np.std(omega)
    omega_outliers = np.abs(omega - omega_median) > 3 * max(omega_std, 1.0)

    print()
    print("=" * 60)
    print("TRACKING STABILITY REPORT")
    print("=" * 60)
    print(f"  Duration       : {ts[-1]:.1f} s")
    print(f"  Observations   : {len(rows)}")
    print(f"  Avg FPS        : {len(rows) / max(ts[-1], 0.01):.1f}")
    print()
    print(f"  X  mean={x_mean:.2f} cm  std={x_std:.2f} cm  range=[{xs.min():.2f}, {xs.max():.2f}]")
    print(f"  Y  mean={y_mean:.2f} cm  std={y_std:.2f} cm  range=[{ys.min():.2f}, {ys.max():.2f}]")
    print(f"  Position outliers (>3σ): {pos_outliers.sum()} / {len(rows)}  ({100*pos_outliers.mean():.1f}%)")
    print()
    print(f"  Angular velocity: median={omega_median:.1f} deg/s  std={omega_std:.1f} deg/s")
    print(f"  Omega range     : [{omega.min():.1f}, {omega.max():.1f}] deg/s")
    print(f"  Omega outliers  : {omega_outliers.sum()} / {len(omega)}  ({100*omega_outliers.mean():.1f}%)")
    print("=" * 60)

    # Show worst outliers
    if pos_outliers.sum() > 0:
        idxs = np.where(pos_outliers)[0]
        print(f"\nPosition outlier samples (first 10):")
        for i in idxs[:10]:
            print(f"  t={ts[i]:.3f}  x={xs[i]:.2f}  y={ys[i]:.2f}  theta={thetas[i]:.1f}")

    if omega_outliers.sum() > 0:
        idxs = np.where(omega_outliers)[0]
        print(f"\nAngular velocity outlier samples (first 10):")
        for i in idxs[:10]:
            print(f"  t={ts[i+1]:.3f}  dtheta={dtheta[i]:.1f}  dt={dt[i]*1000:.0f}ms  omega={omega[i]:.1f} deg/s")

    # Plot
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

        axes[0].plot(ts, xs, "b-", linewidth=0.5, label="x")
        axes[0].axhline(x_mean, color="b", linestyle="--", alpha=0.5)
        if pos_outliers.sum() > 0:
            axes[0].plot(ts[x_outliers], xs[x_outliers], "ro", markersize=3, label="outlier")
        axes[0].set_ylabel("X (cm)")
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(ts, ys, "g-", linewidth=0.5, label="y")
        axes[1].axhline(y_mean, color="g", linestyle="--", alpha=0.5)
        if pos_outliers.sum() > 0:
            axes[1].plot(ts[y_outliers], ys[y_outliers], "ro", markersize=3, label="outlier")
        axes[1].set_ylabel("Y (cm)")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        axes[2].plot(ts, thetas, "m-", linewidth=0.5, label="theta")
        axes[2].set_ylabel("Theta (deg)")
        axes[2].set_xlabel("Time (s)")
        axes[2].legend()
        axes[2].grid(True, alpha=0.3)

        fig.suptitle(f"Spin Tracking Diagnostic (car {car_id})")
        fig.tight_layout()

        plot_path = args.output.replace(".csv", "_plot.png")
        fig.savefig(plot_path, dpi=150)
        print(f"\n[diag] Plot saved to {plot_path}")
    except ImportError:
        print("\n[diag] matplotlib not installed, skipping plot.")


if __name__ == "__main__":
    main()
