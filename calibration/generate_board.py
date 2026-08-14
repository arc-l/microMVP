#!/usr/bin/env python3
"""
Generate a ChArUco calibration board and save as PDF + PNG.

Uses DICT_6X6_250 to avoid conflicts with car markers (4x4) and obstacle markers (5x5).

Usage:
    python calibration/generate_board.py
    python calibration/generate_board.py --cols 7 --rows 5 --square 30 --marker 22 -o my_board
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def generate_board(
    cols: int,
    rows: int,
    square_mm: float,
    marker_mm: float,
    dpi: int,
    margin_mm: float,
) -> tuple[np.ndarray, cv2.aruco.CharucoBoard]:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    board = cv2.aruco.CharucoBoard((cols, rows), square_mm / 1000.0, marker_mm / 1000.0, dictionary)

    board_width_mm = cols * square_mm
    board_height_mm = rows * square_mm
    total_width_mm = board_width_mm + 2 * margin_mm
    total_height_mm = board_height_mm + 2 * margin_mm

    px_per_mm = dpi / 25.4
    img_w = int(round(total_width_mm * px_per_mm))
    img_h = int(round(total_height_mm * px_per_mm))
    margin_px = int(round(margin_mm * px_per_mm))

    img = board.generateImage((img_w - 2 * margin_px, img_h - 2 * margin_px))

    canvas = np.full((img_h, img_w), 255, dtype=np.uint8)
    canvas[margin_px : margin_px + img.shape[0], margin_px : margin_px + img.shape[1]] = img

    return canvas, board


def save_pdf(image: np.ndarray, path: Path, dpi: int) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[WARN] matplotlib not installed, skipping PDF. Install: pip install matplotlib")
        return

    h, w = image.shape[:2]
    fig_w = w / dpi
    fig_h = h / dpi
    fig, ax = plt.subplots(figsize=(fig_w, fig_h), dpi=dpi)
    ax.imshow(image, cmap="gray", interpolation="nearest")
    ax.axis("off")
    fig.subplots_adjust(left=0, right=1, top=1, bottom=0)
    fig.savefig(str(path), dpi=dpi, bbox_inches="tight", pad_inches=0)
    plt.close(fig)
    print(f"  PDF saved: {path}")


def main() -> None:
    ap = argparse.ArgumentParser(description="Generate ChArUco calibration board")
    ap.add_argument("--cols", type=int, default=7, help="number of chessboard columns (default: 7)")
    ap.add_argument("--rows", type=int, default=5, help="number of chessboard rows (default: 5)")
    ap.add_argument("--square", type=float, default=30.0, help="square side length in mm (default: 30)")
    ap.add_argument("--marker", type=float, default=22.0, help="ArUco marker side length in mm (default: 22)")
    ap.add_argument("--dpi", type=int, default=300, help="output DPI (default: 300)")
    ap.add_argument("--margin", type=float, default=10.0, help="white margin in mm (default: 10)")
    ap.add_argument("-o", "--output", default="charuco_board", help="output filename (without extension)")
    args = ap.parse_args()

    out_dir = Path(__file__).parent
    out_stem = out_dir / args.output

    print(f"Generating ChArUco board: {args.cols}x{args.rows}, "
          f"square={args.square}mm, marker={args.marker}mm, dict=DICT_6X6_250")

    image, board = generate_board(
        args.cols, args.rows, args.square, args.marker, args.dpi, args.margin,
    )

    png_path = out_stem.with_suffix(".png")
    cv2.imwrite(str(png_path), image)
    print(f"  PNG saved: {png_path}")

    save_pdf(image, out_stem.with_suffix(".pdf"), args.dpi)

    board_w_mm = args.cols * args.square
    board_h_mm = args.rows * args.square
    print(f"\nBoard physical size: {board_w_mm:.0f} x {board_h_mm:.0f} mm")
    print(f"Print at 100% scale (no fit-to-page). Verify a square measures {args.square:.0f} mm.")


if __name__ == "__main__":
    main()
