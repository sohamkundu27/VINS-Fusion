#!/usr/bin/env python3
"""
Render KITTI feature-extraction visualizations (fast, cv2-based).

For each frame it overlays on the left image:
  - blue dots   : tracked feature pixels at t
  - green arrows: KLT flow t -> t+1
  - yellow text : per-feature depth (m)
  - 3-line velocity block:
        GT   vel: [..] m/s |v|=..   (green)
        PRED vel: [..] m/s |v|=..   (yellow)
        Error:    [..] m/s |err|=.. (red if |err|>0.5 else white)

Reads the augmented per-frame .npz (must contain pred_velocity_cam etc. — run
add_pred_velocity.py first). cv2 + JPEG keeps every-frame output ~40-80 KB/frame
instead of ~600 KB matplotlib PNGs.

Usage:
  python make_kitti_viz.py [--every N] [--ext jpg|png] [--seqs 00 01 05]
  --every 1  -> every frame (default)   --every 100 -> sparse
"""
import glob, argparse
from pathlib import Path
import numpy as np
import cv2

KITTI = "/home/soham/datasets/kitti/dataset"
OUTROOT = "/home/soham/datasets/kitti/extracted"

# BGR colors
BLUE, GREEN, YELLOW, RED, WHITE = (255, 0, 0), (0, 255, 0), (0, 255, 255), (0, 0, 255), (255, 255, 255)


def render(seq, every, ext):
    outdir = f"{OUTROOT}/seq_{seq}"
    vizdir = f"{outdir}/viz_full"
    Path(vizdir).mkdir(parents=True, exist_ok=True)
    npzs = sorted(glob.glob(f"{outdir}/frame_*.npz"))  # one .npz per frame
    n = 0
    for t, npz in enumerate(npzs):
        if t % every != 0:                      # --every lets you subsample (1 = all frames)
            continue
        d = np.load(npz)                         # load this frame's extracted arrays
        # read the left image in COLOR so the overlays show up in colour
        img = cv2.imread(f"{KITTI}/sequences/{seq}/image_0/{t:06d}.png")
        if img is None:
            continue
        pt, pt1, dep = d["feature_pixels_t"], d["feature_pixels_t1"], d["depths"]
        # --- draw each tracked feature: dot at t, arrow to t+1, depth label ---
        for (x, y), (x1, y1), dd in zip(pt, pt1, dep):
            p0 = (int(round(x)), int(round(y)))
            cv2.circle(img, p0, 2, BLUE, -1)                       # feature location at t
            if not (x1 == 0 and y1 == 0):                          # skip last-frame zero tracks
                cv2.arrowedLine(img, p0, (int(round(x1)), int(round(y1))), GREEN, 1, tipLength=0.3)
            cv2.putText(img, f"{dd:.0f}", (p0[0] + 3, p0[1] - 3), cv2.FONT_HERSHEY_SIMPLEX,
                        0.3, YELLOW, 1, cv2.LINE_AA)                # depth in metres
        # --- 3-line velocity readout (GT / PRED / Error) ---
        g, p = d["gt_velocity_cam"], d["pred_velocity_cam"]
        err = d["velocity_error"]; em = float(d["velocity_error_magnitude"])
        lines = [
            (f"GT   vel: [{g[0]:6.2f},{g[1]:6.2f},{g[2]:6.2f}] m/s |v|={np.linalg.norm(g):.2f}", GREEN),
            (f"PRED vel: [{p[0]:6.2f},{p[1]:6.2f},{p[2]:6.2f}] m/s |v|={np.linalg.norm(p):.2f}", YELLOW),
            (f"Error:    [{err[0]:6.2f},{err[1]:6.2f},{err[2]:6.2f}] m/s |err|={em:.2f}",
             RED if em > 0.5 else WHITE),       # error turns red when |err| > 0.5 m/s
        ]
        for i, (txt, col) in enumerate(lines):
            y0 = 16 + i * 18                     # stack the three lines vertically
            # draw black outline first, then coloured text on top, for readability
            cv2.putText(img, txt, (8, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(img, txt, (8, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
        # write compact JPEG (default) or lossless PNG depending on --ext
        out = f"{vizdir}/frame_{t:06d}.{ext}"
        if ext == "jpg":
            cv2.imwrite(out, img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        else:
            cv2.imwrite(out, img)
        n += 1
    print(f"seq {seq}: wrote {n} frames -> {vizdir}")
    return n


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--every", type=int, default=1)
    ap.add_argument("--ext", choices=["jpg", "png"], default="jpg")
    ap.add_argument("--seqs", nargs="+", default=["00", "01", "05"])
    a = ap.parse_args()
    total = sum(render(s, a.every, a.ext) for s in a.seqs)
    print(f"TOTAL {total} frames")
