#!/usr/bin/env python3
"""
Emit KITTI feature tracks + depth as CSV/TXT, in the same schema as the 4Seasons
tracks_depth files:

    frame_idx,timestamp,u,v,depth_m,u_next,v_next

Source is the already-extracted per-frame .npz (feature_pixels_t -> u,v;
feature_pixels_t1 -> u_next,v_next; depths -> depth_m). Every row is one feature
that has BOTH a valid track and a valid depth. The final frame (zero tracks) is
skipped, mirroring the 4Seasons output.

Writes the FULL csv/txt under ~/datasets/kitti/extracted/seq_XX/ (large, kept
local) and a 500-row tracks_depth_sample.csv into the repo for browsing.
"""
import os, glob
import numpy as np

HOME = os.path.expanduser("~")
OUTROOT = os.path.join(HOME, "datasets/kitti/extracted")
REPO = os.path.join(HOME, "github/VINS-Fusion/results/kitti/extracted")
SEQS = ["00", "01", "05"]
HEADER = "frame_idx,timestamp,u,v,depth_m,u_next,v_next"
SAMPLE_ROWS = 500


def process(seq):
    outdir = os.path.join(OUTROOT, f"seq_{seq}")
    npzs = sorted(glob.glob(os.path.join(outdir, "frame_*.npz")))
    csv_path = os.path.join(outdir, "tracks_depth.csv")
    txt_path = os.path.join(outdir, "tracks_depth.txt")
    sample_path = os.path.join(REPO, f"seq_{seq}", "tracks_depth_sample.csv")
    os.makedirs(os.path.dirname(sample_path), exist_ok=True)

    n_rows = 0
    sample_lines = [HEADER]
    with open(csv_path, "w") as fc, open(txt_path, "w") as ft:
        fc.write(HEADER + "\n")
        ft.write(HEADER.replace(",", " ") + "\n")
        for npz in npzs:                                   # frames already sorted 0..N-1
            d = np.load(npz)
            t = int(d["frame_idx"])
            ts = float(d["timestamp"])
            pt, pt1, dep = d["feature_pixels_t"], d["feature_pixels_t1"], d["depths"]
            # skip the last frame: its tracks are zeroed (no t+1)
            if pt.shape[0] == 0 or np.allclose(pt1, 0):
                continue
            for (u, v), (un, vn), dd in zip(pt, pt1, dep):
                line = "%d,%.9f,%.3f,%.3f,%.4f,%.3f,%.3f" % (t, ts, u, v, dd, un, vn)
                fc.write(line + "\n")
                ft.write(line.replace(",", " ") + "\n")
                if len(sample_lines) <= SAMPLE_ROWS:
                    sample_lines.append(line)
                n_rows += 1

    with open(sample_path, "w") as fs:
        fs.write("\n".join(sample_lines) + "\n")
    print(f"seq {seq}: {n_rows} rows -> {csv_path}  (sample -> {sample_path})")
    return seq, n_rows


if __name__ == "__main__":
    print("=== KITTI feature tracks + depth ===")
    res = [process(s) for s in SEQS]
    print("\n| Seq | feature rows |")
    print("|-----|--------------|")
    for s, n in res:
        print(f"| {s} | {n} |")
