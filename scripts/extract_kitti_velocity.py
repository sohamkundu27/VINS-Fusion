#!/usr/bin/env python3
"""
Emit KITTI ground-truth velocity in the CAMERA frame as CSV/TXT, in the same
schema as the 4Seasons velocity files:

    frame_idx,timestamp,vx_cam,vy_cam,vz_cam,speed_mps

Velocity is the finite difference of the KITTI ground-truth trajectory rotated
into the camera frame (identical formula to the extracted .npz gt_velocity_cam):
    v_world = (pos[t+1] - pos[t]) / (time[t+1] - time[t])
    v_cam   = R[t].T @ v_world
The final frame (no t+1) has no velocity and is omitted, mirroring 4Seasons.
"""
import os
import numpy as np

HOME = os.path.expanduser("~")
KITTI = os.path.join(HOME, "datasets/kitti/dataset")
OUTROOT = os.path.join(HOME, "datasets/kitti/extracted")
SEQS = ["00", "01", "05"]


def load_poses(seq):
    R, P = [], []
    for line in open(os.path.join(KITTI, "poses", f"{seq}.txt")):
        m = np.array(list(map(float, line.split()))).reshape(3, 4)  # 3x4 cam->world
        R.append(m[:, :3]); P.append(m[:, 3])
    return np.array(R), np.array(P)


def load_times(seq):
    return np.array([float(x) for x in open(os.path.join(KITTI, "sequences", seq, "times.txt"))])


def process(seq):
    R, P = load_poses(seq)
    times = load_times(seq)
    nf = len(P)
    rows = []
    for t in range(nf - 1):                       # skip last frame (no t+1)
        dt = times[t + 1] - times[t]
        if dt <= 0:
            continue
        v_world = (P[t + 1] - P[t]) / dt           # world-frame velocity
        v_cam = R[t].T @ v_world                   # rotate into camera frame
        speed = float(np.linalg.norm(v_cam))
        rows.append((t, times[t], v_cam[0], v_cam[1], v_cam[2], speed))

    outdir = os.path.join(OUTROOT, f"seq_{seq}")
    os.makedirs(outdir, exist_ok=True)
    csv_path = os.path.join(outdir, "velocity.csv")
    txt_path = os.path.join(outdir, "velocity.txt")
    header = "frame_idx,timestamp,vx_cam,vy_cam,vz_cam,speed_mps"
    with open(csv_path, "w") as fc, open(txt_path, "w") as ft:
        fc.write(header + "\n")
        ft.write(header.replace(",", " ") + "\n")
        for r in rows:
            line = "%d,%.9f,%.5f,%.5f,%.5f,%.5f" % r
            fc.write(line + "\n")
            ft.write(line.replace(",", " ") + "\n")
    speeds = np.array([r[5] for r in rows])
    print(f"seq {seq}: {len(rows)} rows -> {csv_path}  (mean speed {speeds.mean():.2f} m/s)")
    return seq, len(rows), float(speeds.mean())


if __name__ == "__main__":
    print("=== KITTI ground-truth camera-frame velocity ===")
    res = [process(s) for s in SEQS]
    print("\n| Seq | rows | mean speed (m/s) |")
    print("|-----|------|------------------|")
    for s, n, sp in res:
        print(f"| {s} | {n} | {sp:.2f} |")
