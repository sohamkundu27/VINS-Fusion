#!/usr/bin/env python3
"""
Augment the KITTI extracted dataset with VINS-Fusion *predicted* velocity.

For each sequence (00, 01, 05):
  - load ~/datasets/kitti/output/vio_XX.txt  (KITTI 3x4, one pose/frame, 1:1 with images)
  predicted velocity (camera frame) via finite differences on the estimate:
        v_world_pred = (pos_est[t+1] - pos_est[t]) / dt
        v_cam_pred   = R_est[t].T @ v_world_pred
    (vio.csv has vx,vy,vz columns but they are all-zero for these stereo-only/no-IMU
     runs — VINS only estimates Vs with an IMU — so finite differences is used.)
  - add to each frame_*.npz:
        pred_velocity_cam (3,) f32, velocity_error (3,) f32 [pred-gt],
        velocity_error_magnitude () f32
  - regenerate viz for every 100th frame with GT/PRED/Error overlay
  - update metadata.json with mean_velocity_error_rmse
Velocity in the body frame is alignment-invariant, so no trajectory alignment is needed.
"""
import json, glob
from pathlib import Path
import numpy as np
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

KITTI = "/home/soham/datasets/kitti/dataset"
OUTROOT = "/home/soham/datasets/kitti/extracted"
VIODIR = "/home/soham/datasets/kitti/output"
SEQS = ["00", "01", "05"]


def load_vio_txt(path):
    """KITTI 3x4 -> R[t] (3x3), pos[t] (3); one per frame."""
    R, P = [], []
    for line in open(path):
        v = list(map(float, line.split()))
        if len(v) < 12:
            continue
        m = np.array(v).reshape(3, 4)
        R.append(m[:, :3]); P.append(m[:, 3])
    return np.array(R), np.array(P)


def load_times(seq):
    return np.array([float(x) for x in open(f"{KITTI}/sequences/{seq}/times.txt")])


def csv_velocity_usable(seq):
    """Return True only if vio_XX.csv has non-trivial vx,vy,vz (it doesn't for no-IMU runs)."""
    p = f"{VIODIR}/vio_{seq}.csv"
    if not Path(p).exists():
        return False
    sp = []
    for line in open(p):
        c = [x for x in line.strip().rstrip(",").split(",") if x != ""]
        if len(c) >= 11:
            sp.append(abs(float(c[8])) + abs(float(c[9])) + abs(float(c[10])))
    return bool(sp) and (np.max(sp) > 1e-3)


def predicted_velocities(seq):
    """VINS predicted velocity in the camera frame for every frame.

    Mirrors the GT velocity computation but on the ESTIMATED trajectory (vio.txt):
      v_world = (pos_est[t+1] - pos_est[t]) / dt    (finite difference)
      v_cam   = R_est[t].T @ v_world                (rotate world -> camera frame)
    Because each velocity is projected into its own camera frame, GT and prediction
    are directly comparable without any trajectory alignment. Last frame -> zeros.
    """
    R, P = load_vio_txt(f"{VIODIR}/vio_{seq}.txt")  # R_est[t], pos_est[t]
    times = load_times(seq)                                       # capture times (s)
    nf = len(P)
    vcam = np.zeros((nf, 3), np.float32)                          # default 0 (covers last frame)
    for t in range(nf - 1):
        dt = times[t + 1] - times[t]
        if dt <= 0:                                              # guard against bad timestamps
            continue
        v_world = (P[t + 1] - P[t]) / dt                         # world-frame velocity
        vcam[t] = (R[t].T @ v_world).astype(np.float32)          # -> camera/body frame
    return vcam


def regen_viz(seq, t, vcam_pred_t, gt_vel_t, p_t, p_t1, depths, vizpath):
    imL = cv2.imread(f"{KITTI}/sequences/{seq}/image_0/{t:06d}.png",
                     cv2.IMREAD_GRAYSCALE)
    err = vcam_pred_t - gt_vel_t
    errmag = float(np.linalg.norm(err))
    fig, ax = plt.subplots(figsize=(14, 4.5))
    ax.imshow(imL, cmap="gray")
    for (x, y), (x1, y1), d in zip(p_t, p_t1, depths):
        ax.plot(x, y, 'o', color='blue', ms=3)
        ax.annotate("", xy=(x1, y1), xytext=(x, y),
                    arrowprops=dict(arrowstyle="->", color='lime', lw=0.8))
        ax.text(x + 3, y - 3, f"{d:.0f}", color='yellow', fontsize=5)
    g, p = gt_vel_t, vcam_pred_t
    lines = [
        (f"GT   vel: [{g[0]:6.2f},{g[1]:6.2f},{g[2]:6.2f}] m/s  |v|={np.linalg.norm(g):.2f}", 'lime'),
        (f"PRED vel: [{p[0]:6.2f},{p[1]:6.2f},{p[2]:6.2f}] m/s  |v|={np.linalg.norm(p):.2f}", 'yellow'),
        (f"Error:    [{err[0]:6.2f},{err[1]:6.2f},{err[2]:6.2f}] m/s  |err|={errmag:.2f}",
         'red' if errmag > 0.5 else 'white'),
    ]
    for i, (txt, col) in enumerate(lines):
        ax.text(10, 22 + i * 20, txt, color=col, fontsize=10, family='monospace',
                backgroundcolor='black')
    ax.set_axis_off()
    plt.tight_layout()
    plt.savefig(vizpath, dpi=110, bbox_inches='tight')
    plt.close(fig)


def process(seq):
    outdir = f"{OUTROOT}/seq_{seq}"
    vizdir = f"{outdir}/viz"
    Path(vizdir).mkdir(parents=True, exist_ok=True)
    npzs = sorted(glob.glob(f"{outdir}/frame_*.npz"))
    nf = len(npzs)
    use_csv = csv_velocity_usable(seq)
    vcam_pred = predicted_velocities(seq)

    gt_speeds, pred_speeds, sq_err = [], [], []
    for t in range(nf):
        d = dict(np.load(npzs[t]))
        gt_vel = d["gt_velocity_cam"].astype(np.float32)
        pred = vcam_pred[t].astype(np.float32)
        verr = (pred - gt_vel).astype(np.float32)
        vmag = np.float32(np.linalg.norm(verr))
        d["pred_velocity_cam"] = pred
        d["velocity_error"] = verr
        d["velocity_error_magnitude"] = vmag
        np.savez(npzs[t], **d)

        if t < nf - 1:  # exclude last frame (both zero by construction)
            gt_speeds.append(float(np.linalg.norm(gt_vel)))
            pred_speeds.append(float(np.linalg.norm(pred)))
            sq_err.append(float(vmag) ** 2)

        if t % 100 == 0:
            regen_viz(seq, t, pred, gt_vel, d["feature_pixels_t"], d["feature_pixels_t1"],
                      d["depths"], f"{vizdir}/frame_{t:06d}.png")

    vel_rmse = float(np.sqrt(np.mean(sq_err)))
    meta = json.load(open(f"{outdir}/metadata.json"))
    meta["mean_velocity_error_rmse"] = vel_rmse
    meta["pred_velocity_source"] = "vio.csv vx,vy,vz" if use_csv else "finite-diff on vio.txt (no-IMU run: csv velocity all-zero)"
    json.dump(meta, open(f"{outdir}/metadata.json", "w"), indent=2)

    return dict(seq=seq, gt_speed=float(np.mean(gt_speeds)),
                pred_speed=float(np.mean(pred_speeds)), vel_rmse=vel_rmse,
                csv=use_csv)


if __name__ == "__main__":
    rows = [process(s) for s in SEQS]
    print("\n==== PREDICTED vs GROUND-TRUTH VELOCITY ====")
    print("| Seq | GT mean speed | PRED mean speed | Vel RMSE |")
    print("|-----|---------------|-----------------|----------|")
    for r in rows:
        print(f"| {r['seq']} | {r['gt_speed']:.2f} m/s | {r['pred_speed']:.2f} m/s | {r['vel_rmse']:.2f} m/s |")
    print("\n(predicted velocity source: finite differences on vio.txt — "
          "stereo-only runs have no IMU, so vio.csv vx,vy,vz are all zero)")
