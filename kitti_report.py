#!/usr/bin/env python3
"""
VINS-Fusion KITTI evaluation
Metrics: ATE, RPE, attitude error, F-score, velocity error
Output:  datasets/kitti/output/results_table.md
"""

import io, copy
import numpy as np
from evo.core import metrics
from evo.tools import file_interface

# ── Config ─────────────────────────────────────────────────────────────────
BASE      = "/home/soham/datasets/kitti"
OUT       = f"{BASE}/output"
THRESHOLD = 2.0   # metres  — APE threshold for F-score "correct" classification
RPE_DELTA = 100   # frames  — window size for relative pose error

SEQUENCES = {"00": "City", "01": "Highway", "05": "Residential"}

# ── Helpers ────────────────────────────────────────────────────────────────
def clean_load(path):
    """Load KITTI pose file, stripping trailing spaces that evo rejects."""
    cleaned = "\n".join(line.rstrip() for line in open(path))
    return file_interface.read_kitti_poses_file(io.StringIO(cleaned))

def load_times(path):
    with open(path) as f:
        return np.array([float(l.strip()) for l in f])

# ── Per-sequence computation ───────────────────────────────────────────────
def compute(seq):
    vio_path   = f"{OUT}/vio_{seq}.txt"
    gt_path    = f"{BASE}/dataset/poses/{seq}.txt"
    times_path = f"{BASE}/dataset/sequences/{seq}/times.txt"

    traj_ref = clean_load(gt_path)

    # SE(3) Umeyama alignment — rigid-body least-squares fit, no scale change.
    # Brings the VIO world frame into alignment with the KITTI world frame so
    # the initial pose offset doesn't inflate every metric.
    traj_est = clean_load(vio_path)
    traj_est.align(traj_ref, correct_scale=False)   # modifies traj_est in-place

    # ── 1. ATE — translation part ─────────────────────────────────────────
    ate = metrics.APE(metrics.PoseRelation.translation_part)
    ate.process_data((traj_ref, traj_est))
    ate_err    = np.array(ate.error)
    ate_median = np.median(ate_err)
    ate_rmse   = np.sqrt(np.mean(ate_err**2))

    # ── 2. RPE — translation, delta=RPE_DELTA frames ─────────────────────
    rpe = metrics.RPE(
        metrics.PoseRelation.translation_part,
        delta=RPE_DELTA,
        delta_unit=metrics.Unit.frames,
    )
    rpe.process_data((traj_ref, traj_est))
    rpe_err    = np.array(rpe.error)
    rpe_median = np.median(rpe_err)
    rpe_rmse   = np.sqrt(np.mean(rpe_err**2))

    # ── 3. Attitude error — rotation angle (degrees) ──────────────────────
    # evo's SO(3) validator is strict and rejects floating-point drift after
    # alignment, so we compute the geodesic angle manually:
    #   R_err = R_ref^T · R_est,  angle = arccos((tr(R_err) - 1) / 2)
    att_err = []
    for P_ref, P_est in zip(traj_ref.poses_se3, traj_est.poses_se3):
        R_err  = P_ref[:3, :3].T @ P_est[:3, :3]
        cosval = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
        att_err.append(np.degrees(np.arccos(cosval)))
    att_err    = np.array(att_err)
    att_median = np.median(att_err)
    att_rmse   = np.sqrt(np.mean(att_err**2))

    # ── 4. F-score ────────────────────────────────────────────────────────
    # With 1-to-1 frame correspondence precision = recall = fraction(ATE < threshold).
    # F = 2·P·R / (P+R) reduces to P (= R) in this symmetric case,
    # i.e. F-score equals the fraction of poses within the threshold.
    N       = len(ate_err)
    correct = int(np.sum(ate_err < THRESHOLD))
    p = r   = correct / N
    fscore  = (2 * p * r / (p + r)) if (p + r) > 0 else 0.0

    # ── 5. Velocity error — finite differences ────────────────────────────
    # Use aligned positions so both trajectories are in the same frame.
    # Translation t in the alignment cancels in finite differences;
    # rotation R is what actually matters for direction of velocity vectors.
    pos_est = traj_est.positions_xyz    # (N, 3) after alignment
    pos_gt  = traj_ref.positions_xyz    # (N, 3)
    times   = load_times(times_path)    # per-frame timestamps from KITTI

    vel_err = []
    for i in range(len(times) - 1):
        dt    = times[i+1] - times[i]
        v_est = (pos_est[i+1] - pos_est[i]) / dt
        v_gt  = (pos_gt[i+1]  - pos_gt[i])  / dt
        vel_err.append(np.linalg.norm(v_est - v_gt))

    vel_err  = np.array(vel_err)
    vel_mean = vel_err.mean()
    vel_std  = vel_err.std()
    vel_rmse = np.sqrt((vel_err**2).mean())

    return dict(
        ate_median=ate_median, ate_rmse=ate_rmse,
        rpe_median=rpe_median, rpe_rmse=rpe_rmse,
        att_median=att_median, att_rmse=att_rmse,
        fscore=fscore,
        vel_mean=vel_mean, vel_std=vel_std, vel_rmse=vel_rmse,
    )

# ── Run all sequences ──────────────────────────────────────────────────────
results = {}
for seq in SEQUENCES:
    print(f"Computing sequence {seq}…", flush=True)
    results[seq] = compute(seq)

# ── Build markdown tables ──────────────────────────────────────────────────
COL = (
    "| Sequence | Type "
    "| ATE median (m) | ATE RMSE (m) "
    "| RPE median (m) | RPE RMSE (m) "
    "| Attitude median (deg) | Attitude RMSE (deg) "
    "| F-score "
    "| Vel error mean (m/s) | Vel error std (m/s) | Vel error RMSE (m/s) |"
)
SEP = "|" + "|".join(["---"] * 12) + "|"

rows = [COL, SEP]
for seq, label in SEQUENCES.items():
    r = results[seq]
    rows.append(
        f"| {seq} | {label} "
        f"| {r['ate_median']:.4f} | {r['ate_rmse']:.4f} "
        f"| {r['rpe_median']:.4f} | {r['rpe_rmse']:.4f} "
        f"| {r['att_median']:.4f} | {r['att_rmse']:.4f} "
        f"| {r['fscore']:.4f} "
        f"| {r['vel_mean']:.4f} | {r['vel_std']:.4f} | {r['vel_rmse']:.4f} |"
    )

results_table = "\n".join(rows)

ref_table = (
    "| Sequence | ATE mean (m) | ATE RMSE (m) |\n"
    "|----------|------|------|\n"
    "| 00 | 1.18 | 1.24 |\n"
    "| 05 | 1.19 | 1.26 |\n"
    "| 01 | N/A (not reported in this paper) | N/A |"
)

output = f"""\
## VINS-Fusion KITTI Evaluation Results

{results_table}

*ATE/RPE alignment: SE(3) Umeyama (no scale correction)*
*RPE delta: {RPE_DELTA} frames*
*F-score threshold: {THRESHOLD} m*

---

## Published VINS-Fusion Reference Values (Lvio-Fusion paper, arXiv:2106.06783)

{ref_table}
"""

print("\n" + output)
with open(f"{OUT}/results_table.md", "w") as f:
    f.write(output)
print(f"Saved → {OUT}/results_table.md")
