#!/usr/bin/env python3
"""
Compute VINS-Fusion error metrics on a 4Seasons sequence.

Inputs:
  vio.csv   (VINS output)  : ts_ns, x,y,z, qw,qx,qy,qz, vx,vy,vz
  gt_tum.txt (ground truth): ts_s tx ty tz qx qy qz qw   (IMU frame, from gt_convert.py)

Metrics (SE(3) Umeyama alignment, no scale):
  ATE median/RMSE (m), RPE median/RMSE (m, delta=100 frames),
  Attitude RMSE (deg), F-score @2m, Velocity mean/std/RMSE error (m/s).
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation
from evo.core import metrics, sync
from evo.core.trajectory import PoseTrajectory3D

THRESHOLD = 2.0
RPE_DELTA = 100

def load_vio_csv(path):
    stamps, poses = [], []
    with open(path) as f:
        for line in f:
            parts = [p for p in line.strip().rstrip(",").split(",") if p != ""]
            if len(parts) < 8:
                continue
            try:
                ts = float(parts[0]) / 1e9
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                qw, qx, qy, qz = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
            except ValueError:
                continue
            R = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
            T = np.eye(4); T[:3, :3] = R; T[:3, 3] = [x, y, z]
            stamps.append(ts); poses.append(T)
    return PoseTrajectory3D(poses_se3=poses, timestamps=np.array(stamps))

def load_gt_tum(path):
    stamps, poses = [], []
    with open(path) as f:
        for line in f:
            v = line.strip().split()
            if len(v) < 8:
                continue
            ts = float(v[0])
            x, y, z = float(v[1]), float(v[2]), float(v[3])
            qx, qy, qz, qw = float(v[4]), float(v[5]), float(v[6]), float(v[7])
            R = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
            T = np.eye(4); T[:3, :3] = R; T[:3, 3] = [x, y, z]
            stamps.append(ts); poses.append(T)
    return PoseTrajectory3D(poses_se3=poses, timestamps=np.array(stamps))

def main():
    vio_path, gt_path, label = sys.argv[1], sys.argv[2], sys.argv[3]
    traj_est = load_vio_csv(vio_path)
    traj_ref = load_gt_tum(gt_path)

    ref, est = sync.associate_trajectories(traj_ref, traj_est, max_diff=0.02)
    est.align(ref, correct_scale=False)
    N = est.num_poses

    # --- ATE: Absolute Trajectory Error = per-pose translational distance between
    #     the aligned estimate and ground truth (global consistency). ---
    ate = metrics.APE(metrics.PoseRelation.translation_part)
    ate.process_data((ref, est))
    ate_err = np.array(ate.error)

    # --- RPE: Relative Pose Error over a 100-frame window = local drift that does
    #     NOT accumulate previous error (translational part only). ---
    rpe = metrics.RPE(metrics.PoseRelation.translation_part,
                      delta=RPE_DELTA, delta_unit=metrics.Unit.frames,
                      rel_delta_tol=0.1, all_pairs=False)
    rpe.process_data((ref, est))
    rpe_err = np.array(rpe.error)

    # --- Attitude error: geodesic angle of the relative rotation R_ref^T R_est.
    #     angle = arccos((trace(R_err) - 1) / 2). Computed by hand because evo's
    #     SO(3) validator rejects matrices after float drift from alignment. ---
    att = []
    for P_ref, P_est in zip(ref.poses_se3, est.poses_se3):
        R_err = P_ref[:3, :3].T @ P_est[:3, :3]                         # relative rotation
        att.append(np.degrees(np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1, 1))))
    att = np.array(att)

    # --- F-score @2m: fraction of poses within 2 m. With 1:1 correspondence
    #     precision == recall == inliers/N, so F = that same fraction. ---
    n_in = int(np.sum(ate_err < THRESHOLD))
    p = r = n_in / N
    fscore = (2 * p * r / (p + r)) if (p + r) > 0 else 0.0

    # --- Velocity error: per-step speed from finite differences on positions
    #     (||Δpos|| / Δt), estimate vs ground truth, compared element-wise. ---
    def speeds(traj):
        pos = traj.positions_xyz
        t = traj.timestamps
        dt = np.diff(t)
        dp = np.diff(pos, axis=0)
        good = dt > 1e-6
        return np.linalg.norm(dp[good], axis=1) / dt[good]
    sp_est = speeds(est)
    sp_ref = speeds(ref)
    m = min(len(sp_est), len(sp_ref))
    vel_err = sp_est[:m] - sp_ref[:m]

    def rmse(a): return float(np.sqrt(np.mean(a ** 2)))
    print("==== %s ====" % label)
    print("matched poses : %d" % N)
    print("ATE  median   : %.4f m" % np.median(ate_err))
    print("ATE  RMSE     : %.4f m" % rmse(ate_err))
    print("RPE  median   : %.4f m" % np.median(rpe_err))
    print("RPE  RMSE     : %.4f m" % rmse(rpe_err))
    print("Att  median   : %.4f deg" % np.median(att))
    print("Att  RMSE     : %.4f deg" % rmse(att))
    print("F-score @2m   : %.4f" % fscore)
    print("Vel  mean err : %.4f m/s" % np.mean(np.abs(vel_err)))
    print("Vel  std  err : %.4f m/s" % np.std(vel_err))
    print("Vel  RMSE err : %.4f m/s" % rmse(vel_err))
    # machine-readable line for the report builder
    print("CSV,%s,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f" % (
        label, N, np.median(ate_err), rmse(ate_err), np.median(rpe_err), rmse(rpe_err),
        np.median(att), rmse(att), fscore,
        np.mean(np.abs(vel_err)), np.std(vel_err), rmse(vel_err)))

if __name__ == "__main__":
    main()
