import io
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from evo.core import metrics
from evo.tools import file_interface

OUT_DIR = "/home/soham/datasets/kitti/output"

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.size": 11,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "figure.dpi": 150,
})


def load_kitti_poses(filepath):
    poses = []
    with open(filepath) as f:
        for line in f:
            vals = list(map(float, line.strip().split()))
            mat = np.eye(4)
            mat[:3, :] = np.array(vals).reshape(3, 4)
            poses.append(mat)
    return poses


def load_timestamps(filepath):
    with open(filepath) as f:
        return [float(line.strip()) for line in f]


def compute_velocity_error(est_poses, gt_poses, timestamps):
    errors = []
    for i in range(len(timestamps) - 1):
        dt = timestamps[i+1] - timestamps[i]
        est_vel = (est_poses[i+1][:3, 3] - est_poses[i][:3, 3]) / dt
        gt_vel  = (gt_poses[i+1][:3, 3]  - gt_poses[i][:3, 3])  / dt
        errors.append(np.linalg.norm(est_vel - gt_vel))
    return np.mean(errors), np.sqrt(np.mean(np.array(errors)**2))


def _clean(path):
    cleaned = "\n".join(line.rstrip() for line in open(path))
    return file_interface.read_kitti_poses_file(io.StringIO(cleaned))


def path_length(poses):
    pts = np.array([p[:3, 3] for p in poses])
    return np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])


def visualize(seq, est_poses, gt_poses, ate_errors, rpe_errors, vel_errors, timestamps, out_path):
    gt_xyz  = np.array([p[:3, 3] for p in gt_poses])
    est_xyz = np.array([p[:3, 3] for p in est_poses])
    dist    = path_length(gt_poses)

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle(f"VINS-Fusion  —  KITTI Sequence {seq}", fontsize=14, fontweight="bold", y=1.01)

    # ── Panel 1: bird's-eye trajectory (X vs Z in camera frame) ──
    ax = axes[0]
    ax.plot(gt_xyz[:, 0],  gt_xyz[:, 2],  color="#444444", linewidth=1.4, label="Ground truth", zorder=2)
    ax.plot(est_xyz[:, 0], est_xyz[:, 2], color="#E05A2B", linewidth=1.2, label="VINS-Fusion",  zorder=3, alpha=0.85)
    ax.scatter(gt_xyz[0, 0],   gt_xyz[0, 2],   s=60, color="#2196F3", zorder=5, label="Start")
    ax.scatter(gt_xyz[-1, 0],  gt_xyz[-1, 2],  s=60, color="#4CAF50", marker="*", zorder=5, label="End")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Z (m)")
    ax.set_title("Bird's-eye trajectory")
    ax.set_aspect("equal")
    ax.legend(fontsize=9, loc="best")
    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)

    # ── Panel 2: per-pose APE error over path distance ──
    ax = axes[1]
    ax.plot(dist, ate_errors, color="#E05A2B", linewidth=1.0, alpha=0.8)
    ax.axhline(np.mean(ate_errors), color="#555", linewidth=1.2, linestyle="--",
               label=f"mean {np.mean(ate_errors):.2f} m")
    ax.fill_between(dist, 0, ate_errors, color="#E05A2B", alpha=0.15)
    ax.set_xlabel("Path distance (m)"); ax.set_ylabel("APE (m)")
    ax.set_title("Absolute Pose Error over distance")
    ax.legend(fontsize=9)
    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)
    ax.yaxis.set_major_formatter(ticker.FormatStrFormatter("%.1f"))

    # ── Panel 3: velocity error over time ──
    ax = axes[2]
    t = np.array(timestamps[:-1]) - timestamps[0]
    ax.plot(t, vel_errors, color="#5B6EAE", linewidth=0.9, alpha=0.8)
    ax.axhline(np.mean(vel_errors), color="#555", linewidth=1.2, linestyle="--",
               label=f"mean {np.mean(vel_errors):.2f} m/s")
    ax.fill_between(t, 0, vel_errors, color="#5B6EAE", alpha=0.15)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Velocity error (m/s)")
    ax.set_title("Velocity error over time")
    ax.legend(fontsize=9)
    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)

    fig.tight_layout()
    fig.savefig(out_path, bbox_inches="tight", dpi=150)
    plt.close(fig)
    print(f"  Saved → {out_path}")


def evaluate(seq, vio_path, gt_path, times_path):
    print(f"\n=== Sequence {seq} ===")

    est_poses  = load_kitti_poses(vio_path)
    gt_poses   = load_kitti_poses(gt_path)
    timestamps = load_timestamps(times_path)

    traj_est = _clean(vio_path)
    traj_ref = _clean(gt_path)

    # ATE
    ate = metrics.APE(metrics.PoseRelation.translation_part)
    ate.process_data((traj_ref, traj_est))
    ate_errors = list(ate.error)
    print(f"ATE mean:  {ate.get_statistic(metrics.StatisticsType.mean):.4f} m")
    print(f"ATE rmse:  {ate.get_statistic(metrics.StatisticsType.rmse):.4f} m")

    # RPE
    rpe = metrics.RPE(metrics.PoseRelation.translation_part)
    rpe.process_data((traj_ref, traj_est))
    rpe_errors = list(rpe.error)
    print(f"RPE mean:  {rpe.get_statistic(metrics.StatisticsType.mean):.4f} m")
    print(f"RPE rmse:  {rpe.get_statistic(metrics.StatisticsType.rmse):.4f} m")

    # Velocity
    vel_errors = []
    for i in range(len(timestamps) - 1):
        dt = timestamps[i+1] - timestamps[i]
        est_vel = (est_poses[i+1][:3, 3] - est_poses[i][:3, 3]) / dt
        gt_vel  = (gt_poses[i+1][:3, 3]  - gt_poses[i][:3, 3])  / dt
        vel_errors.append(np.linalg.norm(est_vel - gt_vel))
    vel_arr = np.array(vel_errors)
    print(f"Vel mean:  {vel_arr.mean():.4f} m/s")
    print(f"Vel rmse:  {np.sqrt((vel_arr**2).mean()):.4f} m/s")

    visualize(
        seq        = seq,
        est_poses  = est_poses,
        gt_poses   = gt_poses,
        ate_errors = ate_errors,
        rpe_errors = rpe_errors,
        vel_errors = vel_errors,
        timestamps = timestamps,
        out_path   = f"{OUT_DIR}/seq_{seq}.png",
    )


BASE = "/home/soham/datasets/kitti"

evaluate("00",
    vio_path   = f"{BASE}/output/vio_00.txt",
    gt_path    = f"{BASE}/dataset/poses/00.txt",
    times_path = f"{BASE}/dataset/sequences/00/times.txt")

evaluate("01",
    vio_path   = f"{BASE}/output/vio_01.txt",
    gt_path    = f"{BASE}/dataset/poses/01.txt",
    times_path = f"{BASE}/dataset/sequences/01/times.txt")

evaluate("05",
    vio_path   = f"{BASE}/output/vio_05.txt",
    gt_path    = f"{BASE}/dataset/poses/05.txt",
    times_path = f"{BASE}/dataset/sequences/05/times.txt")
