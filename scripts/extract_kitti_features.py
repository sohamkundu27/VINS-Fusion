#!/usr/bin/env python3
"""
Extract per-frame training data from KITTI odometry stereo sequences (00, 01, 05).

Per time step t:
  1. STEREO DEPTH (sparse)  : cv2.StereoSGBM disparity on left+right at t,
                              depth = (fx * baseline) / disparity
  2. FEATURE TRACKS         : goodFeaturesToTrack on left[t] + KLT LK to left[t+1],
                              forward-backward round-trip check (<1 px)
  3. PAIRED depth + tracks  : keep features that have a valid track AND valid depth
                              (0 < depth < 80 m, finite)
  4. GT VELOCITY (cam frame): v_world = (pos[t+1]-pos[t]) / (time[t+1]-time[t]);
                              v_cam = R[t].T @ v_world

Output: ~/datasets/kitti/extracted/seq_XX/
  frame_NNNNNN.npz        per-frame arrays (features, depths, gt velocity)
  metadata.json           sequence-level stats + intrinsics
  viz/frame_NNNNNN.png    overlay every 100th frame
  tracks_depth.{csv,txt}  flat table: frame_idx,timestamp,u,v,depth_m,u_next,v_next
  velocity.{csv,txt}      flat table: frame_idx,timestamp,vx_cam,vy_cam,vz_cam,speed_mps
"""
import sys, json, glob
from pathlib import Path
import numpy as np
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

KITTI = "/home/soham/datasets/kitti/dataset"
OUTROOT = "/home/soham/datasets/kitti/extracted"

# Per-sequence camera intrinsics + stereo baseline (metres)
CAM = {
    "00": dict(fx=718.856, fy=718.856, cx=607.193, cy=185.216, baseline=0.537165),
    "01": dict(fx=718.856, fy=718.856, cx=607.193, cy=185.216, baseline=0.537165),
    "05": dict(fx=707.091, fy=707.091, cx=601.887, cy=183.110, baseline=0.537150),
}

# --- StereoSGBM (settings per spec) ---
BLOCK = 11
NUM_DISP = 128                       # multiple of 16
SGBM = cv2.StereoSGBM_create(
    minDisparity=0, numDisparities=NUM_DISP, blockSize=BLOCK,
    P1=8 * 3 * BLOCK ** 2, P2=32 * 3 * BLOCK ** 2, disp12MaxDiff=1,
    uniquenessRatio=10, speckleWindowSize=100, speckleRange=32)
MAX_DISP = NUM_DISP                  # disparity must be >0 and < this

# --- features / KLT (settings per spec) ---
GFTT = dict(maxCorners=500, qualityLevel=0.01, minDistance=10, blockSize=7)
LK = dict(winSize=(21, 21), maxLevel=3,
          criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
FB_THRESH = 1.0          # forward-backward round-trip error (px)
DEPTH_MIN, DEPTH_MAX = 0.0, 80.0


def load_poses(seq):
    """Read KITTI ground-truth poses file (poses/SEQ.txt).

    Each line is a 3x4 row-major camera-to-world matrix (12 numbers): the first
    3x3 block is rotation R (camera->world), the last column is translation t.
    Returns one (R, t) tuple per frame, index-aligned with the image frames.
    """
    P = []
    for line in open(f"{KITTI}/poses/{seq}.txt"):
        m = np.array(list(map(float, line.split()))).reshape(3, 4)  # 12 floats -> 3x4
        P.append((m[:, :3], m[:, 3]))                                # (R 3x3, t 3,)
    return P


def load_times(seq):
    """Read per-frame capture timestamps (seconds) from sequences/SEQ/times.txt.
    Used as dt for the finite-difference velocity (one value per frame)."""
    return np.array([float(x) for x in open(f"{KITTI}/sequences/{seq}/times.txt")])


def compute_depth_map(imL, imR, fx, baseline):
    """Dense stereo depth via Semi-Global Block Matching.

    Steps:
      1. SGBM.compute returns disparity as a fixed-point int16 scaled by 16,
         so we divide by 16.0 to get sub-pixel disparity in real pixels.
      2. A disparity is only trustworthy when it is strictly positive and below
         the search range (NUM_DISP); everything else is marked invalid.
      3. Convert disparity d to metric depth with the stereo pinhole relation
         depth = (focal_length_px * baseline_m) / disparity_px.
      4. Invalid pixels are left as +inf so they get filtered out downstream.
    """
    disp = SGBM.compute(imL, imR).astype(np.float32) / 16.0   # un-scale SGBM fixed-point
    valid = (disp > 0) & (disp < MAX_DISP)                    # keep only reliable disparities
    depth = np.full(disp.shape, np.inf, np.float32)           # default = invalid
    depth[valid] = (fx * baseline) / disp[valid]              # d -> Z in metres
    return depth


def feature_tracks(imL_t, imL_t1):
    """Detect corners in frame t and track them to t+1 with a consistency check.

    Steps:
      1. goodFeaturesToTrack picks up to 500 strong Shi-Tomasi corners in left[t].
      2. calcOpticalFlowPyrLK (pyramidal Lucas-Kanade) tracks each corner FORWARD
         from t -> t+1, giving p1 and a per-point success flag st1.
      3. We track the result BACKWARD t+1 -> t (p0r). If LK is consistent, p0r
         should land back on the original p0.
      4. forward-backward error = ||p0 - p0r||. Keep a feature only if both LK
         directions succeeded AND the round-trip error is < 1 px (FB_THRESH).
         This is the same robustness filter VINS-Fusion's feature_tracker uses.
    Returns the surviving (p0 at t, p1 at t+1) pixel arrays.
    """
    p0 = cv2.goodFeaturesToTrack(imL_t, **GFTT)
    if p0 is None:                                            # no corners found this frame
        return np.empty((0, 2), np.float32), np.empty((0, 2), np.float32)
    p0 = p0.reshape(-1, 2).astype(np.float32)
    p1, st1, _ = cv2.calcOpticalFlowPyrLK(imL_t, imL_t1, p0, None, **LK)   # forward  t -> t+1
    p0r, st2, _ = cv2.calcOpticalFlowPyrLK(imL_t1, imL_t, p1, None, **LK)  # backward t+1 -> t
    fb = np.linalg.norm(p0 - p0r, axis=1)                    # round-trip pixel error
    ok = (st1.ravel() == 1) & (st2.ravel() == 1) & (fb < FB_THRESH)        # both ok + consistent
    return p0[ok], p1[ok]


def lookup_depth(depth_map, pts):
    """Sample the dense depth map at each (sub-pixel) feature location.

    Rounds each feature (x, y) to the nearest integer pixel, clamps it to the
    image bounds, and reads the depth there. Pixels with no valid stereo match
    come back as +inf and are dropped by the caller's depth filter.
    """
    h, w = depth_map.shape
    xi = np.clip(np.round(pts[:, 0]).astype(int), 0, w - 1)  # column index, bounded
    yi = np.clip(np.round(pts[:, 1]).astype(int), 0, h - 1)  # row index, bounded
    return depth_map[yi, xi]


def save_viz(path, imL, p_t, p_t1, depths, v_cam):
    img = cv2.cvtColor(imL, cv2.COLOR_GRAY2BGR)
    fig, ax = plt.subplots(figsize=(14, 4.5))
    ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    for (x, y), (x1, y1), d in zip(p_t, p_t1, depths):
        ax.plot(x, y, 'o', color='blue', ms=3)
        ax.annotate("", xy=(x1, y1), xytext=(x, y),
                    arrowprops=dict(arrowstyle="->", color='lime', lw=0.8))
        ax.text(x + 3, y - 3, f"{d:.0f}", color='yellow', fontsize=5)
    ax.text(10, 25, f"GT vel (cam) [{v_cam[0]:.2f}, {v_cam[1]:.2f}, {v_cam[2]:.2f}] m/s  "
                    f"|v|={np.linalg.norm(v_cam):.2f}",
            color='white', fontsize=10, backgroundcolor='black')
    ax.set_axis_off()
    plt.tight_layout()
    plt.savefig(path, dpi=110, bbox_inches='tight')
    plt.close(fig)


def process_sequence(seq):
    cam = CAM[seq]
    fx, baseline = cam["fx"], cam["baseline"]
    seqdir = f"{KITTI}/sequences/{seq}"
    Ldir, Rdir = f"{seqdir}/image_0", f"{seqdir}/image_1"
    frames = sorted(glob.glob(f"{Ldir}/*.png"))
    nf = len(frames)
    poses, times = load_poses(seq), load_times(seq)
    outdir = f"{OUTROOT}/seq_{seq}"
    vizdir = f"{outdir}/viz"
    Path(vizdir).mkdir(parents=True, exist_ok=True)

    # Flat-table exports written alongside the .npz, in both .csv and .txt:
    #   tracks_depth.{csv,txt} : frame_idx,timestamp,u,v,depth_m,u_next,v_next
    #   velocity.{csv,txt}     : frame_idx,timestamp,vx_cam,vy_cam,vz_cam,speed_mps
    trk_hdr = "frame_idx,timestamp,u,v,depth_m,u_next,v_next"
    vel_hdr = "frame_idx,timestamp,vx_cam,vy_cam,vz_cam,speed_mps"
    trk_csv = open(f"{outdir}/tracks_depth.csv", "w")
    trk_txt = open(f"{outdir}/tracks_depth.txt", "w")
    vel_csv = open(f"{outdir}/velocity.csv", "w")
    vel_txt = open(f"{outdir}/velocity.txt", "w")
    trk_csv.write(trk_hdr + "\n"); trk_txt.write(trk_hdr.replace(",", " ") + "\n")
    vel_csv.write(vel_hdr + "\n"); vel_txt.write(vel_hdr.replace(",", " ") + "\n")

    print(f"\n=== sequence {seq}: {nf} frames ===")
    sum_feat = sum_depth = sum_speed = 0.0
    cnt_feat_frames = 0
    for t in range(nf):
        # --- load the stereo pair for frame t (grayscale, as KITTI ships them) ---
        imL = cv2.imread(f"{Ldir}/{t:06d}.png", cv2.IMREAD_GRAYSCALE)
        imR = cv2.imread(f"{Rdir}/{t:06d}.png", cv2.IMREAD_GRAYSCALE)
        # --- (1) dense stereo depth map for this frame ---
        depth_map = compute_depth_map(imL, imR, fx, baseline)

        last = (t == nf - 1)
        if not last:
            # --- (2) tracks need frame t+1; detect+track+FB-check ---
            imL1 = cv2.imread(f"{Ldir}/{t+1:06d}.png", cv2.IMREAD_GRAYSCALE)
            p_t, p_t1 = feature_tracks(imL, imL1)
        else:
            # last frame has no t+1: keep depth-only corners, zero the (nonexistent) tracks
            p0 = cv2.goodFeaturesToTrack(imL, **GFTT)
            p_t = p0.reshape(-1, 2).astype(np.float32) if p0 is not None else np.empty((0, 2), np.float32)
            p_t1 = np.zeros_like(p_t)

        # --- (3) enforce the pairing constraint: every kept feature must have a
        #         valid metric depth (0 < Z < 80 m, finite). Tracks and depths are
        #         filtered with the SAME mask so the arrays stay row-aligned. ---
        d = lookup_depth(depth_map, p_t) if len(p_t) else np.empty((0,), np.float32)
        keep = np.isfinite(d) & (d > DEPTH_MIN) & (d < DEPTH_MAX)
        p_t, p_t1, d = p_t[keep], p_t1[keep], d[keep].astype(np.float32)
        if last:
            p_t1 = np.zeros_like(p_t)                         # spec: zero tracks on last frame

        # --- (4) ground-truth velocity expressed in the camera frame ---
        if not last:
            R_t, _ = poses[t]                                 # camera->world rotation at t
            dt = times[t + 1] - times[t]                      # real capture interval (s)
            v_world = (poses[t + 1][1] - poses[t][1]) / dt    # finite-diff world velocity
            v_cam = (R_t.T @ v_world).astype(np.float32)      # rotate world->camera frame
        else:
            v_cam = np.zeros(3, np.float32)                   # spec: zero velocity on last frame

        # --- persist this frame: 6 arrays, exactly the PM-requested schema ---
        np.savez(f"{outdir}/frame_{t:06d}.npz",
                 feature_pixels_t=p_t.astype(np.float32),
                 feature_pixels_t1=p_t1.astype(np.float32),
                 depths=d,
                 gt_velocity_cam=v_cam,
                 timestamp=np.float64(times[t]),
                 frame_idx=int(t))

        # --- append this frame to the flat CSV/TXT tables (skip the last frame:
        #     its tracks/velocity are zero placeholders) ---
        if not last:
            for (u, v), (un, vn), dd in zip(p_t, p_t1, d):
                row = "%d,%.9f,%.3f,%.3f,%.4f,%.3f,%.3f" % (t, times[t], u, v, dd, un, vn)
                trk_csv.write(row + "\n"); trk_txt.write(row.replace(",", " ") + "\n")
            speed = float(np.linalg.norm(v_cam))
            vrow = "%d,%.9f,%.5f,%.5f,%.5f,%.5f" % (t, times[t], v_cam[0], v_cam[1], v_cam[2], speed)
            vel_csv.write(vrow + "\n"); vel_txt.write(vrow.replace(",", " ") + "\n")

        if len(d):
            sum_feat += len(d); sum_depth += float(np.mean(d)); cnt_feat_frames += 1
        sum_speed += float(np.linalg.norm(v_cam))

        if t % 100 == 0:
            save_viz(f"{vizdir}/frame_{t:06d}.png", imL, p_t, p_t1, d, v_cam)
            md = float(np.mean(d)) if len(d) else 0.0
            print(f"  frame {t}/{nf} | features tracked: {len(d)} | "
                  f"mean depth: {md:.1f}m | gt speed: {np.linalg.norm(v_cam):.2f} m/s")

    for f in (trk_csv, trk_txt, vel_csv, vel_txt):
        f.close()

    meta = dict(sequence=seq, num_frames=nf,
                fx=cam["fx"], fy=cam["fy"], cx=cam["cx"], cy=cam["cy"],
                baseline=baseline, image_width=int(imL.shape[1]), image_height=int(imL.shape[0]),
                mean_features_per_frame=sum_feat / max(cnt_feat_frames, 1),
                mean_depth_per_frame=sum_depth / max(cnt_feat_frames, 1),
                mean_gt_speed=sum_speed / nf,
                last_frame_has_zero_tracks=True)
    json.dump(meta, open(f"{outdir}/metadata.json", "w"), indent=2)
    print(f"  -> {outdir}  (avg feat={meta['mean_features_per_frame']:.1f}, "
          f"avg depth={meta['mean_depth_per_frame']:.1f}m, avg speed={meta['mean_gt_speed']:.2f} m/s)")
    return meta


if __name__ == "__main__":
    seqs = sys.argv[1:] or ["00", "01", "05"]
    Path(OUTROOT).mkdir(parents=True, exist_ok=True)
    metas = [process_sequence(s) for s in seqs]
    print("\n==== FINAL SUMMARY ====")
    print("| Seq | Frames | Avg features/frame | Avg depth (m) | Avg GT speed (m/s) |")
    print("|-----|--------|--------------------|---------------|--------------------|")
    for m in metas:
        print(f"| {m['sequence']} | {m['num_frames']} | {m['mean_features_per_frame']:.1f} "
              f"| {m['mean_depth_per_frame']:.1f} | {m['mean_gt_speed']:.2f} |")
