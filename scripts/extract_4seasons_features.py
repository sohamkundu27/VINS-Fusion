#!/usr/bin/env python3
"""
Standalone OpenCV extractor for 4Seasons sequences. Produces three per-frame
data products on the OFFICIAL undistorted (stereo-rectified) images:

  1. FEATURE TRACKS  : KLT forward-backward optical flow, left[t] -> left[t+1]
  2. DEPTH (sparse)  : stereo match left[t] <-> right[t], disparity -> depth
  3. GT VELOCITY     : differentiate GNSS reference poses, expressed in CAMERA frame

Rectified pinhole intrinsics (from calibration/undistorted_calib_0.txt):
  fx = fy = 501.4757919305817 ; cx = 421.7953735163109 ; cy = 167.65799492501083
Stereo baseline (undistorted_calib_stereo.txt): 0.3004961618953472 m
  depth = fx * baseline / disparity         (rectified => disparity = u_left - u_right)

Outputs (per sequence, in features/<label>/):
  tracks_depth.csv / .txt : frame_idx, timestamp, u, v, depth_m, u_next, v_next
  velocity.csv     / .txt : frame_idx, timestamp, vx_cam, vy_cam, vz_cam, speed_mps
  summary.txt
"""
import os, sys, glob
import numpy as np
import cv2
from scipy.spatial.transform import Rotation, Slerp

FX = 501.4757919305817
FY = 501.4757919305817
CX = 421.7953735163109
CY = 167.65799492501083
BASELINE = 0.3004961618953472

# KLT / feature params
MAX_CORNERS = 500
QUALITY = 0.01
MIN_DIST = 12
LK = dict(winSize=(21, 21), maxLevel=3,
          criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
FB_THRESH = 1.0          # forward-backward consistency (px)
STEREO_V_TOL = 1.5       # rectified: vertical disparity must be ~0 (px)
DISP_MIN = 0.5           # px  -> max depth ~ FX*B/0.5 ~ 300m
DEPTH_MIN, DEPTH_MAX = 1.0, 100.0   # keep plausible driving depths (m)


def find_cam_dir(seqroot, cam):
    for pat in ["undistorted_images/%s" % cam, "*/%s" % cam, cam]:
        hits = [h for h in glob.glob(os.path.join(seqroot, pat)) if os.path.isdir(h)]
        if hits:
            return hits[0]
    raise RuntimeError("no %s dir under %s" % (cam, seqroot))


def load_gnss(path):
    rows = []
    for line in open(path):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        v = line.replace(",", " ").split()
        rows.append([float(v[0]) / 1e9, float(v[1]), float(v[2]), float(v[3]),
                     float(v[4]), float(v[5]), float(v[6]), float(v[7])])  # ts, xyz, qxyzw
    a = np.array(rows)
    return a[:, 0], a[:, 1:4], a[:, 4:8]


def camera_frame_velocity(gnss_ts, gnss_pos, gnss_quat, img_ts):
    """World velocity by central difference at GNSS samples, interpolated to
    image timestamps, rotated into the camera frame using slerp'd orientation."""
    # world velocity at gnss samples
    vw = np.zeros_like(gnss_pos)
    vw[1:-1] = (gnss_pos[2:] - gnss_pos[:-2]) / (gnss_ts[2:] - gnss_ts[:-2])[:, None]
    vw[0] = (gnss_pos[1] - gnss_pos[0]) / (gnss_ts[1] - gnss_ts[0])
    vw[-1] = (gnss_pos[-1] - gnss_pos[-2]) / (gnss_ts[-1] - gnss_ts[-2])
    # clamp image ts to gnss coverage
    lo, hi = gnss_ts[0], gnss_ts[-1]
    valid = (img_ts >= lo) & (img_ts <= hi)
    vwi = np.empty((len(img_ts), 3))
    for k in range(3):
        vwi[:, k] = np.interp(img_ts, gnss_ts, vw[:, k])
    slerp = Slerp(gnss_ts, Rotation.from_quat(gnss_quat))
    tcl = np.clip(img_ts, lo, hi)
    Rwi = slerp(tcl)  # R_world_cam at each image ts
    vcam = np.empty((len(img_ts), 3))
    for i in range(len(img_ts)):
        vcam[i] = Rwi[i].as_matrix().T @ vwi[i]   # v_cam = R_world_cam^T * v_world
    return vcam, valid


def process(seqroot, gnss_path, label, outdir):
    os.makedirs(outdir, exist_ok=True)
    cam0 = find_cam_dir(seqroot, "cam0")
    cam1 = find_cam_dir(seqroot, "cam1")
    f0 = sorted(glob.glob(os.path.join(cam0, "*.png")), key=lambda p: int(os.path.splitext(os.path.basename(p))[0]))
    names = [os.path.splitext(os.path.basename(p))[0] for p in f0]
    ts = np.array([int(n) / 1e9 for n in names])
    right = {os.path.splitext(os.path.basename(p))[0]: p for p in glob.glob(os.path.join(cam1, "*.png"))}
    print("[%s] frames=%d" % (label, len(f0)))

    gts, gpos, gquat = load_gnss(gnss_path)
    vcam, vvalid = camera_frame_velocity(gts, gpos, gquat, ts)

    trk = open(os.path.join(outdir, "tracks_depth.csv"), "w")
    trk.write("frame_idx,timestamp,u,v,depth_m,u_next,v_next\n")
    vel = open(os.path.join(outdir, "velocity.csv"), "w")
    vel.write("frame_idx,timestamp,vx_cam,vy_cam,vz_cam,speed_mps\n")

    n_feat_total = 0
    for i in range(len(f0) - 1):
        name = names[i]
        if name not in right:
            continue
        Lt = cv2.imread(f0[i], cv2.IMREAD_GRAYSCALE)
        Lt1 = cv2.imread(f0[i + 1], cv2.IMREAD_GRAYSCALE)
        Rt = cv2.imread(right[name], cv2.IMREAD_GRAYSCALE)
        if Lt is None or Lt1 is None or Rt is None:
            continue
        pts = cv2.goodFeaturesToTrack(Lt, MAX_CORNERS, QUALITY, MIN_DIST)
        if pts is None:
            continue
        pts = pts.reshape(-1, 2).astype(np.float32)

        # temporal flow left[t] -> left[t+1] with forward-backward check
        nxt, st, _ = cv2.calcOpticalFlowPyrLK(Lt, Lt1, pts, None, **LK)
        back, st2, _ = cv2.calcOpticalFlowPyrLK(Lt1, Lt, nxt, None, **LK)
        fb = np.linalg.norm(pts - back, axis=1)
        ok_t = (st.ravel() == 1) & (st2.ravel() == 1) & (fb < FB_THRESH)

        # stereo flow left[t] -> right[t] for disparity/depth
        rpt, sr, _ = cv2.calcOpticalFlowPyrLK(Lt, Rt, pts, None, **LK)
        disp = pts[:, 0] - rpt[:, 0]
        dv = np.abs(pts[:, 1] - rpt[:, 1])
        ok_s = (sr.ravel() == 1) & (disp > DISP_MIN) & (dv < STEREO_V_TOL)
        depth = np.where(disp > DISP_MIN, FX * BASELINE / np.maximum(disp, 1e-6), -1.0)
        ok_s = ok_s & (depth > DEPTH_MIN) & (depth < DEPTH_MAX)

        keep = ok_t & ok_s
        for j in np.where(keep)[0]:
            trk.write("%d,%.9f,%.3f,%.3f,%.4f,%.3f,%.3f\n" %
                      (i, ts[i], pts[j, 0], pts[j, 1], depth[j], nxt[j, 0], nxt[j, 1]))
        n_feat_total += int(keep.sum())

        if vvalid[i]:
            v = vcam[i]
            vel.write("%d,%.9f,%.5f,%.5f,%.5f,%.5f\n" %
                      (i, ts[i], v[0], v[1], v[2], float(np.linalg.norm(v))))
        if i % 2000 == 0:
            print("  [%s] frame %d/%d  feats so far=%d" % (label, i, len(f0), n_feat_total))
    trk.close(); vel.close()

    # txt mirrors (space-separated)
    for base in ["tracks_depth", "velocity"]:
        with open(os.path.join(outdir, base + ".csv")) as fin, \
             open(os.path.join(outdir, base + ".txt"), "w") as fout:
            for line in fin:
                fout.write(line.replace(",", " "))

    with open(os.path.join(outdir, "summary.txt"), "w") as s:
        s.write("sequence: %s\n" % label)
        s.write("frames: %d\n" % len(f0))
        s.write("total kept features (tracked+depth): %d\n" % n_feat_total)
        s.write("mean features/frame: %.1f\n" % (n_feat_total / max(len(f0) - 1, 1)))
        s.write("rectified fx=%.4f cx=%.4f cy=%.4f baseline=%.6f m\n" % (FX, CX, CY, BASELINE))
        s.write("depth range kept: [%.1f, %.1f] m\n" % (DEPTH_MIN, DEPTH_MAX))
    print("[%s] DONE total_features=%d -> %s" % (label, n_feat_total, outdir))


if __name__ == "__main__":
    seqroot, gnss_path, label, outdir = sys.argv[1:5]
    process(seqroot, gnss_path, label, outdir)
