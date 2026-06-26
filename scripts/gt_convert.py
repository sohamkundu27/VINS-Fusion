#!/usr/bin/env python3
"""
Convert 4Seasons GNSSPoses.txt (reference / ground-truth poses) to TUM format
for evo, transformed from the camera frame into the IMU/body frame so it lines
up with VINS-Fusion's output frame.

GNSSPoses.txt columns:
  timestamp_ns, tx, ty, tz, qx, qy, qz, qw, scale, fusion_quality
These are T_w_cam0 (cam0 pose in the reference world frame).

VINS outputs T_w_imu (IMU pose). To compare like-with-like we apply the fixed
camera->imu extrinsic:  T_w_imu = T_w_cam0 * T_cam0_imu
T_cam0_imu is taken from calibration/camchain.yaml (cam0.T_cam_imu), i.e. the
transform that maps a point in IMU frame to cam0 frame; its inverse is cam0->imu.

Output TUM:  timestamp_seconds tx ty tz qx qy qz qw
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation

# T_cam0_imu from camchain.yaml (maps IMU-frame point -> cam0 frame)
T_cam0_imu = np.array([
 [-0.9998852242642406, -0.013522961078544133, 0.006831385051241187, 0.17541216744862287],
 [-0.006890161859766396, 0.004304637029338462, -0.9999669973402087, 0.0036894333751345677],
 [0.01349310815180704, -0.9998992947410829, -0.004397318352110671, -0.05810612695941222],
 [0.0, 0.0, 0.0, 1.0]])

def inv(T):
    R = T[:3, :3]; t = T[:3, 3]
    Ti = np.eye(4); Ti[:3, :3] = R.T; Ti[:3, 3] = -R.T @ t
    return Ti

T_cam0_to_imu = inv(T_cam0_imu)  # cam0 -> imu

def main():
    src, dst = sys.argv[1], sys.argv[2]
    out = []
    with open(src) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            v = line.replace(",", " ").split()
            ts_ns = int(v[0])
            tx, ty, tz = float(v[1]), float(v[2]), float(v[3])
            qx, qy, qz, qw = float(v[4]), float(v[5]), float(v[6]), float(v[7])
            T_w_cam = np.eye(4)
            T_w_cam[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
            T_w_cam[:3, 3] = [tx, ty, tz]
            T_w_imu = T_w_cam @ T_cam0_to_imu
            q = Rotation.from_matrix(T_w_imu[:3, :3]).as_quat()  # xyzw
            t = T_w_imu[:3, 3]
            ts_s = ts_ns / 1e9
            out.append("%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f" %
                       (ts_s, t[0], t[1], t[2], q[0], q[1], q[2], q[3]))
    with open(dst, "w") as f:
        f.write("\n".join(out) + "\n")
    print("wrote %d poses -> %s" % (len(out), dst))

if __name__ == "__main__":
    main()
