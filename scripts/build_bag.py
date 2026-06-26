#!/usr/bin/env python
"""
Build a ROS bag from a 4Seasons sequence (distorted stereo images + IMU).
Runs INSIDE the ros:vins-fusion container (Python2 / ROS Kinetic).

Topics produced:
  /cam0/image_raw  sensor_msgs/Image (mono8)
  /cam1/image_raw  sensor_msgs/Image (mono8)
  /imu0            sensor_msgs/Imu

All messages written in global timestamp order so `rosbag play` reproduces
the real-time stream VINS-Fusion expects.

Usage (inside container):
  python build_bag.py <sequence_dir> <imu_txt> <out_bag>
where <sequence_dir> contains distorted_images/cam0/*.png and cam1/*.png
"""
import sys, os, glob
import rospy, rosbag
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge
import cv2

def find_cam_dir(seqdir, cam):
    # robustly locate the cam0/cam1 image folder regardless of parent name
    for pat in ["distorted_images/%s" % cam,
                "undistorted_images/%s" % cam,
                "*/%s" % cam, cam]:
        hits = glob.glob(os.path.join(seqdir, pat))
        hits = [h for h in hits if os.path.isdir(h)]
        if hits:
            return hits[0]
    raise RuntimeError("cannot find image dir for %s under %s" % (cam, seqdir))

def stamp_from_ns(ns):
    return rospy.Time(int(ns) // 1000000000, int(ns) % 1000000000)

def main():
    seqdir, imu_txt, out_bag = sys.argv[1], sys.argv[2], sys.argv[3]
    cam0 = find_cam_dir(seqdir, "cam0")
    cam1 = find_cam_dir(seqdir, "cam1")
    print("cam0 dir: %s" % cam0)
    print("cam1 dir: %s" % cam1)

    # image timestamp(ns) -> filename, keyed off cam0; require matching cam1 file
    c0 = {}
    for p in glob.glob(os.path.join(cam0, "*.png")):
        ts = os.path.splitext(os.path.basename(p))[0]
        c0[ts] = p
    c1 = {}
    for p in glob.glob(os.path.join(cam1, "*.png")):
        ts = os.path.splitext(os.path.basename(p))[0]
        c1[ts] = p
    common = sorted(set(c0.keys()) & set(c1.keys()), key=lambda s: int(s))
    print("cam0=%d cam1=%d common stereo pairs=%d" % (len(c0), len(c1), len(common)))

    # IMU rows: timestamp_ns gx gy gz ax ay az
    imu_rows = []
    with open(imu_txt) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            v = line.split()
            imu_rows.append((int(v[0]), float(v[1]), float(v[2]), float(v[3]),
                             float(v[4]), float(v[5]), float(v[6])))
    imu_rows.sort(key=lambda r: r[0])
    print("imu samples=%d" % len(imu_rows))

    # Build a single merged, time-ordered event list
    events = []
    for ts in common:
        events.append((int(ts), "img", ts))
    for r in imu_rows:
        events.append((r[0], "imu", r))
    events.sort(key=lambda e: e[0])

    bridge = CvBridge()
    bag = rosbag.Bag(out_bag, "w")
    n_img = n_imu = 0
    try:
        for ev_ts, kind, payload in events:
            t = stamp_from_ns(ev_ts)
            if kind == "imu":
                _, gx, gy, gz, ax, ay, az = payload
                m = Imu()
                m.header.stamp = t
                m.header.frame_id = "imu0"
                m.angular_velocity.x = gx
                m.angular_velocity.y = gy
                m.angular_velocity.z = gz
                m.linear_acceleration.x = ax
                m.linear_acceleration.y = ay
                m.linear_acceleration.z = az
                m.orientation_covariance[0] = -1  # orientation not provided
                bag.write("/imu0", m, t)
                n_imu += 1
            else:
                ts = payload
                im0 = cv2.imread(c0[ts], cv2.IMREAD_GRAYSCALE)
                im1 = cv2.imread(c1[ts], cv2.IMREAD_GRAYSCALE)
                if im0 is None or im1 is None:
                    continue
                msg0 = bridge.cv2_to_imgmsg(im0, encoding="mono8")
                msg0.header.stamp = t; msg0.header.frame_id = "cam0"
                msg1 = bridge.cv2_to_imgmsg(im1, encoding="mono8")
                msg1.header.stamp = t; msg1.header.frame_id = "cam1"
                bag.write("/cam0/image_raw", msg0, t)
                bag.write("/cam1/image_raw", msg1, t)
                n_img += 1
                if n_img % 1000 == 0:
                    print("  ... %d stereo pairs written" % n_img)
    finally:
        bag.close()
    print("DONE bag=%s  stereo_pairs=%d  imu=%d" % (out_bag, n_img, n_imu))

if __name__ == "__main__":
    main()
