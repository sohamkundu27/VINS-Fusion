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
import sys, glob
import rospy, rosbag
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge
import cv2

def find_cam_dir(seqdir, cam):
    # robustly locate the cam0/cam1 image folder regardless of parent name
    for pat in ["distorted_images/%s" % cam,
                "undistorted_images/%s" % cam,
                "*/%s" % cam, cam]:
        # trailing "/" makes glob match directories only; strip it back off
        hits = [h.rstrip("/") for h in glob.glob(seqdir + "/" + pat + "/")]
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

    # 4Seasons names every image file by its capture timestamp in nanoseconds.
    # Index cam0 and cam1 frames by that timestamp string so we can pair them.
    c0 = {}
    for p in glob.glob(cam0 + "/*.png"):
        ts = p.split("/")[-1].rsplit(".", 1)[0]   # "<timestamp_ns>.png" -> "<timestamp_ns>"
        c0[ts] = p
    c1 = {}
    for p in glob.glob(cam1 + "/*.png"):
        ts = p.split("/")[-1].rsplit(".", 1)[0]
        c1[ts] = p
    # keep only timestamps present in BOTH cameras (a complete stereo pair), time-ordered
    common = sorted(set(c0.keys()) & set(c1.keys()), key=lambda s: int(s))
    print("cam0=%d cam1=%d common stereo pairs=%d" % (len(c0), len(c1), len(common)))

    # Parse imu.txt — one sample per line: "timestamp_ns gx gy gz ax ay az"
    # (gyro rad/s, accel m/s^2). Stored as tuples for the merge step.
    imu_rows = []
    with open(imu_txt) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            v = line.split()
            imu_rows.append((int(v[0]), float(v[1]), float(v[2]), float(v[3]),
                             float(v[4]), float(v[5]), float(v[6])))
    imu_rows.sort(key=lambda r: r[0])                   # ensure monotonic timestamps
    print("imu samples=%d" % len(imu_rows))

    # Merge images + IMU into ONE list sorted by timestamp, so the bag reproduces
    # the real sensor stream order VINS expects (IMU interleaved between frames).
    events = []
    for ts in common:
        events.append((int(ts), "img", ts))            # tag each event with its kind
    for r in imu_rows:
        events.append((r[0], "imu", r))
    events.sort(key=lambda e: e[0])                     # global chronological order

    bridge = CvBridge()                                 # converts cv2 images -> ROS Image msgs
    bag = rosbag.Bag(out_bag, "w")
    n_img = n_imu = 0
    try:
        for ev_ts, kind, payload in events:
            t = stamp_from_ns(ev_ts)                    # ROS Time for this event
            if kind == "imu":
                # --- build a sensor_msgs/Imu message ---
                _, gx, gy, gz, ax, ay, az = payload
                m = Imu()
                m.header.stamp = t
                m.header.frame_id = "imu0"
                m.angular_velocity.x = gx               # gyro (rad/s)
                m.angular_velocity.y = gy
                m.angular_velocity.z = gz
                m.linear_acceleration.x = ax            # accel (m/s^2)
                m.linear_acceleration.y = ay
                m.linear_acceleration.z = az
                m.orientation_covariance[0] = -1        # signal "no orientation provided"
                bag.write("/imu0", m, t)
                n_imu += 1
            else:
                # --- build the two sensor_msgs/Image messages for a stereo pair ---
                ts = payload
                im0 = cv2.imread(c0[ts], cv2.IMREAD_GRAYSCALE)   # left  (mono8)
                im1 = cv2.imread(c1[ts], cv2.IMREAD_GRAYSCALE)   # right (mono8)
                if im0 is None or im1 is None:
                    continue
                msg0 = bridge.cv2_to_imgmsg(im0, encoding="mono8")
                msg0.header.stamp = t; msg0.header.frame_id = "cam0"
                msg1 = bridge.cv2_to_imgmsg(im1, encoding="mono8")
                msg1.header.stamp = t; msg1.header.frame_id = "cam1"
                # both images share the SAME stamp so VINS treats them as one stereo frame
                bag.write("/cam0/image_raw", msg0, t)
                bag.write("/cam1/image_raw", msg1, t)
                n_img += 1
                if n_img % 1000 == 0:
                    print("  ... %d stereo pairs written" % n_img)
    finally:
        bag.close()                                     # always flush/close the bag
    print("DONE bag=%s  stereo_pairs=%d  imu=%d" % (out_bag, n_img, n_imu))

if __name__ == "__main__":
    main()
