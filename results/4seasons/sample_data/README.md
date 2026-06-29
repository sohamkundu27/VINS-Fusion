# 4Seasons — raw data samples

A tiny slice of the raw [4Seasons dataset](https://cvg.cit.tum.de/data/datasets/4seasons-dataset)
so you can see what the source data looks like before any extraction. Taken from the
`recording_2020-10-08_09-57-28` (office loop, clear weather) sequence.

## What's here

| Path | What it is |
|------|------------|
| `recording_*/undistorted_images/cam0,cam1/<ts>.png` | 2 stereo pairs (left/right), rectified pinhole, 800×400 mono. Filenames are the capture timestamp in **nanoseconds**. |
| `recording_*/imu_head.txt` | First 200 IMU samples. Columns: `timestamp_ns gx gy gz ax ay az` (gyro rad/s, accel m/s²), 2000 Hz. |
| `recording_*/GNSSPoses_head.txt` | First 30 reference (ground-truth) poses. Columns: `timestamp_ns, tx, ty, tz, qx, qy, qz, qw, scale, quality`. |
| `recording_*/times_head.txt` | First 10 image timestamps. |
| `recording_*/Transformations.txt` | Fixed frame transforms (incl. `TS_cam_imu`) used to relate camera, IMU, and GNSS frames. |
| `calibration/camchain.yaml` | Stereo+IMU calibration: intrinsics, distortion (equidistant/fisheye), `T_cam_imu` extrinsics. |
| `calibration/undistorted_calib_0.txt` | Rectified pinhole intrinsics for cam0 (`fx fy cx cy` + size). |
| `calibration/undistorted_calib_stereo.txt` | Rectified stereo extrinsic (baseline ≈ 0.300 m). |
| `calibration/calib_0.txt` | Original (distorted/fisheye) cam0 calibration. |

## How it maps to the extracted data

These raw images + IMU + poses are what produced the extracted feature tables one level up in
`results/4seasons/features/`:
- the **undistorted images** feed the stereo depth + KLT tracking → `tracks_depth_sample.csv`
- the **GNSSPoses** (rotated into the camera frame) → `velocity_sample.csv`

The full sequences are ~3–4 GB each (14k+ frames) and are **not** committed; this folder is just a
representative peek. See the pipeline scripts in [`../../../scripts/`](../../../scripts/).
