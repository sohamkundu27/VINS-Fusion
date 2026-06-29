# 4Seasons — sample data

A small, representative slice of the raw [4Seasons dataset](https://cvg.cit.tum.de/data/datasets/4seasons-dataset)
so you can see what the source data looks like without downloading the full multi-GB sequences.

## Folders

| Folder | What's in it |
|--------|--------------|
| `calibration/` | Sensor calibration: `camchain.yaml` (stereo + IMU intrinsics, fisheye distortion, `T_cam_imu` extrinsics), the rectified pinhole intrinsics (`undistorted_calib_0.txt`), the rectified stereo baseline (`undistorted_calib_stereo.txt`), and the original distorted calibration (`calib_0.txt`). |
| `recording_2020-10-08_09-57-28/` | A peek at one full sequence (office loop, clear): two stereo image pairs under `undistorted_images/cam0` and `cam1`, plus head snippets of the raw streams — `imu_head.txt`, `GNSSPoses_head.txt` (ground-truth poses), `times_head.txt`, and `Transformations.txt` (fixed frame transforms). |
| `seasons/` | One left-camera frame from each of four recordings — spring, summer, autumn, winter — showing the dataset's cross-season coverage. |
