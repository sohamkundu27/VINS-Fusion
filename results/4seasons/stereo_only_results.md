# 4Seasons — Stereo-only VINS-Fusion results (this study)

Stereo-only (no IMU) VINS-Fusion on the undistorted/rectified pinhole images
(`/cam0/image_raw`, `/cam1/image_raw`), to complement the existing **stereo+IMU** runs in
[`README.md`](../README.md). Same ground truth, same sequences. Docker `vins-fusion-kitti`,
`evo` with SE(3) Umeyama alignment (no scale). Config: `~/datasets/4seasons/stereo_cfg/4seasons_stereo.yaml`.

## Metrics

| Sequence | Recording | Matched poses | ATE median | **ATE RMSE** | RPE median | RPE RMSE | Att median | Att RMSE | F-score @2m | Vel mean err | Vel std err | Vel RMSE err |
|----------|-----------|--------------:|-----------:|-------------:|-----------:|---------:|-----------:|---------:|------------:|-------------:|------------:|-------------:|
| neighbor (clear) | 2020-10-07_14-47-51 | 2868 | 9.40 m | **20.51 m** | 107.08 m | 106.38 m | 177.13° | 176.86° | 0.000 | 0.150 m/s | 0.201 m/s | 0.215 m/s |
| office (clear) | 2020-10-08_09-57-28 | 5960 | 66.50 m | **87.54 m** | 160.65 m | 159.08 m | 178.66° | 178.47° | 0.000 | 0.389 m/s | 0.525 m/s | 0.553 m/s |
| snow | 2021-02-25_13-51-57 | 4192 | 12.73 m | **44.96 m** | 118.47 m | 121.08 m | 176.97° | 176.96° | 0.000 | 0.210 m/s | 0.374 m/s | 0.374 m/s |

## Stereo vs Stereo+IMU (ATE RMSE)

| Sequence | Path | Stereo | Stereo+IMU | Δ | Drift% stereo | Drift% stereo+IMU |
|----------|-----:|-------:|-----------:|--:|--------------:|------------------:|
| neighbor | 2206 m | 20.51 m | 13.70 m | IMU −33 % | 0.93 % | 0.62 % |
| office | 6878 m | 87.54 m | 119.93 m | IMU +37 % | 1.27 % | 1.74 % |
| snow | 3608 m | 44.96 m | 33.24 m | IMU −26 % | 1.25 % | 0.92 % |

IMU helps on neighbor/snow but **hurts on the 6.9 km office loop** (+37 %). The mechanism is not
conclusively isolated: a trajectory-level investigation (SE(3)-aligned APE and yaw drift vs distance)
found the error is progressive open-loop drift with yaw drift *comparable* between the two modes — too
small a yaw difference to fully explain the ATE gap. See
[`../final_dataset_comparison.md`](../final_dataset_comparison.md) limitations §8.

## Caveats

- **RPE / attitude here are frame-convention artifacts** (the stereo config's cam0 axes vs the IMU/body
  frame in which GT attitude is expressed). They are ~107–180° and are **not** meaningful accuracy
  figures. Only **ATE-translation** is valid for the stereo-only runs, which is what the comparison uses.
- **F-score @2m = 0** on every sequence because absolute position error exceeds the 2 m threshold over
  these km-scale open-loop routes — expected for open-loop VIO at this scale, not a failure.
- These are open-loop (no `loop_fusion`). Pose-graph closure would reduce ATE on the looping routes.

Part of the cross-dataset study — see [`../final_dataset_comparison.md`](../final_dataset_comparison.md).
