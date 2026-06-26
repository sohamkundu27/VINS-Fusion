# Ground Stereo Odometry — Results & Extracted Feature Dataset

This directory holds the evaluation results and extracted training data from running
**VINS-Fusion** on the **KITTI Odometry** and **4Seasons** datasets. It contains the
per-frame extracted feature/depth/velocity dataset (samples), the full 4Seasons
visual-inertial evaluation report, and publication-quality plots. The extraction
scripts live in [`../scripts/`](../scripts/).

---

## 1. KITTI Odometry results (stereo, no IMU)

ATE RMSE (m), SE(3) Umeyama alignment, sequences 00 / 01 / 05:

| Seq | Environment | Baseline (default cfg) | Tuned (`max_solver_time` 0.08→0.5) | + Loop closure |
|-----|-------------|-----------------------:|-----------------------------------:|---------------:|
| 00  | City        | 13.61 | 14.71 | **5.01** |
| 01  | Highway     | 37.70 | **7.34** | — (no loops on highway) |
| 05  | Residential | 5.94  | 5.97  | **3.71** |

- The single decisive tuning knob was **`max_solver_time`** (0.08 s → 0.5 s): on the long, fast
  highway seq 01 the 80 ms cap was starving Ceres every frame, inflating drift to 37 m; the full
  budget cut it to 7.3 m (−80 %).
- **Loop closure** (`loop_fusion_node`) closes the remaining gap on the looped city/residential
  sequences — this is what the VINS-Fusion KITTI benchmark numbers actually use.

## 2. 4Seasons results (stereo + IMU, open-loop VIO, no loop closure)

SE(3)-aligned, RPE Δ=100 frames, attitude via geodesic SO(3) angle:

| Sequence | Env / Weather | ATE RMSE | RPE RMSE | Att RMSE | Vel RMSE |
|----------|---------------|---------:|---------:|---------:|---------:|
| `recording_2020-10-08_09-57-28` | Office, clear   | 119.93 m | 3.21 m | 4.95° | 0.49 m/s |
| `recording_2020-10-07_14-47-51` | Neighbor, clear | 13.70 m  | 1.74 m | 3.93° | 0.19 m/s |
| `recording_2021-02-25_13-51-57` | Neighbor, snow  | 33.24 m  | 2.13 m | 4.72° | 0.34 m/s |

Full analysis (frame conventions, yaw-drift decomposition, weather A/B): [`4seasons/4seasons_report.pdf`](4seasons/4seasons_report.pdf).
Plots in [`4seasons/plots/`](4seasons/plots/).

## 3. Comparison vs Paper 5 (Sharafutdinov et al., *Comparison of modern open-source Visual SLAM approaches*, arXiv:2108.01654)

VINS-Fusion stereo on KITTI, ATE RMSE (m) / rotation RMSE (deg):

| Seq | Ours (open-loop VIO) | **Ours (+ loop closure)** | Paper 5 (VINS-Fusion stereo) |
|-----|---------------------:|--------------------------:|-----------------------------:|
| 00  | 14.71 / 2.72° | **5.01 / 3.55°** | 5.20 / 3.22° |
| 05  | 5.97 / 2.98°  | **3.71 / 2.08°** | 4.79 / 3.02° |

With loop closure enabled we **match Paper 5 on seq 00 (5.01 vs 5.20 m)** and **beat it on seq 05
(3.71 vs 4.79 m)**. The earlier 2–3× gap was entirely explained by Paper 5 running the `loop_fusion`
node while our first runs were pure open-loop VIO — not a tuning or calibration difference.

## 4. Extracted feature dataset

At each time step *t* we save **stereo depth**, **feature tracks** *t → t+1*, and **ground-truth
velocity in the camera frame**, with the constraint that **every feature has both a valid track and a
valid depth**.

**KITTI** — one `.npz` per frame ([`kitti/extracted/seq_XX/`](kitti/extracted/); repo holds the first
5 frames + all viz, full set is generated locally):

| Array | Shape | Meaning |
|-------|-------|---------|
| `feature_pixels_t`  | (N,2) f32 | (u,v) at *t* |
| `feature_pixels_t1` | (N,2) f32 | (u,v) at *t+1* (KLT, forward-backward checked) |
| `depths`            | (N,) f32  | metric depth at *t* (SGBM, 0–80 m) |
| `gt_velocity_cam`   | (3,) f32  | GT velocity in **camera frame** `R_tᵀ·v_world` |
| `pred_velocity_cam` | (3,) f32  | VINS-Fusion predicted velocity in camera frame (finite-diff on `vio.txt`) |
| `velocity_error`    | (3,) f32  | `pred − gt` (camera frame) |
| `velocity_error_magnitude` | () f32 | `‖pred − gt‖` |
| `timestamp`         | f64       | seconds |
| `frame_idx`         | int       | frame index |

VINS predicted velocity uses finite differences on the estimated trajectory (`vio.txt`)
projected into the camera frame; the stereo-only/no-IMU runs leave `vio.csv` `vx,vy,vz` at zero,
so they are not used. Velocity in the body frame is alignment-invariant, so GT and prediction are
directly comparable without trajectory alignment:

| Seq | GT mean speed | PRED mean speed | Vel error RMSE |
|-----|--------------:|----------------:|---------------:|
| 00  | 7.91 m/s | 7.89 m/s | 0.28 m/s |
| 01  | 21.51 m/s | 21.45 m/s | 6.51 m/s |
| 05  | 7.67 m/s | 7.75 m/s | 0.19 m/s |

Mean speeds match GT to within 0.1 m/s; seq 01's high RMSE is the fast highway run (sparse features,
per-frame scale jitter) — the same sequence that needed the `max_solver_time` fix.

**4Seasons** — CSV/TXT per recording ([`4seasons/features/`](4seasons/features/); repo holds 500-row
samples):
- `tracks_depth_sample.csv`: `frame_idx, timestamp, u, v, depth_m, u_next, v_next` (each row = one
  feature that has **both** a track and a depth)
- `velocity_sample.csv`: `frame_idx, timestamp, vx_cam, vy_cam, vz_cam, speed_mps` (camera frame)

Dataset scale (full, generated locally): KITTI 8 403 frames; 4Seasons 9.38 M paired features across
38 601 frames.

**Per-sequence overlay videos** ([`kitti/extracted/seq_XX/viz_seq_XX.mp4`](kitti/extracted/)) show
**every frame** with blue feature dots, green KLT flow arrows, per-feature depth labels, and the
GT / PRED / Error velocity block (red error text when `|err| > 0.5 m/s`). Generated by
[`../scripts/make_kitti_viz.py`](../scripts/make_kitti_viz.py) (full-resolution JPEG frames are kept
locally; the committed MP4s are downscaled H.264 to stay repo-friendly).

## 5. Reproducing

All scripts in [`../scripts/`](../scripts/):

| Script | Purpose |
|--------|---------|
| `extract_kitti_features.py`     | KITTI → per-frame `.npz` (SGBM depth + KLT tracks + GT velocity) |
| `extract_4seasons_features.py`  | 4Seasons → `tracks_depth`/`velocity` CSV+TXT |
| `build_bag.py`                  | 4Seasons images+IMU → ROS bag (4Seasons isn't distributed as bags) |
| `gt_convert.py`                 | 4Seasons `GNSSPoses.txt` → TUM, transformed into the IMU frame |
| `metrics_4seasons.py`           | ATE/RPE/attitude/F-score/velocity vs ground truth |

VINS-Fusion 4Seasons config (stereo+IMU, Kannala-Brandt fisheye): [`../config/4seasons/`](../config/4seasons/).

## 6. Key findings

- **The ATE gap to published numbers was a *loop-closure* difference, not tuning.** With
  `loop_fusion` enabled we match/beat Paper 5; our open-loop rotation accuracy is already competitive.
- **`max_solver_time` dominates open-loop drift** on long, fast sequences (seq 01: −80 % ATE).
- **With an IMU, ATE is almost entirely yaw drift** — roll/pitch stay gravity-locked at 1–2°, while
  yaw drifts monotonically (0→15° over the 6.9 km office route), producing the large open-loop ATE.
- **Snow measurably degrades but does not break VIO**: on the same neighborhood route, ATE rose 13.7→33.2 m
  and RPE 1.74→2.13 m, yet VINS tracked >99.8 % of frames with no loss.
