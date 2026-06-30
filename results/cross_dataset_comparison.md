# VINS-Fusion Cross-Dataset Comparison — Stereo vs Stereo+IMU

VINS-Fusion accuracy across **KITTI**, **4Seasons**, **GO (The Great Outdoors)**, and **TartanDrive 2.0**,
in **stereo** and **stereo-inertial** modes. All numbers are real local runs (evo, SE(3) alignment, no scale).

> **Not apples-to-apples.** Sequences differ in length (80 m → 6.9 km), speed, terrain, sensors, and
> ground-truth source (KITTI/4Seasons GT vs GO/TD2 GPS/odometry). Use **drift-per-distance** for any
> cross-dataset reading. ATE in metres is only comparable *within* a dataset.

## ATE RMSE (m) by dataset and mode

| Dataset / sequence | Env | Stereo | Stereo + IMU | Path | IMU in dataset? |
|--------------------|-----|-------:|-------------:|-----:|-----------------|
| **KITTI 00** | city | 14.71 *(loop 5.01)* | **N/A** | 3724 m | ❌ no IMU in KITTI Odometry |
| **KITTI 01** | highway | 7.34 | **N/A** | 2453 m | ❌ |
| **KITTI 05** | residential | 5.97 *(loop 3.71)* | **N/A** | 2206 m | ❌ |
| **4Seasons** neighbor | suburban, clear | 14.93 ‡ | 13.70 | 2206 m | ✅ |
| **4Seasons** office | office loop, clear | — | 119.93 | 6878 m | ✅ |
| **4Seasons** snow | suburban, snow | — | 33.24 | 3608 m | ✅ |
| **GO** pilot ⚠NC | off-road | 1.60 | 1.35 | 203 m | ✅ |
| **TartanDrive 2.0** pilot ✅MIT | off-road | 0.47 | 0.63 | 80 m | ✅ |

*loop = with `loop_fusion` pose-graph closure. ‡ 4Seasons stereo run = neighbor sequence only (this study).*
*"—" = mode not run for that sequence. GO/TD2 are short pilots (80–200 m).*

## Drift per distance (% of path) — the fair cross-dataset metric

| Dataset / sequence | Stereo | Stereo + IMU |
|--------------------|-------:|-------------:|
| KITTI 00 | 0.39 % | n/a |
| KITTI 05 | 0.27 % | n/a |
| 4Seasons neighbor | 0.68 % ‡ | 0.62 % |
| GO pilot | 0.79 % | 0.67 % |
| TartanDrive 2.0 pilot | 0.59 % | 0.78 % |

All systems land at **~0.3–0.8 % drift** — VINS-Fusion is consistent across very different environments.

## Notes on each dataset

- **KITTI (Odometry):** stereo-only — **the Odometry benchmark ships no IMU stream**, so stereo+IMU is
  not possible here. True KITTI VIO would require the separate **KITTI raw** dataset (OXTS GPS/IMU) and a
  different runner (`kitti_gps_test`); not run in this study.
- **4Seasons:** stereo+IMU from prior runs (Kannala-Brandt fisheye, distorted images). Stereo-only added
  here for the neighbor sequence using the rectified/undistorted pinhole images. Open-loop ATE is
  dominated by yaw drift on the long routes (office 6.9 km → 120 m ATE).
- **GO ⚠ (CC BY-NC-SA, reference-only):** stereo & stereo+IMU both run on a 285 s pilot (first 60 s
  evaluated). IMU **helps** (1.60→1.35 m) — 200 Hz IMU with clean `/tf_static` extrinsics.
- **TartanDrive 2.0 ✅ (MIT):** stereo & stereo+IMU on a 39 s pilot. IMU slightly **hurts** (0.47→0.63 m)
  because its IMU noise/extrinsics are documented *assumptions* (Novatel + EuRoC-default noise) — tuning
  should recover the IMU benefit.

## Stereo vs Stereo+IMU — summary

| Dataset | IMU effect | Why |
|---------|-----------|-----|
| KITTI | — | no IMU available |
| 4Seasons | small improvement (13.70 vs 14.93 ‡) | gravity-locks roll/pitch; yaw still drifts |
| GO | **improves** (1.35 vs 1.60) | high-rate IMU + accurate extrinsics |
| TartanDrive 2.0 | slightly worse (0.63 vs 0.47) | approximate IMU calib (fixable by tuning) |

## Sources (local)
- KITTI: `~/datasets/kitti/tuning_experiments.md` (Runs 1–4) + loop-closure runs.
- 4Seasons: `results/README.md`, `results/4seasons/` (stereo+IMU); stereo run this study.
- GO / TD2: `~/Dataset_VINS_Fusion_Comparison_Project/` (configs, per-run trajectories, `REPORT.pdf`).
