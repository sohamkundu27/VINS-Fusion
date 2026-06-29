# VINS-Fusion — KITTI & 4Seasons benchmarking / feature extraction

A fork of [HKUST VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) (optimization-based
multi-sensor visual-inertial state estimator) extended with benchmarking and per-frame feature/depth/velocity
extraction on the **KITTI Odometry** and **4Seasons** datasets. The upstream estimator source is unchanged;
this repo adds the `config/`, `scripts/`, and `results/` work around it.

## Repository structure

```
VINS-Fusion/
├── vins_estimator/   core VINS-Fusion estimator (upstream)
├── loop_fusion/      visual loop closure / pose-graph node (upstream)
├── global_fusion/    GPS / global fusion node (upstream)
├── camera_models/    camera model library — pinhole, KANNALA_BRANDT, MEI (upstream)
├── config/           sensor configs (see below)
├── docker/           Dockerfile + build scripts for VINS-Fusion
├── support_files/    images/assets referenced by the upstream README
├── scripts/          our extraction & evaluation scripts
└── results/          our benchmark results, extracted datasets, plots, report
```

### What's in each folder

| Folder | Contents |
|--------|----------|
| `vins_estimator/`, `loop_fusion/`, `global_fusion/`, `camera_models/` | Unmodified upstream VINS-Fusion source: the estimator, loop-closure node, GPS-fusion node, and camera-model library. |
| `config/` | Per-dataset sensor configs. Includes `config/kitti_odom/` (stereo, used for the KITTI runs) and `config/4seasons/` (stereo + IMU, Kannala-Brandt fisheye — `4seasons_config.yaml`, `cam0.yaml`, `cam1.yaml`), plus the original EuRoC / realsense configs. |
| `scripts/` | The Python pipeline: `extract_kitti_features.py` (stereo depth + KLT tracks + GT velocity → `.npz`/csv/txt), `add_pred_velocity.py`, `make_kitti_viz.py`, plus the 4Seasons helpers `build_bag.py`, `gt_convert.py`, `metrics_4seasons.py`, and `extract_4seasons_features.py`. |
| `results/` | Everything we produced — see `results/README.md`. KITTI extracted dataset (`results/kitti/extracted/`: per-frame `.npz`, metadata, viz videos, tracks/velocity tables) and 4Seasons evaluation (`results/4seasons/`: report PDF, plots, feature samples, and raw `sample_data/`). |
| `docker/`, `support_files/` | Upstream Docker build and README image assets. |

## Built on

This is a fork of **VINS-Fusion** by Tong Qin, Shaozu Cao, Jie Pan, Peiliang Li, and Shaojie Shen
(Aerial Robotics Group, HKUST). For prerequisites, build, and run instructions for the estimator
itself, see the [upstream repository](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion).

## License

The VINS-Fusion source is released under the [GPLv3](http://www.gnu.org/licenses/) license, following
the upstream project.
