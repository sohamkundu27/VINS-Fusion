# VINS-Fusion KITTI Reference Metrics from Published Papers

## Coverage Gaps Summary

The following gaps exist across all five surveyed papers — these are measurements your work provides that no single paper covers:

- **Sequence 01 has zero per-sequence data for VINS-Fusion in any paper.** None of the five papers report any metric for sequence 01 individually. Your ATE, RPE, attitude, F-score, and velocity results for seq 01 are entirely novel in the context of these works.
- **No per-sequence RPE (translation or rotation) for VINS-Fusion on sequences 00 or 05** appears in any paper. Paper 2 (arXiv:2507.11241) is the only source with RPE for VINS-Fusion on KITTI, but it is averaged over all 11 sequences (00–10), not broken down per-sequence.
- **No F-score or velocity error metric** for VINS-Fusion on KITTI odometry appears in any of the five papers. These are unique contributions of your evaluation.
- **Attitude error (rotation ATE, per-sequence)** for sequences 00 and 05 is only available from Paper 5 (arXiv:2108.01654), and sequence 01 has no rotation ATE data anywhere.
- Papers 1 and 5 disagree significantly on ATE for VINS-Fusion (e.g., seq 00: 1.24 m RMSE in Paper 1 vs 5.20 m RMSE in Paper 5). This is because **Paper 1 uses stereo+IMU+GPS** while **Paper 5 uses stereo-only** — both run under VINS-Fusion but with fundamentally different sensor configurations. Your results (stereo+IMU, no GPS) fill the middle ground that no paper directly covers.

---

## Reference Comparison Table

| Sequence | Metric | Value | Method | Sensor Config | Source Paper |
|----------|--------|-------|--------|---------------|--------------|
| 00 | ATE mean (m) | 1.18 | VINS-Fusion | Stereo + IMU + GPS | Lvio-Fusion, arXiv:2106.06783, Table I |
| 00 | ATE RMSE (m) | 1.24 | VINS-Fusion | Stereo + IMU + GPS | Lvio-Fusion, arXiv:2106.06783, Table I |
| 00 | ATE position RMSE (m) | 5.20 | VINS-Fusion | Stereo only (KITTI has no IMU data) | SLAM Comparison, arXiv:2108.01654, Table 8 |
| 00 | ATE position max (m) | 13.80 | VINS-Fusion | Stereo only | SLAM Comparison, arXiv:2108.01654, Table 8 |
| 00 | ATE rotation RMSE (deg) | 3.22 | VINS-Fusion | Stereo only | SLAM Comparison, arXiv:2108.01654, Table 8 |
| 00 | ATE rotation max (deg) | 7.93 | VINS-Fusion | Stereo only | SLAM Comparison, arXiv:2108.01654, Table 8 |
| 00 | RPE translation (m) — avg over seqs 00–10 | 0.076 | VINS-Fusion | Stereo only (kitti_odom config) | Localization Comparison, arXiv:2507.11241, Table V |
| 00 | RPE rotation [-] — avg over seqs 00–10 | 0.013 | VINS-Fusion | Stereo only | Localization Comparison, arXiv:2507.11241, Table V |
| 00 | RTE avg all seqs (%) | 2.64 | VINS-Fusion | Stereo only (KITTI leaderboard) | JCS 2022, doi:10.3844/jcssp.2022.1030.1037, Fig. 7 |
| 00 | RRE avg all seqs (deg/hm) | 1.01 | VINS-Fusion | Stereo only (KITTI leaderboard) | JCS 2022, doi:10.3844/jcssp.2022.1030.1037, Fig. 7 |
| 00 | F-score | No published data found | — | — | — |
| 00 | Velocity error | No published data found | — | — | — |
| 01 | ATE mean (m) | No published data found | — | — | — |
| 01 | ATE RMSE (m) | No published data found | — | — | — |
| 01 | ATE rotation RMSE (deg) | No published data found | — | — | — |
| 01 | RPE translation (m) | No published data found | — | — | — |
| 01 | RPE rotation | No published data found | — | — | — |
| 01 | F-score | No published data found | — | — | — |
| 01 | Velocity error | No published data found | — | — | — |
| 05 | ATE mean (m) | 1.19 | VINS-Fusion | Stereo + IMU + GPS | Lvio-Fusion, arXiv:2106.06783, Table I |
| 05 | ATE RMSE (m) | 1.26 | VINS-Fusion | Stereo + IMU + GPS | Lvio-Fusion, arXiv:2106.06783, Table I |
| 05 | ATE position RMSE (m) | 4.79 | VINS-Fusion | Stereo only | SLAM Comparison, arXiv:2108.01654, Table 8 |
| 05 | ATE position max (m) | 16.78 | VINS-Fusion | Stereo only | SLAM Comparison, arXiv:2108.01654, Table 8 |
| 05 | ATE rotation RMSE (deg) | 3.02 | VINS-Fusion | Stereo only | SLAM Comparison, arXiv:2108.01654, Table 8 |
| 05 | ATE rotation max (deg) | 7.49 | VINS-Fusion | Stereo only | SLAM Comparison, arXiv:2108.01654, Table 8 |
| 05 | t_rel (%) | 11.6 | **VINS-Mono** ⚠️ NOT VINS-Fusion | Monocular + IMU | Causal Transformer, arXiv:2409.08769, Table 1 |
| 05 | r_rel (deg/100m) | 1.26 | **VINS-Mono** ⚠️ NOT VINS-Fusion | Monocular + IMU | Causal Transformer, arXiv:2409.08769, Table 1 |
| 05 | RPE translation (m) — avg over seqs 00–10 | 0.076 | VINS-Fusion | Stereo only | Localization Comparison, arXiv:2507.11241, Table V |
| 05 | F-score | No published data found | — | — | — |
| 05 | Velocity error | No published data found | — | — | — |

---

## Notes on Other Available Sequences

Papers 1 and 5 also report VINS-Fusion data for sequences **02, 06, 07, 09** that are not included above. If needed later:

| Seq | ATE mean (m) | ATE RMSE (m) | Source |
|-----|-------------|-------------|--------|
| 02  | 2.56 | 3.62 | Lvio-Fusion Table I (stereo+IMU+GPS) |
| 06  | 0.97 | 1.03 | Lvio-Fusion Table I (stereo+IMU+GPS) |
| 07  | 0.88 | 0.98 | Lvio-Fusion Table I (stereo+IMU+GPS) |
| 09  | 0.83 | 0.90 | Lvio-Fusion Table I (stereo+IMU+GPS) |
| 02  | 18.17 m RMSE | — | SLAM Comparison Table 8 (stereo only) |
| 06  | 2.50 m RMSE  | — | SLAM Comparison Table 8 (stereo only) |

---

## Paper-by-Paper Analysis

### Paper 1 — Lvio-Fusion (arXiv:2106.06783)
- **Metrics:** ATE mean and ATE RMSE (metres), from Table I
- **Method:** VINS-Fusion (explicitly named as baseline)
- **Sensor config:** Stereo + IMU + **GPS** — the paper explicitly states "VINS-Fusion is a visual-inertial odometer and fuses GPS with local estimations." GPS fusion is active; this is not a pure VIO run.
- **Sequences covered:** 00, 02, 05, 06, 07, 09 — sequence **01 is absent**
- **Per-sequence:** Yes — each row is one sequence

### Paper 2 — Comparison of Localization Algorithms (arXiv:2507.11241)
- **Metrics:** APE (full, translation in metres, rotation unitless) and RPE (full, translation in metres, rotation unitless), from Table V
- **Method:** VINS-Fusion
- **Sensor config:** Stereo only — "the Kitti Odometry dataset has no inertial measurement data available" (noted re: OpenVINS); VINS-Fusion was run with the kitti_odom example config (stereo, no IMU)
- **Sequences covered:** All 11 (00–10) succeeded, but **results are averaged over all sequences** — Table V gives one row per algorithm, not per sequence
- **Per-sequence:** No — Table V is a dataset-level average

### Paper 3 — Causal Transformer for VIO (arXiv:2409.08769)
- **Metrics:** t_rel (%) and r_rel (degrees per 100m), from Table 1
- **Method:** ⚠️ **VINS-Mono** — monocular, not VINS-Fusion (stereo). Flag clearly: different sensor modality and algorithm variant. Only useful as a rough ceiling on monocular performance.
- **Sensor config:** Monocular + IMU
- **Sequences covered:** Test sequences 05, 07, 10 only — sequences 00 and 01 are training sequences and are not reported
- **Per-sequence:** Yes — results shown per test sequence

### Paper 4 — JCS 2022 (doi:10.3844/jcssp.2022.1030.1037)
- **Metrics:** RTE (relative translation error, %) and RRE (relative rotation error, deg/hm), from Figures 4–7 — per-sequence data is in charts, not tables, so exact per-sequence values for seq 00, 01, 05 cannot be extracted numerically
- **Method:** VINS-Fusion (one of 13 VO methods compared; data sourced from KITTI leaderboard)
- **Sensor config:** Stereo only (KITTI leaderboard submission; KITTI has no IMU)
- **Sequences covered:** All 11 sequences, but reported as a single overall average in text (RTE avg = 2.64%, RRE avg = 1.01 deg/hm)
- **Per-sequence:** Only in figures — not extractable as exact numbers for 00, 01, 05

### Paper 5 — Comparison of Modern Open-Source Visual SLAM (arXiv:2108.01654)
- **Metrics:** ATE position RMSE and max (metres), ATE rotation RMSE and max (degrees), from Table 8
- **Method:** VINS-Fusion
- **Sensor config:** Stereo only — "Only the algorithms that could work with stereo data are provided"; KITTI has no IMU data
- **Sequences covered:** 00, 02, 05, 06 — sequence **01 is absent**
- **Per-sequence:** Yes — each column is one sequence
- **Ranking vs. others:** VINS-Fusion performs significantly worse than ORB-SLAM2 and OpenVSLAM. Seq 00: VINS-Fusion 5.20 m vs ORB-SLAM2 0.89 m and OpenVSLAM 0.88 m. Seq 05: VINS-Fusion 4.79 m vs ORB-SLAM2 0.39 m and OpenVSLAM 0.46 m. Paper states: "VINS-Fusion and LDSO both obtain high errors on KITTI sequences, which impugns the usage of these approaches on public roads."
