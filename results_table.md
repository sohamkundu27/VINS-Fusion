## VINS-Fusion KITTI Evaluation Results

| Sequence | Type | ATE median (m) | ATE RMSE (m) | RPE median (m) | RPE RMSE (m) | Attitude median (deg) | Attitude RMSE (deg) | F-score | Vel error mean (m/s) | Vel error std (m/s) | Vel error RMSE (m/s) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 00 | City | 10.8853 | 13.6065 | 0.7746 | 1.0369 | 2.5457 | 2.6723 | 0.0000 | 0.3276 | 0.2078 | 0.3879 |
| 01 | Highway | 37.1459 | 37.7043 | 8.8928 | 20.4637 | 2.7760 | 3.6471 | 0.0000 | 2.5143 | 5.9722 | 6.4799 |
| 05 | Residential | 4.3262 | 5.9367 | 0.9464 | 1.0799 | 2.1399 | 2.8738 | 0.0304 | 0.3348 | 0.2131 | 0.3968 |

*ATE/RPE alignment: SE(3) Umeyama (no scale correction)*
*RPE delta: 100 frames*
*F-score threshold: 2.0 m*

---

## Published VINS-Fusion Reference Values (Lvio-Fusion paper, arXiv:2106.06783)

| Sequence | ATE mean (m) | ATE RMSE (m) |
|----------|------|------|
| 00 | 1.18 | 1.24 |
| 05 | 1.19 | 1.26 |
| 01 | N/A (not reported in this paper) | N/A |
