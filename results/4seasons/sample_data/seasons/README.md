# One frame per season

Single left-camera (cam0) undistorted frame from four different 4Seasons recordings, one per
season — the whole point of the dataset is the *same kind of roads across seasons/weather*.

| File | Recording | Season | Notes |
|------|-----------|--------|-------|
| `spring_2020-04-07_*.png` | recording_2020-04-07_10-20-32 | Spring (April) | |
| `summer_2020-06-12_*.png` | recording_2020-06-12_10-10-57 | Summer (June) | full foliage, strong sun/shadows |
| `autumn_2020-10-08_*.png` | recording_2020-10-08_09-57-28 | Autumn (October) | the office-loop sequence used in the eval |
| `winter_2021-02-25_*.png` | recording_2021-02-25_13-51-57 | Winter (February) | bare trees, low sun (the dataset tags this run "snow", but the sampled frame is a clear, dry cold day — no snow on the ground) |

Spring and summer frames were pulled directly from the remote dataset zips via HTTP range requests
(`remotezip`) without downloading the full multi-GB sequences; autumn and winter come from the
locally-processed sequences. Filenames keep the original nanosecond capture timestamp.
