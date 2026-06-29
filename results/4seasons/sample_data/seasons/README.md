# Frames per season

Ten left-camera (cam0) undistorted frames from each of four 4Seasons recordings — one recording per
season — to show the dataset's cross-season coverage. Each season has its own subfolder; frames are
evenly spaced through the drive and named `NN_<timestamp_ns>.png`.

| Folder | Recording | Season | Notes |
|--------|-----------|--------|-------|
| `spring/` | recording_2020-04-07_10-20-32 | Spring (April) | bare early-spring trees |
| `summer/` | recording_2020-06-12_10-10-57 | Summer (June) | full foliage, strong sun/shadows |
| `autumn/` | recording_2020-10-08_09-57-28 | Autumn (October) | the office-loop sequence used in the eval |
| `winter/` | recording_2021-02-25_13-51-57 | Winter (February) | bare trees, low sun (the dataset tags this run "snow", but the sampled frames are clear, dry cold days — no snow on the ground) |

Spring and summer frames were pulled directly from the remote dataset zips via HTTP range requests
(`remotezip`) without downloading the full multi-GB sequences; autumn and winter come from the
locally-processed sequences.
