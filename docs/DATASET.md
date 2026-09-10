# Recorded-data status

The selected source is the official
`RobustFieldAutonomyLab/jackal_dataset_20170608` Google Drive folder. Its first
segment has these published properties:

```text
filename: 2017-06-08-15-49-45_0.bag
Google Drive file ID: 1lPC1o5c58bP08gD6rVWefEAMzAF_aCNf
listed size: 831281741 bytes
expected LiDAR topic: /velodyne_points
```

Acquisition attempts for segment 0 and segment 1 on 2026-09-10 both failed
with Google Drive’s server response: “Too many users have viewed or downloaded
this file recently.” A web search found no trustworthy mirror. The failure is
external and occurs before any bytes are supplied, so size, SHA-256, connection
metadata, fields, timestamps, conversion, and full-segment playback remain
**blocked**.

`scripts/fetch_dataset.sh [0|1|2|3]` records size and SHA-256 after a successful
download. `scripts/convert_dataset.sh [0|1|2|3]` converts only
`/velodyne_points` to SQLite3 metadata version 5 using the Humble typestore,
then verifies the result with `ros2 bag info`.

When Drive permits access, run:

```bash
./scripts/fetch_dataset.sh 0
./scripts/convert_dataset.sh 0
```

Recorded runtime, loop closure, and trajectory plausibility cannot be inferred
from the passing synthetic suite.

