# Dataset storage

`raw/` holds the ignored ROS 1 Jackal VLP-16 bag. `converted/` holds the
ignored ROS 2 Humble SQLite3 conversion containing only `/velodyne_points`.

Run `scripts/fetch_dataset.sh`, then `scripts/convert_dataset.sh`. The fetch
helper records filename, Google Drive file ID, byte size, and SHA-256 beside
this file. The conversion excludes recorded TF, odometry, and IMU so they
cannot conflict with the local reference frame chain.

