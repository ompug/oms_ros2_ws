# Troubleshooting

If setup reports that sudo authentication is required, run the installer from
an interactive terminal. Review the apt simulation in `logs/apt/`; the script
does not proceed if core desktop packages would be removed.

If the build cannot find GTSAM, verify `apt-cache policy ros-humble-gtsam` and
that `/opt/ros/humble/setup.bash` was sourced. Preserve the exact package and
CMake error before considering the documented source fallback.

If no scans arrive, compare `points_topic` with the publisher and inspect
`ros2 topic info --verbose /velodyne_points`. The input requires float32 `x`,
`y`, `z`, and `intensity`; a frame name does not transform coordinates.

If scans are rejected as sparse, confirm one complete rotating VLP-16-like scan
per message, ordered around the sweep. Nonuniform vertical channels and
solid-state patterns need a different projection model.

If RViz has no data, start RViz/subscribers before bag playback because several
debug clouds publish conditionally. Confirm the fixed frame is `map` and the
reference static transforms are enabled for desktop data.

If shutdown exceeds ten seconds, save the launch log and check for project
processes before retrying. `test.sh` signals only the process group it starts.

