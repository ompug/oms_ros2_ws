# Verification record

## Automated local suite

Run `./test.sh`. It creates a timestamped evidence directory under `logs/`
with a machine-readable `result.json`, command logs, endpoint details, and
per-QoS runtime measurements.

The suite checks:

- ROS discovery and a C++ talker to Python listener exchange.
- Package tests, including VLP-16 row projection and symmetric ground slopes.
- Discovery and parameters of all four composed nodes.
- Ctrl-C shutdown before data and after synthetic playback, bounded to 10 s.
- Empty, all-nonfinite, and missing-intensity input rejection.
- Best-effort and reliable PointCloud2 publishers against sensor-data QoS.
- Nonempty ground, segmented, and feature output plus advancing, finite,
  normalized odometry.
- A single parent per observed TF child and the reference static relationships.
- Immediate rejection of invalid scan dimensions.

Synthetic scans model a flat ground plane and sector-based vertical surfaces.
They establish deterministic projection and message plumbing; they do not
establish trajectory accuracy or recorded-data SLAM quality.

## Current results — 2026-09-10

| Check | Status | Evidence |
|---|---|---|
| Initial machine inventory | passed | `logs/inventory-initial-20260910T193825Z.tsv` |
| Pinned upstream import | passed | local commits and `docs/SOURCE_LOCK.md` |
| Shell/Python/XML static syntax | passed | Local syntax checks |
| ROS installation and package build | passed | `logs/build-20260910T201931Z.log` |
| Rosdep dependency resolution | passed | `logs/rosdep-update-final-20260910.log` and `logs/rosdep-install-final-20260910.log` |
| Clean rebuild | passed | Final portable warning-free build: `logs/build-20260910T213023Z.log` |
| Automated synthetic runtime suite | passed | Initial `logs/test-20260910T201952Z/result.json`; final post-clean-build `logs/test-20260910T213124Z/result.json` |
| Recorded Jackal segment, run 1 | blocked | Official Google Drive quota; see `docs/DATASET.md` |
| Recorded Jackal segment, run 2 | blocked | Run 1 data unavailable |
| RViz OpenGL initialization and topic subscription | passed | `logs/rviz-20260910T202149Z/` |
| RViz screenshot inspection | blocked | GNOME denied D-Bus capture; Qt capture was black under Wayland |
| Loop-closure behavior | not tested | Requires adequate recorded trajectory |
| Trajectory accuracy | not tested | Dataset has no independent ground truth in this workflow |
| Robot integration | blocked | Robot inspection is explicitly outside the desktop phase |

Recorded-data acceptance requires advancing input timestamps, nonempty feature
and map clouds, multiple keyframes, finite poses, normalized quaternions, and a
trajectory visually plausible for the scene. Message existence alone is not a
pass.
