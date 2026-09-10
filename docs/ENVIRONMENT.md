# Environment record

## Initial state — 2026-09-10

| Item | Observed value |
|---|---|
| Host | Personal computer; no robot connection |
| OS | Ubuntu 22.04.5 LTS (Jammy) |
| Architecture | x86_64 / amd64 |
| Kernel | 6.8.0-138-generic |
| CPU | Intel Core Ultra 7 155H, 22 logical CPUs |
| Memory | 14 GiB total, about 12 GiB initially available |
| Swap | None |
| Root disk | About 83 GiB initially available |
| Locale | `LANG=en_US.UTF-8`, `LC_ALL=C.UTF-8` |
| Python | 3.10.12 |
| Display | Wayland, `DISPLAY=:0`; RViz rendering not yet verified |
| ROS at start | `/opt/ros` absent |
| Build tools at start | Git, compiler, CMake, colcon, rosdep absent |

The initial dpkg inventory contains 1,498 entries and is stored as
`logs/inventory-initial-20260910T193825Z.tsv` (SHA-256
`47b1d515efcc5b4d8f0c6d01f5bc482a25147ee9cb2f7569df265e53cdc12f93`).
The matching apt source backup is
`logs/apt/sources.list-initial-20260910T193825Z` (SHA-256
`7b0e5f1fec6dd91161344296b15c56c3ca06f52ce46c4efbcd9c3e7aebfb5134`).

Jammy main, restricted, universe, multiverse, and security were enabled.
Jammy updates entries were commented out, and no ROS repository was present.

## Installed state

Pending `install_dependencies.sh`. That script generates
`docs/INSTALLED_VERSIONS.md` and records the official
`ros2-apt-source` package metadata, apt simulation, complete installation log,
before/after inventories, and installed versions. Append the observed ROS,
GTSAM, PCL, Eigen, Boost, compiler, CMake, colcon, rosdep, and RViz versions
after it completes.
