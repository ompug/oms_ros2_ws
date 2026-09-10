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

## Installed state — 2026-09-10

`install_dependencies.sh` installed the official ROS 2 Humble desktop package,
the native build toolchain, PCL, Eigen, Boost, and the official Humble GTSAM
4.2 package. The exact versions are in `docs/INSTALLED_VERSIONS.md`.

The post-install dpkg inventory is
`logs/inventory-after-20260910T202635Z.tsv` (SHA-256
`9428143d146d308c202ba6a9944679ca7445daafdfefa78671e0dc37b5330f42`).
The newly installed package comparison is
`logs/inventory-installed-20260910T202635Z.tsv` (SHA-256
`09249e63dd4751a6057be6435974b913274f2918633683d658306689583b649a`).

ROS and the native dependencies are installed and tested. Rosdep was
initialized system-wide, and the Humble index was populated beneath the
project-local `.ros/` cache. `rosdep install` reports that all required rosdeps
are installed.

RViz created an OpenGL render window and subscribed to the configured point
cloud topics under the Wayland session. Desktop screenshot capture could not be
used for visual inspection because GNOME denied D-Bus capture and Qt returned a
black Wayland image; see `docs/TESTING.md`.
