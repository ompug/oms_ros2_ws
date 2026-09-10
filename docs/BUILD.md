# Build and dependency procedure

`install_dependencies.sh` is the only system setup entry point. It validates
Jammy amd64, saves apt configuration and package inventories, restores
`jammy-updates`, installs the official signed `ros2-apt-source` package, and
simulates the full transaction before installing. It aborts if apt proposes
removing desktop, systemd, udev, or NetworkManager packages.

The preferred graph-optimization dependency is `ros-humble-gtsam`. No PPA or
source fallback is used unless that official package fails demonstrably.
Rosdep operates with its user cache under `.ros/`.

`build.sh` clears overlay variables in a subshell, sources only the Humble
underlay, limits CMake to two build jobs, runs packages sequentially, and saves
the complete output under `logs/`. `clean_build.sh` accepts only the canonical
workspace path, rejects symlinked generated directories, removes exactly
`build/`, `install/`, and `log/`, then invokes the standard build.

```bash
./build.sh
./clean_build.sh
```

Build status: **passed** on the desktop host. Both packages completed a clean
Release build, the `lego_loam_sr` executable is discoverable, and `ldd` reports
no missing libraries. It links against the official Humble GTSAM 4.2 package;
the source fallback was not needed. The final warning-free clean build is
`logs/build-20260910T203151Z.log`; executable discovery and dynamic-link
verification are in `logs/build-verification-final-20260910.txt`.
