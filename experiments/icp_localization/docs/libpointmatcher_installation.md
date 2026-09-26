# libpointmatcher installation and verification

The official project is [norlab-ulaval/libpointmatcher](https://github.com/norlab-ulaval/libpointmatcher).
Its [release page](https://github.com/norlab-ulaval/libpointmatcher/releases)
lists 1.4.4 as the latest stable release at inspection time, and its
[compilation guide](https://libpointmatcher.readthedocs.io/en/stable/Compilation/)
lists Ubuntu 22.04 x86_64 among supported platforms. We used tagged releases
in `third_party/`, outside `src/`, with a workspace-local install prefix.

- libpointmatcher tag 1.4.4, commit `3eb448330b2c35f7718e690e66f0c1057115e877`.
- libnabo tag 1.1.2, commit `cd95c8b95e8620e6c4ba722ed0f56974b4740873`.
- GCC/G++ 11.4.0, CMake 3.22.1, Eigen 3.4.0, Boost 1.74, yaml-cpp 0.7.
- Release build, examples and unit tests enabled for libpointmatcher; local
  prefix `/home/ompug/oms_ros2_ws/third_party/install`.

The installed `third_party/install/bin/pmicp` is the upstream command-line
tool. The official unit suite completed with **92/92 tests passing**. The
experiment C++ program links against this installation via CMake and uses
the official ICP C++ API. Python bindings were not required.

Rebuild without touching system install paths:

```bash
git clone --branch 1.1.2 --depth 1 --recurse-submodules https://github.com/norlab-ulaval/libnabo.git third_party/libnabo
git clone --branch 1.4.4 --depth 1 --recurse-submodules https://github.com/norlab-ulaval/libpointmatcher.git third_party/libpointmatcher
cmake -S third_party/libnabo -B third_party/build/libnabo -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PWD/third_party/install"
cmake --build third_party/build/libnabo -j 4
cmake --install third_party/build/libnabo
cmake -S third_party/libpointmatcher -B third_party/build/libpointmatcher -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PWD/third_party/install" -DCMAKE_PREFIX_PATH="$PWD/third_party/install" -DBUILD_TESTS=ON -DBUILD_EXAMPLES=ON
cmake --build third_party/build/libpointmatcher -j 4
cmake --install third_party/build/libpointmatcher
third_party/build/libpointmatcher/utest/utest --path third_party/libpointmatcher/examples/data/
source experiments/icp_localization/scripts/setup_libpointmatcher.sh
cmake -S experiments/icp_localization/src -B experiments/icp_localization/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$PWD/third_party/install"
cmake --build experiments/icp_localization/build -j 2
```

The 1.4.4 loader does **not** implement `DATA binary` PCD input, although
the format is common in ROS/PCL. The first smoke test exposed this; the
experiment's `src/icp_localize.cpp` has a small validated float32 binary PCD
reader for the generated map and scans. The frozen PCD files remain standard
binary PCDs and are never rewritten to work around the upstream loader.
