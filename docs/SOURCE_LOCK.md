# Source and dependency lock

## LeGO-LOAM

- Upstream: `https://github.com/fishros/LeGO-LOAM-ROS2`
- Commit: `088856332b71e4bc4cc0f08cd8282ac370ca444a`
- Import archive SHA-256: `38094bd6d24b28082ede8d228998554196261e74d0879eb0d9d9e084014dbf03`
- Import: download the GitHub commit archive, verify the SHA-256, and extract
  its top-level contents into `src/`.
- License: upstream BSD license is preserved as `src/LICENSE`.

## Dependencies

System packages are resolved from Ubuntu Jammy and the official ROS 2 apt
repository by `install_dependencies.sh`. The selected package versions and
the before/after package inventory are written under `logs/` during install.
Dataset tooling uses `rosbags==0.11.5`; the fully resolved Python environment
is recorded in `docs/python-requirements.lock` after installation.

