# Scout Mini porting checklist

Do not enable the desktop reference transforms on the robot until this
checklist is complete.

## Platform inventory

- Onboard CPU architecture: **TBD on robot**
- OS and version: **TBD on robot**
- ROS distribution: **TBD on robot**
- Available RAM, swap, and storage: **TBD on robot**
- Scout ROS 2 driver repository and exact revision: **TBD on robot**
- Verified `/cmd_vel` message type and topic: **TBD on robot**
- Wheel-odometry topic, frame IDs, covariance, and TF owner: **TBD on robot**
- URDF and `robot_state_publisher` launch owner: **TBD on robot**

## LiDAR contract

- Manufacturer and model: **TBD on robot**
- Driver and exact revision: **TBD on robot**
- Channels, scan rate, vertical angles, return mode: **TBD on robot**
- PointCloud2 topic, frame, fields, ordering, and QoS: **TBD on robot**
- Timestamp source and synchronization: **TBD on robot**
- Measured LiDAR-to-base translation and rotation: **TBD on robot**
- Mounting height and angle: **TBD on robot**

| Future interface | Planned mapping |
|---|---|
| Verified Scout LiDAR topic | Pass as `points_topic`; value TBD |
| Verified LiDAR frame and extrinsics | Replace desktop static assumptions |
| `/integrated_to_init` | Downstream consumer and frame conversion TBD |
| `/laser_cloud_surround` | Visualization; navigation representation TBD |
| Scout wheel odometry | Validation or fusion design TBD |

Inspect TF publishers before launch and assign exactly one owner to each child
frame. Evaluate a `map -> odom -> base_link -> lidar_frame` chain after the
robot’s existing URDF and driver behavior are known. Wheel odometry fusion is
outside this baseline and must not be added by remapping alone.

## Migration procedure

1. Transfer tracked files only; exclude generated overlays, logs, caches,
   virtual environments, and datasets unless the bag is needed for a robot-side
   performance test.
2. Run the dependency installer only if the robot is verified as Jammy amd64;
   otherwise adapt package installation deliberately and record the platform.
3. Build natively on the robot with `CMAKE_BUILD_PARALLEL_LEVEL=1` initially.
4. Run `test.sh` disconnected from the motor controller and with localhost ROS
   discovery.
5. Inspect the real LiDAR message and TF graph before changing configuration.
6. Launch with `publish_reference_tf:=false`, measured extrinsics, and the
   verified LiDAR topic. Do not publish velocity commands from this workspace.

