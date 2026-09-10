# Parameter reference

| Parameter | Default | Units | Consumer |
|---|---:|---|---|
| `laser.num_vertical_scans` | 16 | channels | projection, association |
| `laser.num_horizontal_scans` | 1800 | bins/revolution | projection, association |
| `laser.vertical_angle_bottom` | -15 | degrees | projection |
| `laser.vertical_angle_top` | 15 | degrees | projection |
| `laser.sensor_mount_angle` | 0 | degrees | ground classification |
| `laser.ground_scan_index` | 7 | zero-based ring | projection |
| `laser.scan_period` | 0.1 | seconds | association |
| `image_projection.segment_theta` | 60 | degrees | segmentation |
| `image_projection.segment_valid_point_num` | 5 | points | segmentation |
| `image_projection.segment_valid_line_num` | 3 | rings | segmentation |
| `featureAssociation.edge_threshold` | 0.1 | curvature | feature extraction |
| `featureAssociation.surf_threshold` | 0.1 | curvature | feature extraction |
| `featureAssociation.nearest_feature_search_distance` | 5 | metres | odometry |
| `mapping.mapping_frequency_divider` | 5 | scans | association to mapping |
| `mapping.enable_loop_closure` | true | boolean | mapping |
| `mapping.surrounding_keyframe_search_radius` | 50 | metres | mapping |
| `mapping.surrounding_keyframe_search_num` | 50 | keyframes | mapping |
| `mapping.history_keyframe_search_radius` | 7 | metres | loop closure |
| `mapping.history_keyframe_search_num` | 25 | keyframes each side | loop closure |
| `mapping.history_keyframe_fitness_score` | 0.3 | ICP score | loop closure |
| `mapping.global_map_visualization_search_radius` | 500 | metres | map display |

For the VLP-16 defaults, horizontal resolution is 0.2 degrees and vertical
resolution is 2 degrees. The implementation also preserves upstream fixed
voxel sizes (0.2 m corners; 0.4 m surfaces, outliers, history, and global map),
1 m key-pose voxels, and its fixed keyframe/optimization thresholds.

Changing only channel count and top/bottom FOV is insufficient for LiDARs with
nonuniform vertical angles. This port does not consume `ring` or per-point time.

