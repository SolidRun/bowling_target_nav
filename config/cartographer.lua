-- Cartographer Configuration for RZ/V2N
-- 2D LiDAR + wheel odometry SLAM
--
-- TF tree:  map -> odom -> base_link -> laser
--   cartographer publishes: map -> odom (localization correction)
--   odometry_node publishes: odom -> base_link (wheel odometry)
--   robot_state_publisher:   base_link -> laser (from URDF)
--
-- IMPORTANT: Global SLAM (loop closure) is disabled to prevent
-- abseil mutex deadlock on aarch64 (V2N board). This means:
--   - Only local submap optimization occurs
--   - Map drift accumulates over long sessions
--   - Restart mapping for fresh maps if drift is noticeable

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  -- TF tree: map -> odom -> base_link -> laser
  -- cartographer publishes: map -> odom
  -- odometry_node publishes: odom -> base_link
  -- robot_state_publisher: base_link -> laser (from URDF)
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 1

TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.02

-- Submaps config (lower = faster map updates)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- Disable global SLAM completely to avoid mutex deadlock on V2N
-- (optimize_every_n_nodes = 0 disables optimizer, but constraint builder
--  still runs and causes deadlock with main thread on aarch64)
POSE_GRAPH.optimize_every_n_nodes = 0
POSE_GRAPH.constraint_builder.sampling_ratio = 0.0
POSE_GRAPH.global_sampling_ratio = 0.0
POSE_GRAPH.constraint_builder.min_score = 0.99
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.99
POSE_GRAPH.max_num_final_iterations = 1

return options
