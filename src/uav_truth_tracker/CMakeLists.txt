cmake_minimum_required(VERSION 3.0.2)
project(uav_truth_tracker)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2_ros
  tf2_sensor_msgs
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/analyze_prediction_log.py
  scripts/capture_decision_node.py
  scripts/camera_tf_broadcaster.py
  scripts/chase_evaluator_node.py
  scripts/check_lidar_detection_alignment.py
  scripts/collect_uav_yolo_dataset_node.py
  scripts/filter_pcd_ground.py
  scripts/forest_world_markers.py
  scripts/gazebo_odom_bridge.py
  scripts/gazebo_uav_marker.py
  scripts/mavros_hover_node.py
  scripts/mavros_tf_bridge.py
  scripts/pcd_pointcloud_publisher.py
  scripts/pointcloud_to_pointcloud2_relay.py
  scripts/static_uav_marker.py
  scripts/summarize_prediction_results.py
  scripts/target_depth_detector_node.py
  scripts/target_lidar_detector_node.py
  scripts/target_reacquire_manager_node.py
  scripts/target_sensor_fusion_node.py
  scripts/target_state_estimator_node.py
  scripts/target_truth_compare_node.py
  scripts/target_yolo_detector_node.py
  scripts/train_uav_yolo.py
  scripts/truth_tracker_node.py
  scripts/uav0_goal_follower_node.py
  scripts/uav0_velocity_guidance_node.py
  scripts/ugv_robot_model_helper.py
  scripts/uav_pointcloud_map_recorder.py
  scripts/uav_pointcloud_to_world.py
  scripts/uav_elevation_cloud_projector.py
  scripts/uav1_motion_node.py
  scripts/uav_model_markers.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/collect_uav_yolo_multiworld.sh
  scripts/run_kalman_tuning_sweep.sh
  scripts/run_estimator_model_tests.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
