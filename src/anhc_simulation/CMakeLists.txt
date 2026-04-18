cmake_minimum_required(VERSION 3.8)
project(anhc_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_sim REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)
install(DIRECTORY maps/ DESTINATION share/${PROJECT_NAME}/maps)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(
  PROGRAMS
  scripts/anhc_scan_frame_relay.py
  scripts/anhc_odom_tf_broadcaster.py
  scripts/anhc_odom_tf_node.py
  scripts/anhc_cmd_vel_idle_gate.py
  scripts/anhc_gz_frame_static_tf.py
  scripts/anhc_initialpose_odom_stamp.py
  scripts/anhc_initialpose_to_gz.py
  scripts/anhc_joint_state_stamp_guard.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
