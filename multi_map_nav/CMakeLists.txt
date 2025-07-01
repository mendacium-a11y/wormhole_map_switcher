cmake_minimum_required(VERSION 3.8)
project(multi_map_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------------------
# Dependencies
# ----------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(custom_msg REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(Threads REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# SQLite3
find_library(SQLITE3_LIBRARY sqlite3 REQUIRED)

# ----------------------------------
# Include directories
# ----------------------------------
include_directories(include)


# ----------------------------------
# Executables
# ----------------------------------

# Wormhole Saver Node
add_executable(wormhole_saver
  src/wormhole_saver.cpp
  src/wormhole_db.cpp
  src/pose_saver_db.cpp
)

target_link_libraries(wormhole_saver ${SQLITE3_LIBRARY} Threads::Threads)

ament_target_dependencies(wormhole_saver
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  action_msgs
  custom_msg
)

# Save Pose Node
add_executable(pose_saver
  src/save_pose_node.cpp
  src/pose_saver_db.cpp
)

target_link_libraries(pose_saver ${SQLITE3_LIBRARY} Threads::Threads)

ament_target_dependencies(pose_saver
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  custom_msg
)

# Multi Map Action Server Node
add_executable(multi_map_action_server
  src/multi_map_action_server.cpp
  src/pose_saver_db.cpp
  src/wormhole_db.cpp
)

target_link_libraries(multi_map_action_server ${SQLITE3_LIBRARY} Threads::Threads)

ament_target_dependencies(multi_map_action_server
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  action_msgs
  nav2_msgs
  custom_msg
)

# ----------------------------------
# Installation
# ----------------------------------
install(TARGETS
  wormhole_saver
  pose_saver
  multi_map_action_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# ----------------------------------
# Package exports
# ----------------------------------
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_package()
