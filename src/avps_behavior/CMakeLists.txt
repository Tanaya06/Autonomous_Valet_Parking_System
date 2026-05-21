cmake_minimum_required(VERSION 3.8)
project(avps_behavior)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# Jazzy: behaviortree_cpp (v4), not behaviortree_cpp_v3
find_package(behaviortree_cpp REQUIRED)

add_executable(behavior_tree_node
  src/bt_nodes/navigate_to_zone.cpp
  src/behavior_tree_node.cpp
)

target_include_directories(behavior_tree_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(behavior_tree_node
  rclcpp rclcpp_action geometry_msgs
  nav2_msgs tf2_geometry_msgs behaviortree_cpp
)

install(TARGETS behavior_tree_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch behavior_trees
  DESTINATION share/${PROJECT_NAME} OPTIONAL
)

ament_package()

# cmake_minimum_required(VERSION 3.8)
# project(avps_behavior)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(nav2_msgs REQUIRED)
# find_package(behaviortree_cpp REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()
