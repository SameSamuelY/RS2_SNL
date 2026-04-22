cmake_minimum_required(VERSION 3.8)
project(ur3_planner)

# Force C++17 (required for MoveIt 2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)


add_executable(ur3_planner src/ur3_planner.cpp)
target_include_directories(ur3_planner PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  ur3_planner
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "geometry_msgs"
  "controller_manager_msgs"
)
install(TARGETS ur3_planner
  DESTINATION lib/${PROJECT_NAME})

  
add_executable(ur3_planner_listener src/ur3_planner_listener.cpp)
target_include_directories(ur3_planner_listener PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  ur3_planner_listener
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "geometry_msgs"
  "controller_manager_msgs"
)
install(TARGETS ur3_planner_listener
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
