cmake_minimum_required(VERSION 3.8)
project(ur3_mtc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(controller_manager_msgs REQUIRED)


add_executable(mtc_node src/mtc_node.cpp)
target_include_directories(mtc_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtc_node
  moveit_task_constructor_core
  rclcpp
)
install(TARGETS mtc_node
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(mtc_pick_place src/mtc_pick_place.cpp)
target_include_directories(mtc_pick_place PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtc_pick_place
  rclcpp
  moveit_msgs
  moveit_ros_planning_interface
  moveit_task_constructor_core
  shape_msgs
)
install(TARGETS mtc_pick_place
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(mtc_pick_place_kinematics src/mtc_pick_place_kinematics.cpp)
target_include_directories(mtc_pick_place_kinematics PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtc_pick_place_kinematics
  rclcpp
  moveit_msgs
  moveit_ros_planning_interface
  moveit_task_constructor_core
  geometry_msgs
  shape_msgs
)
install(TARGETS mtc_pick_place_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(mtc_pick_place_listener src/mtc_pick_place_listener.cpp)
target_include_directories(mtc_pick_place_listener PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtc_pick_place_listener
  rclcpp
  moveit_msgs
  moveit_ros_planning_interface
  moveit_task_constructor_core
  geometry_msgs
  shape_msgs
  std_msgs
  nlohmann_json
  controller_manager_msgs
)
install(TARGETS mtc_pick_place_listener
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS 
  scripts/set_kinematics.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

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
