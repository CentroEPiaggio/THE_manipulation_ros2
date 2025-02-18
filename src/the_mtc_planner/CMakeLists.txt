cmake_minimum_required(VERSION 3.8)
project(the_mtc_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(the_interfaces REQUIRED)
find_package(moveit_ros REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  rclcpp
  tf2_eigen
  sensor_msgs
  std_srvs
  tf2_geometry_msgs
  the_interfaces
  moveit_ros
  moveit_visual_tools
  Eigen3
  
)
add_library(${PROJECT_NAME}_extr_calibration SHARED src/calib_task_planner.cpp)

target_include_directories(${PROJECT_NAME}_extr_calibration PUBLIC

$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

$<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}_extr_calibration ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(extr_calibration_node src/calib_task_planner_node.cpp)
ament_target_dependencies(extr_calibration_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
#ament_target_dependencies(extr_calib_mgi_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(extr_calibration_node ${PROJECT_NAME}_extr_calibration)
#target_link_libraries(extr_calib_mgi_node ${PROJECT_NAME}_extr_calibration)

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

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_extr_calibration
  EXPORT export_${PROJECT_NAME}_extr_calibration
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}_extr_calibration
)

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  
)


install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS extr_calibration_node 
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
