cmake_minimum_required(VERSION 3.20)

project(iekf_localizer
  VERSION 1.0.0
  DESCRIPTION "Invariant EKF localizer ROS2 node"
  LANGUAGES CXX)

# Default to a Release build if the user doesn't specify one
if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type specified, defaulting to 'Release'.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# system
find_package(Eigen3 REQUIRED)
find_package(manif REQUIRED)
# local
find_package(kitti_msgs REQUIRED)

add_executable(${PROJECT_NAME}_node
  src/iekf_localizer_node.cpp
  src/iekf_localizer.cpp)

target_include_directories(${PROJECT_NAME}_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${manif_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}_node PRIVATE
  ${rclcpp_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${tf2_ros_TARGETS}
  ${tf2_eigen_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  ${kitti_msgs_TARGETS}
  Eigen3::Eigen
  ${manif_LIBRARIES})

target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)  # Require C++17

target_compile_options(${PROJECT_NAME}_node PRIVATE
  -Wall
  -Wextra
  -Wpedantic)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

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

  # find_package(ament_cmake_gtest REQUIRED)
  # Add ament_add_gtest(...) blocks here once unit tests are written
  # for IEKFLocalizer, mirroring ekf_localizer's test setup.
endif()

ament_package()
