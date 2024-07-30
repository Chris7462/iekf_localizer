cmake_minimum_required(VERSION 3.8)
project(iekf_localizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(manif REQUIRED)

add_executable(iekf_localizer_node
  src/iekf_localizer_node.cpp
  src/iekf_localizer.cpp)

target_include_directories(iekf_localizer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${manif_INCLUDE_DIRS})

ament_target_dependencies(iekf_localizer_node PUBLIC
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs)

target_link_libraries(iekf_localizer_node PUBLIC
  Eigen3::Eigen
  ${manif_LIBRARIES})

target_compile_features(iekf_localizer_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS iekf_localizer_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
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
endif()

ament_package()
