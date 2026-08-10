cmake_minimum_required(VERSION 3.8)
project(fast_ground_segmenter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)


add_executable(ground_segmenter_node
  src/main.cpp
  src/ground_segmenter_node.cpp
  src/point_cloud_filter.cpp
  src/polar_grid.cpp
  src/ground_line_fitter.cpp)

target_compile_features(ground_segmenter_node PRIVATE cxx_std_17)

target_include_directories(ground_segmenter_node PRIVATE 
 include
 ${PCL_INCLUDE_DIRS})

target_link_libraries(ground_segmenter_node
  ${PCL_LIBRARIES})

target_compile_definitions(ground_segmenter_node PRIVATE 
    ${PCL_DEFINITIONS})

ament_target_dependencies(ground_segmenter_node
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions)


install(TARGETS 
  ground_segmenter_node
  DESTINATION lib/${PROJECT_NAME})

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
