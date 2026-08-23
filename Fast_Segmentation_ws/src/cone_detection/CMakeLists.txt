cmake_minimum_required(VERSION 3.8)
project(cone_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS
    common
    search
    segmentation)

add_library(adaptive_clusterer
    src/adaptive_clusterer.cpp)

target_compile_features(adaptive_clusterer PUBLIC cxx_std_17)

target_include_directories(adaptive_clusterer
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${PCL_INCLUDE_DIRS})

target_link_libraries(adaptive_clusterer
    PUBLIC 
      ${PCL_LIBRARIES})

target_compile_definitions(adaptive_clusterer
    PRIVATE 
      ${PCL_DEFINITIONS})

install(
  TARGETS adaptive_clusterer
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib 
  LIBRARY DESTINATION lib 
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include 
)

ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(PCL)

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
