cmake_minimum_required(VERSION 3.8)
project(racecar_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY models urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
