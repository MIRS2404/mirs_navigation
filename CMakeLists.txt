cmake_minimum_required(VERSION 3.8)
project(mirs_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  params
  rviz
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()