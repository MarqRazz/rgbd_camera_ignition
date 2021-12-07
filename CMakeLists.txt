cmake_minimum_required(VERSION 3.5)

project(rgbd_camera_ignition)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY
    rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(
  DIRECTORY
    urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(
  DIRECTORY
        worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

ament_package()
