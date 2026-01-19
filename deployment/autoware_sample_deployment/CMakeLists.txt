cmake_minimum_required(VERSION 3.14)
project(autoware_sample_deployment)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  urdf
)

# Add a dummy library to make a target
add_library(${PROJECT_NAME} SHARED src/empty.cpp)

autoware_system_designer_build_deploy(${PROJECT_NAME} AutowareSample.system)
