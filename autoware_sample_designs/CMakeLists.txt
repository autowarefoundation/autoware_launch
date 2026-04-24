cmake_minimum_required(VERSION 3.14)
project(autoware_sample_designs)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  urdf
)

add_custom_target(${PROJECT_NAME})

autoware_system_designer_build_deploy(${PROJECT_NAME} AutowareSample.system)
