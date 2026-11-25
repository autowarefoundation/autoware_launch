cmake_minimum_required(VERSION 3.14)
project(autoware_sample_designs)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  urdf
)

add_custom_target(${PROJECT_NAME})

# Crop boxes of the vehicle body and the tire steering envelope.
autoware_system_designer_generate_param_files(crop_box
  SCRIPT scripts/generate_crop_box_params.py
  ARGS ${CMAKE_CURRENT_SOURCE_DIR}/config/sample_vehicle/vehicle_info.param.yaml config/sample_vehicle
  OUTPUTS config/sample_vehicle/crop_box_self.param.yaml config/sample_vehicle/crop_box_wheels.param.yaml
  DEPENDS config/sample_vehicle/vehicle_info.param.yaml
)

# Robot description of the sample vehicle with the sample sensor kit calibration.
autoware_system_designer_generate_robot_description(
  XACRO urdf/vehicle.xacro
  CONFIG_DIRS config/sample_vehicle
)

autoware_system_designer_build_deploy(${PROJECT_NAME} AutowareSample.system)
