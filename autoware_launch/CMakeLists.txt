cmake_minimum_required(VERSION 3.5)
project(autoware_launch)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  rviz
)

# Copy the script to the build directory and ensure it is executable
add_custom_target(copy_qt5ctsetup ALL
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/rviz/qt5ctsetup.sh ${CMAKE_CURRENT_BINARY_DIR}/rviz/qt5ctsetup.sh
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/rviz
  COMMAND chmod +x ${CMAKE_CURRENT_BINARY_DIR}/rviz/qt5ctsetup.sh
)

# Create a dummy target
add_custom_target(dummy_target ALL
  COMMAND ${CMAKE_COMMAND} -E echo "Running autoware build dummy target"
)

# Custom target to run the script after build
add_custom_target(post_build ALL
  COMMAND ${CMAKE_COMMAND} -E echo "Running post-build script"
  COMMAND ${CMAKE_COMMAND} -E env bash ${CMAKE_CURRENT_BINARY_DIR}/rviz/qt5ctsetup.sh
  DEPENDS dummy_target copy_qt5ctsetup
)

# Ensure post_build target runs after dummy_target and copy_qt5ctsetup
add_dependencies(post_build dummy_target)
add_dependencies(post_build copy_qt5ctsetup)
