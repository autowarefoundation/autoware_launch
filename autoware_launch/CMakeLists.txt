cmake_minimum_required(VERSION 3.5)
project(autoware_launch)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # static centerline optimizer
  add_launch_test(
    test/static_centerline_optimizer/test_static_centerline_optimizer.test.py
    TIMEOUT "30"
  )
  install(DIRECTORY
    test/static_centerline_optimizer/data/
    DESTINATION share/${PROJECT_NAME}/test/static_centerline_optimizer/data/
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  rviz
)
