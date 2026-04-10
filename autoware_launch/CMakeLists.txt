cmake_minimum_required(VERSION 3.5)
project(autoware_launch)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  PROGRAMS
    script/inject_diag_main.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME inject_diag_main
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)

install(
  PROGRAMS
    script/inject_diag_sub.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME inject_diag_sub
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  rviz
)
