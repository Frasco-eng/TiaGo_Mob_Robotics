cmake_minimum_required(VERSION 3.4.0)
project(tiago_exam)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ament_cmake_auto REQUIRED)
# Add these specifically if not already picked up by ament_auto
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)



install(
  PROGRAMS 
    scripts/tuck_arm.py
    scripts/map_saver_watcher.py
    scripts/localization.py
    scripts/marker_searcher.py
    scripts/naive_explorer.py
    scripts/aruco_noter.py
  DESTINATION lib/${PROJECT_NAME}
)


install (DIRECTORY 
  maps
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(sensor_msgs REQUIRED)

endif()

ament_auto_package(INSTALL_TO_SHARE launch)
