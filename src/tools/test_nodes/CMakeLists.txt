cmake_minimum_required(VERSION 3.5)
project(test_nodes)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(NODE_NAME1 motor_report_test_exe)
ament_auto_add_executable(${NODE_NAME1}
  src/motor_report_test.cpp)

set(NODE_NAME2 gnss_test_exe)
ament_auto_add_executable(${NODE_NAME2}
        src/gnss_test.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
