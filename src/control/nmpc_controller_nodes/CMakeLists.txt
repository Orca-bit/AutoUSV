cmake_minimum_required(VERSION 3.6)

project(nmpc_controller_nodes)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

### Build
ament_auto_add_library(nmpc_controller_node SHARED
  src/nmpc_controller_nodes/nmpc_controller_node.cpp)

usv_set_compile_options(nmpc_controller_node)
target_include_directories(nmpc_controller_node SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})

set(NODE_NAME nmpc_controller_node_exe)
ament_auto_add_executable(${NODE_NAME}
  src/nmpc_controller_nodes/nmpc_controller_main.cpp)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Linters
  ament_lint_auto_find_test_dependencies()

  find_package(autoware_testing REQUIRED)
  add_smoke_test(${PROJECT_NAME} ${NODE_NAME})

  # Unit test
  ament_add_gtest(nmpc_controller_nodes_unit_tests
    test/gtest_main.cpp
    test/sanity_check.cpp)
  usv_set_compile_options(nmpc_controller_nodes_unit_tests)
  target_link_libraries(nmpc_controller_nodes_unit_tests nmpc_controller_node)
  add_dependencies(nmpc_controller_nodes_unit_tests nmpc_controller_node)
endif()

ament_auto_package(INSTALL_TO_SHARE param)
