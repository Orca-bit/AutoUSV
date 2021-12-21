cmake_minimum_required(VERSION 3.5)
project(vehicle_interface)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Library
ament_auto_add_library(${PROJECT_NAME} SHARED
        src/platform_interface.cpp
        src/vehicle_interface_node.cpp)
usv_set_compile_options(${PROJECT_NAME})
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-conversion)

set(NODE_NAME vehicle_interface_exe)
ament_auto_add_executable(${NODE_NAME}
  src/vehicle_interface_main.cpp)

if (BUILD_TESTING)
    # Lint
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    # Test
endif ()

ament_auto_package(INSTALL_TO_SHARE param launch)
