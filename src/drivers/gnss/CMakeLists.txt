cmake_minimum_required(VERSION 3.6)
project(gnss)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_auto_add_library(Convert SHARED
        src/geo_pos_conv.cpp)
ament_auto_add_library(${PROJECT_NAME} SHARED
        src/gnss_interface.cpp
        src/gnss_interface_node.cpp)
target_link_libraries(${PROJECT_NAME} Convert)
usv_set_compile_options(${PROJECT_NAME})

set(NODE_NAME gnss_interface_exe)
ament_auto_add_executable(${NODE_NAME}
        src/gnss_interface_main.cpp)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()

    ament_add_gtest(gnss_unit_tests
            test/gtest_main.cpp
            test/llh_conv_test.cpp)
    usv_set_compile_options(gnss_unit_tests)
    target_link_libraries(gnss_unit_tests Convert)
    add_dependencies(gnss_unit_tests Convert)
endif ()

ament_auto_package(INSTALL_TO_SHARE param launch)
