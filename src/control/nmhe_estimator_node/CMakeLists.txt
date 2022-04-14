cmake_minimum_required(VERSION 3.5)
project(nmhe_estimator_node)

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

set(acados_include $ENV{ACADOS_SOURCE_DIR}/include)
# set(acados_lib $ENV{ACADOS_SOURCE_DIR}/lib)
set(acados_c_code_gen ${nmhe_estimator_DIR}/../../c_generated_code)

### Build
ament_auto_add_library(nmhe_estimator_node SHARED
        src/nmhe_estimator_node.cpp)
# usv_set_compile_options(nmhe_estimator_node)
target_include_directories(nmhe_estimator_node SYSTEM PUBLIC)
target_include_directories(nmhe_estimator_node PUBLIC
        ${acados_c_code_gen}
        ${acados_include}
        ${acados_include}/blasfeo/include
        ${acados_include}/hpipm/include
        # ${acados_include}/acados
        )

set(NODE_NAME nmhe_estimator_node_exe)
ament_auto_add_executable(${NODE_NAME}
        src/nmhe_estimator_main.cpp)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_auto_package()
