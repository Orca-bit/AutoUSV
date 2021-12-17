cmake_minimum_required(VERSION 3.6)

project(nmpc_controller)

# Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(acados_include $ENV{ACADOS_SOURCE_DIR}/include)
set(acados_lib $ENV{ACADOS_SOURCE_DIR}/lib)
set(acados_c_code_gen ${PROJECT_SOURCE_DIR}/scripts/c_generated_code)

include_directories(
        ${acados_c_code_gen}
        ${acados_include}
        ${acados_include}/blasfeo/include
        ${acados_include}/hpipm/include
        ${acados_include}/acados
)

link_directories(
        ${acados_c_code_gen}
)

# Build library
ament_auto_add_library(${PROJECT_NAME} SHARED
        src/nmpc_controller/config.cpp
        src/nmpc_controller/nmpc_controller.cpp
        src/nmpc_controller/references.cpp
        src/nmpc_controller/debug.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${acados_c_code_gen}/libacados_ocp_solver_autousv.so
        ${acados_lib}/libacados.so
        )

### Test
if (BUILD_TESTING)
    # Linters
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    # Unit test
    apex_test_tools_add_gtest(nmpc_controller_unit_tests
            test/gtest_main.cpp
            test/sanity_checks.cpp)

    ament_target_dependencies(nmpc_controller_unit_tests "controller_common")
    target_link_libraries(nmpc_controller_unit_tests ${PROJECT_NAME})
    add_dependencies(nmpc_controller_unit_tests ${PROJECT_NAME})
endif ()

ament_auto_package()
