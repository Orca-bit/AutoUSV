cmake_minimum_required(VERSION 3.6)
project(nmhe_estimator)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

# if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#    add_compile_options(-Wall -Wextra -Wpedantic)
# endif ()

# Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(acados_include $ENV{ACADOS_SOURCE_DIR}/include)
set(acados_c_code_gen ${PROJECT_SOURCE_DIR}/scripts/c_generated_code)

ament_auto_add_library(${PROJECT_NAME} SHARED
        src/nmhe_estimator/config.cpp
        src/nmhe_estimator/nmhe_estimator.cpp
        src/nmhe_estimator/references.cpp
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        ${acados_c_code_gen}
        ${acados_include}
        ${acados_include}/blasfeo/include
        ${acados_include}/hpipm/include
        )

target_link_libraries(${PROJECT_NAME}
        ${acados_c_code_gen}/libacados_ocp_solver_mhe_autousv_ode_with_noisy_param.so
        )

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

install(DIRECTORY scripts/c_generated_code DESTINATION share/)
install(FILES ${acados_c_code_gen}/libacados_ocp_solver_mhe_autousv_ode_with_noisy_param.so DESTINATION lib/)

ament_auto_package()
