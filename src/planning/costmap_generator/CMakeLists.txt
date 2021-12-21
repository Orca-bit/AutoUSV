cmake_minimum_required(VERSION 3.5)
project(costmap_generator)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(COSTMAP_GENERATOR_LIB costmap_generator)

ament_auto_add_library(${COSTMAP_GENERATOR_LIB} SHARED
        src/costmap_generator/costmap_generator.cpp
        src/costmap_generator/points_to_costmap.cpp
        )
usv_set_compile_options(${COSTMAP_GENERATOR_LIB})

# skipping Werrors from external `grid_map_cv` package
target_compile_options(${COSTMAP_GENERATOR_LIB} PRIVATE -Wno-old-style-cast -Wno-sign-conversion -Wno-conversion -Wno-double-conversion -Wno-double-promotion)

if (BUILD_TESTING)
  # run linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  # find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(${COSTMAP_GENERATOR_LIB}_test test/test_costmap_generator.cpp TIMEOUT 100)
  # usv_set_compile_options(${COSTMAP_GENERATOR_LIB}_test)
  # target_link_libraries(${COSTMAP_GENERATOR_LIB}_test ${COSTMAP_GENERATOR_LIB})
endif ()

ament_auto_package(INSTALL_TO_SHARE)
