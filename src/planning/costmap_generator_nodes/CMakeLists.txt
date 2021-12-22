cmake_minimum_required(VERSION 3.5)
project(costmap_generator_nodes)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(COSTMAP_GENERATOR_NODE_NAME costmap_generator_node)

ament_auto_add_library(${COSTMAP_GENERATOR_NODE_NAME} SHARED
  src/costmap_generator_nodes/costmap_generator_node.cpp
)
usv_set_compile_options(${COSTMAP_GENERATOR_NODE_NAME})
# skipping Werrors from external `grid_map_cv` package
target_compile_options(${COSTMAP_GENERATOR_NODE_NAME} PRIVATE -Wno-old-style-cast -Wno-sign-conversion -Wno-conversion -Wno-double-conversion -Wno-double-promotion)

rclcpp_components_register_node(${COSTMAP_GENERATOR_NODE_NAME}
  PLUGIN "usv::planning::costmap_generator::CostmapGeneratorNode"
  EXECUTABLE ${COSTMAP_GENERATOR_NODE_NAME}_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(autoware_testing REQUIRED)
  add_smoke_test(${PROJECT_NAME} ${COSTMAP_GENERATOR_NODE_NAME}_exe)

  add_ros_test(
    test/costmap_generator_node.test.py
    TIMEOUT "15"
  )
endif()

ament_auto_package(INSTALL_TO_SHARE launch param)
