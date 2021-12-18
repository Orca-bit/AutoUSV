cmake_minimum_required(VERSION 3.5)
project(astar_planner_nodes)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(ASTAR_PLANNER_NODE_LIB astar_planner_node)
ament_auto_add_library(${ASTAR_PLANNER_NODE_LIB} SHARED
  src/astar_planner_nodes/astar_planner_node.cpp)
usv_set_compile_options(${ASTAR_PLANNER_NODE_LIB})


rclcpp_components_register_node(${ASTAR_PLANNER_NODE_LIB}
  PLUGIN "usv::planning::astar_planner::AstarPlannerNode"
  EXECUTABLE ${ASTAR_PLANNER_NODE_LIB}_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(autoware_testing REQUIRED)
  add_smoke_test(${PROJECT_NAME} ${ASTAR_PLANNER_NODE_LIB}_exe)

  add_ros_test(
    test/astar_planner.test.py
    TIMEOUT "15"
  )
endif()

ament_auto_package(INSTALL_TO_SHARE launch param)
