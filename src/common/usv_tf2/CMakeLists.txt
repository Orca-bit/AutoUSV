cmake_minimum_required(VERSION 3.5)

### Export headers
project(usv_tf2)

## dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Linters
  ament_lint_auto_find_test_dependencies()
  # Unit test
  ament_add_gtest(test_tf2_usv_msgs test/test_tf2_usv_msgs.cpp)
  usv_set_compile_options(test_tf2_usv_msgs)
  target_include_directories(test_tf2_usv_msgs PRIVATE "include")
  ament_target_dependencies(test_tf2_usv_msgs
    "usv_common"
    "usv_msgs"
    "geometry_msgs"
    "orocos_kdl"
    "tf2"
    "tf2_ros"
)
endif()

ament_auto_package()
