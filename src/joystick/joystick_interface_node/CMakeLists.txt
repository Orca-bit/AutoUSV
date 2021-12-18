cmake_minimum_required(VERSION 3.5)
project(joystick_interface_node)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(JOYSTICK_INTERFACE_LIB_NODE joystick_interface_node)

ament_auto_add_library(${JOYSTICK_INTERFACE_LIB_NODE}
  SHARED src/joystick_interface_node.cpp)
usv_set_compile_options(${JOYSTICK_INTERFACE_LIB_NODE})

rclcpp_components_register_node(${JOYSTICK_INTERFACE_LIB_NODE}
        PLUGIN "joystick_interface_node::JoystickInterfaceNode"
        EXECUTABLE ${JOYSTICK_INTERFACE_LIB_NODE}_exe)

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

ament_auto_package(INSTALL_TO_SHARE param launch)
