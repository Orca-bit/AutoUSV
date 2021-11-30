set(_AMENT_PACKAGE_NAME "mpc_controller")
set(mpc_controller_VERSION "1.0.0")
set(mpc_controller_MAINTAINER "liuhao <steepcurve@163.com>")
set(mpc_controller_BUILD_DEPENDS "acado_vendor" "motion_testing" "time_utils" "usv_msgs" "controller_common" "motion_common")
set(mpc_controller_BUILDTOOL_DEPENDS "ament_cmake_auto" "usv_auto_cmake")
set(mpc_controller_BUILD_EXPORT_DEPENDS "usv_msgs" "controller_common" "motion_common")
set(mpc_controller_BUILDTOOL_EXPORT_DEPENDS "usv_auto_cmake")
set(mpc_controller_EXEC_DEPENDS "usv_msgs" "controller_common" "motion_common")
set(mpc_controller_TEST_DEPENDS "apex_test_tools" "ament_lint_auto" "ament_lint_common")
set(mpc_controller_GROUP_DEPENDS )
set(mpc_controller_MEMBER_OF_GROUPS )
set(mpc_controller_DEPRECATED "")
set(mpc_controller_EXPORT_TAGS)
list(APPEND mpc_controller_EXPORT_TAGS "<build_type>ament_cmake</build_type>")
