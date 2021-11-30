# generated from ament_cmake_export_include_directories/cmake/ament_cmake_export_include_directories-extras.cmake.in

set(_exported_include_dirs "${mpc_controller_DIR}/../../../include;/home/liuhao/ros2_ws/usv_ws/src/control/mpc_controller/cmake-build-debug/single_track_dynamics")

# append include directories to mpc_controller_INCLUDE_DIRS
# warn about not existing paths
if(NOT _exported_include_dirs STREQUAL "")
  find_package(ament_cmake_core QUIET REQUIRED)
  foreach(_exported_include_dir ${_exported_include_dirs})
    if(NOT IS_DIRECTORY "${_exported_include_dir}")
      message(WARNING "Package 'mpc_controller' exports the include directory '${_exported_include_dir}' which doesn't exist")
    endif()
    normalize_path(_exported_include_dir "${_exported_include_dir}")
    list(APPEND mpc_controller_INCLUDE_DIRS "${_exported_include_dir}")
  endforeach()
endif()
