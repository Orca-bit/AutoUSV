cmake_minimum_required(VERSION 3.5)

### Export headers
project(usv_common)

## dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Temporarily disable cpplint and uncrustify
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )

  ament_lint_auto_find_test_dependencies()

  file(GLOB_RECURSE FILES_MINUS_SOME
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*
    ${CMAKE_CURRENT_SOURCE_DIR}/test/*
  )

  list(FILTER FILES_MINUS_SOME EXCLUDE REGEX ".*filesystem.hpp")

  # Re-enable cpplint
  find_package(ament_cmake_cpplint)
  ament_cpplint(${FILES_MINUS_SOME})

  # Re-enable uncrustify
  find_package(ament_cmake_uncrustify)
  ament_uncrustify(${FILES_MINUS_SOME})

  # Unit tests
  set(TEST_COMMON test_common_gtest)
  ament_add_gtest(${TEST_COMMON}
          test/gtest_main.cpp
          test/test_bool_comparisons.cpp
          test/test_byte_reader.cpp
          test/test_float_comparisons.cpp
          test/test_mahalanobis_distance.cpp
          test/test_message_field_adapters.cpp
          test/test_template_utils.cpp
          test/test_angle_utils.cpp
          test/test_type_name.cpp
          test/test_type_traits.cpp)
  usv_set_compile_options(${TEST_COMMON})
  target_compile_options(${TEST_COMMON} PRIVATE -Wno-sign-conversion)
  target_include_directories(${TEST_COMMON} PRIVATE include)
  ament_target_dependencies(${TEST_COMMON} builtin_interfaces Eigen3)
endif()

# Ament Exporting
ament_auto_package()
