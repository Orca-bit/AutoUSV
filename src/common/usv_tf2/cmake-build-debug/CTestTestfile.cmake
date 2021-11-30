# CMake generated Testfile for 
# Source directory: /home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2
# Build directory: /home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_tf2_usv_msgs "/usr/bin/python3" "-u" "/opt/ros/foxy/share/ament_cmake_test/cmake/run_test.py" "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug/test_results/usv_tf2/test_tf2_usv_msgs.gtest.xml" "--package-name" "usv_tf2" "--output-file" "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug/ament_cmake_gtest/test_tf2_usv_msgs.txt" "--command" "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug/test_tf2_usv_msgs" "--gtest_output=xml:/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug/test_results/usv_tf2/test_tf2_usv_msgs.gtest.xml")
set_tests_properties(test_tf2_usv_msgs PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug/test_tf2_usv_msgs" TIMEOUT "60" WORKING_DIRECTORY "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug" _BACKTRACE_TRIPLES "/opt/ros/foxy/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/opt/ros/foxy/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/foxy/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/CMakeLists.txt;18;ament_add_gtest;/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/CMakeLists.txt;0;")
add_test(lint_cmake "/usr/bin/python3" "-u" "/opt/ros/foxy/share/ament_cmake_test/cmake/run_test.py" "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug/test_results/usv_tf2/lint_cmake.xunit.xml" "--package-name" "usv_tf2" "--output-file" "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/foxy/bin/ament_lint_cmake" "--xunit-file" "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug/test_results/usv_tf2/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2" _BACKTRACE_TRIPLES "/opt/ros/foxy/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/opt/ros/foxy/share/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;41;ament_add_test;/opt/ros/foxy/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;21;ament_lint_cmake;/opt/ros/foxy/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;0;;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/foxy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/foxy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/opt/ros/foxy/share/ament_cmake_auto/cmake/ament_auto_package.cmake;102;ament_package;/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/CMakeLists.txt;31;ament_auto_package;/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/CMakeLists.txt;0;")
add_test(copyright "/usr/bin/python3" "-u" "/opt/ros/foxy/share/ament_cmake_test/cmake/run_test.py" "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug/test_results/usv_tf2/copyright.xunit.xml" "--package-name" "usv_tf2" "--output-file" "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug/ament_copyright/copyright.txt" "--command" "/opt/ros/foxy/bin/ament_copyright" "--xunit-file" "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug/test_results/usv_tf2/copyright.xunit.xml")
set_tests_properties(copyright PROPERTIES  LABELS "copyright;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2" _BACKTRACE_TRIPLES "/opt/ros/foxy/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/opt/ros/foxy/share/ament_cmake_copyright/cmake/ament_copyright.cmake;41;ament_add_test;/opt/ros/foxy/share/ament_cmake_copyright/cmake/ament_cmake_copyright_lint_hook.cmake;18;ament_copyright;/opt/ros/foxy/share/ament_cmake_copyright/cmake/ament_cmake_copyright_lint_hook.cmake;0;;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/foxy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/foxy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/opt/ros/foxy/share/ament_cmake_auto/cmake/ament_auto_package.cmake;102;ament_package;/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/CMakeLists.txt;31;ament_auto_package;/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/CMakeLists.txt;0;")
add_test(cppcheck "/usr/bin/python3" "-u" "/opt/ros/foxy/share/ament_cmake_test/cmake/run_test.py" "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug/test_results/usv_tf2/cppcheck.xunit.xml" "--package-name" "usv_tf2" "--output-file" "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug/ament_cppcheck/cppcheck.txt" "--command" "/opt/ros/foxy/bin/ament_cppcheck" "--xunit-file" "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug/test_results/usv_tf2/cppcheck.xunit.xml" "--include_dirs" "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/include")
set_tests_properties(cppcheck PROPERTIES  LABELS "cppcheck;linter" TIMEOUT "300" WORKING_DIRECTORY "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2" _BACKTRACE_TRIPLES "/opt/ros/foxy/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/opt/ros/foxy/share/ament_cmake_cppcheck/cmake/ament_cppcheck.cmake;61;ament_add_test;/opt/ros/foxy/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;83;ament_cppcheck;/opt/ros/foxy/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;0;;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/foxy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/foxy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/opt/ros/foxy/share/ament_cmake_auto/cmake/ament_auto_package.cmake;102;ament_package;/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/CMakeLists.txt;31;ament_auto_package;/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/CMakeLists.txt;0;")
add_test(cpplint "/usr/bin/python3" "-u" "/opt/ros/foxy/share/ament_cmake_test/cmake/run_test.py" "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug/test_results/usv_tf2/cpplint.xunit.xml" "--package-name" "usv_tf2" "--output-file" "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug/ament_cpplint/cpplint.txt" "--command" "/opt/ros/foxy/bin/ament_cpplint" "--xunit-file" "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug/test_results/usv_tf2/cpplint.xunit.xml")
set_tests_properties(cpplint PROPERTIES  LABELS "cpplint;linter" TIMEOUT "120" WORKING_DIRECTORY "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2" _BACKTRACE_TRIPLES "/opt/ros/foxy/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/opt/ros/foxy/share/ament_cmake_cpplint/cmake/ament_cpplint.cmake;68;ament_add_test;/opt/ros/foxy/share/ament_cmake_cpplint/cmake/ament_cmake_cpplint_lint_hook.cmake;35;ament_cpplint;/opt/ros/foxy/share/ament_cmake_cpplint/cmake/ament_cmake_cpplint_lint_hook.cmake;0;;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/foxy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/foxy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/opt/ros/foxy/share/ament_cmake_auto/cmake/ament_auto_package.cmake;102;ament_package;/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/CMakeLists.txt;31;ament_auto_package;/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/CMakeLists.txt;0;")
add_test(uncrustify "/usr/bin/python3" "-u" "/opt/ros/foxy/share/ament_cmake_test/cmake/run_test.py" "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug/test_results/usv_tf2/uncrustify.xunit.xml" "--package-name" "usv_tf2" "--output-file" "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug/ament_uncrustify/uncrustify.txt" "--command" "/opt/ros/foxy/bin/ament_uncrustify" "--xunit-file" "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug/test_results/usv_tf2/uncrustify.xunit.xml")
set_tests_properties(uncrustify PROPERTIES  LABELS "uncrustify;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2" _BACKTRACE_TRIPLES "/opt/ros/foxy/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/opt/ros/foxy/share/ament_cmake_uncrustify/cmake/ament_uncrustify.cmake;65;ament_add_test;/opt/ros/foxy/share/ament_cmake_uncrustify/cmake/ament_cmake_uncrustify_lint_hook.cmake;34;ament_uncrustify;/opt/ros/foxy/share/ament_cmake_uncrustify/cmake/ament_cmake_uncrustify_lint_hook.cmake;0;;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/foxy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/foxy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/opt/ros/foxy/share/ament_cmake_auto/cmake/ament_auto_package.cmake;102;ament_package;/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/CMakeLists.txt;31;ament_auto_package;/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/CMakeLists.txt;0;")
add_test(xmllint "/usr/bin/python3" "-u" "/opt/ros/foxy/share/ament_cmake_test/cmake/run_test.py" "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug/test_results/usv_tf2/xmllint.xunit.xml" "--package-name" "usv_tf2" "--output-file" "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug/ament_xmllint/xmllint.txt" "--command" "/opt/ros/foxy/bin/ament_xmllint" "--xunit-file" "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/cmake-build-debug/test_results/usv_tf2/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2" _BACKTRACE_TRIPLES "/opt/ros/foxy/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/opt/ros/foxy/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/opt/ros/foxy/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;18;ament_xmllint;/opt/ros/foxy/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;0;;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/foxy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/foxy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/foxy/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/opt/ros/foxy/share/ament_cmake_auto/cmake/ament_auto_package.cmake;102;ament_package;/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/CMakeLists.txt;31;ament_auto_package;/home/liuhao/ros2_ws/usv_ws/src/common/usv_tf2/CMakeLists.txt;0;")
subdirs("gtest")
