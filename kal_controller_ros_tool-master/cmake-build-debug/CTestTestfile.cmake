# CMake generated Testfile for 
# Source directory: /home/kamran/programming/kal_test_ws/src/kal_controller_ros_tool
# Build directory: /home/kamran/programming/kal_test_ws/src/kal_controller_ros_tool/cmake-build-debug
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(build_tests "/home/kamran/programming/kal_test_ws/src/kal_controller_ros_tool/cmake-build-debug/catkin_generated/env_cached.sh" "bash" "-c" "\"/snap/clion/111/bin/cmake/linux/bin/cmake\" --build /home/kamran/programming/kal_test_ws/src/kal_controller_ros_tool/cmake-build-debug --parallel 8 --target tests && /home/kamran/programming/kal_test_ws/src/mrt_cmake_modules/scripts/init_coverage.py kal_controller_ros_tool /home/kamran/programming/kal_test_ws/src/kal_controller_ros_tool/cmake-build-debug /home/kamran/programming/kal_test_ws/src/kal_controller_ros_tool /home/kamran/programming/kal_test_ws/src/kal_controller_ros_tool/cmake-build-debug/test_results/kal_controller_ros_tool")
set_tests_properties(build_tests PROPERTIES  _BACKTRACE_TRIPLES "/home/kamran/programming/kal_test_ws/src/mrt_cmake_modules/cmake/Modules/MrtTesting.cmake;76;add_test;/home/kamran/programming/kal_test_ws/src/mrt_cmake_modules/cmake/mrt_cmake_modules-macros.cmake;1083;mrt_init_testing;/home/kamran/programming/kal_test_ws/src/kal_controller_ros_tool/CMakeLists.txt;84;mrt_add_ros_tests;/home/kamran/programming/kal_test_ws/src/kal_controller_ros_tool/CMakeLists.txt;0;")
subdirs("gtest")
