#pragma once
#ifndef TEST_PATH_USE_STD_FILESYSTEM

#if __cplusplus >= 201703L && defined(__has_include)
#if __has_include(<filesystem>)
#define TEST_PATH_USE_STD_FILESYSTEM 1
#endif
#endif
#ifndef TEST_PATH_USE_STD_FILESYSTEM
#define TEST_PATH_USE_STD_FILESYSTEM 0
#endif

#if TEST_PATH_USE_STD_FILESYSTEM
#include <filesystem>
#else
#include <boost/filesystem.hpp>
#endif

namespace kal_controller_ros_tool {
namespace test {

#ifdef TEST_PATH_USE_STD_FILESYSTEM
static const std::filesystem::path projectRootDir{"/home/kamran/programming/kal_test_ws/src/kal_controller_ros_tool"};
#else
static const boost::filesystem::path projectRootDir{"/home/kamran/programming/kal_test_ws/src/kal_controller_ros_tool"};
#endif

} // namespace test
} // namespace kal_controller_ros_tool

#endif
