cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rospack(pluginlib)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rospack_add_boost_directories()

rospack_add_library(poco_lite src/Exception.cpp src/File.cpp src/Manifest.cpp src/Mutex.cpp src/Path.cpp src/SharedLibrary.cpp src/StringTokenizer.cpp src/Timestamp.cpp src/UnicodeConverter.cpp src/AtomicCounter.cpp src/UTF16Encoding.cpp src/UTF8Encoding.cpp src/TextEncoding.cpp src/ASCIIEncoding.cpp src/Latin1Encoding.cpp src/Latin9Encoding.cpp src/Windows1252Encoding.cpp src/RWLock.cpp src/DirectoryIterator.cpp src/Bugcheck.cpp src/Environment.cpp src/TextIterator.cpp src/Debugger.cpp src/TextConverter.cpp)
target_link_libraries(poco_lite dl)
