cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

include_directories(/usr/include/suitesparse)
include_directories(~/ros/cs225b/project4/Dropbox/g2o)
include_directories(~/ros/cs225b/project4/Dropbox/g2o/g2o/examples/tutorial_slam2d)
link_directories(~/ros/cs225b/project4/Dropbox/g2o/lib)
rosbuild_add_library(start_my_slam src/Karto.cpp)
rosbuild_add_library(start_my_slam src/Mapper.cpp)
rosbuild_add_library(start_my_slam src/karto_scan_matcher.cpp)
rosbuild_add_library(start_my_slam src/my_slam.cpp)
rosbuild_add_library(start_my_slam ~/ros/cs225b/project4/Dropbox/g2o/g2o/examples/tutorial_slam2d/vertex_se2.cpp)
rosbuild_add_library(start_my_slam ~/ros/cs225b/project4/Dropbox/g2o/g2o/examples/tutorial_slam2d/edge_se2.cpp)
rosbuild_add_executable(start_my_slam src/start_my_slam.cpp)
target_link_libraries(start_my_slam cxsparse csparse g2o_core g2o_solver_csparse g2o_types_slam2d g2o_stuff)
