cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init()

# add dynamic reconfigure api
#rosbuild_find_ros_package(dynamic_reconfigure)
# include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
# gencfg()
rosbuild_genmsg()


rosbuild_add_library(glocalizer_lib
                     src/stuff/stat.cpp
		     src/glocalizer/laserparameters.cpp
		     src/glocalizer/motionmodel.cpp
		     src/glocalizer/localizemap.cpp
		     src/glocalizer/localizer.cpp)


rosbuild_add_executable(bin/glocalizer
                       src/glocalizer_node.cpp)

target_link_libraries(bin/glocalizer glocalizer_lib -lboost_signals)

rosbuild_add_executable(bin/glocalizer_test
                       src/glocalizer/localize_test.cpp)
target_link_libraries(bin/glocalizer_test glocalizer_lib)


rosbuild_add_executable(bin/pgm2lmap
                       src/glocalizer/pgm2lmap.cpp)

target_link_libraries(bin/pgm2lmap glocalizer_lib)

rosbuild_add_executable(bin/clf2lmap
                       src/glocalizer/clf2lmap.cpp)

target_link_libraries(bin/clf2lmap glocalizer_lib)

rosbuild_add_executable(bin/clfmerge
                       src/glocalizer/clfmerge.cpp)

target_link_libraries(bin/clfmerge glocalizer_lib)




