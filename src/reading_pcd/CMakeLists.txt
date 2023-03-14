cmake_minimum_required(VERSION 3.0.2)
project(reading_pcd)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
)

find_package(PCL 1.8 REQUIRED)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(/usr/include/pcl-1.8)

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(pcd_read src/pcd_read.cpp)
target_link_libraries(pcd_read ${PCL_LIBRARIES} ${catkin_LIBRARIES})

