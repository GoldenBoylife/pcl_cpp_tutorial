cmake_minimum_required(VERSION 3.0.2)
project(pcl_cpp_tutorial)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  # pcl
  pcl_ros
  roscpp
  sensor_msgs
)
find_package(PCL REQUIRED)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcl_read src/pcl_read.cpp)
target_link_libraries(pcl_read ${catkin_LIBRARIES})

 
add_executable(pcl_write src/pcl_write.cpp)
target_link_libraries(pcl_write ${catkin_LIBRARIES})

add_executable(pcl_voxel_filter src/pcl_voxel_filter.cpp)
target_link_libraries(pcl_voxel_filter ${catkin_LIBRARIES})

add_executable(pcl_passthrough src/pcl_passthrough.cpp)
target_link_libraries(pcl_passthrough ${catkin_LIBRARIES})