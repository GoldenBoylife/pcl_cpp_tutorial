cmake_minimum_required(VERSION 3.0.2)
project(pcl_cpp_tutorial)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++14") #c++14를써야 chrono_literals 쓸수 있다.

find_package(catkin REQUIRED COMPONENTS
  # pcl
  pcl_ros
  roscpp
  sensor_msgs
)
find_package(PCL 1.5 REQUIRED)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# ##registration에 필요
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# ##

add_executable(pcl_read src/pcl_read.cpp)
target_link_libraries(pcl_read ${catkin_LIBRARIES})

 
add_executable(pcl_write src/pcl_write.cpp)
target_link_libraries(pcl_write ${catkin_LIBRARIES})

add_executable(pcl_voxel_filter src/pcl_voxel_filter.cpp)
target_link_libraries(pcl_voxel_filter ${catkin_LIBRARIES})

add_executable(pcl_passthrough src/pcl_passthrough.cpp)
target_link_libraries(pcl_passthrough ${catkin_LIBRARIES})

add_executable(pcl_registration1 src/pcl_registration.cpp)
target_link_libraries(pcl_registration1 ${catkin_LBRARIES} ${PCL_LIBRARIES})

add_executable(pcl_test src/pcl_test.cpp)
target_link_libraries(pcl_test ${catkin_LIBRARIES})

add_executable(pcl_translation src/pcl_translation.cpp)
target_link_libraries(pcl_translation ${catkin_LIBRARIES})

add_executable(pcl_to_topic src/pcl_to_topic.cpp)
target_link_libraries(pcl_to_topic ${catkin_LIBRARIES})

add_executable(pcl_topic+translation src/pcl_topic+translation.cpp)
target_link_libraries(pcl_topic+translation ${catkin_LIBRARIES})

add_executable(pcl_viewer src/pcl_viewer.cpp)
target_link_libraries(pcl_viewer ${catkin_LIBRARIES}  pcl_visualization)

add_executable(pcl_registration_pcds src/pcl_registration_pcds.cpp)
target_link_libraries(pcl_registration_pcds ${catkin_LIBRARIES} pcl_visualization ${PCL_LIBRARIES})

add_executable(pcl_ndt src/pcl_ndt.cpp) 
target_link_libraries(pcl_ndt ${PCL_LIBRARIES})