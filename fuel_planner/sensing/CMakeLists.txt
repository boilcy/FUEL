PROJECT(sensing_node)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp 
    roslib 
    cv_bridge cmake_modules pcl_ros sensor_msgs geometry_msgs nav_msgs quadrotor_msgs)

catkin_package(
    DEPENDS Eigen OpenCV
    CATKIN_DEPENDS roscpp roslib pcl_ros
)

include_directories(
  SYSTEM 
  include 
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(
  sensing_node
  src/pointcloud_render_node.cpp
)
target_link_libraries(sensing_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
# add_executable(
#   pcl_render_node
#   src/depth_render_node.cpp
# )
# target_link_libraries( pcl_render_node
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${OpenCV_LIBS}
# )


