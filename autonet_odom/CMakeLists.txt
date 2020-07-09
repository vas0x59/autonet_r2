cmake_minimum_required(VERSION 2.8.3)
project(autonet_odom)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
#   cv_bridge
  geometry_msgs
  nav_msgs
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf2 tf2_ros message_generation
)
# find_package( OpenCV REQUIRED )

add_service_files(
        FILES
        SetOdom.srv
)

generate_messages()

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES autonet_odom
        #  CATKIN_DEPENDS cv_birdge nav_msgs roscpp rospy sensor_msgs std_msgs tf
        #  DEPENDS system_lib
        CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(odom src/odom.cpp)


target_link_libraries(odom
  ${catkin_LIBRARIES}
)
catkin_package()