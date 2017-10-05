cmake_minimum_required(VERSION 2.8.3)
project(drone_optique)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cv_bridge
  image_transport
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(optical_flow src/optical_flow.cpp)
target_link_libraries(optical_flow ${catkin_LIBRARIES})
add_dependencies(optical_flow drone_optic_generate_messages_cpp)
 
add_executable(detect_focus src/detect_focus.cpp)
target_link_libraries(detect_focus ${catkin_LIBRARIES})
add_dependencies(detect_focus drone_optic_generate_messages_cpp)

add_executable(detect_pattern src/detect_pattern.cpp)
target_link_libraries(detect_pattern ${catkin_LIBRARIES})
add_dependencies(detect_pattern drone_optic_generate_messages_cpp)

add_executable(track_object src/track_object.cpp)
target_link_libraries(track_object ${catkin_LIBRARIES})
add_dependencies(track_object drone_optic_generate_messages_cpp)


