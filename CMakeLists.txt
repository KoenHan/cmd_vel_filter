cmake_minimum_required(VERSION 3.0.2)
project(cmd_vel_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(cmd_vel_echo src/cmd_vel_echo.cpp)
target_link_libraries(cmd_vel_echo ${catkin_LIBRARIES})
add_dependencies(cmd_vel_echo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

