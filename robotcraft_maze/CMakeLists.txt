cmake_minimum_required(VERSION 2.8.3)
project(robotcraft_maze)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  stage_ros
  std_msgs
  tf
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(maze_basic_task src/maze_basic_task.cpp)
target_link_libraries(maze_basic_task ${catkin_LIBRARIES})
add_executable(maze_driver src/maze_driver.cpp)
target_link_libraries(maze_driver ${catkin_LIBRARIES})
