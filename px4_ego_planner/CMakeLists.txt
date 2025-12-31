cmake_minimum_required(VERSION 3.0.2)
project(px4_ego_planner)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(flight_mode_manager src/flight_mode_manager.cpp)
target_link_libraries(flight_mode_manager ${catkin_LIBRARIES})
add_executable(setgoal_node src/setgoal_node.cpp)
target_link_libraries(setgoal_node ${catkin_LIBRARIES})
