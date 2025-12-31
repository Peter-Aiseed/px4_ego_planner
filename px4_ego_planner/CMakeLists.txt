cmake_minimum_required(VERSION 3.0.2)
project(px4_ego_planner)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES px4_ego_planner
  CATKIN_DEPENDS ego_planner
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(mode_transition_node src/flight_mode_transition.cpp)
target_link_libraries(mode_transition_node ${catkin_LIBRARIES})
add_executable(setgoal_node src/setgoal_node.cpp)
target_link_libraries(setgoal_node ${catkin_LIBRARIES})
