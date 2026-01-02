cmake_minimum_required(VERSION 3.0.2)
project(px4_ego_planner)

## Use C++11 (common in ROS1)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  mavros_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs mavros_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## flight_mode_manager node
add_executable(flight_mode_manager src/flight_mode_manager.cpp)
target_link_libraries(flight_mode_manager ${catkin_LIBRARIES})
add_dependencies(flight_mode_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## setgoal_node node
add_executable(setgoal_node src/setgoal_node.cpp)
target_link_libraries(setgoal_node ${catkin_LIBRARIES})
add_dependencies(setgoal_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(vins_transfer src/vins_transfer.cpp)
target_link_libraries(vins_transfer
  ${catkin_LIBRARIES}
)
add_dependencies(vins_transfer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Optional but recommended: install targets
install(TARGETS flight_mode_manager setgoal_node vins_transfer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
