cmake_minimum_required(VERSION 3.0.2)
project(px4_ego_planner)

## Use C++11 (common in ROS1)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	rospy
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  mavros_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS rospy roscpp std_msgs geometry_msgs nav_msgs mavros_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## simulation flight mode manager node
add_executable(sim_mode_manager src/sim_mode_manager.cpp)
target_link_libraries(sim_mode_manager ${catkin_LIBRARIES})
add_dependencies(sim_mode_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## real-world flight mode manager node
add_executable(real_mode_manager src/real_mode_manager.cpp)
target_link_libraries(real_mode_manager ${catkin_LIBRARIES})
add_dependencies(real_mode_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## setgoal_node node
add_executable(setgoal_node src/setgoal_node.cpp)
target_link_libraries(setgoal_node ${catkin_LIBRARIES})
add_dependencies(setgoal_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## vins_transfer_node
add_executable(vins_transfer src/vins_transfer.cpp)
target_link_libraries(vins_transfer
  ${catkin_LIBRARIES}
)
add_dependencies(vins_transfer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## visualization node
add_executable(visualization_node src/visualization_robot.cpp)
target_link_libraries(visualization_node
  ${catkin_LIBRARIES}
)
add_dependencies(visualization_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Optional but recommended: install targets
install(TARGETS real_mode_manager sim_mode_manager setgoal_node vins_transfer visualization_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  src/track.py
  src/qwen.py
  src/sot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
