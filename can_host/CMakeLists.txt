cmake_minimum_required(VERSION 3.0.2)
project(can_host)
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS scripts/can_host_node scripts/can_host_ros_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)