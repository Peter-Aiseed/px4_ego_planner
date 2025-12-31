# Description
This project forks from [ego_planner](https://github.com/ZJU-FAST-Lab/ego-planner.git), aiming to extend it to PX4-compatiable version. Up to now, the implementation of ego-planner on Gazebo simulator was completed! The real-world test will be conducted soon.
# Environment
A quick overview of my system.
- Ubuntu 20.04
- ROS Noetic
- Gazebo classic 11
- newest version of PX4
- newest version of mavros, mavros_extras, mavros_msgs

I actually installed these softwares according to [PX4-Aovidance](https://github.com/PX4/PX4-Avoidance.git), so, I aslo strongly recommend you to reference this project and double check if you have installed them.
# Build
If you haven't work on ROS, you might need to create a workspace as following.
```
sudo apt install python3-catkin-tools
mkdir -p ~/catkin_ws/src
```
Then go to this directory and download my code.
```
cd ~/catkin_ws/src
git clone https://github.com/hyq123-cmd/px4_ego_planner.git
```
Back to previous level of directory and build this project.
```
cd ..
catkin build px4_ego_planner
```
# Running
Open your terminal and spilt it into four windows (Tmux or Terminator are good tools).
1. In the first window, enter this command to launch PX4, MAVROS, simulator, and RVIZ. Perhaps you need to wait few minutes, since the first time to load a new Gazebo world should take some time.
```
roslaunch px4_ego_planner run_sim.launch
```
2. Until you see the "Ready to takeoff" in the first window, you can enter another command in the second window.
This command is to start the flight mode manager, after that, the drone will start takeoff. Just wait some seconds when the drone loiters.
```
rosrun px4_ego_planner flight_mode_manager
```
3. Launch ego_planner in the third window.
```
roslaunch px4_ego_planner run_ego.launch
```
4. Send a preset goal point in the fourth window, I have already preset a feasible coordinate here.
You can use the nav arrow in Rviz to send a desired goal actually, so, you don't need to start this node if you choose this apporach.
```
rosrun px4_ego_planner setgoal_node 
``` 
