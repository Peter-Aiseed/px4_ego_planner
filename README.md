# Introduction
This project forks from [ego_planner](https://github.com/ZJU-FAST-Lab/ego-planner.git), aiming to extend it to a PX4-compatible version. Up to now, the implementation of ego-planner in both the Gazebo simulator and the real world has been completed!
# Environment
A quick overview of the required configuration.
- Ubuntu 20.04
- ROS Noetic
- Gazebo classic 11
- The latest version of PX4
- The latest versions of mavros, mavros_extras, mavros_msgs
- Eigen3 library

I actually installed these softwares following the guide of [PX4-Aovidance](https://github.com/PX4/PX4-Avoidance.git), so, I also strongly recommend you to refer to this project and double-check that all dependencies are installed correctly.
# Build
If you are new to ROS development, please create a workspace first.
```bash
sudo apt install python3-catkin-tools
mkdir -p ~/catkin_ws/src
```
Then go to the source directory and clone this repository.
```bash
cd ~/catkin_ws/src
git clone https://github.com/hyq123-cmd/px4_ego_planner.git
```
Return to the parent directory and build the project.
```bash
cd ..
catkin build px4_ego_planner
```
Finally, source the catkin setup.bash so that the system can find your rospacks.
```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
# Running in Gazebo
Open a terminal and split it into three windows (Tmux or Terminator are recommended tools).
1. In the first window, enter the following command to launch the PX4, MAVROS, Gazebo, and RViz. Note that the first launch may take a few minutes to load the Gazebo world.
```bash
roslaunch px4_ego_planner run_sim.launch
```
2. Until you see "Ready to takeoff" in the first window, you can enter another command in the second window.
This command is to start the flight mode manager node; the drone will then start takeoff. Just wait a few seconds for the drone to hovers in place.
```bash
rosrun px4_ego_planner sim_mode_manager
```
3. Launch ego_planner in the third window.
```bash
roslaunch px4_ego_planner run_ego_sim.launch
```
4. You can use the **2D Nav Goal** tool in Rviz to specify a goal.

# Real-World Testing
It is pretty difficult to replicate our real-world experiment, since we possibly have different hardware setups and mounting location. However, I would still like to record how we implemented the real-world test for your reference.
## Pose estimation
We mounted a realsense D455 camera on our drone, and ran VINS-Fusion (VINS) algorithm to implement real-time pose estimation. Since the D455 camera provides a relatively accurate IMU, we directly combined the built-in IMU and stereo camera to run VINS. Please refer to our camera configuration files located in "$(rospack find px4_ego_planner)/resource/realsense_d455/".<br>Furthermore, we have written a coordinate transformation node to align the reference frames of VINS and PX4. "vins_transfer.cpp" is responsible for this function.
## Node launch steps
Using a cable to connect your drone and laptop, and launch the following file:
```bash
roslaunch px4_ego_planner run_real.launch
```
After that, make sure everything is running well, you should see some prompts like "Altitude too low xx <= xx." in the terminal. This message indicate that before switching to OFFBOARD mode, the drone should first fly above a certian altitude, and then the mode can be switched from POSITION to OFFBOARD.<br>In this launch file, the mavros, VINS and Rviz are all started automatically.<br>Next, enter the following command in a new terminal.
```bash
roslaunch px4_ego_planner run_ego_real.launch
```
You should see the ego_planner start working, and many colored grids are displayed in the Rviz to represent the obstacles. Then, you can use the mouse to set a goal using 2D Nav Goal tool in Rviz analogous to the simulated case.<br>Finally, you can unplug the cable. Use the remote controller (RC) to take off the drone using POSITION mode until it reaches a certain altitude (typically around 0.5 m), then switch to OFFBOARD mode. The drone will navigate to the goal and avoid obstacles autonomously.<br>By the way, you should keep the throttle stick on RC at around 50 percent position, which allows you to take over control when the drone arrives at the goal or encounters any emergency.
