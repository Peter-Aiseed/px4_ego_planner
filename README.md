# Description
This project forks from [ego_planner](https://github.com/ZJU-FAST-Lab/ego-planner.git), aiming to extend it to a PX4-compatiable version. Up to now, the implementation of ego-planner on Gazebo simulator has been completed! The real-world test will be conducted soon.
# Environment
A quick overview of the required configuration.
- Ubuntu 20.04
- ROS Noetic
- Gazebo classic 11
- The latest version of PX4
- The latest versions of mavros, mavros_extras, mavros_msgs

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
Back to previous level of directory and build the project.
```bash
cd ..
catkin build px4_ego_planner
```
Finally, source the catkin setup.bash so that the system can find your rospacks.
```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
# Running
Open your terminal and split it into four windows (Tmux or Terminator are good tools).
1. In the first window, enter this command to launch the PX4, MAVROS, simulator, and RViz. Perhaps you need to wait few minutes, since the first time to load a new Gazebo world should take some time.
```bash
roslaunch px4_ego_planner run_sim.launch
```
2. Until you see "Ready to takeoff" in the first window, you can enter another command in the second window.
This command is to start the flight mode manager node; the drone will then start takeoff. Just wait a few seconds for the drone to hovers in place.
```bash
rosrun px4_ego_planner flight_mode_manager
```
3. Launch ego_planner in the third window.
```bash
roslaunch px4_ego_planner run_ego.launch
```
4. Send a preset goal point in the fourth window, I have already preset a feasible goal point here. Alternatively, you can use the **2D Nav Goal** tool in Rviz to specify a goal.
```bash
rosrun px4_ego_planner setgoal_node 
``` 
