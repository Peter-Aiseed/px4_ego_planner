# Introduction
This project integrates PX4, QGroundControl (QGC), and [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner.git) with a custom gatekeeper and trajectory server for hybrid mission execution.

The system allows:
- QGC mission input
- PX4 flight control
- EGO-Planner trajectory execution (OFFBOARD)
- Gatekeeper mode translation between PX4 and planner

# Environment
A quick overview of the required configuration.
- Ubuntu 20.04
- ROS Noetic
- Gazebo classic 11
- PX4-Autopilot version v1.16.1
- The latest versions of mavros, mavros_extras, mavros_msgs
- Eigen3 library

You can installed these softwares following the guide of [PX4-Avoidance](https://github.com/PX4/PX4-Avoidance.git), the only different is that i am using PX4 v1.16.1 so make sure you install the right version of [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot/tree/v1.16.1).  
Also make sure .bashrc are sourcing and exporting the right place of PX4-Autopilot (Change to your PX4 and repo path and put it at last of .bashrc).
```
source ~/PX4-Autopilot-v1.16.1/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot-v1.16.1/PX4-Autopilot ~/PX4-Autopilot-v1.16.1/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot-v1.16.1/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot-v1.16.1/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ego_ws/src/px4_ego_planner/px4_ego_planner/models
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/PX4-Autopilot-v1.16.1/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds
export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
```

# Build
If you are new to ROS development, please create a workspace first.
```bash
sudo apt install python3-catkin-tools
mkdir -p ~/catkin_ws/src
```
Then go to the source directory and clone this repository.
```bash
cd ~/catkin_ws/src
git clone https://github.com/Peter-Aiseed/px4_ego_planner.git
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

# How to Run the System
### 1. Start ROS Core
```bash
roscore
```
### 2. Open your QGroundControl
```bash
./QGroundControl.AppImage
```
### 3. Run Full System Script

```bash
px4_ego_planner/scripts/run_system.sh
px4_ego_planner/scripts/record.sh
```

# QGroundControl (QGC)
QGroundControl is used as the main control interface for the system.

You can use QGC to:
- Upload missions with waypoint, takeoff and return and done by Ego-Planner.
- Switch flight modes (Position/Stabilized, Offboard, Mission, RTL, Hold, Takeoff and Land)
- Send Go-To commands done by Ego-Planner.

The gatekeeper node will automatically coordinate PX4 and EGO-Planner behavior based on the selected mode.
