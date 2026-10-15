# Introduction
This project integrates PX4, QGroundControl (QGC), and [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner.git) with a custom gatekeeper and trajectory server for hybrid mission execution.

The system allows:
- QGC mission input
- PX4 flight control
- EGO-Planner trajectory execution (OFFBOARD)
- Gatekeeper mode translation between PX4 and planner


# Build
Create ego_ws/src folder.
```bash
mkdir -p ~/ego_ws/src
```
Then go to the directory and clone this repository.
```bash
cd ~/ego_ws/src
git clone https://github.com/aiseed-tech/px4_ego_planner.git -b nx-1.0
```
Build the environment docker.
```bash
cd px4_ego_planner/scripts
docker build -f noetic-dockerfile -t px4-ego-noetic:nx-1.0 .
```
After build, change the run-docker.sh directory to where you put ego_ws:   
Change this line: (if not put under Projects then delete /Projects)
```bash
 -v $HOME/Projects/ego_ws:/ws_ego \
```
Run docker by script and go into docker:
```bash
./run-docker.sh
docker exec -it noetic bash
```
Go to the directory in docker and build the project.
```bash
cd ws_ego/
catkin build multi_map_server
catkin build
```
