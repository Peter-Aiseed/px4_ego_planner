#!/bin/bash
STAMP=$(date +%F_%H-%M-%S)
DIR=/ws_ego/src/px4_ego_planner/rosbag/
rosparam dump $DIR/ego_debug_$STAMP.params.yaml           # config snapshot
cp $(rospack find ego_planner)/launch/run_real.launch \
   $DIR/ego_debug_$STAMP.launch                            # exact launch used
roslaunch px4_ego_planner record_ego.launch bag_prefix:=ego_debug_$STAMP