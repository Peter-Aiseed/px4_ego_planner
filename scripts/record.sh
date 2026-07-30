#!/bin/bash
STAMP=$(date +%F_%H-%M-%S)
DIR=/ws_ego/src/px4_ego_planner/rosbag
mkdir -p "$DIR"
rosparam dump "$DIR/ego_debug_$STAMP.params.yaml"                       # planner must already be running
cp "$(rospack find ego_planner)/launch/run_ego_real.launch" \
   "$DIR/ego_debug_$STAMP.launch"
roslaunch ego_planner record.launch bag_prefix:="ego_debug_$STAMP"