roslaunch px4_ego_planner replay.launch \
  bag_file:=/ws_ego/src/px4_ego_planner/rosbag/ego_debug_2026-07-25.bag

# <!-- Active replay: re-run the planner against the bag inputs -->
# roslaunch ego_planner replay.launch \
#   bag_file:=/ws_ego/src/px4_ego_planner/rosbag/ego_debug_2026-07-25.bag \
#   active:=true rate:=0.5