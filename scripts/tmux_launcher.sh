#!/bin/bash

SESSION="ego"

# 1. Check if the session already exists
tmux has-session -t $SESSION 2>/dev/null

if [ $? != 0 ]; then
    echo "Spinning up Docker container and initializing windows..."

    # 2. Run the original script to start the Docker container
    ./Projects/ego_ws/src/px4_ego_planner/scripts/run-docker.sh
    sleep 2

    # 3. Create a new detached tmux session (Window 0: core-launch)
    tmux new-session -d -s $SESSION -n "core-launch"

    # ==================== WINDOW 0: CORE & LAUNCH ====================
    # PANE 1 (Top): Exec into Docker and run roscore
    tmux send-keys -t $SESSION:0 "docker exec -it noetic /bin/bash" C-m
    sleep 1
    tmux send-keys -t $SESSION:0 "roscore" C-m
    sleep 2

    # Split Window 0 vertically to create PANE 2 (Bottom)
    tmux split-window -v -t $SESSION:0

    # PANE 2 (Bottom): Exec into Docker and run Ego Planner
    tmux send-keys -t $SESSION:0.1 "docker exec -it noetic /bin/bash" C-m
    sleep 1
    tmux send-keys -t $SESSION:0.1 "roslaunch px4_ego_planner run_real.launch" C-m


    # ==================== WINDOW 1: FULLSCREEN DEBUG ====================
    # 4. Create a completely brand new window (Window 1) named "debug"
    tmux new-window -t $SESSION -n "debug"

    # Exec into Docker and clear screen for a clean workspace
    tmux send-keys -t $SESSION:1 "docker exec -it noetic /bin/bash" C-m
    sleep 1
    tmux send-keys -t $SESSION:1 "clear" C-m

fi

# 5. Attach to the session and drop directly onto Window 1 (your debug window)
tmux attach-session -t $SESSION:debug