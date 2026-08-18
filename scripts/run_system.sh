#!/bin/bash

# 1. Setup Paths
# This script is in .../px4_ego_planner/scripts/
# The Repo Root is one level up
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

echo "==========================================="
echo "   PX4 + EGO-PLANNER UNIFIED STARTUP      "
echo "   Repo Root: $REPO_ROOT"
echo "   Scripts:   $SCRIPT_DIR"
echo "==========================================="

clean_up() {
    echo -e "\n\033[31m[!] SHUTTING DOWN ALL PROCESSES...\033[0m"
    
    # Kill the background PIDs we saved earlier
    kill -9 $GATE_PID $SIM_PID 2>/dev/null
    
    # Force kill by name to free up UDP ports 14550, 14570, etc.
    pkill -9 -f gatekeeper.py
    pkill -9 -f px4
    pkill -9 -f gzserver
    
    echo -e "\033[32m[✔] Ports cleared. System ready for next run.\033[0m"
    exit
}
trap clean_up INT

# 3. Start the Gatekeeper Filter
# Located in the same folder as this script
echo "[1/2] Starting Gatekeeper..."
python3 "$SCRIPT_DIR/gatekeeper.py" &
GATE_PID=$!

# 4. Wait for background processes to initialize
sleep 2

# 5. Run your main ROS Launch file
echo "[2/2] Starting Ego-Planner Simulation..."
echo "-------------------------------------------"

# Launch your simulation (Make sure your workspace is sourced!)
roslaunch px4_ego_planner run_real.launch
SIM_PID=$!

# 6. Wait for the simulation to end (or Ctrl+C)
clean_up
