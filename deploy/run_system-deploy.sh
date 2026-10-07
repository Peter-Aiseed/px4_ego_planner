#!/bin/bash
# Deployment runtime manager for the px4-ego service.
#
# Runs as the container's application process (under tini):
#   1. sources the ROS environment explicitly,
#   2. starts roscore and waits for the ROS master,
#   3. starts gatekeeper.py and run_real.launch,
#   4. supervises them until an intentional stop or an unexpected failure.
#
# This is the production workflow. scripts/run_system.sh remains the
# development startup script and is intentionally untouched.

GATEKEEPER="/ws_ego/src/px4_ego_planner/scripts/gatekeeper.py"

# Test-only seams (never deployment configuration).
SETUP_ROOT="${PX4_EGO_TEST_SETUP_ROOT:-}"
READINESS_TIMEOUT="${PX4_EGO_TEST_READINESS_TIMEOUT:-30}"
case "$READINESS_TIMEOUT" in
  ''|*[!0-9]*)
    echo "run_system-deploy: invalid PX4_EGO_TEST_READINESS_TIMEOUT: '$READINESS_TIMEOUT'" >&2
    exit 1
    ;;
esac

STOPPING=0
ROSCORE_PID=""
GATE_PID=""
ROSLAUNCH_PID=""

on_stop() {
  # A child forked for a managed process but not yet exec'd inherits this
  # handler. Exit it so a wake-up TERM delivered in the fork window still
  # terminates the child instead of being swallowed.
  if [ "${BASHPID:-}" != "$$" ]; then
    exit 0
  fi
  STOPPING=1
  # Wake the supervisor if it is about to enter or is blocked in `wait -n`.
  # TERM the most recently started child; cleanup() then stops the remaining
  # children in the frozen order (roslaunch -> gatekeeper -> roscore).
  if [ -n "$ROSLAUNCH_PID" ]; then
    kill -TERM "$ROSLAUNCH_PID" 2>/dev/null || true
  elif [ -n "$GATE_PID" ]; then
    kill -TERM "$GATE_PID" 2>/dev/null || true
  elif [ -n "$ROSCORE_PID" ]; then
    kill -TERM "$ROSCORE_PID" 2>/dev/null || true
  fi
  # A TERM sent to a just-forked child can be lost before it execs. This
  # short-lived helper always wakes `wait -n`, after which cleanup() stops
  # every child in the frozen order.
  ( sleep 0.3 ) &
}
trap on_stop TERM INT

# --- ROS environment ----------------------------------------------------------
for setup_path in \
  "/opt/ros/noetic/setup.bash" \
  "/ws_ros1/devel/setup.bash" \
  "/ws_ego/devel/setup.bash"
do
  setup_file="${SETUP_ROOT}${setup_path}"
  if [ ! -r "$setup_file" ]; then
    echo "run_system-deploy: missing ROS setup file: $setup_file" >&2
    exit 1
  fi
  # shellcheck disable=SC1090
  . "$setup_file"
done

# --- ROS networking (preserve the existing deployment convention) -------------
if [ -z "${ROS_MASTER_URI:-}" ]; then
  if [ -z "${ROS_IP:-}" ]; then
    ROS_IP="$(hostname -I | cut -d' ' -f1)"
    export ROS_IP
  fi
  export ROS_MASTER_URI="http://${ROS_IP}:11311"
  unset ROS_HOSTNAME
fi

# --- shutdown helpers ---------------------------------------------------------
stop_child() {
  local pid="$1"
  [ -n "$pid" ] || return 0
  kill -TERM "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  stop_child "$ROSLAUNCH_PID"
  stop_child "$GATE_PID"
  stop_child "$ROSCORE_PID"
}

# --- startup ------------------------------------------------------------------
if [ "$STOPPING" = "1" ]; then
  cleanup
  exit 0
fi

roscore &
ROSCORE_PID=$!

deadline_ns=$(( $(date +%s%N) + READINESS_TIMEOUT * 1000000000 ))
while :; do
  if [ "$STOPPING" = "1" ]; then
    cleanup
    exit 0
  fi
  if timeout 5 rosnode list >/dev/null 2>&1; then
    break
  fi
  if [ "$(date +%s%N)" -ge "$deadline_ns" ]; then
    echo "run_system-deploy: ROS master was not ready within ${READINESS_TIMEOUT}s" >&2
    cleanup
    exit 1
  fi
  sleep 0.2
done

if [ "$STOPPING" = "1" ]; then
  cleanup
  exit 0
fi

python3 "$GATEKEEPER" &
GATE_PID=$!

# Give gatekeeper a moment to bind its MAVLink sockets before mavros starts.
sleep 1

if [ "$STOPPING" = "1" ]; then
  cleanup
  exit 0
fi

roslaunch px4_ego_planner run_real.launch &
ROSLAUNCH_PID=$!

if [ "$STOPPING" = "1" ]; then
  cleanup
  exit 0
fi

# Test-only: deliver TERM deterministically in the window between the last
# STOPPING check and `wait -n` (regression coverage for the handled-signal race).
if [ "${PX4_EGO_TEST_SIGNAL_BEFORE_WAIT:-0}" = "1" ]; then
  kill -TERM $$
fi

# --- supervision --------------------------------------------------------------
wait -n
status=$?
if [ "$STOPPING" = "1" ]; then
  cleanup
  exit 0
fi

echo "run_system-deploy: managed process exited unexpectedly (status $status)" >&2
cleanup
exit 1
