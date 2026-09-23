#!/usr/bin/env bash
# RED tests for deploy/run_system-deploy.sh (in-container runtime boundary).
#
# roscore/rosnode/roslaunch/python3/hostname are fakes injected through PATH.
# No real ROS, Docker, or systemd is used.
#
# Test-only seams (must be honored by the implementation):
#   PX4_EGO_TEST_SETUP_ROOT         temp root containing opt/ros/noetic,
#                                   ws_ros1/devel and ws_ego/devel setup files
#   PX4_EGO_TEST_READINESS_TIMEOUT  short readiness deadline for tests
set -u

# Job control makes background jobs keep the default SIGINT disposition, so
# the script under test can actually receive SIGINT (POSIX async-job rule).
set -m

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/../.." && pwd)"
RUN_SCRIPT="$REPO_ROOT/deploy/run_system-deploy.sh"
FAKES="$HERE/fakes"

. "$HERE/helpers.sh"

echo "== run_system-deploy.sh =="

if [ ! -f "$RUN_SCRIPT" ]; then
  fail "production script exists: $RUN_SCRIPT"
  report "run_system-deploy.sh"
  exit 1
fi
pass "production script exists: $RUN_SCRIPT"

TMPROOT="$(mktemp -d)"
trap 'rm -rf "$TMPROOT"' EXIT

make_setup_root() {
  local root="$1" omit="${2:-}"
  mkdir -p "$root/opt/ros/noetic" "$root/ws_ros1/devel" "$root/ws_ego/devel"
  if [ "$omit" != "base" ]; then
    printf 'printf "SOURCE /opt/ros/noetic\\n" >>"${FAKE_EVENTS:-/dev/null}"\n' \
      >"$root/opt/ros/noetic/setup.bash"
  fi
  if [ "$omit" != "ros1" ]; then
    printf 'printf "SOURCE /ws_ros1/devel\\n" >>"${FAKE_EVENTS:-/dev/null}"\n' \
      >"$root/ws_ros1/devel/setup.bash"
  fi
  if [ "$omit" != "ws_ego" ]; then
    printf 'printf "SOURCE /ws_ego/devel\\n" >>"${FAKE_EVENTS:-/dev/null}"\n' \
      >"$root/ws_ego/devel/setup.bash"
  fi
}

# start_runtime <case-dir> [KEY=VAL ...]
# Starts run_system-deploy.sh in the background. Sets EVENTS and SCRIPT_PID.
start_runtime() {
  local case_dir="$1"
  shift
  mkdir -p "$case_dir"
  EVENTS="$case_dir/events.log"
  : >"$EVENTS"
  env -u ROS_MASTER_URI -u ROS_IP -u ROS_HOSTNAME \
    PATH="$FAKES:$PATH" \
    FAKE_EVENTS="$EVENTS" \
    PX4_EGO_TEST_READINESS_TIMEOUT=1 \
    "$@" \
    bash "$RUN_SCRIPT" >"$case_dir/stdout" 2>"$case_dir/stderr" &
  SCRIPT_PID=$!
}

# ---- case 1: missing setup file --------------------------------------------
ROOT="$TMPROOT/setup-missing"; make_setup_root "$ROOT" ws_ego
start_runtime "$TMPROOT/case1" PX4_EGO_TEST_SETUP_ROOT="$ROOT"
finish_runtime "$SCRIPT_PID" 5; status=$?
assert_nonzero "case1: missing setup file fails" "$status"
assert_not_contains "$(cat "$EVENTS")" "START roscore" "case1: no managed child starts"

# ---- case 2: correct startup order -----------------------------------------
ROOT="$TMPROOT/setup-ok"; make_setup_root "$ROOT"
start_runtime "$TMPROOT/case2" PX4_EGO_TEST_SETUP_ROOT="$ROOT" FAKE_ROSNODE_SUCCEED_ON=1
if wait_for_event "$EVENTS" "START roslaunch" 5; then
  pass "case2: roslaunch started"
else
  fail "case2: roslaunch started (events: $(tr '\n' '|' <"$EVENTS"))"
fi
assert_order "$EVENTS" "SOURCE /opt/ros/noetic" "SOURCE /ws_ros1/devel" \
  "case2: base sourced before ros1"
assert_order "$EVENTS" "SOURCE /ws_ros1/devel" "SOURCE /ws_ego/devel" \
  "case2: ros1 sourced before ws_ego"
assert_order "$EVENTS" "SOURCE /ws_ego/devel" "START roscore" \
  "case2: setup sourced before roscore"
assert_order "$EVENTS" "START roscore" "PROBE rosnode" \
  "case2: readiness probed after roscore"
assert_order "$EVENTS" "PROBE rosnode" "START gatekeeper" \
  "case2: gatekeeper starts after readiness"
assert_order "$EVENTS" "START gatekeeper" "START roslaunch" \
  "case2: roslaunch starts after gatekeeper"
kill -TERM "$SCRIPT_PID"
finish_runtime "$SCRIPT_PID" 5; status=$?
assert_zero "case2: intentional TERM exits 0" "$status"

# ---- case 3: readiness retries ---------------------------------------------
ROOT="$TMPROOT/setup-ok"
start_runtime "$TMPROOT/case3" PX4_EGO_TEST_SETUP_ROOT="$ROOT" FAKE_ROSNODE_SUCCEED_ON=3
if wait_for_event "$EVENTS" "START gatekeeper" 5; then
  pass "case3: gatekeeper started after retries"
else
  fail "case3: gatekeeper started after retries (events: $(tr '\n' '|' <"$EVENTS"))"
fi
assert_order "$EVENTS" "PROBE rosnode 1" "START gatekeeper" \
  "case3: no gatekeeper before first probe"
assert_order "$EVENTS" "PROBE rosnode 3" "START gatekeeper" \
  "case3: gatekeeper waits for readiness success"
kill -TERM "$SCRIPT_PID"
finish_runtime "$SCRIPT_PID" 5; status=$?
assert_zero "case3: intentional TERM exits 0" "$status"

# ---- case 4: readiness timeout ---------------------------------------------
ROOT="$TMPROOT/setup-ok"
start_runtime "$TMPROOT/case4" PX4_EGO_TEST_SETUP_ROOT="$ROOT" \
  FAKE_ROSNODE_ALWAYS_FAIL=1 PX4_EGO_TEST_READINESS_TIMEOUT=1
finish_runtime "$SCRIPT_PID" 5; status=$?
assert_nonzero "case4: readiness timeout fails" "$status"
assert_not_contains "$(cat "$EVENTS")" "START gatekeeper" "case4: gatekeeper never starts"
assert_not_contains "$(cat "$EVENTS")" "START roslaunch" "case4: roslaunch never starts"
assert_contains "$(cat "$EVENTS")" "TERM roscore" "case4: roscore cleaned up"

# ---- case 5: TERM during readiness (stop during startup) -------------------
ROOT="$TMPROOT/setup-ok"
start_runtime "$TMPROOT/case5" PX4_EGO_TEST_SETUP_ROOT="$ROOT" \
  FAKE_ROSNODE_ALWAYS_FAIL=1 PX4_EGO_TEST_READINESS_TIMEOUT=30
wait_for_event "$EVENTS" "START roscore" 5 || true
sleep 0.3
kill -TERM "$SCRIPT_PID"
finish_runtime "$SCRIPT_PID" 5; status=$?
assert_zero "case5: TERM during readiness exits 0" "$status"
assert_not_contains "$(cat "$EVENTS")" "START gatekeeper" "case5: no later process starts"
assert_not_contains "$(cat "$EVENTS")" "START roslaunch" "case5: no later process starts"
assert_contains "$(cat "$EVENTS")" "TERM roscore" "case5: roscore cleaned up"

# ---- case 6: stays alive after startup -------------------------------------
ROOT="$TMPROOT/setup-ok"
start_runtime "$TMPROOT/case6" PX4_EGO_TEST_SETUP_ROOT="$ROOT" FAKE_ROSNODE_SUCCEED_ON=1
wait_for_event "$EVENTS" "START roslaunch" 5 || true
sleep 0.3
if kill -0 "$SCRIPT_PID" 2>/dev/null; then
  pass "case6: supervisor stays alive while runtime is active"
else
  fail "case6: supervisor stays alive while runtime is active"
fi
kill -TERM "$SCRIPT_PID"
finish_runtime "$SCRIPT_PID" 5; status=$?
assert_zero "case6: intentional TERM exits 0" "$status"

# ---- case 7: SIGTERM after startup, shutdown order -------------------------
ROOT="$TMPROOT/setup-ok"
start_runtime "$TMPROOT/case7" PX4_EGO_TEST_SETUP_ROOT="$ROOT" FAKE_ROSNODE_SUCCEED_ON=1
wait_for_event "$EVENTS" "START roslaunch" 5 || true
kill -TERM "$SCRIPT_PID"
finish_runtime "$SCRIPT_PID" 5; status=$?
assert_zero "case7: intentional TERM exits 0" "$status"
assert_order "$EVENTS" "TERM roslaunch" "TERM gatekeeper" \
  "case7: roslaunch stopped before gatekeeper"
assert_order "$EVENTS" "TERM gatekeeper" "TERM roscore" \
  "case7: gatekeeper stopped before roscore"

# ---- case 8: SIGINT --------------------------------------------------------
ROOT="$TMPROOT/setup-ok"
start_runtime "$TMPROOT/case8" PX4_EGO_TEST_SETUP_ROOT="$ROOT" FAKE_ROSNODE_SUCCEED_ON=1
wait_for_event "$EVENTS" "START roslaunch" 5 || true
kill -INT "$SCRIPT_PID"
finish_runtime "$SCRIPT_PID" 5; status=$?
assert_zero "case8: intentional INT exits 0" "$status"
assert_order "$EVENTS" "TERM roslaunch" "TERM gatekeeper" \
  "case8: roslaunch stopped before gatekeeper"
assert_order "$EVENTS" "TERM gatekeeper" "TERM roscore" \
  "case8: gatekeeper stopped before roscore"

# ---- case 9: unexpected managed process exit -------------------------------
ROOT="$TMPROOT/setup-ok"
start_runtime "$TMPROOT/case9" PX4_EGO_TEST_SETUP_ROOT="$ROOT" \
  FAKE_ROSNODE_SUCCEED_ON=1 FAKE_ROSLAUNCH_EXIT_IMMEDIATELY=1
finish_runtime "$SCRIPT_PID" 5; status=$?
assert_nonzero "case9: unexpected child exit fails" "$status"
assert_contains "$(cat "$EVENTS")" "TERM gatekeeper" "case9: gatekeeper cleaned up"
assert_contains "$(cat "$EVENTS")" "TERM roscore" "case9: roscore cleaned up"

# ---- case 10: no legacy cleanup --------------------------------------------
CONTENT="$(cat "$RUN_SCRIPT")"
assert_not_contains "$CONTENT" "pkill" "case10: no pkill"
assert_not_contains "$CONTENT" "kill -9" "case10: no kill -9"
assert_not_contains "$CONTENT" "SIM_PID" "case10: no SIM_PID"
assert_not_contains "$CONTENT" "gzserver" "case10: no gzserver cleanup"

# ---- case 11: production container paths -----------------------------------
assert_contains "$CONTENT" "/opt/ros/noetic/setup.bash" "case11: base setup path present"
assert_contains "$CONTENT" "/ws_ros1/devel/setup.bash" "case11: ws_ros1 setup path present"
assert_contains "$CONTENT" "/ws_ego/devel/setup.bash" "case11: ws_ego setup path present"
assert_contains "$CONTENT" "/ws_ego/src/px4_ego_planner/scripts/gatekeeper.py" \
  "case11: gatekeeper path present"
assert_contains "$CONTENT" "roslaunch px4_ego_planner run_real.launch" \
  "case11: launch command present"

# ---- case 12: preset ROS_MASTER_URI is preserved ---------------------------
ROOT="$TMPROOT/setup-ok"
start_runtime "$TMPROOT/case12" PX4_EGO_TEST_SETUP_ROOT="$ROOT" \
  FAKE_ROSNODE_SUCCEED_ON=1 ROS_MASTER_URI=http://preset:11311
wait_for_event "$EVENTS" "START roscore" 5 || true
ROSCORE_LINE="$(grep 'START roscore' "$EVENTS" | head -1)"
assert_contains "$ROSCORE_LINE" "ROS_MASTER_URI=http://preset:11311" \
  "case12: preset ROS_MASTER_URI preserved"
assert_not_contains "$(cat "$EVENTS")" "CALL hostname" \
  "case12: hostname not consulted when ROS_MASTER_URI preset"
kill -TERM "$SCRIPT_PID"
finish_runtime "$SCRIPT_PID" 5; status=$?
assert_zero "case12: intentional TERM exits 0" "$status"

# ---- case 13: preset ROS_IP is preserved -----------------------------------
ROOT="$TMPROOT/setup-ok"
start_runtime "$TMPROOT/case13" PX4_EGO_TEST_SETUP_ROOT="$ROOT" \
  FAKE_ROSNODE_SUCCEED_ON=1 ROS_IP=192.0.2.10
wait_for_event "$EVENTS" "START roscore" 5 || true
ROSCORE_LINE="$(grep 'START roscore' "$EVENTS" | head -1)"
assert_contains "$ROSCORE_LINE" "ROS_IP=192.0.2.10" "case13: preset ROS_IP preserved"
assert_contains "$ROSCORE_LINE" "ROS_MASTER_URI=http://192.0.2.10:11311" \
  "case13: ROS_MASTER_URI derived from preset ROS_IP"
assert_not_contains "$(cat "$EVENTS")" "CALL hostname" \
  "case13: hostname not consulted when ROS_IP preset"
kill -TERM "$SCRIPT_PID"
finish_runtime "$SCRIPT_PID" 5; status=$?
assert_zero "case13: intentional TERM exits 0" "$status"

# ---- case 14: unset networking is derived from hostname --------------------
ROOT="$TMPROOT/setup-ok"
start_runtime "$TMPROOT/case14" PX4_EGO_TEST_SETUP_ROOT="$ROOT" \
  FAKE_ROSNODE_SUCCEED_ON=1 FAKE_HOSTNAME_IP=10.1.2.3
wait_for_event "$EVENTS" "START roscore" 5 || true
ROSCORE_LINE="$(grep 'START roscore' "$EVENTS" | head -1)"
assert_contains "$ROSCORE_LINE" "ROS_IP=10.1.2.3" "case14: ROS_IP derived from hostname"
assert_contains "$ROSCORE_LINE" "ROS_MASTER_URI=http://10.1.2.3:11311" \
  "case14: ROS_MASTER_URI derived from hostname"
assert_contains "$(cat "$EVENTS")" "CALL hostname -I" "case14: hostname consulted when unset"
kill -TERM "$SCRIPT_PID"
finish_runtime "$SCRIPT_PID" 5; status=$?
assert_zero "case14: intentional TERM exits 0" "$status"

# ---- case 15: TERM handled in the window just before wait -n ---------------
# PX4_EGO_TEST_SIGNAL_BEFORE_WAIT makes the script deliver TERM to itself after
# the final STOPPING check and before `wait -n`. The freshly forked roslaunch
# fake can be killed before it installs its TERM trap, so only the children
# that are provably trapped are asserted here; the key property is that the
# supervisor is woken by the handler and exits 0 without a second signal.
ROOT="$TMPROOT/setup-ok"
start_runtime "$TMPROOT/case15" PX4_EGO_TEST_SETUP_ROOT="$ROOT" \
  FAKE_ROSNODE_SUCCEED_ON=1 PX4_EGO_TEST_SIGNAL_BEFORE_WAIT=1
finish_runtime "$SCRIPT_PID" 5; status=$?
assert_zero "case15: signal before wait -n exits 0 without a second signal" "$status"
assert_contains "$(cat "$EVENTS")" "TERM gatekeeper" "case15: gatekeeper TERMed during cleanup"
assert_contains "$(cat "$EVENTS")" "TERM roscore" "case15: roscore TERMed during cleanup"
assert_order "$EVENTS" "TERM gatekeeper" "TERM roscore" "case15: cleanup order preserved"

# ---- case 16: malformed readiness timeout is rejected ----------------------
ROOT="$TMPROOT/setup-ok"
start_runtime "$TMPROOT/case16" PX4_EGO_TEST_SETUP_ROOT="$ROOT" \
  PX4_EGO_TEST_READINESS_TIMEOUT=abc
finish_runtime "$SCRIPT_PID" 5; status=$?
assert_nonzero "case16: malformed readiness timeout fails fast" "$status"
assert_not_contains "$(cat "$EVENTS")" "START roscore" "case16: no managed child starts"

report "run_system-deploy.sh"
