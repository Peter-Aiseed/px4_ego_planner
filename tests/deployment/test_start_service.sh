#!/usr/bin/env bash
# RED tests for deploy/start_service.sh (host launcher boundary).
#
# Uses a fake docker CLI injected through PATH. Never touches real Docker.
# Test-only path overrides (approved seams):
#   PX4_EGO_TEST_HOST_WORKSPACE
#   PX4_EGO_TEST_HOST_LOG_DIR
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/../.." && pwd)"
START_SERVICE="$REPO_ROOT/deploy/start_service.sh"
FAKES="$HERE/fakes"

. "$HERE/helpers.sh"

echo "== start_service.sh =="

if [ ! -f "$START_SERVICE" ]; then
  fail "production script exists: $START_SERVICE"
  report "start_service.sh"
  exit 1
fi
pass "production script exists: $START_SERVICE"

TMPROOT="$(mktemp -d)"
trap 'rm -rf "$TMPROOT"' EXIT

setup_workspace() {
  local ws="$1" with_devel="${2:-1}"
  mkdir -p "$ws"
  if [ "$with_devel" = "1" ]; then
    mkdir -p "$ws/devel"
    : >"$ws/devel/setup.bash"
  fi
}

# run_launcher <case-dir> <container-state> [KEY=VAL ...]
# Sets DOCKER_LOG and RUN_STATUS.
run_launcher() {
  local case_dir="$1" state="$2"
  shift 2
  mkdir -p "$case_dir"
  DOCKER_LOG="$case_dir/docker.log"
  : >"$DOCKER_LOG"
  RUN_STATUS=0
  env \
    PATH="$FAKES:$PATH" \
    FAKE_DOCKER_LOG="$DOCKER_LOG" \
    FAKE_DOCKER_CONTAINER_STATE="$state" \
    "$@" \
    bash "$START_SERVICE" >"$case_dir/stdout" 2>"$case_dir/stderr" || RUN_STATUS=$?
}

# ---- case 1: happy path, absent container, log dir created -----------------
CASE="$TMPROOT/case1"; WS="$CASE/ws"; LOG="$CASE/log"
setup_workspace "$WS"
run_launcher "$CASE" absent \
  PX4_EGO_TEST_HOST_WORKSPACE="$WS" \
  PX4_EGO_TEST_HOST_LOG_DIR="$LOG"
assert_zero "case1: launcher succeeds" "$RUN_STATUS"
assert_dir_exists "$LOG" "case1: host log directory created"
assert_contains "$(cat "$DOCKER_LOG")" "docker run" "case1: docker run invoked"
assert_not_contains "$(cat "$DOCKER_LOG")" "docker rm" "case1: no docker rm when container absent"
assert_not_contains "$(cat "$DOCKER_LOG")" "docker stop" "case1: no docker stop"

# ---- case 2: missing workspace ---------------------------------------------
CASE="$TMPROOT/case2"; WS="$CASE/missing-ws"; LOG="$CASE/log"
run_launcher "$CASE" absent \
  PX4_EGO_TEST_HOST_WORKSPACE="$WS" \
  PX4_EGO_TEST_HOST_LOG_DIR="$LOG"
assert_nonzero "case2: missing workspace fails" "$RUN_STATUS"
assert_not_contains "$(cat "$DOCKER_LOG")" "docker run" "case2: docker run not reached"

# ---- case 3: missing devel/setup.bash --------------------------------------
CASE="$TMPROOT/case3"; WS="$CASE/ws"; LOG="$CASE/log"
setup_workspace "$WS" 0
run_launcher "$CASE" absent \
  PX4_EGO_TEST_HOST_WORKSPACE="$WS" \
  PX4_EGO_TEST_HOST_LOG_DIR="$LOG"
assert_nonzero "case3: missing devel/setup.bash fails" "$RUN_STATUS"
assert_not_contains "$(cat "$DOCKER_LOG")" "docker run" "case3: docker run not reached"

# ---- case 4: nested log directory created with mkdir -p --------------------
CASE="$TMPROOT/case4"; WS="$CASE/ws"; LOG="$CASE/a/b/log"
setup_workspace "$WS"
run_launcher "$CASE" absent \
  PX4_EGO_TEST_HOST_WORKSPACE="$WS" \
  PX4_EGO_TEST_HOST_LOG_DIR="$LOG"
assert_zero "case4: launcher succeeds" "$RUN_STATUS"
assert_dir_exists "$LOG" "case4: nested log directory created"

# ---- case 5: docker daemon unreachable -------------------------------------
CASE="$TMPROOT/case5"; WS="$CASE/ws"; LOG="$CASE/log"
setup_workspace "$WS"
run_launcher "$CASE" absent \
  PX4_EGO_TEST_HOST_WORKSPACE="$WS" \
  PX4_EGO_TEST_HOST_LOG_DIR="$LOG" \
  FAKE_DOCKER_INFO_FAIL=1
assert_nonzero "case5: docker daemon failure fails" "$RUN_STATUS"
assert_not_contains "$(cat "$DOCKER_LOG")" "docker run" "case5: docker run not reached"

# ---- case 6: image missing -------------------------------------------------
CASE="$TMPROOT/case6"; WS="$CASE/ws"; LOG="$CASE/log"
setup_workspace "$WS"
run_launcher "$CASE" absent \
  PX4_EGO_TEST_HOST_WORKSPACE="$WS" \
  PX4_EGO_TEST_HOST_LOG_DIR="$LOG" \
  FAKE_DOCKER_IMAGE_MISSING=1
assert_nonzero "case6: missing image fails" "$RUN_STATUS"
assert_not_contains "$(cat "$DOCKER_LOG")" "docker run" "case6: docker run not reached"

# ---- case 7: same-name container running -----------------------------------
CASE="$TMPROOT/case7"; WS="$CASE/ws"; LOG="$CASE/log"
setup_workspace "$WS"
run_launcher "$CASE" running \
  PX4_EGO_TEST_HOST_WORKSPACE="$WS" \
  PX4_EGO_TEST_HOST_LOG_DIR="$LOG"
assert_nonzero "case7: running container fails the launcher" "$RUN_STATUS"
assert_not_contains "$(cat "$DOCKER_LOG")" "docker stop" "case7: running container not stopped"
assert_not_contains "$(cat "$DOCKER_LOG")" "docker rm" "case7: running container not removed"
assert_not_contains "$(cat "$DOCKER_LOG")" "docker run" "case7: no second container started"

# ---- case 8: same-name container stopped -----------------------------------
CASE="$TMPROOT/case8"; WS="$CASE/ws"; LOG="$CASE/log"
setup_workspace "$WS"
run_launcher "$CASE" stopped \
  PX4_EGO_TEST_HOST_WORKSPACE="$WS" \
  PX4_EGO_TEST_HOST_LOG_DIR="$LOG"
assert_zero "case8: launcher succeeds" "$RUN_STATUS"
assert_contains "$(cat "$DOCKER_LOG")" "docker rm px4-ego" "case8: stopped container removed"
assert_contains "$(cat "$DOCKER_LOG")" "docker run" "case8: startup continues"
assert_order "$DOCKER_LOG" "docker rm px4-ego" "docker run" "case8: rm happens before run"

# ---- case 8b: docker rm failure aborts startup -----------------------------
CASE="$TMPROOT/case8b"; WS="$CASE/ws"; LOG="$CASE/log"
setup_workspace "$WS"
run_launcher "$CASE" stopped \
  PX4_EGO_TEST_HOST_WORKSPACE="$WS" \
  PX4_EGO_TEST_HOST_LOG_DIR="$LOG" \
  FAKE_DOCKER_RM_FAIL=1
assert_nonzero "case8b: rm failure fails the launcher" "$RUN_STATUS"
assert_not_contains "$(cat "$DOCKER_LOG")" "docker run" "case8b: docker run not reached"

# ---- case 9: absent container ----------------------------------------------
CASE="$TMPROOT/case9"; WS="$CASE/ws"; LOG="$CASE/log"
setup_workspace "$WS"
run_launcher "$CASE" absent \
  PX4_EGO_TEST_HOST_WORKSPACE="$WS" \
  PX4_EGO_TEST_HOST_LOG_DIR="$LOG"
assert_zero "case9: launcher succeeds" "$RUN_STATUS"
assert_contains "$(cat "$DOCKER_LOG")" "docker run" "case9: docker run invoked"
assert_not_contains "$(cat "$DOCKER_LOG")" "docker rm" "case9: no docker rm"

# ---- case 10-13: exact docker run contract ---------------------------------
CASE="$TMPROOT/case10"; WS="$CASE/ws"; LOG="$CASE/log"
setup_workspace "$WS"
run_launcher "$CASE" absent \
  PX4_EGO_TEST_HOST_WORKSPACE="$WS" \
  PX4_EGO_TEST_HOST_LOG_DIR="$LOG"
assert_zero "case10: launcher succeeds" "$RUN_STATUS"
RUNLINE="$(grep 'docker run' "$DOCKER_LOG" | head -1)"
assert_contains "$RUNLINE" " --rm " "case10: --rm present"
assert_contains "$RUNLINE" " --init " "case10: --init present"
assert_contains "$RUNLINE" " --name px4-ego " "case10: --name px4-ego present"
assert_contains "$RUNLINE" " --network host " "case10: --network host present"
assert_contains "$RUNLINE" " --privileged " "case10: --privileged present"
assert_contains "$RUNLINE" " -v /dev:/dev " "case10: /dev bind mount present"
assert_contains "$RUNLINE" " -v $WS:/ws_ego " "case10: workspace mount present"
assert_contains "$RUNLINE" " -v $LOG:/var/log/px4-ego/ros " "case10: log mount present"
assert_contains "$RUNLINE" " -e ROS_LOG_DIR=/var/log/px4-ego/ros " "case10: ROS_LOG_DIR present"
assert_contains "$RUNLINE" " -e PYTHONUNBUFFERED=1 " "case10: PYTHONUNBUFFERED present"
assert_contains "$RUNLINE" "px4-ego-noetic:nx-1.0" "case10: image present"
assert_contains "$RUNLINE" "/bin/bash /ws_ego/src/px4_ego_planner/deploy/run_system-deploy.sh" \
  "case10: runtime script command present"

assert_not_contains " $RUNLINE " " -d " "case11: no detached mode"
assert_not_contains " $RUNLINE " " -it " "case12: no interactive tty"
assert_not_contains "$RUNLINE" "DISPLAY" "case13: no DISPLAY"
assert_not_contains "$RUNLINE" "Xauthority" "case13: no Xauthority"
assert_not_contains "$RUNLINE" ".X11-unix" "case13: no X11 socket"
assert_not_contains "$RUNLINE" "LIBGL" "case13: no LIBGL"
assert_not_contains "$RUNLINE" "xhost" "case13: no xhost"
assert_not_contains "$RUNLINE" "/dev/dri" "case13: no /dev/dri"
assert_not_contains "$(cat "$DOCKER_LOG")" "rm -f" "case13: docker rm -f never used"

# ---- case 14: docker run exit status propagates ----------------------------
CASE="$TMPROOT/case14"; WS="$CASE/ws"; LOG="$CASE/log"
setup_workspace "$WS"
run_launcher "$CASE" absent \
  PX4_EGO_TEST_HOST_WORKSPACE="$WS" \
  PX4_EGO_TEST_HOST_LOG_DIR="$LOG" \
  FAKE_DOCKER_RUN_EXIT=7
assert_status_eq "case14: docker exit status propagates" "7" "$RUN_STATUS"

report "start_service.sh"
