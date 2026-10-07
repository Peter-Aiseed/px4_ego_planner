#!/usr/bin/env bash
# RED tests for deploy/px4-ego.service (static text assertions only).
# No systemd is invoked or installed.
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/../.." && pwd)"
UNIT="$REPO_ROOT/deploy/px4-ego.service"

. "$HERE/helpers.sh"

echo "== px4-ego.service =="

if [ ! -f "$UNIT" ]; then
  fail "production unit exists: $UNIT"
  report "px4-ego.service"
  exit 1
fi
pass "production unit exists: $UNIT"

CONTENT="$(cat "$UNIT")"

assert_contains "$CONTENT" "Requires=docker.service" "unit requires docker.service"
assert_contains "$CONTENT" "After=docker.service" "unit starts after docker.service"
assert_contains "$CONTENT" "Type=simple" "unit type is simple"
assert_contains "$CONTENT" \
  "ExecStart=/usr/bin/bash /home/nvidia/ego_ws/src/px4_ego_planner/deploy/start_service.sh" \
  "unit ExecStart points at the host launcher"
assert_contains "$CONTENT" "ExecStop=-/usr/bin/docker stop -t 30 px4-ego" \
  "unit ExecStop stops the container with a 30s timeout"
assert_contains "$CONTENT" "Restart=on-failure" "unit restarts on failure"
assert_contains "$CONTENT" "RestartSec=5" "unit restart delay is 5 seconds"
assert_contains "$CONTENT" "WantedBy=multi-user.target" "unit is enabled for multi-user.target"

TIMEOUT_VALUE="$(sed -n 's/^TimeoutStopSec=\([0-9][0-9]*\).*/\1/p' "$UNIT" | head -1)"
if [ -n "$TIMEOUT_VALUE" ] && [ "$TIMEOUT_VALUE" -gt 30 ]; then
  pass "unit TimeoutStopSec > 30 (found ${TIMEOUT_VALUE})"
else
  fail "unit TimeoutStopSec > 30 (found '${TIMEOUT_VALUE:-none}')"
fi

assert_not_contains "$CONTENT" "User=" "unit has no User="
assert_not_contains "$CONTENT" "Group=" "unit has no Group="
assert_not_contains "$CONTENT" "docker exec" "unit does not use docker exec"
assert_not_contains "$CONTENT" "tmux" "unit does not use tmux"
assert_not_contains "$CONTENT" "docker run" "unit does not start Docker directly"

report "px4-ego.service"
