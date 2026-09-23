#!/usr/bin/env bash
# Tests for tests/deployment/helpers.sh itself.
#
# Proves that a watchdog termination cannot be mistaken for a genuine nonzero
# exit: finish_runtime returns the reserved sentinel 124 and assert_nonzero
# rejects it.
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$HERE/helpers.sh"

echo "== helpers.sh =="

# A hung process must yield the sentinel 124, not its SIGKILL status.
sleep 30 >/dev/null 2>&1 &
hung_pid=$!
finish_runtime "$hung_pid" 1
watchdog_status=$?
if [ "$watchdog_status" = "124" ]; then
  pass "watchdog termination returns sentinel 124"
else
  fail "watchdog termination returns sentinel 124 (got '$watchdog_status')"
fi

# A process that exits by itself keeps its own status.
( exit 7 ) &
quick_pid=$!
finish_runtime "$quick_pid" 5
quick_status=$?
if [ "$quick_status" = "7" ]; then
  pass "self-terminated process keeps its own exit status"
else
  fail "self-terminated process keeps its own exit status (got '$quick_status')"
fi

# assert_nonzero must reject the sentinel but still accept a genuine failure.
if ( assert_nonzero "watchdog sentinel must not satisfy assert_nonzero" "124" ) >/dev/null 2>&1; then
  fail "assert_nonzero accepted the watchdog sentinel 124"
else
  pass "assert_nonzero rejects the watchdog sentinel 124"
fi

if ( assert_nonzero "genuine nonzero is accepted" "1" ) >/dev/null 2>&1; then
  pass "assert_nonzero accepts a genuine nonzero exit"
else
  fail "assert_nonzero rejected a genuine nonzero exit"
fi

report "helpers.sh"
