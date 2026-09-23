#!/usr/bin/env bash
# Deployment-layer test runner.
#
# These tests exercise the future deploy/ scripts through fakes only:
# no real Docker, ROS, systemd, or hardware is used.
#
# A failure here means a real regression: all behavioral assertions execute
# against the deploy/ scripts through the fakes.
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

failed=0

for test_file in "$HERE"/test_*.sh; do
  [ -e "$test_file" ] || continue
  printf '===================================================================\n'
  printf 'Running %s\n' "$(basename "$test_file")"
  if bash "$test_file"; then
    printf 'RESULT: PASS (%s)\n' "$(basename "$test_file")"
  else
    printf 'RESULT: FAIL (%s)\n' "$(basename "$test_file")"
    failed=$((failed + 1))
  fi
done

printf '===================================================================\n'
if [ "$failed" -eq 0 ]; then
  printf 'ALL TEST FILES PASSED\n'
  exit 0
fi
printf '%d TEST FILE(S) FAILED\n' "$failed"
exit 1
