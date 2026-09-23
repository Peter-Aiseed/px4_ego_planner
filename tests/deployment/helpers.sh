#!/usr/bin/env bash
# Minimal assertion helpers for the deployment tests.
# Sourced by tests/deployment/test_*.sh. No external test framework.

PASS_COUNT=0
FAIL_COUNT=0

pass() {
  PASS_COUNT=$((PASS_COUNT + 1))
  printf '  PASS: %s\n' "$1"
}

fail() {
  FAIL_COUNT=$((FAIL_COUNT + 1))
  printf '  FAIL: %s\n' "$1" >&2
}

assert_file_exists() {
  if [ -f "$1" ]; then pass "$2"; else fail "$2 (missing file: $1)"; fi
}

assert_dir_exists() {
  if [ -d "$1" ]; then pass "$2"; else fail "$2 (missing directory: $1)"; fi
}

assert_contains() {
  case "$1" in
    *"$2"*) pass "$3" ;;
    *) fail "$3 (expected to contain: $2)" ;;
  esac
}

assert_not_contains() {
  case "$1" in
    *"$2"*) fail "$3 (unexpectedly contains: $2)" ;;
    *) pass "$3" ;;
  esac
}

assert_status_eq() {
  if [ "$2" = "$3" ]; then
    pass "$1"
  else
    fail "$1 (expected status '$2', got '$3')"
  fi
}

assert_nonzero() {
  case "$2" in
    ''|*[!0-9]*) fail "$1 (expected nonzero status, got '$2')"; return 1 ;;
    0) fail "$1 (expected nonzero status, got 0)"; return 1 ;;
    124) fail "$1 (watchdog terminated the process; not a genuine exit)"; return 1 ;;
    *) pass "$1"; return 0 ;;
  esac
}

assert_zero() {
  if [ "$2" = "0" ]; then pass "$1"; else fail "$1 (expected status 0, got '$2')"; fi
}

report() {
  local name="$1"
  printf '\n%s: %d passed, %d failed\n' "$name" "$PASS_COUNT" "$FAIL_COUNT"
  [ "$FAIL_COUNT" -eq 0 ]
}

# Wait until <pattern> appears in <file>, up to <seconds>.
wait_for_event() {
  local file="$1" pattern="$2" timeout_s="${3:-5}" i=0 max
  max=$((timeout_s * 20))
  while [ "$i" -lt "$max" ]; do
    if [ -f "$file" ] && grep -q -- "$pattern" "$file"; then
      return 0
    fi
    sleep 0.05
    i=$((i + 1))
  done
  return 1
}

# Wait for a background process to exit, with a SIGKILL watchdog.
# Usage: finish_runtime <pid> <timeout_s>
# Returns the child's exit status, or 137 if the watchdog had to kill it.
finish_runtime() {
  local pid="$1" timeout_s="${2:-5}" watchdog status marker
  marker="$(mktemp)"
  ( sleep "$timeout_s"; echo fired >"$marker"; kill -KILL "$pid" 2>/dev/null ) &
  watchdog=$!
  wait "$pid"
  status=$?
  kill "$watchdog" 2>/dev/null || true
  wait "$watchdog" 2>/dev/null || true
  # 124 is reserved for "the watchdog had to kill the process", so tests can
  # distinguish it from a genuine nonzero exit.
  if [ -s "$marker" ] && [ "$status" = "137" ]; then
    rm -f "$marker"
    return 124
  fi
  rm -f "$marker"
  return "$status"
}

# First line number in <file> matching <pattern> (empty if absent).
line_of() {
  grep -n -- "$2" "$1" 2>/dev/null | head -1 | cut -d: -f1
}

# Assert <first> appears before <second> in <file>.
assert_order() {
  local file="$1" first="$2" second="$3" desc="$4" a b
  a="$(line_of "$file" "$first")"
  b="$(line_of "$file" "$second")"
  if [ -n "$a" ] && [ -n "$b" ] && [ "$a" -lt "$b" ]; then
    pass "$desc"
  else
    fail "$desc (line '$first'=$a, line '$second'=$b)"
  fi
}
