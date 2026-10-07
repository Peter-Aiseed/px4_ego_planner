#!/bin/bash
# Host-side launcher for the px4-ego deployment service.
#
# Invoked by deploy/px4-ego.service (as root). Validates the deployment
# prerequisites, applies the same-name container policy, then execs an
# attached, disposable Docker container so Docker's exit status propagates
# to systemd.
#
# This is the production workflow. scripts/run-docker.sh remains the
# development launcher and is intentionally untouched.

IMAGE="px4-ego-noetic:nx-1.0"
CONTAINER_NAME="px4-ego"

# Test-only path overrides (never deployment configuration).
HOST_WORKSPACE="${PX4_EGO_TEST_HOST_WORKSPACE:-/home/nvidia/ego_ws}"
HOST_LOG_DIR="${PX4_EGO_TEST_HOST_LOG_DIR:-/home/nvidia/ego_ws/log}"

CONTAINER_LOG_DIR="/var/log/px4-ego/ros"
RUNTIME_SCRIPT="/ws_ego/src/px4_ego_planner/deploy/run_system-deploy.sh"

fail() {
  echo "start_service: $*" >&2
  exit 1
}

# --- prerequisites ------------------------------------------------------------
[ -d "$HOST_WORKSPACE" ] || fail "host workspace not found: $HOST_WORKSPACE"
[ -f "$HOST_WORKSPACE/devel/setup.bash" ] || fail "workspace is not built: missing $HOST_WORKSPACE/devel/setup.bash"

mkdir -p "$HOST_LOG_DIR" || fail "cannot create log directory: $HOST_LOG_DIR"

docker info >/dev/null 2>&1 || fail "Docker daemon is not reachable"
docker image inspect "$IMAGE" >/dev/null 2>&1 || fail "Docker image not found: $IMAGE"

# --- same-name container policy ----------------------------------------------
container_state="$(docker container inspect --format '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null)" || container_state=""

case "$container_state" in
  true)
    fail "container '$CONTAINER_NAME' is already running; refusing to stop, remove, or replace it"
    ;;
  false)
    docker rm "$CONTAINER_NAME" >/dev/null || fail "cannot remove stopped container: $CONTAINER_NAME"
    ;;
esac

# --- start the service container ---------------------------------------------
exec docker run \
  --rm \
  --init \
  --name "$CONTAINER_NAME" \
  --network host \
  --privileged \
  -v /dev:/dev \
  -v "$HOST_WORKSPACE:/ws_ego" \
  -v "$HOST_LOG_DIR:$CONTAINER_LOG_DIR" \
  -e ROS_LOG_DIR="$CONTAINER_LOG_DIR" \
  -e PYTHONUNBUFFERED=1 \
  "$IMAGE" \
  /bin/bash "$RUNTIME_SCRIPT"
