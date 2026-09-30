#!/usr/bin/env bash
# Build MAVROS 2.14.0 + the two upstream "Promise already satisfied" fixes
# into an overlay at ~/mavros_ws, leaving the apt install untouched.
#
#   patches/mavros/0001  param.cpp   (upstream 6ea0636e, PR #2093)
#   patches/mavros/0002  command.cpp (upstream 97a14ae1, issue #2159)
#
# On 2026-09-30 an unpatched command.cpp aborted mavros_node when PX4 sent
# a duplicate COMMAND_ACK for DO_SET_MODE. rover.launch.py uses this
# overlay only when its mavros_node exists, so a failed or partial build
# falls back to /opt/ros/humble.
#
# Run on the Jetson as flash (no sudo). Build is ~20-40 min; run it with
# the rover parked. Restart the stack afterwards to pick it up.
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${MAVROS_OVERLAY_WS:-$HOME/mavros_ws}"
TAG="2.14.0"
JOBS="${MAVROS_BUILD_JOBS:-2}"

installed="$(dpkg-query -W -f='${Version}' ros-humble-mavros 2>/dev/null || true)"
case "$installed" in
  2.14.0*) ;;
  *)
    echo "apt ros-humble-mavros is '$installed', patches target $TAG; stop." >&2
    exit 1
    ;;
esac

# shellcheck disable=SC1091
set +u; source /opt/ros/humble/setup.bash; set -u

mkdir -p "$WS/src"
if [ ! -d "$WS/src/mavros/.git" ]; then
  git clone --branch "$TAG" --depth 1 https://github.com/mavlink/mavros.git "$WS/src/mavros"
fi

cd "$WS/src/mavros"
git reset --hard -q "$TAG"
git clean -fdq
git -c user.name=dyx -c user.email=dyx@localhost am -q "$REPO_DIR"/patches/mavros/*.patch
git log --oneline -3

# Only the mavros package: mavros_msgs / mavros_extras stay from apt
# (same 2.14.0 source, so the plugin ABI matches).
touch "$WS/src/mavros/libmavconn/COLCON_IGNORE" \
      "$WS/src/mavros/mavros_msgs/COLCON_IGNORE" \
      "$WS/src/mavros/mavros_extras/COLCON_IGNORE" \
      "$WS/src/mavros/mavros_examples/COLCON_IGNORE" 2>/dev/null || true

cd "$WS"
MAKEFLAGS="-j$JOBS" nice -n 10 colcon build \
  --packages-select mavros \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

node="$WS/install/mavros/lib/mavros/mavros_node"
test -x "$node" || { echo "build finished but $node is missing" >&2; exit 1; }
echo
echo "Patched MAVROS built: $node"
echo "Restart the rover stack to use it; the launch log will say which MAVROS started."
