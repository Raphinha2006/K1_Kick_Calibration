#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/humble/setup.bash
source /opt/booster/BoosterRos2Interface/install/setup.bash
source "$ROOT/install/local_setup.bash"
export ROS_DOMAIN_ID=171
unset FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_LOCALHOST_ONLY=1
cd "$ROOT"
exec /usr/bin/python3 "$ROOT/tests/ros_isolated.py"
