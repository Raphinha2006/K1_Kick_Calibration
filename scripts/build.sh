#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/humble/setup.bash
source /opt/booster/BoosterRos2Interface/install/setup.bash
cd "$ROOT"
/usr/bin/python3 -c 'from booster_interface.msg import BatteryState, RobotStatesMsg, LowState'
colcon build --symlink-install --packages-select brain booster_msgs vision_interface vision kick_calibration \
  --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
