#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export PYTHONPATH="$ROOT/src/kick_calibration${PYTHONPATH:+:$PYTHONPATH}"
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
  source /opt/booster/BoosterRos2Interface/install/setup.bash
  source "$ROOT/install/local_setup.bash"
fi
exec python3 -m kick_calibration.dataset "$@"
