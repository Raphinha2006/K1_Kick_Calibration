#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
mode=--dry-run
extra=()
for argument in "$@"; do
  case "$argument" in
    --real|--dry-run) mode="$argument" ;;
    --no-vision) extra+=(--no-vision) ;;
    *) echo "Usage: $0 [--dry-run|--real] [--no-vision]"; exit 2 ;;
  esac
done
if [ ! -f "$ROOT/install/setup.bash" ]; then
  echo "Build first: ./scripts/build.sh"; exit 1
fi
source /opt/ros/humble/setup.bash
# Use the CURRENT system booster_interface (includes battery and robot states).
# Do not source AAA/person_pass: their old overlay masks these message types.
source /opt/booster/BoosterRos2Interface/install/setup.bash
source "$ROOT/install/local_setup.bash"
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml
cd "$ROOT"
options=(--root "$ROOT")
if [ "$mode" = --real ]; then options+=(--real); fi
exec /usr/bin/python3 -m kick_calibration.session "${options[@]}" "${extra[@]}"
