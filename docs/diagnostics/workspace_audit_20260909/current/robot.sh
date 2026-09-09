#!/usr/bin/env bash
set -eo pipefail
RINBO_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$RINBO_WORKSPACE/install/setup.bash" ]]; then
  echo '尚未建立 ROS 工作區。請先安裝 ROS 2 Humble 並編譯此工作區。' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source "$RINBO_WORKSPACE/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
export PYTHONPATH="$RINBO_WORKSPACE/src/rinbo_control${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 -m rinbo_control.console "$@"
