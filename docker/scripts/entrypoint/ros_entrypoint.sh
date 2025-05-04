#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
# setup custom workspace environment
if [ -f "$ROS_WS_PATH/install/setup.bash" ]; then
  source "$ROS_WS_PATH/install/setup.bash"
fi
exec "$@"
