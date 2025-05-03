#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
# setup custom workspace environment
source "$ROS_WS_PATH/install/setup.bash"
exec "$@"
