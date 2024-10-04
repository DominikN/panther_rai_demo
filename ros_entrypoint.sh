#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --

export PATH="$HOME/.local/bin:$PATH"
source ./setup_shell.sh

exec "$@"