#!/bin/bash
set -e

# Source ROS
source /opt/ros/humble/setup.bash

# If workspace has not been built, build it
if [ ! -f /raspberry_pi_ws/install/setup.bash ]; then
  echo "Building ROS 2 workspace..."
  cd /raspberry_pi_ws
  rm -rf build install log  # clean to avoid host/container path mismatch
  colcon build
fi

# Source workspace
source /raspberry_pi_ws/install/setup.bash

# Run passed command
exec "$@"
