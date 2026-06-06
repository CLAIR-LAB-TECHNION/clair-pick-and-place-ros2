#!/usr/bin/env bash
# Source ROS 2 Humble and this workspace. Use: source setup_env.sh
source /opt/ros/humble/setup.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
