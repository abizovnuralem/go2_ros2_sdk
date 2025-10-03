#!/bin/bash
set -euo pipefail

if [ -z "${ROS_DISTRO:-}" ]; then
  echo "ROS_DISTRO not set; defaulting to humble"
  export ROS_DISTRO=humble
fi

# Prevent ROS setup scripts from tripping `set -u` when AMENT_TRACE_SETUP_FILES is unset
: "${AMENT_TRACE_SETUP_FILES:=0}"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

# Ensure rosdep database is up to date
if [ ! -f "$HOME/.ros/rosdep_updated" ]; then
  echo "Updating rosdep database..."
  rosdep update
  touch "$HOME/.ros/rosdep_updated"
fi

# Refresh apt cache once to improve rosdep installations (if apt-get is available)
if command -v apt-get >/dev/null 2>&1; then
  if [ ! -f "$HOME/.ros/apt_cache_updated" ]; then
    echo "Refreshing apt package index..."
    sudo apt-get update
    touch "$HOME/.ros/apt_cache_updated"
  fi
fi

# Install workspace dependencies (best effort)
if [ -d /workspaces/go2_ros2_sdk ]; then
  cd /workspaces/go2_ros2_sdk
  echo "Resolving ROS dependencies (best effort)..."
  rosdep install --from-paths . --ignore-src -r -y || true
fi

# Initialize colcon mixins metadata (if not already present)
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml 2>/dev/null || true
colcon mixin update default 2>/dev/null || true

echo "Devcontainer setup complete."
