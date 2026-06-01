#!/bin/bash
# install-deps.sh [minimal|full]
set -euo pipefail
MODE="${1:-minimal}"
# minimal | full | nav2

printf 'Acquire::http::Timeout "300";\nAcquire::Retries "3";\n' \
  | sudo tee /etc/apt/apt.conf.d/80-retries >/dev/null

sudo apt-get update

if [[ "$MODE" == "minimal" ]]; then
  echo "=== minimal: CycloneDDS (Unitree DDS) + ping ==="
  sudo apt-get install -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp \
    iputils-ping
elif [[ "$MODE" == "nav2" ]]; then
  echo "=== nav2: full + Nav2 + SLAM + teleop ==="
  sudo apt-get install -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rviz2 \
    ros-humble-tf2-tools \
    ros-humble-tf2-ros \
    ros-humble-nav2-map-server \
    ros-humble-nav2-planner \
    ros-humble-nav2-navfn-planner \
    ros-humble-nav2-smac-planner \
    ros-humble-nav2-dwb-controller \
    ros-humble-nav2-controller \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-msgs \
    ros-humble-slam-toolbox \
    ros-humble-teleop-twist-keyboard \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    python3-colcon-common-extensions \
    git \
    iputils-ping \
    net-tools
else
  echo "=== full: + RViz, colcon, tf2 ==="
  sudo apt-get install -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rviz2 \
    ros-humble-tf2-tools \
    ros-humble-tf2-ros \
    python3-colcon-common-extensions \
    git \
    iputils-ping \
    net-tools
fi

