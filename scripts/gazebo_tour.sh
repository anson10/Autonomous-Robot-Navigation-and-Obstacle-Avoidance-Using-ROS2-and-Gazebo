#!/bin/bash
# Headless Gazebo waypoint tour, run inside the project Docker image.
# Usage (from the repo root):
#   docker build -t obstacle-avoidance .
#   scripts/gazebo_tour.sh sector        # or apf / follow_the_gap
#   IMPL=python scripts/gazebo_tour.sh apf   # Python node instead of the C++ one
# Results land in results/gazebo/<impl>/<algorithm>/ (odom.csv, status.jsonl, launch.log).
set -euo pipefail
ALGO=${1:-sector}
IMPL=${IMPL:-cpp}
DURATION=${DURATION:-240}
WAYPOINTS="0.0,0.62; 0.68,0.0; 0.0,-0.62; -0.68,0.0; -1.8,1.8; 1.8,1.8; 1.8,-1.8; -1.8,-1.8"
OUT="$(pwd)/results/gazebo/${IMPL}/${ALGO}"
mkdir -p "$OUT"
docker run --rm \
  -e ROS_DOMAIN_ID=$((RANDOM % 100 + 1)) \
  -v "$(pwd)/src:/ros2_ws/src:ro" -v "$(pwd)/scripts:/scripts:ro" -v "$OUT:/out" \
  obstacle-avoidance bash -c "
    source /opt/ros/humble/setup.bash
    cd /ros2_ws && colcon build --symlink-install > /out/build.log 2>&1
    source install/setup.bash
    ros2 launch my_robot_navigation robot_navigation.launch.py gui:=false \
      impl:=${IMPL} algorithm:=${ALGO} waypoints:='${WAYPOINTS}' > /out/launch.log 2>&1 &
    python3 /scripts/record_run.py /out ${DURATION}
    pkill -INT -f 'ros2 launch' || true; sleep 3; pkill -9 -f gzserver || true
    chown -R $(id -u):$(id -g) /out"
echo "Saved to $OUT"
