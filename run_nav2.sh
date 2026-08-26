#!/usr/bin/env bash
# One-shot Nav2 launcher: sources ROS + this workspace, optionally rebuilds,
# then brings up the ESP32 bridge, RPLIDAR, robot_state_publisher, and the
# full nav2 stack (map_server + AMCL + planner/controller/BT navigator)
# together via nav2_launch.py, localized against a saved map.
#
# Usage:
#   ./run_nav2.sh                                       # use maps/my_map.yaml
#   ./run_nav2.sh --map=/path/to/other_map.yaml
#   ./run_nav2.sh --build                                # colcon build robot_bringup first
#   ./run_nav2.sh --rviz                                 # also launch RViz
#   ./run_nav2.sh --serial-port=/dev/ttyUSB1 --lidar-port=/dev/ttyUSB0
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_SETUP="/opt/ros/humble/setup.bash"

BUILD=false
USE_RVIZ=false
SERIAL_PORT="/dev/ttyUSB1"
LIDAR_PORT="/dev/ttyUSB0"
MAP_FILE="$SCRIPT_DIR/maps/my_map.yaml"

for arg in "$@"; do
  case "$arg" in
    --build) BUILD=true ;;
    --rviz) USE_RVIZ=true ;;
    --map=*) MAP_FILE="${arg#*=}" ;;
    --serial-port=*) SERIAL_PORT="${arg#*=}" ;;
    --lidar-port=*) LIDAR_PORT="${arg#*=}" ;;
    -h|--help)
      echo "Usage: $0 [--build] [--rviz] [--map=/path/to/map.yaml] [--serial-port=/dev/ttyUSBx] [--lidar-port=/dev/ttyUSBx]"
      exit 0
      ;;
    *)
      echo "Unknown option: $arg" >&2
      exit 1
      ;;
  esac
done

source "$ROS_DISTRO_SETUP"

echo ">> Stopping any leftover robot processes from a previous run..."
pkill -f "ros2 launch robot_bringup" 2>/dev/null || true
pkill -f "serial_bridge_node" 2>/dev/null || true
pkill -f "rplidar" 2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true
pkill -f "joint_state_publisher" 2>/dev/null || true
pkill -f "async_slam_toolbox_node" 2>/dev/null || true
pkill -f "map_server" 2>/dev/null || true
pkill -f "amcl" 2>/dev/null || true
pkill -f "controller_server" 2>/dev/null || true
pkill -f "planner_server" 2>/dev/null || true
pkill -f "smoother_server" 2>/dev/null || true
pkill -f "behavior_server" 2>/dev/null || true
pkill -f "bt_navigator" 2>/dev/null || true
pkill -f "waypoint_follower" 2>/dev/null || true
pkill -f "velocity_smoother" 2>/dev/null || true
pkill -f "lifecycle_manager" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true
sleep 1

if $BUILD; then
  echo ">> Building robot_bringup..."
  (cd "$SCRIPT_DIR" && colcon build --packages-select robot_bringup)
fi

source "$SCRIPT_DIR/install/setup.bash"

if [ ! -f "$MAP_FILE" ]; then
  echo "Map file not found: $MAP_FILE" >&2
  exit 1
fi

echo ">> Launching Nav2 stack (map=$MAP_FILE serial_port=$SERIAL_PORT lidar_port=$LIDAR_PORT)"
exec ros2 launch robot_bringup nav2_launch.py \
  map:="$MAP_FILE" \
  serial_port:="$SERIAL_PORT" \
  lidar_port:="$LIDAR_PORT" \
  use_rviz:="$USE_RVIZ"
