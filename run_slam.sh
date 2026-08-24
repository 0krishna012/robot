#!/usr/bin/env bash
# One-shot SLAM launcher: sources ROS + this workspace, optionally rebuilds,
# then brings up the ESP32 bridge, RPLIDAR, robot_state_publisher, and
# slam_toolbox together via slam_launch.py.
#
# Usage:
#   ./run_slam.sh                                 # just launch
#   ./run_slam.sh --build                         # colcon build robot_bringup first
#   ./run_slam.sh --serial-port=/dev/ttyUSB1 --lidar-port=/dev/ttyUSB0
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_SETUP="/opt/ros/humble/setup.bash"

BUILD=false
SERIAL_PORT="/dev/ttyUSB1"
LIDAR_PORT="/dev/ttyUSB0"

for arg in "$@"; do
  case "$arg" in
    --build) BUILD=true ;;
    --serial-port=*) SERIAL_PORT="${arg#*=}" ;;
    --lidar-port=*) LIDAR_PORT="${arg#*=}" ;;
    -h|--help)
      echo "Usage: $0 [--build] [--serial-port=/dev/ttyUSBx] [--lidar-port=/dev/ttyUSBx]"
      exit 0
      ;;
    *)
      echo "Unknown option: $arg" >&2
      exit 1
      ;;
  esac
done

source "$ROS_DISTRO_SETUP"

if $BUILD; then
  echo ">> Building robot_bringup..."
  (cd "$SCRIPT_DIR" && colcon build --packages-select robot_bringup)
fi

source "$SCRIPT_DIR/install/setup.bash"

echo ">> Launching SLAM stack (serial_port=$SERIAL_PORT lidar_port=$LIDAR_PORT)"
exec ros2 launch robot_bringup slam_launch.py \
  serial_port:="$SERIAL_PORT" \
  lidar_port:="$LIDAR_PORT"
