#!/usr/bin/env bash
# Source this file: source scripts/setup/source_wsl_ros_env.sh

if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
else
  echo "ERROR: ROS setup.bash not found under /opt/ros/{jazzy,humble}" >&2
  return 1 2>/dev/null || exit 1
fi

export TURTLEBOT3_MODEL="burger"
export ROS_DOMAIN_ID="0"
export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
export FASTDDS_BUILTIN_TRANSPORTS="UDPv4"
export ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET"
export ROS_LOCALHOST_ONLY=0

echo "ROS env loaded:"
echo "  TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "  FASTDDS_BUILTIN_TRANSPORTS=$FASTDDS_BUILTIN_TRANSPORTS"
echo "  ROS_AUTOMATIC_DISCOVERY_RANGE=$ROS_AUTOMATIC_DISCOVERY_RANGE"
echo "  ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
