#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run as root: sudo $0"
  exit 1
fi

if ! command -v lsb_release >/dev/null 2>&1; then
  apt update
  apt install -y lsb-release
fi

CODENAME="$(lsb_release -cs)"
ARCH="$(dpkg --print-architecture)"

if [[ "$CODENAME" != "noble" && "$CODENAME" != "jammy" ]]; then
  echo "Unsupported Ubuntu codename: $CODENAME"
  echo "This script supports Ubuntu 24.04 (noble) and 22.04 (jammy)."
  exit 1
fi

apt update
apt install -y curl gnupg2 software-properties-common ca-certificates
add-apt-repository -y universe

ROS_KEYRING="/usr/share/keyrings/ros-archive-keyring.gpg"
ROS_LIST="/etc/apt/sources.list.d/ros2.list"

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "$ROS_KEYRING"

echo "deb [arch=${ARCH} signed-by=${ROS_KEYRING}] http://packages.ros.org/ros2/ubuntu ${CODENAME} main" > "$ROS_LIST"

apt update

if [[ "$CODENAME" == "noble" ]]; then
  ROS_DESKTOP_PKG="ros-jazzy-desktop"
  ROS_TB3_GAZEBO_PKG="ros-jazzy-turtlebot3-gazebo"
  ROS_SETUP_LINE="source /opt/ros/jazzy/setup.bash"
else
  ROS_DESKTOP_PKG="ros-humble-desktop"
  ROS_TB3_GAZEBO_PKG="ros-humble-turtlebot3-gazebo"
  ROS_SETUP_LINE="source /opt/ros/humble/setup.bash"
fi

apt install -y \
  docker.io \
  docker-compose-v2 \
  "$ROS_DESKTOP_PKG" \
  "$ROS_TB3_GAZEBO_PKG"

if ! id -nG "${SUDO_USER:-$USER}" | grep -qw docker; then
  usermod -aG docker "${SUDO_USER:-$USER}" || true
fi

TARGET_HOME="${HOME}"
if [[ -n "${SUDO_USER:-}" ]]; then
  TARGET_HOME="$(getent passwd "$SUDO_USER" | cut -d: -f6)"
fi

BASHRC_PATH="${TARGET_HOME}/.bashrc"

append_if_missing() {
  local line="$1"
  if ! grep -Fqx "$line" "$BASHRC_PATH"; then
    echo "$line" >> "$BASHRC_PATH"
  fi
}

append_if_missing "$ROS_SETUP_LINE"
append_if_missing "export TURTLEBOT3_MODEL=burger"
append_if_missing "export ROS_DOMAIN_ID=0"
append_if_missing "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
append_if_missing "export FASTDDS_BUILTIN_TRANSPORTS=UDPv4"
append_if_missing "export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET"
append_if_missing "export ROS_LOCALHOST_ONLY=0"

echo

echo "Installation finished."
echo "Run the following in your shell:"
echo "  newgrp docker"
echo "  source ~/.bashrc"
