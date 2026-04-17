#!/usr/bin/env bash
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y \
  locales \
  software-properties-common \
  curl \
  gnupg \
  lsb-release \
  ca-certificates \
  git \
  python3-pip \
  python3-opencv \
  python3-pytest

locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

add-apt-repository -y universe
apt-get update

ROS_APT_SOURCE_VERSION="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F 'tag_name' | awk -F'\"' '{print $4}')"
UBUNTU_CODENAME_VALUE="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")"
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME_VALUE}_all.deb"
dpkg -i /tmp/ros2-apt-source.deb

curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

apt-get update
apt-get upgrade -y

apt-get install -y \
  ros-dev-tools \
  python3-colcon-common-extensions \
  ros-jazzy-ros-base \
  ros-jazzy-vision-opencv \
  ros-jazzy-apriltag-ros \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-ros-gz \
  gz-harmonic
