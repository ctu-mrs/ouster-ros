#!/bin/bash
set -e

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

echo "Starting install"

MY_PATH=`pwd`

sudo apt-get -y install git

echo "clone uav_core"
cd
git clone https://github.com/ctu-mrs/uav_core.git
cd uav_core

echo "running the main install.sh"
./installation/install.sh

echo "clone camera_base"
cd
git clone https://github.com/ctu-mrs/camera_base.git

echo "installing ouster-ros"
cd $MY_PATH/install
./install.sh

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s ~/camera_base camera_base
ln -s "$MY_PATH" ouster-ros
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/catkin_ws

echo "install ended"
