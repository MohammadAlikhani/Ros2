# Ros2
> **Author:** Mohammad Alikhani Najafabadi — mohammad.najafabadi@estudiantat.upc.edu  
> **Date:** 13 August 2025

## Introduction
This repository contains practice projects and exercises for learning and exploring ROS 2.

## Installing Ros2

### Setting up system for ROS 2

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt install curl gnupg2 lsb-release -y
```

##  Adding the ROS 2 GPG key

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
```
## Adding the ROS 2 apt repository

```bash
echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
``

## Installing Ros2 along with essential pakages
```bash
sudo apt install ros-humble-desktop
sudo apt install ros-humble-rqt-tf-tree ros-humble-tf2-tools
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-camera-info-manager
sudo apt install ros-humble-camera-calibration-parsers ros-humble-camera-info-manager
sudo apt install ros-humble-ur-description
sudo apt install ros-humble-aruco-ros
sudo apt install ros-humble-controller-manager
sudo apt install ros-humble-ros2-controllers ros-humble-ros2-control
sudo apt install ros-humble-rqt-joint-trajectory-controller
sudo apt install ros-humble-rqt-controller-manager
sudo apt install ros-humble-ros-gz ros-humble-gz-ros2-control ros-humble-ros-gz-bridge
sudo apt install ros-humble-ros-gz-image ros-humble-ros-gz-interfaces ros-humble-ros-gz-sim
sudo apt install ros-humble-ros-gz-sim-demos ros-humble-ur-simulation-gz
```
## Other important utilities
```bash
sudo apt install python3-colcon-common-extensions
sudo apt install python3-colcon-clean
sudo apt install git gitk qgit build-essential wget curl
sudo apt install liburdfdom-tools bash-completion
sudo apt install python3-ament-lint python3-ament-lint-cmake
## Sourcing Ros2 In all terminal sessions
```

## Sourcing ROS 2 in all terminal sessions
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
``
