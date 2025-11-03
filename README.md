# Ros2
> **Author:** Marco Alikhani Najafabadi   
> **Date:** 13 August 2025

## Introduction
This repository contains practice projects and exercises for learning and exploring ROS 2.
ROS 2 Humble is officially supported on Ubuntu 22.04 (Jammy) only, so first check Ubuntu version with:
```bash
lsb_release -a
```

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
```

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
```


## ROS 2 on Shared Networks — Setup Guide

> To Configure `ROS_DOMAIN_ID` and `ROS_LOCALHOST_ONLY` correctly so that the ROS 2 nodes don’t collide with other peoples nodes on a shared LAN.
---

- **Solo on one machine?**  
  Put `ROS_LOCALHOST_ONLY=1` (loopback only). Pick any `ROS_DOMAIN_ID` to avoid collisions.
- **Working across multiple PCs?**  
  Use `ROS_LOCALHOST_ONLY=0` and agree on a unique `ROS_DOMAIN_ID` for all people in the group.

---

## Picking a Unique Domain ID

Choose a number **0–232** that no other nearby group uses.

```bash
# Example
export ROS_DOMAIN_ID=42
```



Nodes **only** discover each other when their `ROS_DOMAIN_ID` matches.
---

## Apply Settings

```bash
# Solo (local-only):
export ROS_LOCALHOST_ONLY=1

# OR Team (LAN):
# export ROS_LOCALHOST_ONLY=0

# Always set a domain in shared environments:
export ROS_DOMAIN_ID=42   # <-- change to your group's chosen ID

# Refresh discovery (optional but handy):
ros2 daemon stop || true
```

---

## Make It Persistent

Append to your shell startup file.

```bash
# For bash:
echo 'export ROS_LOCALHOST_ONLY=1' >> ~/.bashrc   # or 0 if multi-machine
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
source ~/.bashrc

# For zsh:
# echo 'export ROS_LOCALHOST_ONLY=1' >> ~/.zshrc
# echo 'export ROS_DOMAIN_ID=42' >> ~/.zshrc
# source ~/.zshrc
```

---

## Verify

```bash
printenv ROS_LOCALHOST_ONLY
printenv ROS_DOMAIN_ID
ros2 daemon stop
```

---

## Quick Functional Test (Single Machine)

Open **Terminal A** (with the env vars set):

```bash
ros2 run demo_nodes_cpp talker
```

Open **Terminal B** (same env vars):

```bash
ros2 run demo_nodes_cpp listener
```

The output will be:

```
I heard: "Hello World: N"
```

---

##  Multi‑Machine Test (Team on the Same LAN)

On **every machine**:

- Same `ROS_DOMAIN_ID` (e.g., `42`)
- `ROS_LOCALHOST_ONLY=0`
- Same network (no VPN between you; same Wi‑Fi/Ethernet segment)
- Firewalls allow DDS discovery/traffic on the LAN

**Test:**

```bash
# Machine A:
ros2 run demo_nodes_cpp talker

# Machine B:
ros2 topic echo /chatter

# Machine C (optional):
ros2 node list
```

You should see `/talker` and `/listener` and messages on `/chatter`.








