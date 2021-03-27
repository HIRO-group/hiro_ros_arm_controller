# HIRO ROS Arm Controller
`hiro_ros_arm_controller`

Documentation Link: **http://hiro-group.ronc.one/hiro_ros_arm_controller/**

A high level API (in the form of a ROS package) for controlling robotic arms in ROS.
It works for both simulation and real robots, but has been primarily tested on the Franka Panda.

# Supported Robotic Arms
- Franka Emika Panda (recommended)
- Rethink Robotics Sawyer

# Quick Installation
Assuming that you already have a catkin workspace:
```sh
mkdir -p ~/catkin_ws/src
cd src
git clone git@github.com:HIRO-group/hiro_ros_arm_controller.git
cd hiro_ros_arm_controller
./install.sh
cd ../..
catkin build   # or catkin_make
```

# Detailed Installation

We have an `install.sh` script that will install the following packages:

- Installs this package

- The Franka Panda Gazebo Simulator package [here](https://github.com/HIRO-group/panda_simulation)

- External dependencies needed for the `panda_simulation` package

- The Sawyer Gazebo simulator

## To run `install.sh`
Make sure that you have cloned this repository from the `src` folder of a catkin workspace (eg: from `catkin_ws/src`). If you haven't, the script will give an error, and it won't build the whole workspace.

Additionally, you should clone this repository in an **empty** catkin workspace. For example, if you have the `franka_ros` package in the workspace, it will be deleted in favor of our forked version that works with this package.

### Usage:
```sh
./install.sh --git-option https|ssh --franka-build apt|source
```


### Examples
1. Simply Run (Default options= `ssh` && `apt`)
```sh
./install.sh
```

2. Use options
```sh
./install.sh --git-option ssh --franka-build source
```
This command will build `libfranka` from source and use ssh for git.

### Options
- `--git-option` <br>
specifies if we clone the `HIRO` repos via https or ssh.

- `franka_build` <br>
specifies whether we want to build `libfranka` from source or install it via `apt`.

# API
In any of your ros packages, simply import our package.

Joint Position Control Example (in real life)
```py
from hiro_ros_arm_controller.RobotController import PandaController

if __name__ == '__main__':
    rospy.init_node('robot_controller', anonymous=True)
    controller = PandaController(is_sim=False)

    controller.publish_positions([0, 0, 0, -0.0698, 0, 0, 0])
```

This example creates a ROS node, creates a PandaController with `is_sim` set to `False` (so, real robot control), and publishes a list of desired joint positions to follow.
