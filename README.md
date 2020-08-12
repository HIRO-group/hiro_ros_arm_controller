# hiro_ros_arm_controller
A high level API for controlling robotic arms in ROS.
It works for both simulation and real robots.

# Supported Robotic Arms
- Franka Emika Panda
- Rethink Robotics Sawyer

# Installation
Assuming that you already have a catkin workspace
TODO: We need to fix this
```
mkdir -p ~/catkin_ws/src
cd src
git clone git@github.com:HIRO-group/hiro_ros_arm_controller.git
git clone git@github.com:HIRO-group/panda_simulation.git   # or use https://github.com/erdalpekel/panda_simulation
git clone git@github.com:HIRO-group/franka_ros.git
cd ..
catkin build   # or catkin make
```

# APIs

Joint Position Control Example
```
if __name__ == '__main__':
    rospy.init_node('robot_controller', anonymous=True)
    controller = PandaController(is_sim=False)

    controller.publish_positions([0, 0, 0, -0.0698, 0, 0, 0])
```
