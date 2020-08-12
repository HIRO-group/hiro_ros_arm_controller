# hiro_ros_arm_controller
A high level API for controlling robotic arms in ROS

# Supported Robotic Arms
- Franka Emika Panda
- Rethink Robotics Sawyer

# Installation
Assuming that you already have a catkin workspace
```
mkdir -p ~/catkin_ws/src
cd src
git clone git@github.com:HIRO-group/hiro_ros_arm_controller.git
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
