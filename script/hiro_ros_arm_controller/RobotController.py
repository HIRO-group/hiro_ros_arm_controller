#!/usr/bin/env python
"""
This code serves as the generic RobotController
class.
"""

import rospy
import numpy as np
import intera_interface
from RobotArm import RobotArm


class RobotController(object):
    """
    Robot controller object.
    """
    def __init__(self, num_joints, is_sim=True,
                 controller_names=None):
        """
        robot controller for n joints.

        Arguments
        ---------
        `num_joints`: `int` - number of joints on the robot.

        `is_sim`: `bool` - whether the robot is being controlled in sim or not.

        `controller_names`: `List[str]` - list of the controller names.
        """
        self.is_sim = is_sim
        self.arm = self.get_arm(num_joints, is_sim)
        self.pose_name = ''

    def get_arm(self, num_joints, is_sim=True):
        """
        gets the robotic limb.

        Arguments
        ---------
        `num_joints`: `int` - number of joints on the robot

        `is_sim`: `bool`: if the robot is controlled in simulation or not

        Returns
        -------
        `RobotArm` - a robot arm object.

        """
        return RobotArm(num_joints=num_joints, is_sim=is_sim)

    @property
    def joint_names(self):
        """
        get joint names.

        Returns
        -------
        `List[str]`: list of joint names.
        """
        return self.arm.joint_names()

    @property
    def joint_velocities(self):
        """
        get joint velocities.

        Returns
        -------
        `List[float]`: list of joint velocities.
        """
        return self.arm.joint_velocities()

    @property
    def joint_angles(self):
        """
        get joint angles.

        Returns
        -------
        `List[float]`: list of joint angles.
        """
        return self.arm.joint_angles()

    def joint_angle(self, joint_name):
        """
        get joint angle givem `joint_name`.

        Arguments
        ---------
        `joint_name`: `str` - the joint name.

        Returns
        -------
        `float`: joint angle of the specified joint name.
        """
        return self.arm.joint_angle(joint_name)

    def joint_velocity(self, joint_name):
        """
        get joint velocity given `joint_name`.

        Arguments
        ---------
        `joint_name`: `str` - the joint name.

        Returns
        -------
        `float`: joint velocity of the specified joint name.
        """
        return self.arm.joint_velocity(joint_name)

    def set_joint_position_speed(self, speed=1.0):
        """
        change joint position speed.

        Arguments
        ---------
        `speed`: `float` - desired (max) speed of the joints.

        Returns
        -------
        `None`, as this hasn't been implemented yet.
        """
        return self.arm.set_joint_position_speed(speed)

    def send_once(self):
        """
        sends a trajectory message once. This is a hardcoded trajectory
        and should only be used for test purposes.
        """
        traj = np.zeros((1, 3, 7))
        self.arm.set_joint_trajectory(traj)
        rospy.sleep(1)
        self.arm.set_joint_trajectory(traj)
        rospy.sleep(1)

    def publish_positions(self, joint_positions, sleep=2.0):
        """
        moves robot to specificed `joint_positions`.

        Arguments
        ---------
        `joint_positions`: `List[float]` - list of desired joint positions.

        `sleep`: `float`: amount of sleep until the arm will stop.
        """
        self.arm.move_to_joint_positions(joint_positions, sleep)

    def send_velocities(self, joint_velocities):
        """
        sends a velocity command.

        Arguments
        ---------
        `joint_velocities`: `List[float]` - list of desired velocities.
        """
        self.arm.set_joint_velocities(joint_velocities)

    def publish_velocities(self, joint_velocities, sleep=2.0):
        """
        unlike `send_velocities`, this function will
        command joints to move at specified velocities for `sleep`
        seconds before stopping.

        Arguments
        ---------
        `joint_velocities`: `List[float]` - list of joint velocities

        `sleep`: `float` - time to sleep.
        """
        self.arm.set_joint_velocities(joint_velocities)
        rospy.sleep(sleep)
        self.arm.set_joint_velocities(np.zeros(self.arm.num_joints))

    def publish_trajectory(self, positions, velocities, accelerations, sleep):
        """
        publishes a robot trajectory. Takes care of positions, velocities,
        and accelerations, which are more intuitive than a 3-d tensor.

        Arguments
        ---------
        `positions`: List[List[float]] - a *2-d* matrix of joint positions, where the shape is
        (num_points, num_joints).

        `velocities`: List[List[float]] - a *2-d* matrix of joint velocities, where the shape is
        (num_points, num_joints).

        `accelerations`: List[List[float]] - a *2-d* matrix of joint accelerations, where the shape is
        (num_points, num_joints).

        `sleep`: `float` - how much time to sleep for.
        """
        # shape is amount of poses, 3 rows for pos,vel,acc, with num_joints
        traj = np.zeros((len(positions), 3, self.arm.num_joints))
        for idx, (pos, vel, acc) in enumerate(zip(positions, velocities, accelerations)):
            traj[idx] = np.vstack((pos, vel, acc))
        self.arm.set_joint_trajectory(traj)
        rospy.sleep(sleep)

    def set_positions_list(self, poses, sleep):
        """
        sets position list.

        `poses`: List - containing the following - 
            - Each entry in `poses` consists of three components: positions,
            _, and the pose_name.

        `sleep`: `float`: The amount of time to sleep for.
        """
        for each_pose in poses:
            positions, _, pose_name = each_pose[0], each_pose[1], each_pose[2]  # noqa: F841
            self.pose_name = pose_name
            self.publish_positions(positions, sleep)

    def set_velocities_list(self, poses, sleep):
        """
        sets velocities list.

        `poses`: List - containing the following - 
            - Each entry in `poses` consists of three components: _,
            velocities, and the pose_name.

        `sleep`: `float`: The amount of time to sleep for.
        """
        for each_pose in poses:
            _, velocities, pose_name = each_pose[0], each_pose[1], each_pose[2]  # noqa: F841
            self.pose_name = pose_name
            self.publish_velocities(velocities, sleep)

    def set_trajectory_list(self, poses, sleep):
        """
       sets trajectory list.

        Arguments
        ---------
        `poses`: List - containing the following -
            - Each entry in `poses` consists of three components: positions,
            velocities, and the pose_name.

        `sleep`: `float`: The amount of time to sleep for.
        """
        for each_pose in poses:
            positions, velocities, pose_name = each_pose[0], each_pose[1], each_pose[2]
            accelerations = np.zeros(7)
            self.pose_name = pose_name
            self.publish_trajectory(positions, velocities, accelerations, sleep)


class PandaController(RobotController):
    """
    PandaController class
    """
    def __init__(self, num_joints=7, is_sim=True):
        """
        the Panda Controller class.

        Arguments
        ---------
        `num_joints`: `int` - number of joints.

        `is_sim`: `bool` - whether the robot is being controlled in simulation or not.
        """
        super(PandaController, self).__init__(num_joints, is_sim)


class SawyerController(RobotController):
    """
    SawyerController class.
    """
    def __init__(self, num_joints=7, is_sim=True, limb_name="right"):
        """
        the Sawyer Controller class.

        Arguments
        ---------
        `num_joints`: `int` - number of joints.

        `is_sim`: `bool` - whether the robot is being controlled in simulation or not.

        `limb_name`: `str` - the limb name of the arm for Sawyer.
        """
        self.limb_name = limb_name
        super(SawyerController, self).__init__(num_joints, is_sim)
        self.positions = {name: 0.0 for name in self.joint_names}
        self.velocities = {name: 0.0 for name in self.joint_names}

    def get_arm(self, num_joints, is_sim=True):
        """
        gets the arm for Sawyer.

        Arguments
        ---------
        `num_joints`: `int` - number of joints on the arm.

        `is_sim`: `bool` - whether the robot is being controlled in sim or not.

        Returns
        -------
        `arm`: `intera_interface.Limb` - gets the limb for Sawyer
        """

        arm = intera_interface.Limb(self.limb_name)
        self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        self._rs.enable()
        self.num_joints = num_joints
        rospy.sleep(2)
        return arm

    def send_once(self):
        """
        sends a trajectory message once.
        """
        positions = [0] * self.num_joints
        velocities = [0] * self.num_joints
        accelerations = [0] * self.num_joints

        self.arm.set_joint_trajectory(self.joint_names(), positions, velocities, accelerations)

    def update_position_vec(self, positions):
        """
        updates the positions that will be sent to Sawyer.

        Arguments
        ---------
        `positions`: `List[float]` - the list of joint positions to send
        to Sawyer.
        """
        for joint_name, position in zip(self.joint_names, positions):
            self.positions[joint_name] = float(position)

    def update_velocity_vec(self, velocities):
        """
        updates the velocities that will be sent to Sawyer.

        Arguments
        ---------
        `velocities`: `List[float]` - the list of joint velocities to send
        to Sawyer.
        """
        for joint_name, velocity in zip(self.joint_names, velocities):
            self.velocities[joint_name] = float(velocity)

    def publish_positions(self, joint_positions, sleep=2.0):
        """
        moves robot to specified `joint_positions`.

        Arguments
        ---------
        `joint_positions`: `List[float]` - desired joint positions.

        `sleep`: `float` - sleeps for that amount of time.
        """
        self.update_position_vec(joint_positions)
        self.arm.move_to_joint_positions(self.positions, sleep)

    def send_velocities(self, joint_velocities):
        """
        sends a velocity command.

        Arguments
        ----------
        `joint_velocities`: `List[float]` - list of desired velocities.
        """
        self.update_velocity_vec(joint_velocities)
        self.arm.set_joint_velocities(self.velocities)

    def publish_velocities(self, joint_velocities, sleep=2.0):
        """
        unlike `send_velocities`, this function will
        command joints to move at specified velocities for `sleep`
        seconds before stopping.

        Arguments
        ---------
        `joint_velocities`: `List[float]` - desired joint velocities.

        `sleep`: `float` - amount of time to sleep for before stopping.
        """
        self.update_velocity_vec(joint_velocities)

        self.arm.set_joint_velocities(self.velocities)
        self.update_velocity_vec(joint_velocities)

        rospy.sleep(sleep)
        self.update_velocity_vec(np.zeros(self.num_joints))
        self.arm.set_joint_velocities(self.velocities)

    def publish_trajectory(self, positions, velocities, accelerations, sleep=2.0):
        """
                publishes a robot trajectory. Takes care of positions, velocities,
        and accelerations, which are more intuitive than a 3-d tensor.

        Arguments
        ---------
        `positions`: List[List[float]] - a *2-d* matrix of joint positions, where the shape is
        (num_points, num_joints).

        `velocities`: List[List[float]] - a *2-d* matrix of joint velocities, where the shape is
        (num_points, num_joints).

        `accelerations`: List[List[float]] - a *2-d* matrix of joint accelerations, where the shape is
        (num_points, num_joints).

        `sleep`: `float` - how much time to sleep for.
        """
        # shape is amount of poses, 3 rows for pos,vel,acc, with num_joints
        self.arm.set_joint_trajectory(self.joint_names, positions, velocities, accelerations)
        rospy.sleep(sleep)


if __name__ == '__main__':
    """
    this is a simple example of creating a panda controller
    and publishing positions to all of the joints.
    """
    rospy.init_node('robot_controller')
    controller = PandaController(is_sim=True)

    # publish the "zero" positions of the Panda.
    # note that one of the positions is not 0, due to the limits of the panda.
    controller.publish_positions([0, 0, 0, -0.0698, 0, 0, 0])
