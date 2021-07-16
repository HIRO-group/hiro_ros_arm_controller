// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <PandaJointVelocityContactController.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>

#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <franka/robot_state.h>
#include <ros/ros.h>
#include "pseudo_inversion.h"
#include <numeric>
#include <Eigen/Core>

namespace hiro_panda {

bool PandaJointVelocityContactController::init(hardware_interface::RobotHW* robot_hardware,
                                        ros::NodeHandle& node_handle) {
    setValueTrackers();
    setPublishers(node_handle);
    setSignalParsers(node_handle);
    return setupController(robot_hardware, node_handle);
}


// Helper functions to clean up the robot setup
void PandaJointVelocityContactController::setPublishers( ros::NodeHandle& node_handle){
  ext_cart_force_pub_x = node_handle.advertise<std_msgs::Float64>("ext_cart_force_x", 1);
  ext_cart_force_pub_y = node_handle.advertise<std_msgs::Float64>("ext_cart_force_y", 1);
  ext_cart_force_pub_z = node_handle.advertise<std_msgs::Float64>("ext_cart_force_z", 1);


  x_dot_pub = node_handle.advertise<std_msgs::Float64>("x_dot", 1);
  y_dot_pub = node_handle.advertise<std_msgs::Float64>("y_dot", 1);
  z_dot_pub = node_handle.advertise<std_msgs::Float64>("z_dot", 1);

  cart_ext_pub1 = node_handle.advertise<std_msgs::Float64>("cart_ext_pub1", 1);
  cart_ext_pub2 = node_handle.advertise<std_msgs::Float64>("cart_ext_pub2", 1);
  cart_ext_pub3 = node_handle.advertise<std_msgs::Float64>("cart_ext_pub3", 1);
  cart_ext_sum = node_handle.advertise<std_msgs::Float64>("cart_ext_sum", 1);

  external_wrench_pub = node_handle.advertise<std_msgs::Float64MultiArray>("external_wrench", 1);
  external_signal_pub = node_handle.advertise<std_msgs::Float64MultiArray>("external_signal", 1);

}

bool PandaJointVelocityContactController::setupController(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle){
  
  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "PandaJointVelocityContactController: Error getting velocity joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("PandaJointVelocityContactController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("PandaJointVelocityContactController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "PandaJointVelocityContactController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  last_time_called = ros::Time::now().toSec();

  sub_command_ = node_handle.subscribe<std_msgs::Float64MultiArray>("command", 10, &PandaJointVelocityContactController::jointCommandCb, this);


  auto* model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("PandaJointVelocityContactController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle("panda_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "PandaJointVelocityContactController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("PandaJointVelocityContactController: Could not get state interface from hardware");
    return false;
  }

  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle("panda_robot"));

  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "PandaJointVelocityContactController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;

}


void PandaJointVelocityContactController::setSignalParsers( ros::NodeHandle& node_handle){
  this->signal_parser_x = zScore(node_handle, "x");
  this->signal_parser_y = zScore(node_handle, "y");
  this->signal_parser_z = zScore(node_handle, "z");
  this->signal_parser_std_dev = zScore(node_handle, "std_dev");
}

void PandaJointVelocityContactController::setValueTrackers(){
  for (int i = 0; i < 7; i++)
  {
    this->prev_qd[i] = 0.0;
  }
  this->loops_without_signal = 0;
  this->loops_with_signal = 0;
}

Eigen::MatrixXd PandaJointVelocityContactController::getCartesianVelocity(
                                       Eigen::Map<Eigen::Matrix<double, 6, 7>> &jacobian,
                                       Eigen::Map<Eigen::Matrix<double, 7, 1>> &q_dot,
                                       bool publish_velocity){
  Eigen::MatrixXd x_dot = jacobian * q_dot;
  if(publish_velocity){

    std_msgs::Float64 x_dot_msg;
    x_dot_msg.data = x_dot(0);

    std_msgs::Float64 y_dot_msg;
    y_dot_msg.data = x_dot(1);

    std_msgs::Float64 z_dot_msg;
    z_dot_msg.data = x_dot(2);

    x_dot_pub.publish(x_dot_msg);
    y_dot_pub.publish(y_dot_msg);
    z_dot_pub.publish(z_dot_msg);  
  }
  return x_dot;
}

void PandaJointVelocityContactController::updateSignalThresholds(Eigen::MatrixXd& x_dot){
  signal_parser_x.updateThreshold(x_dot(0));
  signal_parser_y.updateThreshold(x_dot(1));
  signal_parser_z.updateThreshold(x_dot(2));
}

Eigen::Map<Eigen::Matrix<double, 7, 1>> PandaJointVelocityContactController::updateJointAcceleration(franka::RobotState &robot_state, 
                                                             std::array<double, 7> &joint_accs,
                                                             const ros::Duration& period){
    for (int i = 0; i < 7; i++){ 
    joint_accs[i] = (robot_state.dq[i] - prev_qd[i]) / (period.toSec() * 10);
    prev_qd[i] = robot_state.dq[i];
  }
  
  return Eigen::Map<Eigen::Matrix<double, 7, 1>>(joint_accs.data());
}

  Eigen::MatrixXd PandaJointVelocityContactController::getExternalWrench(
                                    Eigen::Map<Eigen::Matrix<double, 6, 7>>& pinv,
                                    Eigen::Map<Eigen::Matrix<double, 7, 1>>& tau_measured,
                                    Eigen::Map<Eigen::Matrix<double, 7, 1>>& gravity,
                                    Eigen::Map<Eigen::Matrix<double, 7, 1>>& coriolis_matrix,
                                    Eigen::Map<Eigen::Matrix<double, 7, 7>>& mass_matrix,
                                    Eigen::Map<Eigen::Matrix<double, 7, 1>>& ddq,
                                    Eigen::Map<Eigen::Matrix<double, 6, 1>>& wrench,
                                    bool publish_values){
  
  Eigen::VectorXd tau_ext = tau_measured - gravity - tau_ext_initial_;
  Eigen::MatrixXd ext_cartesian_wrench = (pinv  * (tau_measured - gravity -  coriolis_matrix - (0.1 * (mass_matrix * ddq)))) - wrench;

  std_msgs::Float64 F_x_msg;
  F_x_msg.data = ext_cartesian_wrench(0);

  std_msgs::Float64 F_y_msg;
  F_y_msg.data = ext_cartesian_wrench(1);

  std_msgs::Float64 F_z_msg;
  F_z_msg.data = ext_cartesian_wrench(2);

  if(publish_values){
    ext_cart_force_pub_x.publish(F_x_msg);
    ext_cart_force_pub_y.publish(F_y_msg);
    ext_cart_force_pub_z.publish(F_z_msg);
  }
  return ext_cartesian_wrench;

}

void PandaJointVelocityContactController::starting(const ros::Time& /* time */) {
    elapsed_time_ = ros::Duration(0.0);
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 7> gravity_array = model_handle_->getGravity();
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
    // Bias correction for the current external torque
    tau_ext_initial_ = tau_measured - gravity;
}

void PandaJointVelocityContactController::update(const ros::Time& time,
                                          const ros::Duration& period) {
    // Get current Franka::RobotState
    elapsed_time_ += period;
    franka::RobotState robot_state = state_handle_->getRobotState();

    // Get robot dynamics
    // TODO(peasant98): How can we move this so it's just one function call?
    std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    std::array<double, 7> gravity_array = model_handle_->getGravity();
    std::array<double, 49> mass = model_handle_->getMass();
    std::array<double, 7> coriolis = model_handle_->getCoriolis();
    std::array<double, 7> joint_accs;

    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_dot(robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 6, 1>> wrench(robot_state.O_F_ext_hat_K.data());
    Eigen::Map<Eigen::Matrix<double, 7, 7>> mass_matrix(mass.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis_matrix(coriolis.data());

    // get cartesian velocities
    Eigen::MatrixXd x_dot = getCartesianVelocity(jacobian, q_dot, true);
    updateSignalThresholds(x_dot);

    // invert jac transpose
    Eigen::MatrixXd jacobian_transpose_pinv = jacobian.transpose().completeOrthogonalDecomposition().pseudoInverse();
    Eigen::Map<Eigen::Matrix<double, 6, 7>> pinv(jacobian_transpose_pinv.data());
    
    // Calculate the external wrench
    Eigen::Map<Eigen::Matrix<double, 7, 1>> ddq = updateJointAcceleration(robot_state, joint_accs, period);
    Eigen::MatrixXd ext_cartesian_wrench = getExternalWrench(pinv, tau_measured, gravity,
                                                            coriolis_matrix, mass_matrix, 
                                                            ddq, wrench, true);


    std::tuple<bool, float> x_signal_ret = signal_parser_x.getSignal(ext_cartesian_wrench(0));
    std::tuple<bool, float> y_signal_ret = signal_parser_y.getSignal(ext_cartesian_wrench(1));
    std::tuple<bool, float> z_signal_ret = signal_parser_z.getSignal(ext_cartesian_wrench(2));
    // std::tuple<bool, float> std_dev_signal_ret = signal_parser_std_dev.getSignal(ext_cartesian_wrench(2));
    
    std::vector<double> external_wrench{wrench(0), wrench(1), wrench(2)};
    std::vector<double> external_signal{std::get<1>(x_signal_ret), std::get<1>(y_signal_ret), std::get<1>(z_signal_ret)};
    
    std_msgs::Float64MultiArray external_wrench_msg;
    std_msgs::Float64MultiArray external_signal_msg;
    
    external_wrench_msg.data = external_wrench;
    external_signal_msg.data = external_signal;

    external_wrench_pub.publish(external_wrench_msg);
    external_signal_pub.publish(external_signal_msg);

    if (std::get<0>(y_signal_ret)){
      loops_with_signal++;
      if (loops_with_signal > 10){
        loops_without_signal = 0;
        pause_movement = true;
      }

    } else{
      loops_with_signal = 0;
    }

    // If there is no command for more than 0.1 sec, set velocity to 0.0
    if (ros::Time::now().toSec() - last_time_called > 0.05) {
        for (int i = 0; i < 7; i++) velocity_joint_handles_[i].setCommand(0.0);
    } else {  // If command recieved, send the command to the controller
        for (int i = 0; i < 7; i++) {
            // Print out to the terminal just for the 1st joint for debugging.
            // Order: Commanded Joint Velocity << Acutal Commanded Joint Velocity << Current Joint Velocity
            // if (i == 0) ROS_INFO_STREAM("Panda_joint" << i+1 << " " << joint_velocities[i] << " " << robot_state.dq_d[i] << " " << robot_state.dq[i]);
            // Send command
            velocity_joint_handles_[i].setCommand(joint_velocities[i]);
        }
    }
}

void PandaJointVelocityContactController::stopping(const ros::Time& /*time*/) {
    // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
    // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAUL`T
    // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void PandaJointVelocityContactController::jointCommandCb(const std_msgs::Float64MultiArray::ConstPtr& joint_velocity_commands) {
    if (joint_velocity_commands->data.size() != 7) {
        ROS_ERROR_STREAM("PandaJointVelocityContactController: Wrong number of joint velocity commands, got "
                        << joint_velocity_commands->data.size() << " instead of 7 commands!");
    }

    // Receive Joint Velocity Commands from a topic and save them in joint_velocities.
    for (int i = 0; i < 7; i++) joint_velocities[i] = joint_velocity_commands->data[i];
    // Save the time when the last topic was published
    last_time_called = ros::Time::now().toSec();
}


}  // namespace hiro_panda

PLUGINLIB_EXPORT_CLASS(hiro_panda::PandaJointVelocityContactController,
                       controller_interface::ControllerBase)
