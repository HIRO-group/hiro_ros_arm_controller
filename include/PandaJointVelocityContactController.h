// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>

#include <Eigen/Core>


#include <hiro_ros_arm_controller/zScore.h>


namespace hiro_panda {

class PandaJointVelocityContactController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::VelocityJointInterface,
                                           franka_hw::FrankaModelInterface,
                                           franka_hw::FrankaStateInterface> {
 public:
    bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
    void update(const ros::Time&, const ros::Duration& period) override;
    void starting(const ros::Time&) override;
    void stopping(const ros::Time&) override;

   int loops_without_signal;
   int loops_with_signal;

 private:


   // helper functions to clean up: init
   bool setupController(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle);
   void setPublishers( ros::NodeHandle& node_handle);
   void setSignalParsers( ros::NodeHandle& node_handle);
   void setValueTrackers();



   // helper functions to clean up: update
   Eigen::MatrixXd getCartesianVelocity(Eigen::Map<Eigen::Matrix<double, 6, 7>> &jacobian,
                                          Eigen::Map<Eigen::Matrix<double, 7, 1>> &q_dot,
                                          bool publish_velocity);

   void updateSignalThresholds(Eigen::MatrixXd& x_dot);
   Eigen::Map<Eigen::Matrix<double, 7, 1>> updateJointAcceleration(franka::RobotState &robot_state, 
                                 std::array<double, 7> &joint_accs, 
                                 const ros::Duration& period);

   Eigen::MatrixXd getExternalWrench(Eigen::Map<Eigen::Matrix<double, 6, 7>>& pinv,
                                       Eigen::Map<Eigen::Matrix<double, 7, 1>>& tau_measured,
                                       Eigen::Map<Eigen::Matrix<double, 7, 1>>& gravity,
                                       Eigen::Map<Eigen::Matrix<double, 7, 1>>& coriolis_matrix,
                                       Eigen::Map<Eigen::Matrix<double, 7, 7>>& mass_matrix,
                                       Eigen::Map<Eigen::Matrix<double, 7, 1>>& ddq,
                                       Eigen::Map<Eigen::Matrix<double, 6, 1>>& wrench,
                                       bool publish_values);


   std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
   std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
   hardware_interface::VelocityJointInterface* velocity_joint_interface_;
   std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
   ros::Duration elapsed_time_;

   double desired_mass_{0.0};
   double target_mass_{0.0};
   double k_p_{0.0};
   double k_i_{0.0};
   double target_k_p_{0.0};
   double target_k_i_{0.0};
   double filter_gain_{0.001};
   Eigen::Matrix<double, 7, 1> tau_ext_initial_;
   std::array<double, 7> prev_qd;
   Eigen::Matrix<double, 7, 1> tau_error_;
   static constexpr double kDeltaTauMax{1.0};


   void jointCommandCb(const std_msgs::Float64MultiArray::ConstPtr& joint_velocity_commands);
   ros::Subscriber sub_command_;
   std::array<double, 7> joint_velocities{};
   double last_time_called;


   // external cartesian forces  Publishers
   ros::Publisher ext_cart_force_pub_x;
   ros::Publisher ext_cart_force_pub_y;
   ros::Publisher ext_cart_force_pub_z;


   ros::Publisher x_dot_pub;
   ros::Publisher y_dot_pub;
   ros::Publisher z_dot_pub;

   ros::Publisher cart_ext_pub1;
   ros::Publisher cart_ext_pub2;
   ros::Publisher cart_ext_pub3;
   ros::Publisher cart_ext_sum;

   ros::Publisher signal_pub;
   ros::Publisher mean_pub;
   ros::Publisher std_dev_positive_pub;
   ros::Publisher std_dev_negative_pub;

   ros::Publisher external_wrench_pub;


   zScore signal_parser_x;
   zScore signal_parser_y;
   zScore signal_parser_z;
   // zScore signal_parser_sum;


   bool pause_movement = false;
};

}  // namespace hiro_panda
