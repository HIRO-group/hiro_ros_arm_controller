// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cylindrical_obstacles.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

#include "pseudo_inversion.h"
#include <visualization_msgs/Marker.h>


// Caleb Escobedo - This code was shared and belongs to Kelly Merckaert, if this code is
// expanded on or used for any research contact kelly.merckaert@vub.be


namespace hiro_panda{

bool CylindricalObstacles::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CylindricalObstacles: Could not read parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CylindricalObstacles: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("Kp", Kp_) || Kp_.size() != 7) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("Kd", Kd_) || Kd_.size() != 7) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no d_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("pred_samples", pred_samples_)) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no pred_samples parameter provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("pred_samplingtime", pred_samplingtime_)) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no pred_samplingtime parameter provided, aborting "
        "controller init!");
    return false;
  }
  if (!node_handle.getParam("pred_interval", pred_interval_)) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no pred_interval parameter provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("tau_limit", tau_limit_) || tau_limit_.size() != 7) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no tau_limit_ parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("kappa_tau", kappa_tau_)) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no kappa_tau parameter provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("delta_tau", delta_tau_)) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no delta_tau parameter provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("dotq_limit", dotq_limit_) || dotq_limit_.size() != 7) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no dotq_limit_ parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("kappa_dotq", kappa_dotq_)) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no kappa_dotq parameter provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("delta_dotq", delta_dotq_)) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no delta_dotq parameter provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("q_lowerlimit", q_lowerlimit_) || q_lowerlimit_.size() != 7) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no q_lowerlimit parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("q_upperlimit", q_upperlimit_) || q_upperlimit_.size() != 7) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no q_upperlimit parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("kappa_q", kappa_q_)) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no kappa_q parameter provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("delta_q", delta_q_)) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no delta_q parameter provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("eta", eta_)) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no eta parameter provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("zeta_q", zeta_q_)) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no zeta_q parameter provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("dotp_EE_limit", dotp_EE_limit_) || dotp_EE_limit_.size() != 2) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no dotp_EE_limit parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("kappa_dotp_EE", kappa_dotp_EE_)) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no kappa_dotp_EE parameter provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("delta_dotp_EE", delta_dotp_EE_)) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no delta_dotp_EE parameter provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("kappa_sphere", kappa_sphere_)) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no kappa_sphere parameter provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("delta_sphere", delta_sphere_)) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no delta_sphere parameter provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("zeta_sphere", zeta_sphere_)) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no zeta_sphere parameter provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("kappa_cylinder", kappa_cylinder_)) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no kappa_cylinder parameter provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("delta_cylinder", delta_cylinder_)) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no delta_cylinder parameter provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("zeta_cylinder", zeta_cylinder_)) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no zeta_cylinder parameter provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("kappa_wall", kappa_wall_)) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no kappa_wall parameter provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("delta_wall", delta_wall_)) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no delta_wall parameter provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("zeta_wall", zeta_wall_)) {
    ROS_ERROR(
        "CylindricalObstacles:  Invalid or no zeta_wall parameter provided, aborting "
        "controller init!");
    return false;
  }

  double publish_rate(100.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("CylindricalObstacles: publish_rate not found. Defaulting to "
                    << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CylindricalObstacles: Could not get state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
      state_interface->getHandle(arm_id + "_robot"));
    // std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    // for (size_t i = 0; i < q_start.size(); i++) {
    //   if (std::abs(state_handle_.getRobotState().q_d[i] - q_start[i]) > 0.1) {
    //     ROS_ERROR_STREAM(
    //         "CylindricalObstacles: Robot is not in the expected starting position for "
    //         "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
    //         "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
    //     return false;
    //   }
    // }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CylindricalObstacles: Exception getting state handle: " << e.what());
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CylindricalObstacles: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CylindricalObstacles: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CylindricalObstacles: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CylindricalObstacles: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  jointstates_publisher_ = node_handle.advertise<hiro_ros_arm_controller::JointStates>("/Panda/joint_states", 50);
  trajpredictions_publisher_ = node_handle.advertise<hiro_ros_arm_controller::TrajectoryPredictions>("/Panda/trajectory_predictions",1000);
  DSM_publisher_ = node_handle.advertise<hiro_ros_arm_controller::DSM>("/Panda/DSM", 50);
  marker_sphere1_pub_ = node_handle.advertise<visualization_msgs::Marker>("visualization_marker",1);
  marker_sphere1_delta_pub_ = node_handle.advertise<visualization_msgs::Marker>("visualization_marker",1);
  marker_sphere1_zeta_pub_ = node_handle.advertise<visualization_msgs::Marker>("visualization_marker",1);
  marker_cylinder1_pub_ = node_handle.advertise<visualization_msgs::Marker>("visualization_marker",1);
  marker_cylinder1_delta_pub_ = node_handle.advertise<visualization_msgs::Marker>("visualization_marker",1);
  marker_cylinder1_zeta_pub_ = node_handle.advertise<visualization_msgs::Marker>("visualization_marker",1);
  marker_cylinder2_pub_ = node_handle.advertise<visualization_msgs::Marker>("visualization_marker",1);
  marker_cylinder2_delta_pub_ = node_handle.advertise<visualization_msgs::Marker>("visualization_marker",1);
  marker_cylinder2_zeta_pub_ = node_handle.advertise<visualization_msgs::Marker>("visualization_marker",1);
  marker_wall1_pub_ = node_handle.advertise<visualization_msgs::Marker>("visualization_marker",10);
  marker_wall2_pub_ = node_handle.advertise<visualization_msgs::Marker>("visualization_marker",10);
  marker_wall3_pub_ = node_handle.advertise<visualization_msgs::Marker>("visualization_marker",10);
  marker_wall4_pub_ = node_handle.advertise<visualization_msgs::Marker>("visualization_marker",10);

  return true;
}

void CylindricalObstacles::starting(const ros::Time& /*time*/ ) {
  q_init_ = state_handle_->getRobotState().q;
  q_v_ = q_init_;
  elapsed_time_ = ros::Duration(0.0);
  elapsed_time_trajpred_ = ros::Duration(0.0);

  obst_sphere1_radius_ = 0.10;
  // obst_sphere1_center_ = {0.3, 0.3, 0.45};
  obst_sphere1_center_ = {0.3, 0.3, 0.65};


  obst_cylinder1_center_ = {0.0, 0.6, 0.4};
  obst_cylinder1_radius_ = 0.10;
  obst_cylinder1_height_ = 0.8;
  obst_cylinder2_center_ = {0.6, -0.3, 0.4};
  obst_cylinder2_radius_ = 0.10;
  obst_cylinder2_height_ = 0.8;

  cylinder_centers_.block(0,0,3,1) = obst_cylinder1_center_;
  cylinder_centers_.block(0,1,3,1) = obst_cylinder2_center_;
  cylinder_radii_(0) = obst_cylinder1_radius_;
  cylinder_radii_(1) = obst_cylinder2_radius_;
  cylinder_heights_(0) = obst_cylinder1_height_;
  cylinder_heights_(1) = obst_cylinder2_height_;


  // q_r1_wall_ = {1.32102, -0.0135195, 1.19831, -2.1813, 0.12326, 2.20064, 0.839673};
  // wall1_normal_ = {-0.7071, 0.7071, 0};
  // wall1_distance_ = 0.5;

  q_r1_wall_ = {0.579077, 0.396427, 0.18958, -1.75499, -0.0941473, 2.14948, 0.753129};
  q_r2_wall_ = {-0.265793, 0.201306, -0.309463, -1.8378, 0.098337, 2.04933, 0.916665};
  q_r3_wall_ = {-0.782785, 0.0759472, -1.44927, -2.07567, 0.0656047, 2.16614, 0.895472};
  q_r4_wall_ = {1.32102, -0.0135195, 1.19831, -2.1813, 0.12326, 2.20064, 0.839673};
  q_r1_cylinder_ = {1.24372, 0.160332, 0.297304, -1.84731, 0.0692216, 1.97046, 0.602016};

  wall1_normal_ = {0.7071, 0.7071, 0};
  wall1_distance_ = 0.5;
  wall2_normal_ = {0.7071, -0.7071, 0};
  wall2_distance_ = 0.5;
  wall3_normal_ = {-0.7071, -0.7071, 0};
  wall3_distance_ = 0.5;
  wall4_normal_ = {-0.7071, 0.7071, 0};
  wall4_distance_ = 0.5;

  wall_normals_.block(0,0,3,1) = wall4_normal_;
  wall_normals_.block(0,1,3,1) = wall2_normal_;
  wall_normals_.block(0,2,3,1) = wall3_normal_;
  wall_normals_.block(0,3,3,1) = wall1_normal_;
  wall_distances_ = 0.5;


}

void CylindricalObstacles::update(const ros::Time& /*time*/,
                                             const ros::Duration& period) {
  q_r1_wall_ = {0.579077, 0.396427, 0.18958, -1.75499, -0.0941473, 2.14948, 0.753129};
  q_r2_wall_ = {-0.265793, 0.201306, -0.309463, -1.8378, 0.098337, 2.04933, 0.916665};
  q_r3_wall_ = {-0.782785, 0.0759472, -1.44927, -2.07567, 0.0656047, 2.16614, 0.895472};
  q_r4_wall_ = {1.32102, -0.0135195, 1.19831, -2.1813, 0.12326, 2.20064, 0.839673};
  q_r1_cylinder_ = {1.24372, 0.160332, 0.297304, -1.84731, 0.0692216, 1.97046, 0.602016};

  /*** VISUALIZATION SPHERE ***/
  // uint32_t sphere_shape = visualization_msgs::Marker::SPHERE;
  // visualization_msgs::Marker marker_sphere1;
  // marker_sphere1.header.frame_id = "panda_link0";
  // marker_sphere1.header.stamp = ros::Time::now();
  // marker_sphere1.ns = "shape_1";
  // marker_sphere1.id = 0;
  // marker_sphere1.type = sphere_shape;
  // marker_sphere1.action = visualization_msgs::Marker::ADD;
  // marker_sphere1.pose.position.x = obst_sphere1_center_[0]; //model_handle_->getPose(franka::Frame::kEndEffector)[12];
  // marker_sphere1.pose.position.y = obst_sphere1_center_[1]; //model_handle_->getPose(franka::Frame::kEndEffector)[13];
  // marker_sphere1.pose.position.z = obst_sphere1_center_[2]; //model_handle_->getPose(franka::Frame::kEndEffector)[14];
  // marker_sphere1.pose.orientation.x = 0.0;
  // marker_sphere1.pose.orientation.y = 0.0;
  // marker_sphere1.pose.orientation.z = 0.0;
  // marker_sphere1.pose.orientation.w = 1.0;
  // marker_sphere1.scale.x = obst_sphere1_radius_; //+delta_sphere_; //0.25;
  // marker_sphere1.scale.y = obst_sphere1_radius_; //+delta_sphere_; //0.25;
  // marker_sphere1.scale.z = obst_sphere1_radius_; //+delta_sphere_; //0.5;
  // marker_sphere1.color.r = 1.0f;
  // marker_sphere1.color.g = 0.0f;
  // marker_sphere1.color.b = 0.0f;
  // marker_sphere1.color.a = 0.8;
  // marker_sphere1.lifetime = ros::Duration();
  // marker_sphere1_pub_.publish(marker_sphere1);

  // visualization_msgs::Marker marker_sphere1_delta;
  // marker_sphere1_delta.header.frame_id = "panda_link0";
  // marker_sphere1_delta.header.stamp = ros::Time::now();
  // marker_sphere1_delta.ns = "shape_2";
  // marker_sphere1_delta.id = 0;
  // marker_sphere1_delta.type = sphere_shape;
  // marker_sphere1_delta.action = visualization_msgs::Marker::ADD;
  // marker_sphere1_delta.pose.position.x = obst_sphere1_center_[0]; //model_handle_->getPose(franka::Frame::kEndEffector)[12];
  // marker_sphere1_delta.pose.position.y = obst_sphere1_center_[1]; //model_handle_->getPose(franka::Frame::kEndEffector)[13];
  // marker_sphere1_delta.pose.position.z = obst_sphere1_center_[2]; //model_handle_->getPose(franka::Frame::kEndEffector)[14];
  // marker_sphere1_delta.pose.orientation.x = 0.0;
  // marker_sphere1_delta.pose.orientation.y = 0.0;
  // marker_sphere1_delta.pose.orientation.z = 0.0;
  // marker_sphere1_delta.pose.orientation.w = 1.0;
  // marker_sphere1_delta.scale.x = obst_sphere1_radius_+delta_sphere_; //0.25;
  // marker_sphere1_delta.scale.y = obst_sphere1_radius_+delta_sphere_; //0.25;
  // marker_sphere1_delta.scale.z = obst_sphere1_radius_+delta_sphere_; //0.5;
  // marker_sphere1_delta.color.r = 1.0f;
  // marker_sphere1_delta.color.g = 0.0f;
  // marker_sphere1_delta.color.b = 0.0f;
  // marker_sphere1_delta.color.a = 0.2;
  // marker_sphere1_delta.lifetime = ros::Duration();
  // marker_sphere1_delta_pub_.publish(marker_sphere1_delta);

  // visualization_msgs::Marker marker_sphere1_zeta;
  // marker_sphere1_zeta.header.frame_id = "panda_link0";
  // marker_sphere1_zeta.header.stamp = ros::Time::now();
  // marker_sphere1_zeta.ns = "shape_3";
  // marker_sphere1_zeta.id = 0;
  // marker_sphere1_zeta.type = sphere_shape;
  // marker_sphere1_zeta.action = visualization_msgs::Marker::ADD;
  // marker_sphere1_zeta.pose.position.x = obst_sphere1_center_[0]; //model_handle_->getPose(franka::Frame::kEndEffector)[12];
  // marker_sphere1_zeta.pose.position.y = obst_sphere1_center_[1]; //model_handle_->getPose(franka::Frame::kEndEffector)[13];
  // marker_sphere1_zeta.pose.position.z = obst_sphere1_center_[2]; //model_handle_->getPose(franka::Frame::kEndEffector)[14];
  // marker_sphere1_zeta.pose.orientation.x = 0.0;
  // marker_sphere1_zeta.pose.orientation.y = 0.0;
  // marker_sphere1_zeta.pose.orientation.z = 0.0;
  // marker_sphere1_zeta.pose.orientation.w = 1.0;
  // marker_sphere1_zeta.scale.x = obst_sphere1_radius_+zeta_sphere_; //0.25;
  // marker_sphere1_zeta.scale.y = obst_sphere1_radius_+zeta_sphere_; //0.25;
  // marker_sphere1_zeta.scale.z = obst_sphere1_radius_+zeta_sphere_; //0.5;
  // marker_sphere1_zeta.color.r = 1.0f;
  // marker_sphere1_zeta.color.g = 0.4f;
  // marker_sphere1_zeta.color.b = 0.0f;
  // marker_sphere1_zeta.color.a = 0.2;
  // marker_sphere1_zeta.lifetime = ros::Duration();
  // marker_sphere1_zeta_pub_.publish(marker_sphere1_zeta);

  /*** VISUALIZATION CYLINDER ***/
  uint32_t cylinder_shape = visualization_msgs::Marker::CYLINDER;

  visualization_msgs::Marker marker_cylinder1;
  marker_cylinder1.header.frame_id = "panda_link0";
  marker_cylinder1.header.stamp = ros::Time::now();
  marker_cylinder1.ns = "shape_1";
  marker_cylinder1.id = 0;
  marker_cylinder1.type = cylinder_shape;
  marker_cylinder1.action = visualization_msgs::Marker::ADD;
  marker_cylinder1.pose.position.x = obst_cylinder1_center_[0]; //model_handle_->getPose(franka::Frame::kEndEffector)[12];
  marker_cylinder1.pose.position.y = obst_cylinder1_center_[1]; //model_handle_->getPose(franka::Frame::kEndEffector)[13];
  marker_cylinder1.pose.position.z = obst_cylinder1_center_[2]; //model_handle_->getPose(franka::Frame::kEndEffector)[14];
  marker_cylinder1.pose.orientation.x = 0.0;
  marker_cylinder1.pose.orientation.y = 0.0;
  marker_cylinder1.pose.orientation.z = 0.0;
  marker_cylinder1.pose.orientation.w = 1.0;
  marker_cylinder1.scale.x = 2.0*obst_cylinder1_radius_; //+delta_sphere_; //0.25;
  marker_cylinder1.scale.y = 2.0*obst_cylinder1_radius_; //+delta_sphere_; //0.25;
  marker_cylinder1.scale.z = obst_cylinder1_height_; //+delta_sphere_; //0.5;
  marker_cylinder1.color.r = 1.0f;
  marker_cylinder1.color.g = 0.0f;
  marker_cylinder1.color.b = 0.0f;
  marker_cylinder1.color.a = 0.8;
  marker_cylinder1.lifetime = ros::Duration();
  marker_cylinder1_pub_.publish(marker_cylinder1);

  visualization_msgs::Marker marker_cylinder1_delta;
  marker_cylinder1_delta.header.frame_id = "panda_link0";
  marker_cylinder1_delta.header.stamp = ros::Time::now();
  marker_cylinder1_delta.ns = "shape_2";
  marker_cylinder1_delta.id = 0;
  marker_cylinder1_delta.type = cylinder_shape;
  marker_cylinder1_delta.action = visualization_msgs::Marker::ADD;
  marker_cylinder1_delta.pose.position.x = obst_cylinder1_center_[0]; //model_handle_->getPose(franka::Frame::kEndEffector)[12];
  marker_cylinder1_delta.pose.position.y = obst_cylinder1_center_[1]; //model_handle_->getPose(franka::Frame::kEndEffector)[13];
  marker_cylinder1_delta.pose.position.z = obst_cylinder1_center_[2]; //model_handle_->getPose(franka::Frame::kEndEffector)[14];
  marker_cylinder1_delta.pose.orientation.x = 0.0;
  marker_cylinder1_delta.pose.orientation.y = 0.0;
  marker_cylinder1_delta.pose.orientation.z = 0.0;
  marker_cylinder1_delta.pose.orientation.w = 1.0;
  marker_cylinder1_delta.scale.x = 2.0*(obst_cylinder1_radius_+delta_cylinder_); //0.25;
  marker_cylinder1_delta.scale.y = 2.0*(obst_cylinder1_radius_+delta_cylinder_); //0.25;
  marker_cylinder1_delta.scale.z = obst_cylinder1_height_+2.0*delta_cylinder_; //0.5;
  marker_cylinder1_delta.color.r = 1.0f;
  marker_cylinder1_delta.color.g = 0.0f;
  marker_cylinder1_delta.color.b = 0.0f;
  marker_cylinder1_delta.color.a = 0.2;
  marker_cylinder1_delta.lifetime = ros::Duration();
  marker_cylinder1_delta_pub_.publish(marker_cylinder1_delta);

  visualization_msgs::Marker marker_cylinder1_zeta;
  marker_cylinder1_zeta.header.frame_id = "panda_link0";
  marker_cylinder1_zeta.header.stamp = ros::Time::now();
  marker_cylinder1_zeta.ns = "shape_3";
  marker_cylinder1_zeta.id = 0;
  marker_cylinder1_zeta.type = cylinder_shape;
  marker_cylinder1_zeta.action = visualization_msgs::Marker::ADD;
  marker_cylinder1_zeta.pose.position.x = obst_cylinder1_center_[0]; //model_handle_->getPose(franka::Frame::kEndEffector)[12];
  marker_cylinder1_zeta.pose.position.y = obst_cylinder1_center_[1]; //model_handle_->getPose(franka::Frame::kEndEffector)[13];
  marker_cylinder1_zeta.pose.position.z = obst_cylinder1_center_[2]; //model_handle_->getPose(franka::Frame::kEndEffector)[14];
  marker_cylinder1_zeta.pose.orientation.x = 0.0;
  marker_cylinder1_zeta.pose.orientation.y = 0.0;
  marker_cylinder1_zeta.pose.orientation.z = 0.0;
  marker_cylinder1_zeta.pose.orientation.w = 1.0;
  marker_cylinder1_zeta.scale.x = 2.0*(obst_cylinder1_radius_+zeta_cylinder_); //0.25;
  marker_cylinder1_zeta.scale.y = 2.0*(obst_cylinder1_radius_+zeta_cylinder_); //0.25;
  marker_cylinder1_zeta.scale.z = obst_cylinder1_height_+2.0*zeta_cylinder_; //0.5;
  marker_cylinder1_zeta.color.r = 1.0f;
  marker_cylinder1_zeta.color.g = 0.4f;
  marker_cylinder1_zeta.color.b = 0.0f;
  marker_cylinder1_zeta.color.a = 0.2;
  marker_cylinder1_zeta.lifetime = ros::Duration();
  marker_cylinder1_zeta_pub_.publish(marker_cylinder1_zeta);

  visualization_msgs::Marker marker_cylinder2;
  marker_cylinder2.header.frame_id = "panda_link0";
  marker_cylinder2.header.stamp = ros::Time::now();
  marker_cylinder2.ns = "shape_4";
  marker_cylinder2.id = 0;
  marker_cylinder2.type = cylinder_shape;
  marker_cylinder2.action = visualization_msgs::Marker::ADD;
  marker_cylinder2.pose.position.x = obst_cylinder2_center_[0]; //model_handle_->getPose(franka::Frame::kEndEffector)[12];
  marker_cylinder2.pose.position.y = obst_cylinder2_center_[1]; //model_handle_->getPose(franka::Frame::kEndEffector)[13];
  marker_cylinder2.pose.position.z = obst_cylinder2_center_[2]; //model_handle_->getPose(franka::Frame::kEndEffector)[14];
  marker_cylinder2.pose.orientation.x = 0.0;
  marker_cylinder2.pose.orientation.y = 0.0;
  marker_cylinder2.pose.orientation.z = 0.0;
  marker_cylinder2.pose.orientation.w = 1.0;
  marker_cylinder2.scale.x = 2.0*obst_cylinder2_radius_; //+delta_sphere_; //0.25;
  marker_cylinder2.scale.y = 2.0*obst_cylinder2_radius_; //+delta_sphere_; //0.25;
  marker_cylinder2.scale.z = obst_cylinder2_height_; //+delta_sphere_; //0.5;
  marker_cylinder2.color.r = 1.0f;
  marker_cylinder2.color.g = 0.0f;
  marker_cylinder2.color.b = 0.0f;
  marker_cylinder2.color.a = 0.8;
  marker_cylinder2.lifetime = ros::Duration();
  marker_cylinder2_pub_.publish(marker_cylinder2);

  visualization_msgs::Marker marker_cylinder2_delta;
  marker_cylinder2_delta.header.frame_id = "panda_link0";
  marker_cylinder2_delta.header.stamp = ros::Time::now();
  marker_cylinder2_delta.ns = "shape_5";
  marker_cylinder2_delta.id = 0;
  marker_cylinder2_delta.type = cylinder_shape;
  marker_cylinder2_delta.action = visualization_msgs::Marker::ADD;
  marker_cylinder2_delta.pose.position.x = obst_cylinder2_center_[0]; //model_handle_->getPose(franka::Frame::kEndEffector)[12];
  marker_cylinder2_delta.pose.position.y = obst_cylinder2_center_[1]; //model_handle_->getPose(franka::Frame::kEndEffector)[13];
  marker_cylinder2_delta.pose.position.z = obst_cylinder2_center_[2]; //model_handle_->getPose(franka::Frame::kEndEffector)[14];
  marker_cylinder2_delta.pose.orientation.x = 0.0;
  marker_cylinder2_delta.pose.orientation.y = 0.0;
  marker_cylinder2_delta.pose.orientation.z = 0.0;
  marker_cylinder2_delta.pose.orientation.w = 1.0;
  marker_cylinder2_delta.scale.x = 2.0*(obst_cylinder2_radius_+delta_cylinder_); //0.25;
  marker_cylinder2_delta.scale.y = 2.0*(obst_cylinder2_radius_+delta_cylinder_); //0.25;
  marker_cylinder2_delta.scale.z = obst_cylinder2_height_+2.0*delta_cylinder_; //0.5;
  marker_cylinder2_delta.color.r = 1.0f;
  marker_cylinder2_delta.color.g = 0.0f;
  marker_cylinder2_delta.color.b = 0.0f;
  marker_cylinder2_delta.color.a = 0.2;
  marker_cylinder2_delta.lifetime = ros::Duration();
  marker_cylinder2_delta_pub_.publish(marker_cylinder2_delta);

  visualization_msgs::Marker marker_cylinder2_zeta;
  marker_cylinder2_zeta.header.frame_id = "panda_link0";
  marker_cylinder2_zeta.header.stamp = ros::Time::now();
  marker_cylinder2_zeta.ns = "shape_6";
  marker_cylinder2_zeta.id = 0;
  marker_cylinder2_zeta.type = cylinder_shape;
  marker_cylinder2_zeta.action = visualization_msgs::Marker::ADD;
  marker_cylinder2_zeta.pose.position.x = obst_cylinder2_center_[0]; //model_handle_->getPose(franka::Frame::kEndEffector)[12];
  marker_cylinder2_zeta.pose.position.y = obst_cylinder2_center_[1]; //model_handle_->getPose(franka::Frame::kEndEffector)[13];
  marker_cylinder2_zeta.pose.position.z = obst_cylinder2_center_[2]; //model_handle_->getPose(franka::Frame::kEndEffector)[14];
  marker_cylinder2_zeta.pose.orientation.x = 0.0;
  marker_cylinder2_zeta.pose.orientation.y = 0.0;
  marker_cylinder2_zeta.pose.orientation.z = 0.0;
  marker_cylinder2_zeta.pose.orientation.w = 1.0;
  marker_cylinder2_zeta.scale.x = 2.0*(obst_cylinder2_radius_+zeta_cylinder_); //0.25;
  marker_cylinder2_zeta.scale.y = 2.0*(obst_cylinder2_radius_+zeta_cylinder_); //0.25;
  marker_cylinder2_zeta.scale.z = obst_cylinder2_height_+2.0*zeta_cylinder_; //0.5;
  marker_cylinder2_zeta.color.r = 1.0f;
  marker_cylinder2_zeta.color.g = 0.4f;
  marker_cylinder2_zeta.color.b = 0.0f;
  marker_cylinder2_zeta.color.a = 0.2;
  marker_cylinder2_zeta.lifetime = ros::Duration();
  marker_cylinder2_zeta_pub_.publish(marker_cylinder2_zeta);

  /*** VISUALIZATION WALLS ***/
  /* uint32_t wall_shape = visualization_msgs::Marker::CUBE;
  visualization_msgs::Marker marker_wall1;
  marker_wall1.header.frame_id = "panda_link0";
  marker_wall1.header.stamp = ros::Time::now();
  marker_wall1.ns = "wall_shape_1";
  marker_wall1.id = 0;
  marker_wall1.type = wall_shape;
  marker_wall1.action = visualization_msgs::Marker::ADD;
  marker_wall1.pose.position.x = 0.3535;
  marker_wall1.pose.position.y = 0.3535; //model_handle_->getPose(franka::Frame::kEndEffector)[13];
  marker_wall1.pose.position.z = 0.5;
  marker_wall1.pose.orientation.x = 0.0;
  marker_wall1.pose.orientation.y = 0.0;
  marker_wall1.pose.orientation.z = -0.3826843; //0.0;
  marker_wall1.pose.orientation.w = 0.9238792; //1.0;
  marker_wall1.scale.x = 1.0;
  marker_wall1.scale.y = 0.01;
  marker_wall1.scale.z = 1.0;
  marker_wall1.color.r = 1.0f;
  marker_wall1.color.g = 0.0f;
  marker_wall1.color.b = 0.0f;
  marker_wall1.color.a = 0.7;
  marker_wall1.lifetime = ros::Duration();
  marker_wall1_pub_.publish(marker_wall1);
  visualization_msgs::Marker marker_wall2;
  marker_wall2.header.frame_id = "panda_link0";
  marker_wall2.header.stamp = ros::Time::now();
  marker_wall2.ns = "wall_shape_2";
  marker_wall2.id = 0;
  marker_wall2.type = wall_shape;
  marker_wall2.action = visualization_msgs::Marker::ADD;
  marker_wall2.pose.position.x = 0.3535;
  marker_wall2.pose.position.y = -0.3535; //model_handle_->getPose(franka::Frame::kEndEffector)[13];
  marker_wall2.pose.position.z = 0.5;
  marker_wall2.pose.orientation.x = 0.0;
  marker_wall2.pose.orientation.y = 0.0;
  marker_wall2.pose.orientation.z = -0.3826843; //0.0;
  marker_wall2.pose.orientation.w = 0.9238792; //1.0;
  marker_wall2.scale.x = 0.01;
  marker_wall2.scale.y = 1.0;
  marker_wall2.scale.z = 1.0;
  marker_wall2.color.r = 1.0f;
  marker_wall2.color.g = 0.0f;
  marker_wall2.color.b = 0.0f;
  marker_wall2.color.a = 0.7;
  marker_wall2.lifetime = ros::Duration();
  marker_wall2_pub_.publish(marker_wall2);
  visualization_msgs::Marker marker_wall3;
  marker_wall3.header.frame_id = "panda_link0";
  marker_wall3.header.stamp = ros::Time::now();
  marker_wall3.ns = "wall_shape_3";
  marker_wall3.id = 0;
  marker_wall3.type = wall_shape;
  marker_wall3.action = visualization_msgs::Marker::ADD;
  marker_wall3.pose.position.x = -0.3535;
  marker_wall3.pose.position.y = -0.3535; //model_handle_->getPose(franka::Frame::kEndEffector)[13];
  marker_wall3.pose.position.z = 0.5;
  marker_wall3.pose.orientation.x = 0.0;
  marker_wall3.pose.orientation.y = 0.0;
  marker_wall3.pose.orientation.z = -0.3826843; //0.0;
  marker_wall3.pose.orientation.w = 0.9238792; //1.0;
  marker_wall3.scale.x = 1.0;
  marker_wall3.scale.y = 0.01;
  marker_wall3.scale.z = 1.0;
  marker_wall3.color.r = 1.0f;
  marker_wall3.color.g = 0.0f;
  marker_wall3.color.b = 0.0f;
  marker_wall3.color.a = 0.7;
  marker_wall3.lifetime = ros::Duration();
  marker_wall3_pub_.publish(marker_wall3);
  visualization_msgs::Marker marker_wall4;
  marker_wall4.header.frame_id = "panda_link0";
  marker_wall4.header.stamp = ros::Time::now();
  marker_wall4.ns = "wall_shape_4";
  marker_wall4.id = 0;
  marker_wall4.type = wall_shape;
  marker_wall4.action = visualization_msgs::Marker::ADD;
  marker_wall4.pose.position.x = -0.3535;
  marker_wall4.pose.position.y = 0.3535; //model_handle_->getPose(franka::Frame::kEndEffector)[13];
  marker_wall4.pose.position.z = 0.5;
  marker_wall4.pose.orientation.x = 0.0;
  marker_wall4.pose.orientation.y = 0.0;
  marker_wall4.pose.orientation.z = -0.3826843; //0.0;
  marker_wall4.pose.orientation.w = 0.9238792; //1.0;
  marker_wall4.scale.x = 0.01;
  marker_wall4.scale.y = 1.0;
  marker_wall4.scale.z = 1.0;
  marker_wall4.color.r = 1.0f;
  marker_wall4.color.g = 0.0f;
  marker_wall4.color.b = 0.0f;
  marker_wall4.color.a = 0.7;
  marker_wall4.lifetime = ros::Duration();
  marker_wall4_pub_.publish(marker_wall4); */

  elapsed_time_ += period;
  elapsed_time_trajpred_ += period;

  franka::RobotState robot_state = state_handle_->getRobotState();
  // std::array<double, 49> mass = model_handle_->getMass();
  std::array<double, 7> gravity = model_handle_->getGravity();

  /*** STEP REFERENCE IN SPHERE ***/
  // if (elapsed_time_.toSec() <= 2.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i];
  //   }
  // }
  // else if(elapsed_time_.toSec() > 2.0 && elapsed_time_.toSec() <= 5.0){
  //     q_r_[0] = 0.13;
  //     q_r_[1] = -1.68;
  //     q_r_[2] = -1.72;
  //     q_r_[3] = -2.05;
  //     q_r_[4] = -0.22;
  //     q_r_[5] = 1.82;
  //     q_r_[6] = 0.60;
  // }
  // else if(elapsed_time_.toSec() > 5.0 && elapsed_time_.toSec() <= 9.0){
  //     q_r_[0] = 0.63;
  //     q_r_[1] = -0.70;
  //     q_r_[2] = -0.03;
  //     q_r_[3] = -2.15;
  //     q_r_[4] = -0.25;
  //     q_r_[5] = 2.11;
  //     q_r_[6] = 0.68;
  // }
  // else if(elapsed_time_.toSec() > 9.0 && elapsed_time_.toSec() <= 13.0){
  //     q_r_[0] = 1.40;
  //     q_r_[1] = -0.60;
  //     q_r_[2] = 1.70;
  //     q_r_[3] = -2.30;
  //     q_r_[4] = 0.80;
  //     q_r_[5] = 2.15;
  //     q_r_[6] = 0.50;
  // }
  // else if(elapsed_time_.toSec() > 13.0){
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i];
  //   }
  // }

  /*** BIG STEP REFERENCE ***/
  // if (elapsed_time_.toSec() <= 2.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i];
  //   }
  // }
  // else if(elapsed_time_.toSec() > 2.0 && elapsed_time_.toSec() <= 7.0){
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i] + 1.0;
  //   }
  // }
  // else if(elapsed_time_.toSec() > 7.0 && elapsed_time_.toSec() <= 9.0){
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i] + 0.5;
  //   }
  // }
  // else if(elapsed_time_.toSec() > 9.0){
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i];
  //   }
  // }

  /*** STEP REFERENCES IN WALL ***/
  /* 1 WALL */
  // if (elapsed_time_.toSec() <= 2.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i];
  //   }
  // }
  // else if (elapsed_time_.toSec() > 2.0 && elapsed_time_.toSec() <= 7.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_r1_wall_[i];
  //   }
  // }
  // else if (elapsed_time_.toSec() > 7.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i];
  //   }
  // }

  /* CYLINDER */
  // if (elapsed_time_.toSec() <= 2.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i];
  //   }
  // }
  // else if (elapsed_time_.toSec() > 2.0 && elapsed_time_.toSec() <= 7.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_r1_cylinder_[i];
  //   }
  // }
  // else if (elapsed_time_.toSec() > 7.0 && elapsed_time_.toSec() <= 12.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_r4_wall_[i];
  //   }
  // }
  // else if (elapsed_time_.toSec() > 12.0 && elapsed_time_.toSec() <= 18.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_r1_wall_[i];
  //   }
  // }
  // else if (elapsed_time_.toSec() > 18.0 && elapsed_time_.toSec() <= 26.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_r2_wall_[i];
  //   }
  // }
  // else if (elapsed_time_.toSec() > 26.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i];
  //   }
  // }


  /* 2 WALLS */
  if (elapsed_time_.toSec() <= 2.0) {
    for(int i=0; i<7; i++){
      q_r_[i] = q_init_[i];
    }
  }
  else if (elapsed_time_.toSec() > 2.0 && elapsed_time_.toSec() <= 6.0) {
    std::array<double,7> ref = {0.769735, 1.1159, 0.022749, -1.25744, -0.113347, 2.44036, 0.782462};
    for(int i=0; i<7; i++){
      q_r_[i] = ref[i];
    }
  }
  else if (elapsed_time_.toSec() > 6.0 && elapsed_time_.toSec() <= 10.0) {
    for(int i=0; i<7; i++){
      q_r_[i] = q_r4_wall_[i];
    }
  }
  else if (elapsed_time_.toSec() > 10.0) {
    for(int i=0; i<7; i++){
      q_r_[i] = q_init_[i];
    }
  }


  /* 4 WALLS */
  // if (elapsed_time_.toSec() <= 2.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i];
  //   }
  // }
  // else if (elapsed_time_.toSec() > 2.0 && elapsed_time_.toSec() <= 7.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_r4_wall_[i];
  //   }
  // }
  // // else if (elapsed_time_.toSec() > 7.0 && elapsed_time_.toSec() <= 12.0) {
  // //   std::array<double,7> ref = {0.173628, 0.31322, -0.0187974, -1.81247, 0.0274817, 2.09329, 0.829448};
  // //   for(int i=0; i<7; i++){
  // //     q_r_[i] = ref[i];
  // //   }
  // // }
  // // else if (elapsed_time_.toSec() > 12.0 && elapsed_time_.toSec() <= 17.0) {
  // //   std::array<double,7> ref = {-0.145338, 0.242705, -0.218903, -1.8334, 0.0923884, 2.07359, 0.852029};
  // //   for(int i=0; i<7; i++){
  // //     q_r_[i] = ref[i];
  // //   }
  // // }
  // else if (elapsed_time_.toSec() > /*17.0*/7.0 && elapsed_time_.toSec() <= /*22.0*/12.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_r1_wall_[i];
  //   }
  // }
  // else if (elapsed_time_.toSec() > 12.0 && elapsed_time_.toSec() <= 17.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_r2_wall_[i];
  //   }
  // }
  // else if (elapsed_time_.toSec() > 17.0 && elapsed_time_.toSec() <= 22.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_r3_wall_[i];
  //   }
  // }
  // else if (elapsed_time_.toSec() > 22.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i];
  //   }
  // }

  // if (elapsed_time_.toSec() <= 2.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i];
  //   }
  // }
  // else if (elapsed_time_.toSec() > 2.0 && elapsed_time_.toSec() <= 7.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_r3_wall_[i];
  //   }
  // }
  // else if (elapsed_time_.toSec() > 7.0 && elapsed_time_.toSec() <= 12.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_r2_wall_[i];
  //   }
  // }
  // else if (elapsed_time_.toSec() > 12.0 && elapsed_time_.toSec() <= 17.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i];
  //   }
  // }
  // else if (elapsed_time_.toSec() > 17.0 && elapsed_time_.toSec() <= 22.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_r4_wall_[i];
  //   }
  // }
  // else if (elapsed_time_.toSec() > 22.0 && elapsed_time_.toSec() <= 27.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_r1_wall_[i];
  //   }
  // }
  // else if (elapsed_time_.toSec() > 27.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i];
  //   }
  // }




  /*** STEADY-STATE INADMISSIBLE STEP REFERENCE  ***/
  // if (elapsed_time_.toSec() <= 2.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i];
  //   }
  // }
  // else if(elapsed_time_.toSec() > 2.0 && elapsed_time_.toSec() <= 5.0){
  //   q_r_[1] = q_init_[1] + 0.75;
  // }
  // else if(elapsed_time_.toSec() > 5.0 && elapsed_time_.toSec() <= 9.0){
  //   q_r_[2] = q_init_[2] + 3.5;
  // }
  //  else if(elapsed_time_.toSec() > 9.0 && elapsed_time_.toSec() <= 14.0){
  //   q_r_[2] = q_init_[2];
  // }
  //  else if(elapsed_time_.toSec() > 14.0){
  //   q_r_[1] = q_init_[1];
  // }

  /*** STEP REFERENCES ***/
  // double step_ref = 0.2;
  // if (elapsed_time_.toSec() <= 2.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i];
  //   }
  // }
  // else if(elapsed_time_.toSec() > 2.0 && elapsed_time_.toSec() <= 4.0){
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i] + 1.0*step_ref;
  //   }
  // }
  // else if(elapsed_time_.toSec() > 4.0 && elapsed_time_.toSec() <= 6.0){
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i] + 2.0*step_ref;
  //   }
  // }
  // else if(elapsed_time_.toSec() > 6.0 && elapsed_time_.toSec() <= 8.0){
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i] + 3.0*step_ref;
  //   }
  // }
  // else if(elapsed_time_.toSec() > 8.0 && elapsed_time_.toSec() <= 10.0){
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i] + 4.0*step_ref;
  //   }
  // }
  // else if(elapsed_time_.toSec() > 10.0 && elapsed_time_.toSec() <= 12.0){
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i] + 5.0*step_ref;
  //   }
  // }
  // else if(elapsed_time_.toSec() > 12.0 && elapsed_time_.toSec() <= 14.0){
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i] + 4.0*step_ref;
  //   }
  // }
  // else if(elapsed_time_.toSec() > 14.0 && elapsed_time_.toSec() <= 16.0){
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i] + 3.0*step_ref;
  //   }
  // }
  // else if(elapsed_time_.toSec() > 16.0 && elapsed_time_.toSec() <= 18.0){
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i] + 2.0*step_ref;
  //   }
  // }
  // else if(elapsed_time_.toSec() > 18.0 && elapsed_time_.toSec() <= 20.0){
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i] + 1.0*step_ref;
  //   }
  // }
  // else if(elapsed_time_.toSec() > 20.0 && elapsed_time_.toSec() <= 22.0){
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i];
  //   }
  // }
  // else if(elapsed_time_.toSec() > 22.0 && elapsed_time_.toSec() <= 24.0){
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i] - 1.0*step_ref;
  //   }
  // }
  // else if(elapsed_time_.toSec() > 24.0){
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i];
  //   }
  // }

  /*** SINE REFERENCE ***/
  // double sine_amplitude = 1.0;
  // double sine_period = 50.0;
  // double sine_omega = 2.0 * 3.1415 / sine_period;
  // if (elapsed_time_.toSec() <= 2.0) {
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i];
  //   }
  // }
  // else{
  //   for(int i=0; i<7; i++){
  //     q_r_[i] = q_init_[i] + sine_amplitude*sin(sine_omega*(elapsed_time_.toSec()-2.0));
  //   }
  // }


  // NF + DSM
  if(elapsed_time_ .toSec()== 0.0){
    threadNF_ = std::thread (&CylindricalObstacles::navigationField, this);
    threadDSM_ = std::thread (&CylindricalObstacles::trajectoryBasedDSM, this);
  }
  if(elapsed_time_trajpred_.toSec()>= pred_interval_){
    threadNF_.join();
    threadDSM_.join();

    //qvUpdate
    for(int i=0; i<7; i++){
      q_v_[i] = q_v_[i] + DSM_ * rho_[i] * period.toSec();
    }

    // std::cout << "DSM = " << DSM_ << std::endl;
    elapsed_time_trajpred_ = ros::Duration(0.0);
    threadNF_ = std::thread (&CylindricalObstacles::navigationField, this);
    threadDSM_ = std::thread (&CylindricalObstacles::trajectoryBasedDSM, this);
  }

  if(rate_trigger_()){
    // panda_q_v_positions_ = getJointPositions(q_v_);
    // lambda_q_v_ = getLambda(panda_q_v_positions_);
    // Pij_q_v_ = getPij(panda_q_v_positions_, lambda_q_v_);
    // getSphericalRepulsionTaskSpace(Pij_q_v_); // update rho_rep_sphere_taskspace_
    // getRhoSphere();
  }


  for(size_t i=0; i<7; ++i) {
    tau_commanded_[i] = Kp_[i]*(q_v_[i] - robot_state.q[i]) - Kd_[i]*robot_state.dq[i];
    // tau_commanded_[i] = Kp_[i]*(q_r_[i] - robot_state.q[i]) - Kd_[i]*robot_state.dq[i];
    tau_desired_[i] = tau_commanded_[i] + gravity[i];
    joint_handles_[i].setCommand(tau_commanded_[i]); // gravity already included
  }
  hiro_ros_arm_controller::JointStates jointstates_msg;
  for (size_t i = 0; i < 7; ++i) {
    jointstates_msg.time = elapsed_time_.toSec();
    jointstates_msg.q[i] = robot_state.q[i];
    jointstates_msg.q_r[i] = q_r_[i];
    jointstates_msg.q_v[i] = q_v_[i];
    jointstates_msg.dotq[i] = robot_state.dq[i];
    jointstates_msg.tau_commanded[i] = tau_commanded_[i];
    jointstates_msg.tau_desired[i] = tau_desired_[i];
    jointstates_msg.tau_measured[i] = robot_state.tau_J[i];
  }
  jointstates_publisher_.publish(jointstates_msg);
}

void CylindricalObstacles::navigationField(){
  // attraction field
  for(int i=0; i<7; i++){
    rho_att_[i] = (q_r_[i]-q_v_[i])/(std::max(computeNorm(q_r_,q_v_),eta_));
  }

  // repulsion field: q
  std::array<double,7> q_upperlimit_array;
  std::array<double,7> q_lowerlimit_array;
  for(int i=0; i<7; i++){
    q_upperlimit_array[i] = q_upperlimit_[i];
    q_lowerlimit_array[i] = q_lowerlimit_[i];
  }
  for(int i=0; i<7; i++){
    rho_rep_q_[i] = std::max((zeta_q_ - std::abs(q_v_[i]-q_upperlimit_[i]))/(zeta_q_-delta_q_),0.0) * (q_v_[i]-q_upperlimit_[i])/computeNorm(q_v_,q_upperlimit_array);
    rho_rep_q_[i] = + std::max((zeta_q_ - std::abs(q_v_[i]-q_lowerlimit_[i]))/(zeta_q_-delta_q_),0.0) * (q_v_[i]-q_lowerlimit_[i])/computeNorm(q_v_,q_lowerlimit_array);
  }

  /*** compute Jacobians ***/
  // compute Jacobian of frame 3, 4, 5, 7, end-effector
  // we only look to positions, not to orientations of Pij
  // Jacobian positons: same for frame 1 and 2, same for frame 5 and 6
  // no joints can influence the positions of frames 1 and 2
  std::array<double, 42> joint3_jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kJoint3, q_v_, state_handle_->getRobotState().F_T_EE, state_handle_->getRobotState().EE_T_K);
  std::array<double, 42> joint4_jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kJoint4, q_v_, state_handle_->getRobotState().F_T_EE, state_handle_->getRobotState().EE_T_K);
  std::array<double, 42> joint5_jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kJoint5, q_v_, state_handle_->getRobotState().F_T_EE, state_handle_->getRobotState().EE_T_K);
  std::array<double, 42> joint7_jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kJoint7, q_v_, state_handle_->getRobotState().F_T_EE, state_handle_->getRobotState().EE_T_K);
  std::array<double, 42> endeffector_jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector, q_v_, state_handle_->getRobotState().F_T_EE, state_handle_->getRobotState().EE_T_K);

  // compute pseudo inverse Jacobians
  Eigen::Map<Eigen::Matrix<double, 6, 7> > joint3_jacobian(joint3_jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7> > joint4_jacobian(joint4_jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7> > joint5_jacobian(joint5_jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7> > joint7_jacobian(joint7_jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7> > endeffector_jacobian(endeffector_jacobian_array.data());

  // // method 1
  // Eigen::Matrix<double, 7, 6> pinv_joint3_jacobian = joint3_jacobian.block(0,0,6,7).completeOrthogonalDecomposition().pseudoInverse();
  // // method 2
  pseudoInverse(joint3_jacobian, joint3_jacobian_pinv_);
  pseudoInverse(joint4_jacobian, joint4_jacobian_pinv_);
  pseudoInverse(joint5_jacobian, joint5_jacobian_pinv_);
  pseudoInverse(joint7_jacobian, joint7_jacobian_pinv_);
  pseudoInverse(endeffector_jacobian, endeffector_jacobian_pinv_);

  panda_q_v_positions_ = getJointPositions(q_v_);

  // repulsion field: spherical obstacle
  if(number_obst_spheres_ >0){
    lambda_q_v_ = getLambda(panda_q_v_positions_);
    Pij_q_v_ = getPij(panda_q_v_positions_, lambda_q_v_);
    getSphericalRepulsionTaskSpace(Pij_q_v_); // update v_Pij_
    getRhoSphere(); // update rho_rep_sphere_
  }

  // repulsion field: cylindrical obstacle
  if(number_obst_cylinders_ >0){
    std::tie(SijTij_q_v_dir_,SijTij_q_v_dist_,mu_q_v_) = getMuSijTij(panda_q_v_positions_);
    getCylindricalRepulsionTaskSpace();
    getCylindricalRepulsionJointSpace();

    // getRhoCylinder();  // update rho_rep_cylinder_

    // std::cout << "1: new repulsion" << std::endl;
    // std::cout << rho_rep_cylinder_ << std::endl;


    // std::tie(mu_q_v_, nu_q_v_) = getMuNu(panda_q_v_positions_, obst_cylinder1_center_, obst_cylinder1_height_);
    // std::tie(Sij_q_v_,Tij_q_v_) = getSijTij(panda_q_v_positions_, obst_cylinder1_center_, obst_cylinder1_height_, mu_q_v_, nu_q_v_);
    // getCylindricalRepulsionTaskSpace_old(Sij_q_v_,Tij_q_v_); // update v_SijTij_
    // getRhoCylinder();  // update rho_rep_cylinder_
    // std::cout << "2: old repulsion" << std::endl;
    // std::cout << rho_rep_cylinder_ << std::endl;
  }

  // repulsion field: wall
  getWallRepulsionTaskSpace(panda_q_v_positions_);
  getRhoWall();

  // total navigation field
  for(int i=0; i<7; i++){
    // rho_[i] = rho_att_[i] + rho_rep_q_[i];
    rho_[i] = rho_att_[i] + rho_rep_q_[i];
    if(number_obst_spheres_ >0){
      rho_[i] += rho_rep_sphere_(i);
    }
    if(number_obst_cylinders_ >0){
      rho_[i] += rho_rep_cylinder_(i);
    }
    if(number_walls_ >0){
      rho_[i] += rho_rep_wall_(i);
    }
  }
}

void CylindricalObstacles::trajectoryBasedDSM(){
  start_DSM_ = std::chrono::system_clock::now();
  DSM_msg_.duration_DSM_tau = 0.0;
  DSM_msg_.duration_DSM_dotq = 0.0;
  DSM_msg_.duration_DSM_q = 0.0;
  DSM_msg_.duration_DSM_dotp_EE = 0.0;
  DSM_msg_.duration_DSM_sphere = 0.0;
  DSM_msg_.duration_DSM_cylinder = 0.0;
  DSM_msg_.duration_DSM_wall = 0.0;
  trajpred_msg_.duration_pred = 0.0;
  DSM_msg_.time_DSM = elapsed_time_.toSec();
  trajpred_msg_.time_pred = elapsed_time_.toSec();

  q_pred_ = state_handle_->getRobotState().q;
  dq_pred_ = state_handle_->getRobotState().dq;
  I_total_pred_ = state_handle_->getRobotState().I_total;
  m_total_pred_ = state_handle_->getRobotState().m_total;
  F_x_Ctotal_pred_ = state_handle_->getRobotState().F_x_Ctotal;

  for(int k=0; k<pred_samples_; k++){
    start_pred_ = std::chrono::system_clock::now();
    mass_pred_ = model_handle_->getMass(q_pred_,I_total_pred_,m_total_pred_,F_x_Ctotal_pred_);
    coriolis_pred_= model_handle_->getCoriolis(q_pred_,dq_pred_,I_total_pred_,m_total_pred_,F_x_Ctotal_pred_);
    gravity_pred_ = model_handle_->getGravity(q_pred_,m_total_pred_,F_x_Ctotal_pred_);

    for(int i=0; i<7;i++){
        tau_pred_[i] = Kp_[i]*(q_v_[i]-q_pred_[i]) - Kd_[i]*dq_pred_[i] + gravity_pred_[i];
        // tau_pred_[i] = Kp_[i]*(q_r_[i]-q_pred_[i]) - Kd_[i]*dq_pred_[i] + gravity_pred_[i];
    }

    // start DSM tau when using thread

    arma_mass_pred_ = stdToArmaMatrix(mass_pred_);
    arma_coriolis_pred_ = stdToArmaVector(coriolis_pred_);
    arma_gravity_pred_ = stdToArmaVector(gravity_pred_);
    arma_tau_pred_ = stdToArmaVector(tau_pred_);

    arma_ddq_pred_ = arma::solve(arma_mass_pred_, arma_tau_pred_-arma_coriolis_pred_-arma_gravity_pred_, arma::solve_opts::likely_sympd + arma::solve_opts::fast);
    ddq_pred_ = armaToStdVector(arma_ddq_pred_);

    //Simplectic Euler
    for(int i=0; i<7; i++){
      dq_pred_[i] = dq_pred_[i] + ddq_pred_[i]*pred_samplingtime_;
      q_pred_[i] = q_pred_[i] + dq_pred_[i]*pred_samplingtime_;
    }

    // start DSM dotq when using thread
    // start DSM q when using thread

    for (size_t i = 0; i < 7; ++i) {
      int index = i+7*k;
      trajpred_msg_.q_pred[index] = q_pred_[i];
      trajpred_msg_.dotq_pred[index] = dq_pred_[i];
      trajpred_msg_.tau_pred[index] = tau_pred_[i];
    }
    end_pred_ = std::chrono::system_clock::now();
    std::chrono::duration<double> duration_pred_ = end_pred_ - start_pred_;
    trajpred_msg_.duration_pred += duration_pred_.count();

    // start DSM tau for the moment at this place
    start_DSM_tau_ = std::chrono::system_clock::now();
    double DSM_tau_element = DSMtau(tau_pred_);
    if (k==0){
      DSM_tau_ = kappa_tau_*DSM_tau_element;
    }
    else{
      DSM_tau_ = std::min(DSM_tau_,kappa_tau_*DSM_tau_element);
    }
    end_DSM_tau_ = std::chrono::system_clock::now();
    duration_DSM_tau_ = end_DSM_tau_ - start_DSM_tau_;
    DSM_msg_.duration_DSM_tau += duration_DSM_tau_.count();

    // start DSM dotq for the moment at this place
    start_DSM_dotq_ = std::chrono::system_clock::now();
    double DSM_dotq_element = DSMdotq(dq_pred_);
    if (k==0){
      DSM_dotq_ = kappa_dotq_*DSM_dotq_element;
    }
    else{
      DSM_dotq_ = std::min(DSM_dotq_,kappa_dotq_*DSM_dotq_element);
    }
    end_DSM_dotq_ = std::chrono::system_clock::now();
    duration_DSM_dotq_ = end_DSM_dotq_ - start_DSM_dotq_;
    DSM_msg_.duration_DSM_dotq += duration_DSM_dotq_.count();

    // start DSM q for the moment at this place
    start_DSM_q_ = std::chrono::system_clock::now();
    double DSM_q_element = DSMq(q_pred_);
    if (k==0){
      DSM_q_ = kappa_q_*DSM_q_element;
    }
    else{
      DSM_q_ = std::min(DSM_q_,kappa_q_*DSM_q_element);
    }
    end_DSM_q_ = std::chrono::system_clock::now();
    duration_DSM_q_ = end_DSM_q_ - start_DSM_q_;
    DSM_msg_.duration_DSM_q += duration_DSM_q_.count();

    // start DSM cartesian velocity (end-effector) constraint
    start_DSM_dotp_EE_ = std::chrono::system_clock::now();
    double DSM_dotp_EE_element = DSMdotpEE(q_pred_, dq_pred_);
    if (k==0){
      DSM_dotp_EE_ = kappa_dotp_EE_*DSM_dotp_EE_element;
    }
    else{
      DSM_dotp_EE_ = std::min(DSM_dotp_EE_,kappa_dotp_EE_*DSM_dotp_EE_element);
    }
    end_DSM_dotp_EE_ = std::chrono::system_clock::now();
    duration_DSM_dotp_EE_ = end_DSM_dotp_EE_ - start_DSM_dotp_EE_;
    DSM_msg_.duration_DSM_dotp_EE += duration_DSM_dotp_EE_.count();

    // start DSM sphere for the moment at this place
    if(number_obst_spheres_ >0){
      start_DSM_sphere_ = std::chrono::system_clock::now();
      double DSM_sphere_element = DSMsphere(q_pred_);
      if (k==0){
        DSM_sphere_ = kappa_sphere_*DSM_sphere_element;
      }
      else{
        DSM_sphere_ = std::min(DSM_sphere_,kappa_sphere_*DSM_sphere_element);
      }
      end_DSM_sphere_ = std::chrono::system_clock::now();
      duration_DSM_sphere_ = end_DSM_sphere_ - start_DSM_sphere_;
      DSM_msg_.duration_DSM_sphere += duration_DSM_sphere_.count();
    }

    // start DSM cylinder for the moment at this place
    if(number_obst_cylinders_ >0){
      start_DSM_cylinder_ = std::chrono::system_clock::now();
      double DSM_cylinder_element = DSMcylinder(q_pred_);
      if (k==0){
        DSM_cylinder_ = kappa_cylinder_*DSM_cylinder_element;
      }
      else{
        DSM_cylinder_ = std::min(DSM_cylinder_,kappa_cylinder_*DSM_cylinder_element);
      }
      end_DSM_cylinder_ = std::chrono::system_clock::now();
      duration_DSM_cylinder_ = end_DSM_cylinder_ - start_DSM_cylinder_;
      DSM_msg_.duration_DSM_cylinder += duration_DSM_cylinder_.count();
    }
    // std::cout << "DSM: " << DSM_cylinder_ << std::endl;

    // start DSM wall for the moment at this place
    if(number_walls_ >0){
      start_DSM_wall_ = std::chrono::system_clock::now();
      double DSM_wall_element = DSMwall(q_pred_);
      if (k==0){
        DSM_wall_ = kappa_wall_*DSM_wall_element;
      }
      else{
        DSM_wall_ = std::min(DSM_wall_,kappa_wall_*DSM_wall_element);
      }
      end_DSM_wall_ = std::chrono::system_clock::now();
      duration_DSM_wall_ = end_DSM_wall_ - start_DSM_wall_;
      DSM_msg_.duration_DSM_wall += duration_DSM_wall_.count();
    }
  }
  trajpredictions_publisher_.publish(trajpred_msg_);
  DSM_msg_.DSM_tau = DSM_tau_;
  DSM_msg_.DSM_dotq = DSM_dotq_;
  DSM_msg_.DSM_q = DSM_q_;
  DSM_msg_.DSM_dotp_EE = DSM_dotp_EE_;
  DSM_msg_.DSM_sphere = DSM_sphere_;
  DSM_msg_.DSM_cylinder = DSM_cylinder_;
  DSM_msg_.DSM_wall = DSM_wall_;

  DSM_ = std::min(DSM_tau_,DSM_dotq_);
  DSM_ = std::min(DSM_,DSM_q_);
  DSM_ = std::min(DSM_,DSM_dotp_EE_);
  if(number_obst_spheres_ >0){
    DSM_ = std::min(DSM_,DSM_sphere_);
  }
  if(number_obst_cylinders_ >0){
    DSM_ = std::min(DSM_,DSM_cylinder_);
  }
  if(number_walls_ >0){
    DSM_ = std::min(DSM_,DSM_wall_);
  }
  DSM_ = std::max(DSM_,0.0);
  // DSM_=1.0;
  // std::cout << DSM_ << std::endl;

  end_DSM_ = std::chrono::system_clock::now();
  duration_DSM_ = end_DSM_ - start_DSM_;
  DSM_msg_.duration_DSM = duration_DSM_.count();
  DSM_msg_.DSM = DSM_;
  DSM_publisher_.publish(DSM_msg_);

  //return DSM_;
}

double CylindricalObstacles::DSMtau(std::array<double, 7>& tau_pred){
  double DSM_tau;
  for(int i=0; i<7; i++){
    double DSM_tau_lowerlimit = tau_pred[i] - (1.0+delta_tau_)*(-tau_limit_[i]);
    double DSM_tau_upperlimit = (1.0-delta_tau_)*tau_limit_[i] - tau_pred[i];
    double DSM_tau_temp = std::min(DSM_tau_lowerlimit,DSM_tau_upperlimit);
    if (i==0){
      DSM_tau = DSM_tau_temp;
    }
    else{
      DSM_tau = std::min(DSM_tau,DSM_tau_temp);
    }
  }
  return DSM_tau;
}

double CylindricalObstacles::DSMdotq(std::array<double, 7>& dotq_pred){
  double DSM_dotq;
  for(int i=0; i<7; i++){
    double DSM_dotq_lowerlimit = dotq_pred[i] - (1.0+delta_dotq_)*(-dotq_limit_[i]);
    double DSM_dotq_upperlimit = (1.0-delta_dotq_)*dotq_limit_[i] - dotq_pred[i];
    double DSM_dotq_temp = std::min(DSM_dotq_lowerlimit,DSM_dotq_upperlimit);
    if (i==0){
      DSM_dotq = DSM_dotq_temp;
    }
    else{
      DSM_dotq = std::min(DSM_dotq,DSM_dotq_temp);
    }
  }
  return DSM_dotq;
}

double CylindricalObstacles::DSMq(std::array<double, 7>& q_pred){
  double DSM_q;
  for(int i=0; i<7; i++){
    double DSM_q_lowerlimit = q_pred[i] - (1.0+delta_q_)*q_lowerlimit_[i];
    double DSM_q_upperlimit = (1.0-delta_q_)*q_upperlimit_[i] - q_pred[i];
    double DSM_q_temp = std::min(DSM_q_lowerlimit,DSM_q_upperlimit);
    if (i==0){
      DSM_q = DSM_q_temp;
    }
    else{
      DSM_q = std::min(DSM_q,DSM_q_temp);
    }
  }
  return DSM_q;
}

double CylindricalObstacles::DSMdotpEE(std::array<double, 7>& q_pred, std::array<double, 7>& dotq_pred){
  double DSM_dotp_EE;
  double DSM_dotp_EE_lowerlimit;
  double DSM_dotp_EE_upperlimit;

  std::array<double, 42> endeffector_jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector, q_pred, state_handle_->getRobotState().F_T_EE, state_handle_->getRobotState().EE_T_K);
  Eigen::Map<Eigen::Matrix<double, 6, 7> > endeffector_jacobian(endeffector_jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dotq_pred_eigen(dotq_pred.data());
  Eigen::Matrix<double,6,1> dotp_EE = endeffector_jacobian*dotq_pred_eigen;
  // std::cout << dotp_EE << std::endl;
  // std::cout << std::endl;

  for(int i=0; i<6; i++){
    if(i<3){
      DSM_dotp_EE_lowerlimit = dotp_EE(i) - (1.0+delta_dotp_EE_)*(-dotp_EE_limit_[0]);
      DSM_dotp_EE_upperlimit = (1.0+delta_dotp_EE_)*dotp_EE_limit_[0] - dotp_EE(i) ;
    }
    else{
      DSM_dotp_EE_lowerlimit = dotp_EE(i) - (1.0+delta_dotp_EE_)*(-dotp_EE_limit_[1]);
      DSM_dotp_EE_upperlimit = (1.0+delta_dotp_EE_)*dotp_EE_limit_[1] - dotp_EE(i) ;
    }
    double DSM_dotp_EE_temp = std::min(DSM_dotp_EE_lowerlimit,DSM_dotp_EE_upperlimit);
    if (i==0){
      DSM_dotp_EE = DSM_dotp_EE_temp;
    }
    else{
      DSM_dotp_EE = std::min(DSM_dotp_EE,DSM_dotp_EE_temp);
    }
  }
  return DSM_dotp_EE;
}

double CylindricalObstacles::DSMsphere(std::array<double, 7>& q_pred){
  double DSM_sphere;

  panda_q_pred_positions_ = getJointPositions(q_pred);
  lambda_q_pred_ = getLambda(panda_q_pred_positions_);
  Pij_q_pred_ = getPij(panda_q_pred_positions_, lambda_q_pred_);

  for(int i=0; i<Pij_q_pred_.rows()/3; i++){
    for (int j=0; j<number_obst_spheres_; j++) {
      // double DSM_sphere_temp = (Pij_q_pred_.block(3*i,j,3,1)-obst_sphere1_center_).norm() - (1+delta_sphere_)*obst_sphere1_radius_;
      double DSM_sphere_temp = (Pij_q_pred_.block(3*i,j,3,1)-obst_sphere1_center_).norm() - obst_sphere1_radius_;
      if (i==0 && j==0){
        DSM_sphere = DSM_sphere_temp;
      }
      else{
        DSM_sphere = std::min(DSM_sphere,DSM_sphere_temp);
      }
    }
  }
  return DSM_sphere;
}

double CylindricalObstacles::DSMcylinder(std::array<double, 7>& q_pred){
  double DSM_cylinder;

  panda_q_pred_positions_ = getJointPositions(q_pred);

  std::tie(SijTij_q_pred_dir_,SijTij_q_pred_dist_,mu_q_pred_) = getMuSijTij(panda_q_pred_positions_);

  // std::tie(mu_q_pred_, nu_q_pred_) = getMuNu(panda_q_pred_positions_, obst_cylinder1_center_, obst_cylinder1_height_);
  // std::tie(Sij_q_pred_,Tij_q_pred_) = getSijTij(panda_q_pred_positions_, obst_cylinder1_center_, obst_cylinder1_height_, mu_q_pred_, nu_q_pred_);

  for(int i=0; i<SijTij_q_pred_dir_.rows()/3; i++){
    for (int j=0; j<number_obst_cylinders_; j++) {
      double DSM_cylinder_temp = SijTij_q_pred_dist_(i,j) - cylinder_radii_(j);
      // double DSM_cylinder_temp = (Sij_q_pred_.block(3*i,j,3,1)-Tij_q_pred_.block(3*i,j,3,1)).norm() - obst_cylinder1_radius_;
      if (i==0 && j==0){
        DSM_cylinder = DSM_cylinder_temp;
      }
      else{
        DSM_cylinder = std::min(DSM_cylinder,DSM_cylinder_temp);
      }
    }
  }
  return DSM_cylinder;
}

double CylindricalObstacles::DSMwall(std::array<double, 7>& q_pred){
  double DSM_wall;
  double DSM_wall_temp;

  panda_q_pred_positions_ = getJointPositions(q_pred);

  for (int j=0; j<number_walls_; j++) {
    for(int i=0; i<panda_q_pred_positions_.cols()-1; i++){
      Eigen::Matrix<double, 3, 1> wall_normal = wall_normals_.block(0,j,3,1);
      DSM_wall_temp = wall_distances_-wall_normal.transpose().dot(panda_q_pred_positions_.col(i+1));
      if (i==0 && j==0){
        DSM_wall = DSM_wall_temp;
      }
      else{
        DSM_wall = std::min(DSM_wall,DSM_wall_temp);
      }
    }
  }
  return DSM_wall;
}

Eigen::Matrix<double, 3, 6> CylindricalObstacles::getJointPositions(std::array<double, 7>& q_argument){
  Eigen::Matrix<double, 3,6 > panda_jointpositions;
  for(int i=0; i<3; i++){
    // no kJoint2 and kJoint6, because
    // position kJoint2 = position kJoint1
    // position kJoint6 = position kJoint5
    panda_jointpositions(i,0) = model_handle_->getPose(franka::Frame::kJoint1, q_argument, state_handle_->getRobotState().F_T_EE , state_handle_->getRobotState().EE_T_K)[12+i];
    panda_jointpositions(i,1) = model_handle_->getPose(franka::Frame::kJoint3, q_argument, state_handle_->getRobotState().F_T_EE , state_handle_->getRobotState().EE_T_K)[12+i];
    panda_jointpositions(i,2) = model_handle_->getPose(franka::Frame::kJoint4, q_argument, state_handle_->getRobotState().F_T_EE , state_handle_->getRobotState().EE_T_K)[12+i];
    panda_jointpositions(i,3) = model_handle_->getPose(franka::Frame::kJoint5, q_argument, state_handle_->getRobotState().F_T_EE , state_handle_->getRobotState().EE_T_K)[12+i];
    panda_jointpositions(i,4)= model_handle_->getPose(franka::Frame::kJoint7, q_argument, state_handle_->getRobotState().F_T_EE , state_handle_->getRobotState().EE_T_K)[12+i];
    panda_jointpositions(i,5) = model_handle_->getPose(franka::Frame::kEndEffector, q_argument, state_handle_->getRobotState().F_T_EE , state_handle_->getRobotState().EE_T_K)[12+i];
  }
  // ROS_INFO_STREAM("panda_jointpositions :" <<  panda_jointpositions(0,0) << " " << panda_jointpositions(1,0)  << " " << panda_jointpositions(2,0) );
  // ROS_INFO_STREAM(" ");
  return panda_jointpositions;
}

Eigen::Matrix<double, 5, 1> CylindricalObstacles::getLambda(Eigen::Matrix<double, 3 ,6>& panda_jointpositions){
  // std::cout << "in computeLambda" << std::endl;
  Eigen::Matrix<double, 5, 1> lambda;  // size =(#links, #spheres)

  for (int i=0; i<panda_jointpositions.cols()-1;i++) {// #joints-1 (-1, because in code +1 to denote next joint)
    double denum = (panda_jointpositions.col(i+1) - panda_jointpositions.col(i)).dot(panda_jointpositions.col(i+1) - panda_jointpositions.col(i));
    for (int j=0; j<number_obst_spheres_; j++) {  // #obstacles
      if (denum <= 0.001) { // denum is norm has to be >0, if =0 frames
        lambda(i,j) = 0; // in case two consecutive frames are at same position, avoid deviding by zero -> lambda = nan
      }
      else {
        lambda(i,j) = (panda_jointpositions.col(i+1)-panda_jointpositions.col(i)).dot(obst_sphere1_center_-panda_jointpositions.col(i))/denum;
        if (lambda(i,j) < 0) {
          lambda(i,j) = 0;
        }
        else if(lambda(i,j) >1) {
          lambda(i,j) = 1;
        }
      }
    }
  }
  return lambda;
}

Eigen::Matrix<double, 15, 1> CylindricalObstacles::getPij(Eigen::Matrix<double, 3, 6>& panda_jointpositions, Eigen::Matrix<double, 5, 1>& lambda){
  Eigen::Matrix<double, 15, 1> Pij;
  for (int j=0; j<number_obst_spheres_; j++){ // # obstacles
    for (int i=0; i<panda_jointpositions.cols()-1;i++){ // # links
      Pij.block(3*i,j,3,1) = panda_jointpositions.col(i) + lambda(i,j) * (panda_jointpositions.col(i+1)-panda_jointpositions.col(i));
      // ROS_INFO_STREAM("\nframe " << i << " :\n" << panda_jointpositions.col(i) <<
      //                 "\nframe " << i+1 << " :\n" << panda_jointpositions.col(i+1) <<
      //                 "\nframe " << i+1 << " - frame " << i << " :\n" << panda_jointpositions.col(i+1) - panda_jointpositions.col(i) <<
      //                 "\nlambda : " << lambda(i,j) <<
      //                 "\nPij : \n" << panda_jointpositions.col(i) + lambda(i,j) * (panda_jointpositions.col(i+1)-panda_jointpositions.col(i)) <<
      //                 "\nPij block : \n" <<  Pij(3*i,j) << "\n" << Pij(3*i+1,j) << "\n" << Pij(3*i+2,j)  <<
      //                 "\n");
    }
  }
  return Pij;
}

void CylindricalObstacles::getSphericalRepulsionTaskSpace(Eigen::Matrix<double, 15, 1>& Pij) {
  // std::cout <<  "in getSphericalRepulsionTaskSpace " << std::endl;
  Eigen::Matrix<double, 3, 1> pijmincj;
  for(int j=0; j<number_obst_spheres_; j++) { // # obstacles
    for (int i=0; i<Pij.rows()/3; i++) { // # links
      pijmincj = Pij.block(3*i,j,3,1) - obst_sphere1_center_;
      v_Pij_.block(3*i,j,3,1) =  std::max((zeta_sphere_ - (pijmincj.norm()-obst_sphere1_radius_))/(zeta_sphere_-delta_sphere_),0.0) * pijmincj; // / pijmincj.norm();
      // ROS_INFO_STREAM("\nframe " << i <<
      //                 "\nrho task : \n" <<  rho_rep_sphere_taskspace_(3*i,j) << "\n" << rho_rep_sphere_taskspace_(3*i+1,j) << "\n" << rho_rep_sphere_taskspace_(3*i+2,j));
    }
  }
}

void CylindricalObstacles::getRhoSphere(){
  // repulsion field rho_rep_sphere_
  Eigen::Matrix<double,7,1> rho_temp;
  rho_rep_sphere_.setZero();
  for (int j=0; j<number_obst_spheres_; j++){
    rho_temp.setZero();
    rho_temp += lambda_q_v_(0,j) * joint3_jacobian_pinv_.block(0,0,7,3) * v_Pij_.block(0,j,3,1);
    rho_temp += (1-lambda_q_v_(1,j)) * joint3_jacobian_pinv_.block(0,0,7,3) * v_Pij_.block(3,j,3,1);
    rho_temp += lambda_q_v_(1,j) * joint4_jacobian_pinv_.block(0,0,7,3) * v_Pij_.block(3,j,3,1);
    rho_temp += (1-lambda_q_v_(2,j)) * joint4_jacobian_pinv_.block(0,0,7,3) * v_Pij_.block(6,j,3,1);
    rho_temp += lambda_q_v_(2,j) * joint5_jacobian_pinv_.block(0,0,7,3) * v_Pij_.block(6,j,3,1);
    rho_temp += (1-lambda_q_v_(3,j)) * joint5_jacobian_pinv_.block(0,0,7,3) * v_Pij_.block(9,j,3,1);
    rho_temp += lambda_q_v_(3,j) * joint7_jacobian_pinv_.block(0,0,7,3) * v_Pij_.block(9,j,3,1);
    rho_temp += (1-lambda_q_v_(4,j)) * joint7_jacobian_pinv_.block(0,0,7,3) * v_Pij_.block(12,j,3,1);
    rho_temp += lambda_q_v_(4,j) * endeffector_jacobian_pinv_.block(0,0,7,3) * v_Pij_.block(12,j,3,1);

    rho_rep_sphere_ += rho_temp / std::max(rho_temp.norm(),0.001);
  }
}

std::tuple<Eigen::Matrix<double,15,2>,Eigen::Matrix<double,5,2>,Eigen::Matrix<double,5,2>> CylindricalObstacles::getMuSijTij(Eigen::Matrix<double, 3, 6>& panda_jointpositions){
  double mu;
  double nu;
  Eigen::Matrix<double, 3, 1> Sij;
  Eigen::Matrix<double, 3, 1> Tij;
  Eigen::Matrix<double, 15, 2> SijTij_dir;
  Eigen::Matrix<double, 5, 2> SijTij_dist;
  Eigen::Matrix<double, 5, 2> mu_final;

  Eigen::Matrix<double, 3, 1> a;
  Eigen::Matrix<double, 3, 1> b;
  Eigen::Matrix<double, 3, 1> c_0;
  Eigen::Matrix<double, 3, 1> c_1;

  Eigen::Matrix<double, 3, 1> cylinder_start;
  Eigen::Matrix<double, 3, 1> cylinder_end;

  for (int i=0; i<panda_jointpositions.cols()-1;i++) {// #joints-1 (-1, because in code +1 to denote next joint)
    a = panda_jointpositions.col(i+1)-panda_jointpositions.col(i);

    for (int j=0; j<number_obst_cylinders_; j++){ // # obstacles
      cylinder_start = {cylinder_centers_(0,j),cylinder_centers_(1,j),cylinder_centers_(2,j) - cylinder_heights_(j)/2};
      cylinder_end = {cylinder_centers_(0,j),cylinder_centers_(1,j),cylinder_centers_(2,j) + cylinder_heights_(j)/2};
      b = cylinder_end - cylinder_start;
      c_0 = cylinder_start - panda_jointpositions.col(i);
      c_1 = cylinder_end - panda_jointpositions.col(i);

      if ( ((a/a.norm()).cross(b/b.norm())).norm() < 0.001 ) // case of parallel segments
      {
        double d_0 = (a/a.norm()).dot(c_0);
        double d_1 = (a/a.norm()).dot(c_1);

        if(d_0<=0.0 && d_1<=0.0) // cylinder before link in z-direction (viewpoint of panda_jointpositions.col(i) to panda_jointpositions.col(i+1))
        {
          mu = 0.0; // mu = 0, Sij = panda_jointpositions.col(i)
          if (std::abs(d_0) < std::abs(d_1))
          {
            nu = 0.0; // nu = 0, Tij = cylinder_start
          }
          else if(std::abs(d_0) > std::abs(d_1))
          {
            nu = 1.0; // nu = 1, Tij = cylinder_end
          }
        }
        else if(d_0>=a.norm() && d_1>=a.norm()) // cylinder after link in z-direction (viewpoint of panda_jointpositions.col(i) to panda_jointpositions.col(i+1))
        {
          mu = 1.0; // mu = 1, Sij = panda_jointpositions.col(i+1)
          if (std::abs(d_0)<std::abs(d_1))
          {
            nu = 0.0; // nu = 0, Tij = cylinder_start
          }
          else if(std::abs(d_0)>std::abs(d_1))
          {
            nu = 1.0; // nu = 1, Tij = cylinder_end
          }
        }
        else // cylinder and link (partly) overlapping in z-direction
        {
          double nu_parallel = (b.dot((panda_jointpositions.col(i)+panda_jointpositions.col(i+1))/2-cylinder_start))/(b.dot(b));
          if (0.0<=nu_parallel && nu_parallel <=1.0)
          {
            mu = 0.5; // mu =0.5, Sij = (panda_jointpositions.col(i)+panda_jointpositions.col(i+1))/2
            nu = nu_parallel; // nu computed as in point-line case
          }
          else if(0.0<=d_0 && d_0<=a.norm()) // = if nu_parallel < 0
          {
            if (d_1>a.norm())
            {
              mu = 1.0; // mu = 1, Sij = panda_jointpositions.col(i+1)
              nu = (b.dot(panda_jointpositions.col(i+1)-cylinder_start))/(b.dot(b)); // nu computed as in point-line case
            }
            else if(d_1 < 0.0)
            {
              mu = 0.0; // mu = 0, Sij = panda_jointpositions.col(i)
              nu = (b.dot(panda_jointpositions.col(i)-cylinder_start))/(b.dot(b)); // nu computed as in point-line case
            }
          }
          else if (0.0<=d_1 && d_1<=a.norm()) // % = if nu_parallel > 1
          {
            if (d_0>a.norm())
            {
              mu = 1.0; // mu = 1, Sij = panda_jointpositions.col(i+1)
              nu = (b.dot(panda_jointpositions.col(i+1)-cylinder_start))/(b.dot(b)); // nu computed as in point-line case
            }
            else if (d_0 < 0.0)
            {
              mu = 0.0; // mu = 0, Sij = panda_jointpositions.col(i)
              nu = (b.dot(panda_jointpositions.col(i)-cylinder_start))/(b.dot(b)); // nu computed as in point-line case
            }
          }
        }
        Sij = panda_jointpositions.col(i) + mu * (panda_jointpositions.col(i+1)-panda_jointpositions.col(i));
        Tij = cylinder_start + nu * (cylinder_end-cylinder_start);
        SijTij_dir.block(3*i,j,3,1) = Sij-Tij;
        SijTij_dist(i,j) = SijTij_dir.block(3*i,j,3,1).norm();
      }
      else { // case of skew segments
        mu = (b.dot(b)*c_0.dot(a)-c_0.dot(b)*b.dot(a))/(b.dot(b)*a.dot(a)-a.dot(b)*b.dot(a)); // mu computed for skew line-line case
        nu = (a.dot(a)/b.dot(a)) * (b.dot(b)*c_0.dot(a)-c_0.dot(b)*b.dot(a))/(b.dot(b)*a.dot(a)-b.dot(a)*b.dot(a))-(c_0.dot(a)/b.dot(a)); // nu computed for skew line-line case
        if ( (mu >=0.0 && mu<=1.0) && (nu>=0.0 && nu<=1.0) ){ // mu in [0,1] and nu in [0,1]
          Sij = panda_jointpositions.col(i) + mu * (panda_jointpositions.col(i+1)-panda_jointpositions.col(i));
          Tij = cylinder_start + nu * (cylinder_end-cylinder_start);
          SijTij_dir.block(3*i,j,3,1) = Sij-Tij;
          SijTij_dist(i,j) = SijTij_dir.block(3*i,j,3,1).norm();
        }
        else if ( (mu<0.0 || mu>1.0) && (nu>=0.0 && nu<=1.0) ){ // mu not in [0,1] and nu in [0,1]
          if (mu<0.0){
            mu = 0.0;
          }
          else if (mu>1.0){
            mu = 1.0;
          }
          Sij  = panda_jointpositions.col(i) + mu * (panda_jointpositions.col(i+1)-panda_jointpositions.col(i));
          nu = (b.dot(Sij-cylinder_start))/(b.dot(b)); // nu computed as in point-line case
          Tij = cylinder_start + nu * (cylinder_end-cylinder_start);
          SijTij_dir.block(3*i,j,3,1) = Sij-Tij;

          if(nu<0.0){
            nu = 0.0;
          }
          else if (nu>1.0){
            nu = 1.0;
          }
          Tij = cylinder_start + nu * (cylinder_end-cylinder_start);
          SijTij_dist(i,j) = (Sij-Tij).norm();
        }
        else if ( (mu >=0.0 && mu<=1.0) && (nu<0.0 || nu>1.0) ){ // mu in [0,1] and nu not in [0,1]
          if (nu<0.0){
            nu = 0.0;
          }
          else if (nu>1.0){
            nu = 1.0;
          }
          Tij = cylinder_start + nu * (cylinder_end-cylinder_start);
          mu = (a.dot(Tij-panda_jointpositions.col(i)))/(a.dot(a)); // mu computed as in point-line case
          Sij = panda_jointpositions.col(i) + mu * (panda_jointpositions.col(i+1)-panda_jointpositions.col(i));

          Eigen::Matrix<double, 3, 1> f = a.cross(Sij-Tij);
          Eigen::Matrix<double, 3, 1> f_norm = f/f.norm();
          Eigen::Matrix<double, 3, 1> SijTij_direction = f_norm.cross(a);
          // SijTij_dir.block(3*i,j,3,1) = f_norm.cross(a);
          if (SijTij_direction.dot(Sij-Tij) < 0.0){
            SijTij_dir.block(3*i,j,3,1) = - SijTij_direction;
          }
          else {
            SijTij_dir.block(3*i,j,3,1) = SijTij_direction;
          }
          if (mu<0.0){
            mu = 0.0;
          }
          else if (mu>1.0){
            mu = 1.0;
          }
          Sij = panda_jointpositions.col(i) + mu * (panda_jointpositions.col(i+1)-panda_jointpositions.col(i));
          SijTij_dist(i,j) = (Sij-Tij).norm();
        }
        else { // mu not in [0,1] and nu not in [0,1]
          Eigen::Matrix<double, 3, 1> Sij_for_mu0 = panda_jointpositions.col(i);
          Eigen::Matrix<double, 3, 1> Sij_for_mu1 = panda_jointpositions.col(i+1);
          double nu_for_mu0 = (b.dot(Sij_for_mu0-cylinder_start))/(b.dot(b)); // nu computed as in point-line case
          double nu_for_mu1 = (b.dot(Sij_for_mu1-cylinder_start))/(b.dot(b)); // nu computed as in point-line case
          Eigen::Matrix<double, 3, 1> Tij_for_mu0 = cylinder_start + nu_for_mu0 * (cylinder_end-cylinder_start);
          Eigen::Matrix<double, 3, 1> Tij_for_mu1 = cylinder_start + nu_for_mu1 * (cylinder_end-cylinder_start);

          if(nu_for_mu0<0.0){
            nu_for_mu0 = 0.0;
          }
          else if(nu_for_mu0>1.0){
            nu_for_mu0 = 1.0;
          }
          if(nu_for_mu1<0.0){
            nu_for_mu1 = 0.0;
          }
          else if(nu_for_mu1>1.0){
            nu_for_mu1 = 1.0;
          }
          Eigen::Matrix<double, 3, 1> Tij_for_mu0_corrected = cylinder_start + nu_for_mu0 * (cylinder_end-cylinder_start);
          Eigen::Matrix<double, 3, 1> Tij_for_mu1_corrected = cylinder_start + nu_for_mu1 * (cylinder_end-cylinder_start);

          if( (Sij_for_mu0-Tij_for_mu0_corrected).norm() < (Sij_for_mu1-Tij_for_mu1_corrected).norm() ){
            Sij = Sij_for_mu0;
            Tij = Tij_for_mu0;
            SijTij_dir.block(3*i,j,3,1) = Sij - Tij;
            SijTij_dist(i,j) = (Sij_for_mu0-Tij_for_mu0_corrected).norm();
            mu = 0.0;
          }
          else{
            Sij = Sij_for_mu1;
            Tij = Tij_for_mu1;
            SijTij_dir.block(3*i,j,3,1) = Sij - Tij;
            SijTij_dist(i,j) = (Sij_for_mu1-Tij_for_mu1_corrected).norm();
            mu = 1.0;
          }
        }
      }
      mu_final(i,j) = mu;
    }
  }
  return {SijTij_dir, SijTij_dist, mu_final};
}

// std::tuple<Eigen::Matrix<double,5,1>,Eigen::Matrix<double,5,1>> CylindricalObstacles::getMuNu(Eigen::Matrix<double, 3, 6>& panda_jointpositions, Eigen::Matrix<double, 3, 1>& obst_cylinder_center, double& obst_cylinder_height){
//   Eigen::Matrix<double, 5, 1> mu;  // size =(#links, 2)
//   Eigen::Matrix<double, 5, 1> nu;

//   Eigen::Matrix<double, 3, 1> a;
//   Eigen::Matrix<double, 3, 1> b;
//   Eigen::Matrix<double, 3, 1> c_0;
//   Eigen::Matrix<double, 3, 1> c_1;

//   Eigen::Matrix<double, 3, 1> cylinder_start;
//   Eigen::Matrix<double, 3, 1> cylinder_end;

//   for (int i=0; i<panda_jointpositions.cols()-1;i++) {// #joints-1 (-1, because in code +1 to denote next joint)
//     a = panda_jointpositions.col(i+1)-panda_jointpositions.col(i);

//     for (int j=0; j<number_obst_cylinders_; j++){ // # obstacles
//       cylinder_start = {obst_cylinder_center(0),obst_cylinder_center(1),obst_cylinder_center(2)-obst_cylinder_height/2};
//       cylinder_end = {obst_cylinder_center(0),obst_cylinder_center(1),obst_cylinder_center(2)+obst_cylinder_height/2};
//       b = cylinder_end - cylinder_start;
//       c_0 = cylinder_start - panda_jointpositions.col(i);
//       c_1 = cylinder_end - panda_jointpositions.col(i);

//       if (a.norm() <= 0.001) // case of consecutive frames
//       {
//         mu(i,j) = 0.0; // mu = 0, Sij = panda_jointpositions.col(i)
//         Eigen::Matrix<double, 3, 1> mu_min_cylinder_start;
//         for(int k=0; k<3; k++){
//           mu_min_cylinder_start(k) = mu(i,j)-cylinder_start(k); // SURE ABOUT THIS???
//         }
//         nu(i,j) = (b.dot(mu_min_cylinder_start))/(b.dot(b)); // nu computed as in point-line case
//       }

//       else if ( ((a/a.norm()).cross(b/b.norm())).norm() < 0.01 ) // case of parallel segments
//       {
//         double d_0 = (a/a.norm()).dot(c_0);
//         double d_1 = (a/a.norm()).dot(c_1);

//         if(d_0<=0.0 && d_1<=0.0) // cylinder before link in z-direction (viewpoint of panda_jointpositions.col(i) to panda_jointpositions.col(i+1))
//         {
//           mu(i,j) = 0.0; // mu = 0, Sij = panda_jointpositions.col(i)
//           if (std::abs(d_0) < std::abs(d_1))
//           {
//             nu(i,j) = 0.0; // nu = 0, Tij = cylinder_start
//           }
//           else if(std::abs(d_0) > std::abs(d_1))
//           {
//             nu(i,j) = 1.0; // nu = 1, Tij = cylinder_end
//           }
//         }
//         else if(d_0>=a.norm() && d_1>=a.norm()) // cylinder after link in z-direction (viewpoint of panda_jointpositions.col(i) to panda_jointpositions.col(i+1))
//         {
//           mu(i,j) = 1.0; // mu = 1, Sij = panda_jointpositions.col(i+1)
//           if (std::abs(d_0)<std::abs(d_1))
//           {
//             nu(i,j) = 0.0; // nu = 0, Tij = cylinder_start
//           }
//           else if(std::abs(d_0)>std::abs(d_1))
//           {
//             nu(i,j) = 1.0; // nu = 1, Tij = cylinder_end
//           }
//         }
//         else // cylinder and link (partly) overlapping in z-direction
//         {
//           double nu_parallel = (b.dot((panda_jointpositions.col(i)+panda_jointpositions.col(i+1))/2-cylinder_start))/(b.dot(b));
//           if (0.0<=nu_parallel && nu_parallel <=1.0)
//           {
//             mu(i,j) = 0.5; // mu =0.5, Sij = (panda_jointpositions.col(i)+panda_jointpositions.col(i+1))/2
//             nu(i,j) = nu_parallel; // nu computed as in point-line case
//           }
//           else if(0.0<=d_0 && d_0<=a.norm()) // = if nu_parallel < 0
//           {
//             if (d_1>a.norm())
//             {
//               mu(i,j) = 1.0; // mu = 1, Sij = panda_jointpositions.col(i+1)
//               nu(i,j) = (b.dot(panda_jointpositions.col(i+1)-cylinder_start))/(b.dot(b)); // nu computed as in point-line case
//             }
//             else if(d_1 < 0.0)
//             {
//               mu(i,j) = 0.0; // mu = 0, Sij = panda_jointpositions.col(i)
//               nu(i,j) = (b.dot(panda_jointpositions.col(i)-cylinder_start))/(b.dot(b)); // nu computed as in point-line case
//             }
//           }
//           else if (0.0<=d_1 && d_1<=a.norm()) // % = if nu_parallel > 1
//           {
//             if (d_0>a.norm())
//             {
//               mu(i,j) = 1.0; // mu = 1, Sij = panda_jointpositions.col(i+1)
//               nu(i,j) = (b.dot(panda_jointpositions.col(i+1)-cylinder_start))/(b.dot(b)); // nu computed as in point-line case
//             }
//             else if (d_0 < 0.0)
//             {
//               mu(i,j) = 0.0; // mu = 0, Sij = panda_jointpositions.col(i)
//               nu(i,j) = (b.dot(panda_jointpositions.col(i)-cylinder_start))/(b.dot(b)); // nu computed as in point-line case
//             }
//           }
//         }
//       }

//       else // case of skew segments
//       {
//         mu(i,j) = (b.dot(b)*c_0.dot(a)-c_0.dot(b)*b.dot(a))/(b.dot(b)*a.dot(a)-a.dot(b)*b.dot(a)); // mu computed for skew line-line case
//         if(mu(i,j) < 0.0)
//         {
//           mu(i,j) = 0.0; // mu = 0, Sij = panda_jointpositions.col(i)
//           nu(i,j) = (b.dot(panda_jointpositions.col(i)-cylinder_start))/(b.dot(b)); // nu computed as in point-line case
//         }
//         else if(mu(i,j)>1.0)
//         {
//           mu(i,j) = 1.0; // mu = 1, Sij = panda_jointpositions.col(i+1)
//           nu(i,j) = (b.dot(panda_jointpositions.col(i+1)-cylinder_start))/(b.dot(b)); // nu computed as in point-line case
//         }
//         else
//         {
//           nu(i,j) = (a.dot(a)/b.dot(a)) * (b.dot(b)*c_0.dot(a)-c_0.dot(b)*b.dot(a))/(b.dot(b)*a.dot(a)-b.dot(a)*b.dot(a))-(c_0.dot(a)/b.dot(a)); // nu computed for skew line-line case
//         }

//         if(nu(i,j) < 0.0)
//         {
//           nu(i,j) = 0.0;
//         }
//         else if(nu(i,j) > 1.0)
//         {
//           nu(i,j) = 1.0;
//         }
//       }
//     }
//   }
//   return {mu, nu};
// }


// std::tuple<Eigen::Matrix<double,15,1>,Eigen::Matrix<double,15,1>>CylindricalObstacles::getSijTij(Eigen::Matrix<double, 3, 6>& panda_jointpositions, Eigen::Matrix<double, 3, 1>& obst_cylinder_center, double& obst_cylinder_height, Eigen::Matrix<double, 5, 1>& mu, Eigen::Matrix<double, 5, 1>& nu){
//   Eigen::Matrix<double,15,1> Sij;
//   Eigen::Matrix<double,15,1> Tij;

//   Eigen::Matrix<double, 3, 1> cylinder_start;
//   Eigen::Matrix<double, 3, 1> cylinder_end;

//   for (int j=0; j<number_obst_cylinders_; j++){ // # obstacles
//     cylinder_start = {obst_cylinder_center(0),obst_cylinder_center(1),obst_cylinder_center(2)-obst_cylinder_height/2};
//     cylinder_end = {obst_cylinder_center(0),obst_cylinder_center(1),obst_cylinder_center(2)+obst_cylinder_height/2};
//     for (int i=0; i<panda_jointpositions.cols()-1;i++){ // # links
//       cylinder_start = {obst_cylinder_center(0),obst_cylinder_center(1),obst_cylinder_center(2)-obst_cylinder_height/2};
//       cylinder_end = {obst_cylinder_center(0),obst_cylinder_center(1),obst_cylinder_center(2)+obst_cylinder_height/2};
//       Sij.block(3*i,j,3,1) = panda_jointpositions.col(i) + mu(i,j) * (panda_jointpositions.col(i+1)-panda_jointpositions.col(i));
//       Tij.block(3*i,j,3,1) = cylinder_start + nu(i,j) * (cylinder_end-cylinder_start);
//     }
//   }
//   return {Sij, Tij};
// }

void CylindricalObstacles::getCylindricalRepulsionTaskSpace(){
  // Eigen::Matrix<double,3,1> circulation_upwards = {0,0,1};
  for (int j=0; j<number_obst_cylinders_; j++){ // # obstacles
    for (int i=0; i<SijTij_q_v_dir_.rows()/3; i++) { // # links
      v_SijTij_.block(3*i,j,3,1) =  std::max((zeta_cylinder_ - (SijTij_q_v_dist_(i,j)-cylinder_radii_(j)))/(zeta_cylinder_-delta_cylinder_),0.0)*SijTij_q_v_dir_.block(3*i,j,3,1); // /SijTij_q_v_dir_.block(3*i,j,3,1).norm();
      // v_SijTij_upwards_.block(3*i,j,3,1) = std::max((zeta_cylinder_ - (SijTij_q_v_dist_(i,j)-cylinder_radii_(j)))/(zeta_cylinder_-delta_cylinder_),0.0) * circulation_upwards;
    }
  }
  std::cout << std::endl;
}

void CylindricalObstacles::getCylindricalRepulsionJointSpace(){
  // Eigen::Matrix<double,7,1> rho_temp;
  // rho_rep_cylinder_.setZero();
  // for (int j=0; j<number_obst_cylinders_; j++){
  //   rho_temp.setZero();
  //   rho_temp += mu_q_v_(0,j) * joint3_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(0,j,3,1);
  //   rho_rep_cylinder_ +=  v_SijTij_.block(0,j,3,1).norm() * rho_temp / std::max(rho_temp.norm(),0.001);
  //   std::cout <<"0: " << v_SijTij_.block(0,j,3,1).norm() << " , " << rho_temp.norm() << std::endl;

  //   rho_temp.setZero();
  //   rho_temp += (1-mu_q_v_(1,j)) * joint3_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(3,j,3,1);
  //   rho_temp += mu_q_v_(1,j) * joint4_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(3,j,3,1);
  //   rho_rep_cylinder_ +=  v_SijTij_.block(3,j,3,1).norm() * rho_temp / std::max(rho_temp.norm(),0.001);
  //   std::cout <<"1: " << v_SijTij_.block(3,j,3,1).norm() << " , " << rho_temp.norm() << std::endl;

  //   rho_temp.setZero();
  //   rho_temp += (1-mu_q_v_(2,j)) * joint4_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(6,j,3,1);
  //   rho_temp += mu_q_v_(2,j) * joint5_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(6,j,3,1);
  //   rho_rep_cylinder_ +=  v_SijTij_.block(6,j,3,1).norm() * rho_temp / std::max(rho_temp.norm(),0.001);
  //   std::cout <<"2: "<< v_SijTij_.block(6,j,3,1).norm() << " , " << rho_temp.norm() <<  std::endl;
  //   std::cout <<  std::max((zeta_cylinder_ - (SijTij_q_v_dist_(2,j)-obst_cylinder1_radius_))/(zeta_cylinder_-delta_cylinder_),0.0) << std::endl;

  //   rho_temp.setZero();
  //   rho_temp += (1-mu_q_v_(3,j)) * joint5_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(9,j,3,1);
  //   rho_temp += mu_q_v_(3,j) * joint7_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(9,j,3,1);
  //   rho_rep_cylinder_ +=  v_SijTij_.block(9,j,3,1).norm() * rho_temp / std::max(rho_temp.norm(),0.001);
  //   std::cout <<"3: " << v_SijTij_.block(9,j,3,1).norm() << " , " << rho_temp.norm() << std::endl;

  //   rho_temp.setZero();
  //   rho_temp += (1-mu_q_v_(4,j)) * joint7_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(12,j,3,1);
  //   rho_temp += mu_q_v_(4,j) * endeffector_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(12,j,3,1);
  //   rho_rep_cylinder_ +=  v_SijTij_.block(12,j,3,1).norm() * rho_temp / std::max(rho_temp.norm(),0.001);
  //   std::cout <<"4: " << v_SijTij_.block(12,j,3,1).norm() << " , " << rho_temp.norm() << std::endl;
  // }

  Eigen::Matrix<double,7,1> rho_temp;
  Eigen::Matrix<double,7,1> rho_circ;
  Eigen::Matrix<double,7,1> rho_total_temp;
  Eigen::Matrix<double,3,1> circulation_upwards = {0,0,1};
  double circulation_factor = 0.4; //0.5;
  rho_rep_cylinder_.setZero();
  for (int j=0; j<number_obst_cylinders_; j++){
    rho_temp.setZero();
    rho_circ.setZero();
    rho_total_temp.setZero();
    // rho_temp = mu_q_v_(0,j) * joint3_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(0,j,3,1);
    // // rho_circ = mu_q_v_(0,j) * joint3_jacobian_pinv_.block(0,0,7,3) * v_SijTij_upwards_.block(0,j,3,1);
    // // rho_circ = circulation_factor *  v_SijTij_.block(0,j,3,1) * rho_circ/std::max(rho_circ.norm(),0.001);
    // // rho_total_temp =  v_SijTij_.block(0,j,3,1).norm() * rho_temp / std::max(rho_temp.norm(),0.001);
    // rho_total_temp = rho_temp; // / std::max(rho_temp.norm(),0.001);

    // rho_temp.setZero();
    // rho_temp = (1-mu_q_v_(1,j)) * joint3_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(3,j,3,1);
    // rho_temp += mu_q_v_(1,j) * joint4_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(3,j,3,1);
    // // rho_temp = v_SijTij_.block(3,j,3,1).norm() * rho_temp / std::max(rho_temp.norm(),0.001);
    // rho_temp = rho_temp; // / std::max(rho_temp.norm(),0.001);
    // if(rho_temp.norm() > rho_total_temp.norm()){
    //   // rho_circ = (1-mu_q_v_(1,j)) * joint3_jacobian_pinv_.block(0,0,7,3) * v_SijTij_upwards_.block(3,j,3,1);
    //   // rho_circ += mu_q_v_(1,j) * joint4_jacobian_pinv_.block(0,0,7,3) * v_SijTij_upwards_.block(3,j,3,1);
    //   // rho_circ = circulation_factor *  v_SijTij_.block(3,j,3,1).norm() * rho_circ/std::max(rho_circ.norm(),0.001);
    //   rho_total_temp = rho_temp;
    // }

    // rho_temp.setZero();
    // rho_temp += (1-mu_q_v_(2,j)) * joint4_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(6,j,3,1);
    // rho_temp += mu_q_v_(2,j) * joint5_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(6,j,3,1);
    // // rho_temp = v_SijTij_.block(6,j,3,1).norm() * rho_temp / std::max(rho_temp.norm(),0.001);
    // rho_temp = rho_temp; // / std::max(rho_temp.norm(),0.001);
    // if(rho_temp.norm() > rho_total_temp.norm()){
    //   // rho_circ += (1-mu_q_v_(2,j)) * joint4_jacobian_pinv_.block(0,0,7,3) * v_SijTij_upwards_.block(6,j,3,1);
    //   // rho_circ += mu_q_v_(2,j) * joint5_jacobian_pinv_.block(0,0,7,3) * v_SijTij_upwards_.block(6,j,3,1);
    //   // rho_circ = circulation_factor *  v_SijTij_.block(6,j,3,1).norm() * rho_circ/std::max(rho_circ.norm(),0.001);
    //   rho_total_temp = rho_temp;
    // }

    // rho_temp.setZero();
    // rho_temp += (1-mu_q_v_(3,j)) * joint5_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(9,j,3,1);
    // rho_temp += mu_q_v_(3,j) * joint7_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(9,j,3,1);
    // // rho_temp = v_SijTij_.block(9,j,3,1).norm() * rho_temp / std::max(rho_temp.norm(),0.001);
    // rho_temp = rho_temp; // / std::max(rho_temp.norm(),0.001);
    // if(rho_temp.norm() > rho_total_temp.norm()){
    //   // rho_circ += (1-mu_q_v_(3,j)) * joint5_jacobian_pinv_.block(0,0,7,3) * v_SijTij_upwards_.block(9,j,3,1);
    //   // rho_circ += mu_q_v_(3,j) * joint7_jacobian_pinv_.block(0,0,7,3) * v_SijTij_upwards_.block(9,j,3,1);
    //   // rho_circ = circulation_factor *  v_SijTij_.block(9,j,3,1).norm() * rho_circ/std::max(rho_circ.norm(),0.001);
    //   rho_total_temp = rho_temp;
    // }

    // rho_temp.setZero();
    // rho_temp += (1-mu_q_v_(4,j)) * joint7_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(12,j,3,1);
    // rho_temp += mu_q_v_(4,j) * endeffector_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(12,j,3,1);
    // // rho_temp = v_SijTij_.block(12,j,3,1).norm() * rho_temp / std::max(rho_temp.norm(),0.001);
    // rho_temp = rho_temp; // / std::max(rho_temp.norm(),0.001);
    // if(rho_temp.norm() > rho_total_temp.norm()){
    //   // rho_circ += (1-mu_q_v_(4,j)) * joint7_jacobian_pinv_.block(0,0,7,3) * v_SijTij_upwards_.block(12,j,3,1);
    //   // rho_circ += mu_q_v_(4,j) * endeffector_jacobian_pinv_.block(0,0,7,3) * v_SijTij_upwards_.block(12,j,3,1);
    //   // rho_circ = circulation_factor * v_SijTij_.block(12,j,3,1).norm() * rho_circ/std::max(rho_circ.norm(),0.001);
    //   rho_total_temp = rho_temp;
    // }

    rho_total_temp += mu_q_v_(0,j) * joint3_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(0,j,3,1);
    rho_total_temp += (1-mu_q_v_(1,j)) * joint3_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(3,j,3,1);
    rho_total_temp += mu_q_v_(1,j) * joint4_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(3,j,3,1);
    rho_total_temp += (1-mu_q_v_(2,j)) * joint4_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(6,j,3,1);
    rho_total_temp += mu_q_v_(2,j) * joint5_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(6,j,3,1);
    rho_total_temp += (1-mu_q_v_(3,j)) * joint5_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(9,j,3,1);
    rho_total_temp += mu_q_v_(3,j) * joint7_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(9,j,3,1);
    rho_total_temp += (1-mu_q_v_(4,j)) * joint7_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(12,j,3,1);
    rho_total_temp += mu_q_v_(4,j) * endeffector_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(12,j,3,1);

    rho_temp += (zeta_cylinder_ - (SijTij_q_v_dist_(0,j)-cylinder_radii_(j)))/(zeta_cylinder_-delta_cylinder_) * mu_q_v_(0,j) * joint3_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(0,j,3,1);
    rho_temp += (zeta_cylinder_ - (SijTij_q_v_dist_(1,j)-cylinder_radii_(j)))/(zeta_cylinder_-delta_cylinder_) * (1-mu_q_v_(1,j)) * joint3_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(3,j,3,1);
    rho_temp += (zeta_cylinder_ - (SijTij_q_v_dist_(1,j)-cylinder_radii_(j)))/(zeta_cylinder_-delta_cylinder_) * mu_q_v_(1,j) * joint4_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(3,j,3,1);
    rho_temp += (zeta_cylinder_ - (SijTij_q_v_dist_(2,j)-cylinder_radii_(j)))/(zeta_cylinder_-delta_cylinder_) * (1-mu_q_v_(2,j)) * joint4_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(6,j,3,1);
    rho_temp += (zeta_cylinder_ - (SijTij_q_v_dist_(2,j)-cylinder_radii_(j)))/(zeta_cylinder_-delta_cylinder_) * mu_q_v_(2,j) * joint5_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(6,j,3,1);
    rho_temp += (zeta_cylinder_ - (SijTij_q_v_dist_(3,j)-cylinder_radii_(j)))/(zeta_cylinder_-delta_cylinder_) * (1-mu_q_v_(3,j)) * joint5_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(9,j,3,1);
    rho_temp += (zeta_cylinder_ - (SijTij_q_v_dist_(3,j)-cylinder_radii_(j)))/(zeta_cylinder_-delta_cylinder_) * mu_q_v_(3,j) * joint7_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(9,j,3,1);
    rho_temp += (zeta_cylinder_ - (SijTij_q_v_dist_(4,j)-cylinder_radii_(j)))/(zeta_cylinder_-delta_cylinder_) * (1-mu_q_v_(4,j)) * joint7_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(12,j,3,1);
    rho_temp += (zeta_cylinder_ - (SijTij_q_v_dist_(4,j)-cylinder_radii_(j)))/(zeta_cylinder_-delta_cylinder_) * mu_q_v_(4,j) * endeffector_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(12,j,3,1);

    std::cout << "rho_temp.norm() : " << rho_temp.norm() << std::endl;
    std::cout << "rho_total_temp.norm() : " << rho_total_temp.norm() << std::endl;
    rho_rep_cylinder_ += rho_temp/std::max(rho_total_temp.norm(),0.001);
    std::cout << "rho_rep_cylinder_.norm() : " << rho_rep_cylinder_.norm() << std::endl;
    rho_circ = joint7_jacobian_pinv_.block(0,0,7,3) * circulation_upwards;
    rho_circ = circulation_factor * rho_rep_cylinder_.norm() * rho_circ/std::max(rho_circ.norm(),0.001);
    // std::cout << "rho_rep_cylinder_.norm() : " << rho_rep_cylinder_.norm() << std::endl;
    // std::cout << "rho_circ.norm() : " << rho_circ.norm() << std::endl;
    rho_rep_cylinder_ += rho_circ;

    // rho_total_temp = rho_total_temp/std::max(rho_total_temp.norm(),0.001);
    // std::cout << "rho_total_temp.norm() : " << rho_total_temp.norm() << std::endl;
    // rho_circ = joint5_jacobian_pinv_.block(0,0,7,3) * circulation_upwards;
    // rho_circ = circulation_factor * rho_total_temp.norm() * rho_circ/std::max(rho_circ.norm(),0.001);
    // std::cout << "rho_circ.norm() : " << rho_circ.norm() << std::endl;
    // std::cout << "factor : " << rho_circ.norm()/rho_total_temp.norm() << std::endl;
    // rho_total_temp  += rho_circ;
    // rho_rep_cylinder_  += rho_total_temp;
  }
}

// void CylindricalObstacles::getCylindricalRepulsionTaskSpace_old(Eigen::Matrix<double,15,1>& Sij, Eigen::Matrix<double,15,1>& Tij){
//   Eigen::Matrix<double, 3, 1> sijmintij;
//   for (int j=0; j<number_obst_cylinders_; j++){ // # obstacles
//     for (int i=0; i<Sij.rows()/3; i++) { // # links
//       sijmintij =  Sij.block(3*i,j,3,1) - Tij.block(3*i,j,3,1);
//       v_SijTij_.block(3*i,j,3,1) =  std::max((zeta_cylinder_ - (sijmintij.norm()-obst_cylinder1_radius_))/(zeta_cylinder_-delta_cylinder_),0.0)*sijmintij;
//     }
//     if((zeta_cylinder_ - (sijmintij.norm()-obst_cylinder1_radius_))/(zeta_cylinder_-delta_cylinder_)>0.0){
//     v_SijTij_(8,j) += 1.0;
//     // v_SijTij_(14) += 0.5;
//     }
//   }
//   // std::cout << "v_SijTij_ = \n" <<  v_SijTij_  << std::endl;
//   // std::cout << std::endl;
// }

// void CylindricalObstacles::getRhoCylinder(){
//   // repulsion field rho_rep_cylinder_;
//   Eigen::Matrix<double,7,1> rho_temp;
//   rho_rep_cylinder_.setZero();
//   for (int j=0; j<number_obst_cylinders_; j++){
//     rho_temp.setZero();
//     rho_temp += mu_q_v_(0,j) * joint3_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(0,j,3,1);
//     rho_temp += (1-mu_q_v_(1,j)) * joint3_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(3,j,3,1);
//     rho_temp += mu_q_v_(1,j) * joint4_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(3,j,3,1);
//     rho_temp += (1-mu_q_v_(2,j)) * joint4_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(6,j,3,1);
//     rho_temp += mu_q_v_(2,j) * joint5_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(6,j,3,1);
//     rho_temp += (1-mu_q_v_(3,j)) * joint5_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(9,j,3,1);
//     rho_temp += mu_q_v_(3,j) * joint7_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(9,j,3,1);
//     rho_temp += (1-mu_q_v_(4,j)) * joint7_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(12,j,3,1);
//     rho_temp += mu_q_v_(4,j) * endeffector_jacobian_pinv_.block(0,0,7,3) * v_SijTij_.block(12,j,3,1);

//     rho_rep_cylinder_ += rho_temp / std::max(rho_temp.norm(),0.001);
//   }
//   // std::cout << rho_rep_cylinder_ << std::endl;
//   // std::cout << std::endl;
// }

void CylindricalObstacles::getWallRepulsionTaskSpace(Eigen::Matrix<double, 3, 6>& panda_jointpositions){
  for(int j=0; j<number_walls_; j++) { // # walls
    for (int i=0; i<panda_jointpositions.cols()-1;i++){ // # links
      Eigen::Matrix<double, 3, 1> wall_normal = wall_normals_.block(0,j,3,1);
      v_wall_ij_.block(3*i,j,3,1) =  std::max((zeta_wall_ - (wall1_distance_-wall_normal.transpose().dot(panda_jointpositions.col(i+1))))/(zeta_wall_-delta_wall_),0.0)*(-wall_normal);
    }
  }
}

void CylindricalObstacles::getRhoWall(){
  Eigen::Matrix<double,7,1> rho_temp;
  rho_rep_wall_.setZero();
  for (int j=0; j<number_walls_; j++){
    rho_temp.setZero();
    rho_temp += joint3_jacobian_pinv_.block(0,0,7,3) * v_wall_ij_.block(0,j,3,1);
    rho_temp += joint4_jacobian_pinv_.block(0,0,7,3) * v_wall_ij_.block(3,j,3,1);
    rho_temp += joint5_jacobian_pinv_.block(0,0,7,3) * v_wall_ij_.block(6,j,3,1);
    rho_temp += joint7_jacobian_pinv_.block(0,0,7,3) * v_wall_ij_.block(9,j,3,1);
    rho_temp += endeffector_jacobian_pinv_.block(0,0,7,3) * v_wall_ij_.block(12,j,3,1);

    rho_rep_wall_ += rho_temp / std::max(rho_temp.norm(),0.001);
    // std::cout << rho_rep_wall_ << std::endl;
  }
  // std::cout << std::endl;
}


arma::mat CylindricalObstacles::stdToArmaMatrix(std::array<double, 49>& stdMatrix){
  arma::mat armaMatrix;
  armaMatrix  = arma::zeros<arma::mat>(7,7);
    for (int i = 0; i < 7; i++)
    {
      for (int j = 0; j < 7; j++)
      {
        armaMatrix(i,j) = stdMatrix[i+j*7];
      }
    }
    return armaMatrix;
}

arma::vec CylindricalObstacles::stdToArmaVector(std::array<double, 7>& stdVector){
  arma::vec armaVector;
  armaVector = arma::zeros<arma::vec>(7);
  for(int i=0; i<7; i++){
    armaVector[i] = stdVector[i];
  }
  return armaVector;
}

std::array<double, 7> CylindricalObstacles::armaToStdVector(arma::vec& armaVector){
  std::array<double, 7> stdVector;
  for(int i=0; i<7; i++){
    stdVector[i] = armaVector[i];
  }
  return stdVector;
}

double CylindricalObstacles::computeNorm(std::array<double,7>& a, std::array<double,7>& b){
  double norm = 0.0;
  if ((a.size() != b.size()) || (a.size() == 0 || b.size() == 0)) {
    return -1;
  }
  for (int i = 0; i < a.size(); i++) {
    norm += (a[i] - b[i])*(a[i] - b[i]);
  }
  return sqrt(norm);
}

}  // namespace hiro_ros_arm_controller

PLUGINLIB_EXPORT_CLASS(hiro_panda::CylindricalObstacles,
                       controller_interface::ControllerBase)