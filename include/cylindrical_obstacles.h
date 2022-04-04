// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <chrono>
#include <ctime>
#include <thread>
#include <armadillo>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/hardware_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <Eigen/Dense>
#include <Eigen/QR>

#include <hiro_ros_arm_controller/JointStates.h>
#include <hiro_ros_arm_controller/TrajectoryPredictions.h>
#include <hiro_ros_arm_controller/DSM.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>
#include <franka_hw/franka_state_interface.h>

namespace hiro_panda {

class CylindricalObstacles : public controller_interface::MultiInterfaceController<
                                                  franka_hw::FrankaModelInterface,
                                                  hardware_interface::EffortJointInterface,
                                                  franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& ) override;
  void update(const ros::Time& , const ros::Duration& period) override;

  void navigationField();
  void trajectoryBasedDSM();
  double DSMtau(std::array<double, 7>& tau_pred);
  double DSMdotq(std::array<double, 7>& dotq_pred);
  double DSMq(std::array<double, 7>& q_pred);
  double DSMdotpEE(std::array<double, 7>& q_pred, std::array<double, 7>& dotq_pred);
  double DSMsphere(std::array<double, 7>& q_pred);
  double DSMcylinder(std::array<double, 7>& q_pred);
  double DSMwall(std::array<double, 7>& q_pred);

  // std::array<double, 24> getJointPositions(std::array<double, 7>& q_argument);
  Eigen::Matrix<double, 3, 6> getJointPositions(std::array<double, 7>& q_argument);

  Eigen::Matrix<double, 5, 1> getLambda(Eigen::Matrix<double, 3, 6>& panda_jointpositions);
  Eigen::Matrix<double, 15, 1> getPij(Eigen::Matrix<double, 3, 6>& panda_jointpositions, Eigen::Matrix<double, 5, 1>& lambda);
  void getSphericalRepulsionTaskSpace(Eigen::Matrix<double, 15, 1>& Pij);
  void getRhoSphere();

  // std::tuple<Eigen::Matrix<double,5,1>,Eigen::Matrix<double,5,1>>getMuNu(Eigen::Matrix<double, 3, 6>& panda_jointpositions, Eigen::Matrix<double, 3, 1>& obst_cylinder_center, double& obst_cylinder_height);
  // std::tuple<Eigen::Matrix<double,15,1>,Eigen::Matrix<double,15,1>> getSijTij(Eigen::Matrix<double, 3, 6>& panda_jointpositions, Eigen::Matrix<double, 3, 1>& obst_cylinder_center, double& obst_cylinder_height, Eigen::Matrix<double, 5, 1>& mu, Eigen::Matrix<double, 5, 1>& nu);
  std::tuple<Eigen::Matrix<double,15,2>,Eigen::Matrix<double,5,2>,Eigen::Matrix<double,5,2>>getMuSijTij(Eigen::Matrix<double, 3, 6>& panda_jointpositions);
  // void getCylindricalRepulsionTaskSpace_old(Eigen::Matrix<double,15,1>& Sij,Eigen::Matrix<double,15,1>& Tij);
  void getCylindricalRepulsionTaskSpace();
  void getCylindricalRepulsionJointSpace();
  // void getRhoCylinder();

  void getWallRepulsionTaskSpace(Eigen::Matrix<double, 3, 6>& panda_jointpositions);
  void getRhoWall();

  arma::mat stdToArmaMatrix(std::array<double, 49>& stdMatrix);
  arma::vec stdToArmaVector(std::array<double, 7>& stdVector);
  std::array<double, 7> armaToStdVector(arma::vec& armaVector);
  double computeNorm(std::array<double,7>& a, std::array<double,7>& b);

 private:
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  ros::Duration elapsed_time_;
  ros::Duration elapsed_time_trajpred_;

  std::thread threadNF_;
  std::thread threadDSM_;

  std::array<double, 7> q_init_;

  std::vector<double> Kp_;
  std::vector<double> Kd_;
  double pred_samples_;
  double pred_samplingtime_;
  double pred_interval_;
  std::vector<double> tau_limit_;
  double kappa_tau_;
  double delta_tau_;
  std::vector<double> dotq_limit_;
  double kappa_dotq_;
  double delta_dotq_;
  std::vector<double> q_lowerlimit_;
  std::vector<double> q_upperlimit_;
  double kappa_q_;
  double delta_q_;
  double eta_;
  double zeta_q_;

  std::vector<double> dotp_EE_limit_;
  double kappa_dotp_EE_;
  double delta_dotp_EE_;

  double kappa_sphere_;
  double delta_sphere_;
  double zeta_sphere_;

  double kappa_cylinder_;
  double delta_cylinder_;
  double zeta_cylinder_;

  double zeta_wall_;
  double kappa_wall_;
  double delta_wall_;

  franka_hw::TriggerRate rate_trigger_{1.0};

  ros::Publisher jointstates_publisher_;
  ros::Publisher trajpredictions_publisher_;
  ros::Publisher DSM_publisher_;
  ros::Publisher marker_sphere1_pub_;
  ros::Publisher marker_sphere1_delta_pub_;
  ros::Publisher marker_sphere1_zeta_pub_;
  ros::Publisher marker_cylinder1_pub_;
  ros::Publisher marker_cylinder1_delta_pub_;
  ros::Publisher marker_cylinder1_zeta_pub_;
  ros::Publisher marker_cylinder2_pub_;
  ros::Publisher marker_cylinder2_delta_pub_;
  ros::Publisher marker_cylinder2_zeta_pub_;
  ros::Publisher marker_wall1_pub_;
  ros::Publisher marker_wall2_pub_;
  ros::Publisher marker_wall3_pub_;
  ros::Publisher marker_wall4_pub_;

  Eigen::MatrixXd joint3_jacobian_pinv_;
  Eigen::MatrixXd joint4_jacobian_pinv_;
  Eigen::MatrixXd joint5_jacobian_pinv_;
  Eigen::MatrixXd joint7_jacobian_pinv_;
  Eigen::MatrixXd endeffector_jacobian_pinv_;

  std::array<double,7> q_r_;
  std::array<double,7> q_v_;
  std::array<double,7> tau_commanded_;
  std::array<double,7> tau_desired_;

  std::array<double,7> rho_;
  std::array<double,7> rho_att_;
  std::array<double,7> rho_rep_q_;
  Eigen::Matrix<double,7,1> rho_rep_sphere_;
  Eigen::Matrix<double,7,1> rho_rep_cylinder_;
  Eigen::Matrix<double,7,1> rho_rep_wall_;
  // std::array<double,7> rho_rep_sphere_;

  double DSM_;
  double DSM_tau_;
  double DSM_dotq_;
  double DSM_q_;
  double DSM_dotp_EE_;
  double DSM_sphere_;
  double DSM_cylinder_;
  double DSM_wall_;

  // std::array<double, 24> panda_q_v_positions_;
  // std::array<double, 24> panda_q_pred_positions_;
  Eigen::Matrix<double, 3, 6> panda_q_v_positions_;
  Eigen::Matrix<double, 3, 6> panda_q_pred_positions_;

  int number_obst_spheres_ = 0;
  Eigen::Matrix<double, 3, 1> obst_sphere1_center_;
  double obst_sphere1_radius_;
  Eigen::Matrix<double, 5, 1> lambda_q_v_;
  Eigen::Matrix<double, 5, 1> lambda_q_pred_;
  Eigen::Matrix<double, 15, 1> Pij_q_v_;
  Eigen::Matrix<double, 15, 1> Pij_q_pred_;
  Eigen::Matrix<double, 15, 1> v_Pij_;

  int number_obst_cylinders_ = 0;
  Eigen::Matrix<double, 3, 2> cylinder_centers_;
  Eigen::Matrix<double, 1, 2> cylinder_radii_;
  Eigen::Matrix<double, 1, 2> cylinder_heights_;
  Eigen::Matrix<double, 3, 1> obst_cylinder1_center_;
  double obst_cylinder1_radius_;
  double obst_cylinder1_height_;
  Eigen::Matrix<double, 3, 1> obst_cylinder2_center_;
  double obst_cylinder2_radius_;
  double obst_cylinder2_height_;
  Eigen::Matrix<double, 5, 2> mu_q_v_;
  Eigen::Matrix<double, 5, 2> nu_q_v_;
  Eigen::Matrix<double, 5, 2> mu_q_pred_;
  Eigen::Matrix<double, 5, 2> nu_q_pred_;
  Eigen::Matrix<double, 15, 2> Sij_q_v_;
  Eigen::Matrix<double, 15, 2> Tij_q_v_;
  Eigen::Matrix<double, 15, 2> SijTij_q_v_dir_;
  Eigen::Matrix<double, 5, 2> SijTij_q_v_dist_;
  Eigen::Matrix<double, 15, 2> Sij_q_pred_;
  Eigen::Matrix<double, 15, 2> Tij_q_pred_;
  Eigen::Matrix<double, 15, 2> SijTij_q_pred_dir_;
  Eigen::Matrix<double, 5, 2> SijTij_q_pred_dist_;
  Eigen::Matrix<double, 15, 2> v_SijTij_;
  Eigen::Matrix<double, 15, 2> v_SijTij_upwards_;
  std::array<double,7> q_r1_cylinder_;

  int number_walls_ = 1;
  Eigen::Matrix<double, 3, 4> wall_normals_;
  double wall_distances_;
  Eigen::Matrix<double, 3, 1> wall1_normal_;
  double wall1_distance_;
  Eigen::Matrix<double, 3, 1> wall2_normal_;
  double wall2_distance_;
  Eigen::Matrix<double, 3, 1> wall3_normal_;
  double wall3_distance_;
  Eigen::Matrix<double, 3, 1> wall4_normal_;
  double wall4_distance_;
  Eigen::Matrix<double, 15, 1> v_wall_ij_;
  std::array<double,7> q_r1_wall_;
  std::array<double,7> q_r2_wall_;
  std::array<double,7> q_r3_wall_;
  std::array<double,7> q_r4_wall_;

  std::chrono::time_point<std::chrono::system_clock> start_DSM_, end_DSM_;
  std::chrono::time_point<std::chrono::system_clock> start_pred_, end_pred_;
  std::chrono::time_point<std::chrono::system_clock> start_DSM_tau_, end_DSM_tau_;
  std::chrono::time_point<std::chrono::system_clock> start_DSM_dotq_, end_DSM_dotq_;
  std::chrono::time_point<std::chrono::system_clock> start_DSM_q_, end_DSM_q_;
  std::chrono::time_point<std::chrono::system_clock> start_DSM_dotp_EE_, end_DSM_dotp_EE_;
  std::chrono::time_point<std::chrono::system_clock> start_DSM_sphere_, end_DSM_sphere_;
  std::chrono::time_point<std::chrono::system_clock> start_DSM_cylinder_, end_DSM_cylinder_;
  std::chrono::time_point<std::chrono::system_clock> start_DSM_wall_, end_DSM_wall_;
  std::chrono::duration<double> duration_DSM_, duration_pred_, duration_DSM_tau_, duration_DSM_dotq_, duration_DSM_q_, duration_DSM_dotp_EE_,
                                duration_DSM_sphere_, duration_DSM_cylinder_, duration_DSM_wall_;

  hiro_ros_arm_controller::DSM DSM_msg_;
  hiro_ros_arm_controller::TrajectoryPredictions trajpred_msg_;

  std::array<double,7> q_pred_;
  std::array<double,7> dq_pred_;
  std::array<double,9> I_total_pred_;
  double m_total_pred_;
  std::array<double,3> F_x_Ctotal_pred_;
  std::array<double, 700> q_pred_list_; //unsigned int pred_list_length = 7*pred_samples_;
  std::array<double, 700> dotq_pred_list_;
  std::array<double, 700> tau_pred_list_;
  std::array<double, 49> mass_pred_;
  std::array<double, 7> coriolis_pred_;
  std::array<double, 7> gravity_pred_;
  std::array<double, 7> tau_pred_;
  arma::mat arma_mass_pred_;
  arma::vec arma_coriolis_pred_;
  arma::vec arma_gravity_pred_;
  arma::vec arma_tau_pred_;
  arma::vec arma_ddq_pred_;
  std::array<double, 7> ddq_pred_;
};

}  // namespace hiro_ros_arm_controller