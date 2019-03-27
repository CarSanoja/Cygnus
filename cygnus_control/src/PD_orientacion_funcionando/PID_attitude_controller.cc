/*
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 *
 * This code can be used only for academic, non-commercial use.
 * This code cannot be redistributed under any license, open source or otherwise.
 *
 * CVXGEN license: http://cvxgen.com/docs/license.html
 * FORCES license: http://forces.ethz.ch
 *
 */

#include <mav_lowlevel_attitude_controller/PID_attitude_controller.h>
#include <rosbag/bag.h>


namespace mav_control {

PIDAttitudeController::PIDAttitudeController(const ros::NodeHandle& nh, const ros::NodeHandle private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      initialized_params_(false),
      pitch_error_integration_(0.0),
      roll_error_integration_(0.0),
      n_rotors_(4)
{
  yaw_rate_gain_ = 1.0;
  roll_gain_ = 1.0;
  pitch_gain_ = 1.0;
  roll_integrator_gain_ = 1.0;
  pitch_integrator_gain_ = 1.0;
  p_gain_ = 1.0;
  q_gain_ = 1.0;
  r_gain_ = 1.0;
  max_integrator_error_ = 1.0;
}

PIDAttitudeController::~PIDAttitudeController()
{
}

void PIDAttitudeController::InitializeParams()
{

  

  //Get parameters from RosParam server
  std::vector<double> temporary_inertia_matrix, temporary_allocation_matrix;

  double rotor_force_constant;  //F_i = k_n * rotor_velocity_i^2
  double rotor_moment_constant;  // M_i = k_m * F_i
  double arm_length;

  if (!private_nh_.getParam("rotor_force_constant", rotor_force_constant)) {
    ROS_ERROR(
        "rotor_force_constant in PID attitude controller is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("rotor_moment_constant", rotor_moment_constant)) {
    ROS_ERROR(
        "rotor_moment_constant in PID attitude controller is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("arm_length", arm_length)) {
    ROS_ERROR("arm_length in PID attitude controller is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("inertia", temporary_inertia_matrix)) {
    ROS_ERROR("arm_length in PID attitude controller is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("allocation_matrix", temporary_allocation_matrix)) {
    ROS_ERROR(
        "allocation matrix in PID attitude controller is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("n_rotors", n_rotors_)) {
    ROS_ERROR("n_rotors in PID attitude controller is not loaded from ros parameter server");
    abort();
  }

  if (temporary_allocation_matrix.size() != n_rotors_ * 4) {
    ROS_ERROR("PID attitude controller: dimension of allocation matrix is wrong.");
    abort();
  }

  if (!private_nh_.getParam("max_integrator_error", max_integrator_error_)) {
    ROS_ERROR(
        "max_integrator_error in PID attitude controller is not loaded from ros parameter server");
    abort();
  }

  Eigen::Map<Eigen::MatrixXd> allocation_matrix_map(temporary_allocation_matrix.data(), 4,
                                                    n_rotors_);
  Eigen::Map<Eigen::MatrixXd> inertia_matrix_map(temporary_inertia_matrix.data(), 3, 3);

  Eigen::MatrixXd allocation_matrix(4, n_rotors_);
  allocation_matrix = allocation_matrix_map;

  inertia_ = inertia_matrix_map;

  Eigen::Matrix4d I;
  I.setZero();
  I.block<3, 3>(0, 0) = inertia_;
  I(3, 3) = 1;

  Eigen::Matrix4d K;
  K.setZero();
  K(0, 0) = arm_length * rotor_force_constant;
  K(1, 1) = arm_length * rotor_force_constant;
  K(2, 2) = rotor_moment_constant;
  K(3, 3) = rotor_force_constant;


  angular_acc_to_rotor_velocities_ = allocation_matrix.transpose()
      * (allocation_matrix * allocation_matrix.transpose()).inverse() * K.inverse() * I;
  std::cout << allocation_matrix.transpose()
      * (allocation_matrix * allocation_matrix.transpose()).inverse() << std::endl;
  std::cout << K.inverse() << std::endl;
  std::cout << angular_acc_to_rotor_velocities_ << std::endl;

  attitude_thrust_reference_.setZero();

  initialized_params_ = true;
}

void PIDAttitudeController::SetOdometry(const mav_msgs::EigenOdometry& odometry)
{
  odometry_ = odometry;
}

RetVal PIDAttitudeController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities)
{
  assert(rotor_velocities);
  assert(initialized_params_);

  rotor_velocities->resize(n_rotors_);

  Eigen::Vector3d angular_acceleration;
  //////////////////////////////////////////////////////////////////7
  RetVal errores;
  errores = ComputeDesiredAngularAcc(&angular_acceleration);
  ///////////////////////////////////////////////////////////////////

  Eigen::Vector4d angular_acceleration_thrust;
  angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
  angular_acceleration_thrust(3) = attitude_thrust_reference_(3);

  Eigen::Vector4d cross_term;
  cross_term
      << inertia_.inverse()
          * (odometry_.angular_velocity_B.cross(inertia_ * odometry_.angular_velocity_B)), 0;
  ROS_INFO("REFERENCIAS [%f],[%f],[%f],[%f]",angular_acceleration_thrust(0),angular_acceleration_thrust(1),angular_acceleration_thrust(2),angular_acceleration_thrust(3));
  ROS_INFO("CROSS_TERM [%f],[%f],[%f],[%f]",cross_term(0),cross_term(1),cross_term(2),cross_term(3));

  std::cout << angular_acc_to_rotor_velocities_<< std::endl;
  std::cout << angular_acceleration_thrust - cross_term<< std::endl;
  *rotor_velocities = angular_acc_to_rotor_velocities_ * (angular_acceleration_thrust - cross_term);
  std::cout << *rotor_velocities<< std::endl;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  std::cout << *rotor_velocities<< std::endl;
  *rotor_velocities = rotor_velocities->cwiseSqrt();
  std::cout << *rotor_velocities<< std::endl;
  return errores;
}

double PIDAttitudeController::ComputeAxisVelocity(double error_axis){
  double axis_velocity;
  int x;
  if (error_axis>0) {
    x = 1;
  }
  else {
    x = -1;
  }
  axis_velocity = x*error_axis * 0.06;
  if (axis_velocity > 0.1){
    axis_velocity = 0.1;
  }
  else if (axis_velocity < -0.1){
    axis_velocity = -0.1;
  }
  return axis_velocity;
}

RetVal PIDAttitudeController::ComputeDesiredAngularAcc(Eigen::Vector3d* angular_acc)
{
  assert(angular_acc);

  Eigen::Vector3d current_rpy;
  odometry_.getEulerAngles(&current_rpy);

  double error_roll = attitude_thrust_reference_(0) - current_rpy(0);
  ROS_INFO("attitude_thrust_reference_(0) [%f] ", attitude_thrust_reference_(0));
  ROS_INFO("ACTUAL ROLL [%f] ",current_rpy(0));
  ROS_INFO("ERROR ROLL [%f] ", error_roll);
  double error_pitch = attitude_thrust_reference_(1) - current_rpy(1);
  ROS_INFO("ACTUAL PITCH [%f] ",current_rpy(1));
  ROS_INFO("ERROR PITCH [%f] ", error_pitch);

  roll_error_integration_ += error_roll;
  pitch_error_integration_ += error_pitch;

  if (std::abs(roll_error_integration_) > max_integrator_error_)
    roll_error_integration_ = max_integrator_error_ * roll_error_integration_
        / std::abs(roll_error_integration_);

  if (std::abs(pitch_error_integration_) > max_integrator_error_)
    pitch_error_integration_ = max_integrator_error_ * pitch_error_integration_
        / std::abs(pitch_error_integration_);

  //////////////////
        ROS_INFO("Axis velocities");
  axis_velocity_p = ComputeAxisVelocity(odometry_.position_W(1));
  ROS_INFO("posicion y velocidad p roll [%f]",odometry_.position_W(1));
  ROS_INFO("velocidad deseada [%f]",axis_velocity_p);
  axis_velocity_q = ComputeAxisVelocity(odometry_.position_W(0));
  ROS_INFO("posicion y velocidad pq pitch [%f]",odometry_.position_W(0));
  ROS_INFO("velocidad deseada [%f]",axis_velocity_q);

  //////////////////

  //desired omega = [0;0;0]
  double error_p = 0 - odometry_.angular_velocity_B(0);
  ROS_INFO("ERROR ROLL RATE [%f] ", error_p);
  double error_q = 0 - odometry_.angular_velocity_B(1);
  ROS_INFO("ERROR PITCH RATE [%f] ", error_q);
  double error_r = attitude_thrust_reference_(2) - odometry_.angular_velocity_B.z();
  ROS_INFO("ACTUAL YAw RATE [%f] ",odometry_.angular_velocity_B.z());
  ROS_INFO("ERROR YAW RATE [%f] ", error_r);

  ROS_INFO("ODOMETRIA ANGULAR W");
  ROS_INFO("Velocidad en p [%f]",odometry_.angular_velocity_B(0));
  ROS_INFO("Velocidad en q [%f]",odometry_.angular_velocity_B(1));

  *angular_acc
      << (roll_gain_ * error_roll + p_gain_ * error_p
          + roll_integrator_gain_ * roll_error_integration_), (pitch_gain_ * error_pitch
      + q_gain_ * error_q + pitch_integrator_gain_ * pitch_error_integration_), r_gain_ * error_r;
  ROS_INFO("Señal de control: ");
  ROS_INFO("roll: [%f]",roll_gain_ * error_roll + p_gain_ * error_p + roll_integrator_gain_ * roll_error_integration_);
  ROS_INFO("pitch : [%f]", pitch_gain_ * error_pitch + q_gain_ * error_q + pitch_integrator_gain_ * pitch_error_integration_);
  ROS_INFO("yaw: [%f]", r_gain_ * error_r);


  ////////////////////////////////////////////////////////////////////////////////
  // CONTROL DE VELOCIDAD ANGULAR
  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////
  // CONTROL DE VELOCIDAD ANGULAR
  ////////////////////////////////////////////////////////////////////////////////
/*
  double error_w_roll = roll_gain_ * error_roll + p_gain_ * error_p + roll_integrator_gain_ * roll_error_integration_ - odometry_.angular_velocity_B(0);
  double error_w_pitch = pitch_gain_ * error_pitch + q_gain_ * error_q + pitch_integrator_gain_ * pitch_error_integration_ - odometry_.angular_velocity_B(1);
  double error_r = r_gain_ * error_yaw -odometry_.angular_velocity_B(3);
  ROS_INFO("ACTUAL YAw RATE [%f] ",odometry_.angular_velocity_B.z());
  ROS_INFO("ERROR YAW RATE [%f] ", error_r);

  roll_w_error_integration_ += error_w_roll;
  pitch_w_error_integration_ += error_w_pitch;

  if (std::abs(roll_w_error_integration_) > max_integrator_error_)
    roll_w_error_integration_ = max_integrator_error_ * roll_w_error_integration_
        / std::abs(roll_w_error_integration_);

  if (std::abs(pitch_w_error_integration_) > max_integrator_error_)
    pitch_w_error_integration_ = max_integrator_error_ * pitch_w_error_integration_
        / std::abs(pitch_w_error_integration_);

  double roll_gain_2 =  2.04718049117933;
  double pitch_gain_2 =  3.39352318561591;
  double r_gain_2 =  1.53139901101877;
  double roll_integrator_gain_2 = 1.28467174273952 ;
  double pitch_integrator_gain_2 = 2.7127621271723 ;
*/

  return {error_roll, error_pitch, error_r,  error_p, error_q, roll_error_integration_, pitch_error_integration_ };

}



}



