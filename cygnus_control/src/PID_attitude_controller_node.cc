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

#include <mav_msgs/default_topics.h>

#include <mav_lowlevel_attitude_controller/PID_attitude_controller_node.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include "ros/ros.h"



namespace mav_control {

PIDAttitudeControllerNode::PIDAttitudeControllerNode(const ros::NodeHandle& nh,
                                                     const ros::NodeHandle private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      got_first_attitude_command_(false),
      PID_attitude_controller_(nh, private_nh)
{

  PID_attitude_controller_.InitializeParams();

  command_roll_pitch_yawrate_thrust_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1,
      &PIDAttitudeControllerNode::CommandRollPitchYawRateThrustCallback, this);

  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                &PIDAttitudeControllerNode::OdometryCallback, this,
                                ros::TransportHints().tcpNoDelay());

  motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);
  control_output_pub_ = nh_.advertise<mav_msgs::Actuators>(
      "control_variables", 1);
  //ROS_INFO("LOADING MOTOR [%s] PLUGIN", mav_msgs::default_topics::COMMAND_ACTUATORS);

  //dynamic_reconfigure::Server<mav_linear_mpc::PIDAttitudeConfig>::CallbackType f;
  //f = boost::bind(&PIDAttitudeControllerNode::DynConfigCallback, this, _1, _2);
  //dyn_config_server_.setCallback(f);
}

PIDAttitudeControllerNode::~PIDAttitudeControllerNode()
{
}

void PIDAttitudeControllerNode::CommandRollPitchYawRateThrustCallback(
    const mav_msgs::RollPitchYawrateThrustConstPtr& roll_pitch_yawrate_thrust_reference)
{

  PID_attitude_controller_.SetDesiredAttitude(roll_pitch_yawrate_thrust_reference->roll,
                                              roll_pitch_yawrate_thrust_reference->pitch,
                                              roll_pitch_yawrate_thrust_reference->yaw_rate,
                                              roll_pitch_yawrate_thrust_reference->thrust.z);
  got_first_attitude_command_ = true;
  ROS_INFO_ONCE("CommandRollPitchYawRateThrustCallback got first message.");
}

void PIDAttitudeControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
  ROS_INFO_ONCE("PIDAttitudeController got first odometry message.");


  if (!got_first_attitude_command_)
    return;

  mav_msgs::EigenOdometry odometry;
  eigenOdometryFromMsg(*odometry_msg, &odometry);

  PID_attitude_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  mav_msgs::Actuators turning_velocities_msg;
  turning_velocities_msg.angular_velocities.clear();

  mav_msgs::Actuators control_out_msg;
  control_out_msg.angular_velocities.clear();


  ////////////////////////////////////////////////////////////////////////////////7
  RetVal errores;
  errores = PID_attitude_controller_.CalculateRotorVelocities(&ref_rotor_velocities);
  //FUNCIONA PARA EL CONTROL DEL ROLL
  //float k = 50;
  //float ky = 0.001;
  //float kp = 13.8;

  //Prueba control PITCH - FUNCIONANDO - Valores indicados !
  /*float k_roll = 50;
  float k_pitch =50;
  float kp = 8;
  float kq = 9;
  float ky = 15;
  float k_roll_int = 0;
  float k_pitch_int = 0;*/

  /*ROS_INFO("Velocidades [%f] ", ref_rotor_velocities[0] );
  ROS_INFO("Velocidades [%f] ", ref_rotor_velocities[1] );
  ROS_INFO("Velocidades [%f] ", ref_rotor_velocities[2] );
  ROS_INFO("Velocidades [%f] ", ref_rotor_velocities[3] );
  ROS_INFO("VELOCIDADES NUEVAS CON NUEVO CONTROL ");
  //Adicion de control sobre velocidad por variacion de angulos para reducir el error
  /* ref_rotor_velocities[0] = ref_rotor_velocities[0] - k_roll*errores.error_roll_vel_ - kp*errores.error_roll_p_vel - k_roll_int*errores.roll_error_integratio - k_pitch*errores.error_pitch_vel_ - k_pitch_int*errores.pitch_error_integratio - kq*errores.error_pitch_q_vel - ky*errores.error_yaw_vel_;
   ref_rotor_velocities[1] = ref_rotor_velocities[1] - k_roll*errores.error_roll_vel_ - kp*errores.error_roll_p_vel - k_roll_int*errores.roll_error_integratio + k_pitch*errores.error_pitch_vel_ + k_pitch_int*errores.pitch_error_integratio + kq*errores.error_pitch_q_vel + ky*errores.error_yaw_vel_;
   ref_rotor_velocities[2] = ref_rotor_velocities[2] + k_roll*errores.error_roll_vel_ + kp*errores.error_roll_p_vel + k_roll_int*errores.roll_error_integratio + k_pitch*errores.error_pitch_vel_ + k_pitch_int*errores.pitch_error_integratio + kq*errores.error_pitch_q_vel - ky*errores.error_yaw_vel_;
   ref_rotor_velocities[3] = ref_rotor_velocities[3] + k_roll*errores.error_roll_vel_ + kp*errores.error_roll_p_vel + k_roll_int*errores.roll_error_integratio - k_pitch*errores.error_pitch_vel_ - k_pitch_int*errores.pitch_error_integratio - kq*errores.error_pitch_q_vel + ky*errores.error_yaw_vel_; 
   /*if ( (ref_rotor_velocities[0]>=652) && (ref_rotor_velocities[1]>=652)&&(ref_rotor_velocities[2]>=652)&&(ref_rotor_velocities[3]>=650) ){
    ref_rotor_velocities[0] = ref_rotor_velocities[0] - 2.0;
    ref_rotor_velocities[1] = ref_rotor_velocities[1] - 2.0;
    ref_rotor_velocities[2] = ref_rotor_velocities[2] - 2.0;
    ref_rotor_velocities[3] = ref_rotor_velocities[3] - 2.0;
   }*/
   
   /////////////////////////////////////////////////////////////////////////////////

  

  for (int i = 0; i < ref_rotor_velocities.size(); i++){
    turning_velocities_msg.angular_velocities.push_back(ref_rotor_velocities[i]);
    ROS_INFO("Velocidades [%f] ", ref_rotor_velocities[i] );}
  turning_velocities_msg.header.stamp = odometry_msg->header.stamp;

  control_out_msg.angular_velocities.push_back(errores.tao_x) ;
  control_out_msg.angular_velocities.push_back(errores.tao_y) ;
  control_out_msg.angular_velocities.push_back(errores.tao_z) ;
  control_out_msg.header.stamp = odometry_msg->header.stamp;

  motor_velocity_reference_pub_.publish(turning_velocities_msg);

  
  control_output_pub_.publish(control_out_msg);
  //ros::Duration(0.005).sleep();
}

/*void PIDAttitudeControllerNode::DynConfigCallback(mav_linear_mpc::PIDAttitudeConfig &config,
                                                  uint32_t level)
{

  PID_attitude_controller_.SetPIDParameters(config.roll_gain, config.pitch_gain, config.p_gain,
                                            config.q_gain, config.r_gain, config.roll_int_gain,
                                            config.pitch_int_gain);
}
*/
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PIDAttitudeControllerNode");

  ros::NodeHandle nh, private_nh("~");

  mav_control::PIDAttitudeControllerNode PID_attitude_controller(nh, private_nh);

  ros::spin();

  return 0;
}
