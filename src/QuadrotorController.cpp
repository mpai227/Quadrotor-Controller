
#include <quadrotor_controller/QuadrotorController.h>
#include <geometry_utils/GeometryUtilsROS.h>
#include <geometry_utils/GeometryUtils.h>
#include <parameter_utils/ParameterUtils.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_utils/GeometryUtilsMath.h>

namespace pu = parameter_utils;
namespace gu = geometry_utils;
namespace gr = gu::ros;
namespace gm = gu::math;

QuadrotorController::QuadrotorController() {
  g_vector(2) = 9.81;
  pos_des(0) = 0;
  pos_des(1) = 0;
  pos_des(2) = 1;
  vel_des(0) = 0;
  vel_des(1) = 0;
  vel_des(2) = 0;
  ang_des(0) = 0;
  ang_des(1) = 0;
  ang_des(2) = 0;
  yaw_des = 0;
}

QuadrotorController::~QuadrotorController() {

}

bool QuadrotorController::initialize(const ros::NodeHandle& n) {

  name = ros::names::append(n.getNamespace(), "QuadrotorController");

  if (!loadParameters(n)) {
    ROS_ERROR("%s: failed to load parameters", name.c_str());
    return false;
  }

  if (!registerCallbacks(n)) {
    ROS_ERROR("%s: failed to register callbacks", name.c_str());
    return false;
  }

  return true;

}

bool QuadrotorController::registerCallbacks(const ros::NodeHandle& n) {
  
  ros::NodeHandle nl(n);
  
  odom_sub = 
    nl.subscribe("odom", 10, &QuadrotorController::viconOdomCallback, this);
   
  pd_cmd_pub = nl.advertise<quadrotor_msgs::PDCommand>("pd_cmd", 10, false);

  return true;

} 

bool QuadrotorController::loadParameters(const ros::NodeHandle & n) {

  if (!pu::get("mass", mass)) return false;

  if (!pu::get("Kp_lin_xy", Kp_lin(0,0))) return false;
  if (!pu::get("Kp_lin_xy", Kp_lin(1,1))) return false;
  if (!pu::get("Kp_lin_z", Kp_lin(2,2))) return false;
  if (!pu::get("Kd_lin_xy", Kd_lin(0,0))) return false;
  if (!pu::get("Kd_lin_xy", Kd_lin(1,1))) return false;
  if (!pu::get("Kd_lin_z", Kd_lin(2,2))) return false;
  if (!pu::get("Kp_ang_xy", Kp_ang(0,0))) return false;
  if (!pu::get("Kp_ang_xy", Kp_ang(1,1))) return false;
  if (!pu::get("Kp_ang_z", Kp_ang(2,2))) return false;
  if (!pu::get("Kd_ang_xy", Kd_ang(0,0))) return false;
  if (!pu::get("Kd_ang_xy", Kd_ang(1,1))) return false;
  if (!pu::get("Kd_ang_z", Kd_ang(2,2))) return false;
  
  if (!pu::get("sim_mode", sim_mode)) return false;

  return true;

}

void QuadrotorController::viconOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

  odom = *msg;

  pos_odom = gr::fromROS(odom.pose.pose.position);
  quat_odom = gr::fromROS(odom.pose.pose.orientation);
  att_odom(0) = gu::getRoll(quat_odom);
  att_odom(1) = gu::getPitch(quat_odom);
  att_odom(2) = gu::getYaw(quat_odom); 
  vel_odom = gr::fromROS(odom.twist.twist.linear);
  ang_odom = gr::fromROS(odom.twist.twist.angular);

  err_pos = pos_odom - pos_des;
  err_vel = vel_odom - vel_des;
  err_ang = ang_odom - ang_des;

  u_linear = mass*(g_vector - Kp_lin*err_pos - Kd_lin*err_vel);
  
  phi_des = (u_linear(0)*gm::sin(att_odom(2)) - u_linear(1)*gm::cos(att_odom(2))) / (mass*g_vector(2));
  th_des = (u_linear(0)*gm::cos(att_odom(2)) + u_linear(1)*gm::sin(att_odom(2))) / (mass*g_vector(2));
  gu::Vec3 att_des;
  att_des(0) = phi_des;
  att_des(1) = th_des;
  att_des(2) = yaw_des;
  yaw_delta = -Kp_ang(2,2)*gu::shortest_angular_distance(att_des(2), att_odom(2));
  err_att = att_odom - ang_des;
  double th_cmd;

  if (sim_mode == 1) {
    th_cmd = u_linear(2);
  } else {
    th_cmd = u_linear(2) / (2*mass*g_vector(2));
  }

  pd_cmd.header.stamp = ros::Time::now();
  pd_cmd.roll = (float) phi_des;
  pd_cmd.pitch = (float) th_des;
  pd_cmd.yaw_delta = (float) yaw_delta;
  pd_cmd.thrust = (float) th_cmd;
  pd_cmd.roll_speed = (float) ang_des(0);
  pd_cmd.pitch_speed = (float) ang_des(1);
  pd_cmd.yaw_speed = (float) ang_des(2);
  pd_cmd.kp_roll = (float) Kp_ang(0,0);
  pd_cmd.kp_pitch = (float) Kp_ang(1,1);
  pd_cmd.kd_roll = (float) Kd_ang(0,0);
  pd_cmd.kd_pitch = (float) Kd_ang(1,1);
  pd_cmd.kd_yaw = (float) Kd_ang(2,2);
  pd_cmd.gains_seq = rand()%1000;
  pd_cmd.speeds_seq = rand()%1000;
  pd_cmd_pub.publish(pd_cmd);
  

}
















