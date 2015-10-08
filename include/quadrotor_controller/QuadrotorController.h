#ifndef Quadrotor_Controller_H
#define Quadrotor_Controller_H


#include <ros/ros.h>
#include <quadrotor_msgs/PDCommand.h>
#include <nav_msgs/Odometry.h>
#include <geometry_utils/GeometryUtilsROS.h>

namespace gu = geometry_utils;

class QuadrotorController
{
public:
  QuadrotorController();
  ~QuadrotorController();

  bool initialize(const ros::NodeHandle& n);

private:
  bool registerCallbacks(const ros::NodeHandle& n);
  bool loadParameters(const ros::NodeHandle& n);

  void viconOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  
  std::string name;
 
  ros::Subscriber odom_sub;
  ros::Publisher pd_cmd_pub;

  quadrotor_msgs::PDCommand pd_cmd;
  nav_msgs::Odometry odom;

  //geometry_msgs::Point pos_odom;
  //geometry_msgs::Quaternion att_odom;
  //geometry_msgs::Vector3 vel_odom, ang_odom;
  gu::Quat quat_odom;
  gu::Vec3 pos_odom, att_odom, vel_odom, ang_odom, u_linear, pos_des, vel_des, ang_des, g_vector;
  gu::Mat33 Kp_lin, Kd_lin, Kp_ang, Kd_ang;
  gu::Vec3 err_pos, err_vel, err_ang, err_att;
  double mass, phi_des, th_des, yaw_des, yaw_delta;
  bool sim_mode;
};

#endif
