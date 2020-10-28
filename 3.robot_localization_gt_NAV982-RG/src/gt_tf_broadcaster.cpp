#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include<iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>

#include <sophus/se3.hpp>

#include "robot_localization/math_utils.hpp"
#include "robot_localization/utils.h"

using namespace std;

bool first = true;
bool use_G_frame = false;
bool use_debug = false;
std::string world_frame_id, odom_frame_id, odom_gps_frame_id, gpsInit_frame_id, gps_base_frame_id, base_frame_id_W, base_frame_id_GS, base_frame_id_GB;
geometry_msgs::TransformStamped tf_worldToInit, tf_worldToGPSinit, tf_worldToBase;

ros::Publisher odom_vehicle_pub_W;
nav_msgs::Odometry odom_vehicle_W;
geometry_msgs::TransformStamped T_odom0_base_tf_W;

ros::Publisher odom_vehicle_pub_GSi;
nav_msgs::Odometry odom_vehicle_GSi;
ros::Publisher odom_vehicle_pub_GBi;
nav_msgs::Odometry odom_vehicle_GBi;
geometry_msgs::TransformStamped T_odom0_base_tf_GSi;
geometry_msgs::TransformStamped T_odom0_base_tf_GBi;

std::ofstream pose_file_W;
std::ofstream odom_file_G;
std::ofstream pose_file_G;
std::string output_path;

Eigen::Isometry3d T_body_gps;

long long int counnumber = 0;

double doo_curr_time = 0;
bool is_doo_curr_time_init = false;

double callback_curr_time = 0;
double callback_last_time = 0;
double delta_time = 0;

void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
{

 counnumber ++ ;
 if(counnumber < 10){
    return;
  }
 if (first)
 {
  tf_worldToInit.header.stamp = msg->header.stamp;
  tf_worldToInit.header.frame_id = world_frame_id;
  tf_worldToInit.child_frame_id = odom_gps_frame_id;
  tf_worldToInit.transform.translation.x = msg->pose.pose.position.x;
  tf_worldToInit.transform.translation.y = msg->pose.pose.position.y;
  tf_worldToInit.transform.translation.z = - msg->pose.pose.position.z;
  tf_worldToInit.transform.rotation = msg->pose.pose.orientation;

  tf_worldToGPSinit.header.stamp = msg->header.stamp;
  tf_worldToGPSinit.header.frame_id = world_frame_id;
  tf_worldToGPSinit.child_frame_id = gpsInit_frame_id;
  tf_worldToGPSinit.transform.translation.x = msg->pose.pose.position.x;
  tf_worldToGPSinit.transform.translation.y = msg->pose.pose.position.y;
  tf_worldToGPSinit.transform.translation.z = - msg->pose.pose.position.z;
  tf_worldToGPSinit.transform.rotation.x = 0;
  tf_worldToGPSinit.transform.rotation.y = 0;
  tf_worldToGPSinit.transform.rotation.z = 0;
  tf_worldToGPSinit.transform.rotation.w = 1;

  first = false;

  callback_curr_time = msg->header.stamp.toSec();
  callback_last_time = callback_curr_time;
 }
 callback_curr_time = msg->header.stamp.toSec();
 delta_time = callback_curr_time - callback_last_time;

 tf_worldToBase.header.stamp = msg->header.stamp;
 tf_worldToBase.header.frame_id = world_frame_id;
 tf_worldToBase.child_frame_id = gps_base_frame_id;
 tf_worldToBase.transform.translation.x = msg->pose.pose.position.x;
 tf_worldToBase.transform.translation.y = msg->pose.pose.position.y;
 tf_worldToBase.transform.translation.z = - msg->pose.pose.position.z;
 tf_worldToBase.transform.rotation = msg->pose.pose.orientation;

  // T_SB0_S0
  Eigen::Isometry3d T_odom0_gps0 = Eigen::Isometry3d::Identity();
  // Eigen四元数初始化 w x y z，此为绕x轴旋转pi, z down -> z up
  T_odom0_gps0.linear() =  Eigen::Quaterniond(0,1,0,0).toRotationMatrix();
  // For simplicity (以gps中心为vehicle后轴中心)
  T_odom0_gps0.translation() = Eigen::Vector3d (0,0,0);
  Eigen::Isometry3d T_word_gpsbase = tf2::transformToEigen(tf_worldToBase);
  Eigen::Isometry3d T_word_gps0 = tf2::transformToEigen(tf_worldToInit);
  Eigen::Isometry3d T_G_gps0 = tf2::transformToEigen(tf_worldToGPSinit);
  
  Eigen::Isometry3d T_odom0_base_WB = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_odom0_base_GSB_Si = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_odom0_base_GSB_Bi = Eigen::Isometry3d::Identity();

  // {T_GSB_SBi}
  // Hamilton T_GSB_SBi ok
  Eigen::Isometry3d T_S0_Si = T_word_gps0.inverse() * T_word_gpsbase;
  Eigen::Isometry3d T_SB0_SBi =  T_odom0_gps0 * T_S0_Si * T_odom0_gps0.inverse();
  Eigen::Matrix3d R_W_gps0 = T_word_gps0.linear();
  Eigen::Quaterniond q_W_gps0;
  q_W_gps0 = R_W_gps0;
  double roll_W_gps0, pitch_W_gps0, yaw_W_gps0;
  tf::Matrix3x3(tf::Quaternion(q_W_gps0.x(),q_W_gps0.y(),q_W_gps0.z(),q_W_gps0.w())).getRPY(roll_W_gps0, pitch_W_gps0, yaw_W_gps0, 1);
  // T_GS_S0 Hamilton
  Eigen::Isometry3d T_GS_S0 = Eigen::Isometry3d::Identity();
  T_GS_S0.translation() = T_word_gps0.translation();
  geometry_msgs::Quaternion q_GS_S0 = tf::createQuaternionMsgFromRollPitchYaw(roll_W_gps0, pitch_W_gps0, 0);
  T_GS_S0.linear() = Eigen::Quaterniond(q_GS_S0.w,q_GS_S0.x,q_GS_S0.y,q_GS_S0.z).toRotationMatrix();
  Eigen::Isometry3d T_GSB_SB0 = T_odom0_gps0 * T_GS_S0 * T_odom0_gps0.inverse();
  // GPS起点作为{GSB}系origin
  T_GSB_SB0.translation().setZero();
  Eigen::Isometry3d T_GSB_SBi = T_GSB_SB0 * T_SB0_SBi;
  T_odom0_base_GSB_Si = T_GSB_SBi;
  if(use_debug){
    std::cout.precision(16);
    Eigen::Quaterniond q_T_GSB_SBi = Eigen::Quaterniond(T_GSB_SBi.linear());
    double roll_T_GSB_SBi, pitch_T_GSB_SBi, yaw_T_GSB_SBi;
    tf::Matrix3x3(tf::Quaternion(q_T_GSB_SBi.x(),q_T_GSB_SBi.y(),q_T_GSB_SBi.z(),q_T_GSB_SBi.w())).getRPY(roll_T_GSB_SBi, pitch_T_GSB_SBi, yaw_T_GSB_SBi, 1);
    std::cout<<"gt_tf_broadcaster GPS起点作为{GSB}系下SBi当前位置ENU q_T_GSB_SBi : roll_T_GSB_SBi="<< roll_T_GSB_SBi * 180 / M_PI <<" deg, pitch_T_GSB_SBi="<< pitch_T_GSB_SBi * 180 / M_PI <<" deg, yaw_T_GSB_SBi="<< yaw_T_GSB_SBi * 180 / M_PI <<" deg" << std::endl;
    Eigen::Vector3d t_GSB_SBi = T_GSB_SBi.translation();
    std::cout<<"gt_tf_broadcaster GPS起点作为{GSB}系下SBi当前位置ENU T_GSB_SBi : x="<< t_GSB_SBi[0] <<" m,　y="<< t_GSB_SBi[1] <<" m,　z="<< t_GSB_SBi[2] <<" m" << std::endl;
    std::cout << "-----------------" << std::endl;
  }

  // {T_GSB_Bi}
  // Hamilton T_GSB_Bi ok
  Eigen::Isometry3d T_SB0_Bi =  T_odom0_gps0 * T_S0_Si * T_body_gps;
  Eigen::Isometry3d T_GSB_Bi = T_GSB_SB0 * T_SB0_Bi;
  T_odom0_base_GSB_Bi = T_GSB_Bi;

  // {W}
  // Hamilton T_W_Bi ok
  // T_body_gps JPL
  Eigen::Isometry3d T_B0_Bi = (T_body_gps.inverse()) * T_S0_Si * T_body_gps;
  T_odom0_base_WB = T_B0_Bi;
  if(use_debug){
    //发布车辆姿态欧拉角，相对于W  ENU
    Eigen::Quaterniond q_Bi_W;
    q_Bi_W = T_odom0_base_WB.linear();
    Eigen::Vector3d t_Bi_W = T_odom0_base_WB.translation();
    double roll_Bi_W, pitch_Bi_W, yaw_Bi_W;
    tf::Matrix3x3(tf::Quaternion(q_Bi_W.x(),q_Bi_W.y(),q_Bi_W.z(),q_Bi_W.w())).getRPY(roll_Bi_W, pitch_Bi_W, yaw_Bi_W, 1);     
    std::cout.precision(16);
    std::cout<<"gt_tf_broadcaster 车辆起点作为{W}系下车辆当前姿态ENU q_Bi_W : roll="<< roll_Bi_W * 180 / M_PI <<" deg,　pitch="<< pitch_Bi_W * 180 / M_PI <<" deg,　yaw_="<< yaw_Bi_W * 180 / M_PI <<" deg" << std::endl;
    std::cout<<"gt_tf_broadcaster 车辆起点作为{W}系下车辆当前位置ENU t_Bi_W : x="<< t_Bi_W[0] <<" m,　y="<< t_Bi_W[1] <<" m,　z="<< t_Bi_W[2] <<" m" << std::endl;
    std::cout << std::endl;
  }

  // {W}
  T_odom0_base_tf_W = tf2::eigenToTransform(T_odom0_base_WB);
  T_odom0_base_tf_W.header.stamp = msg->header.stamp;
  T_odom0_base_tf_W.header.frame_id = odom_frame_id;
  T_odom0_base_tf_W.child_frame_id = base_frame_id_W;

  // 速度变换到车辆坐标系下
  Eigen::Matrix3d R_gps_body = (T_body_gps.inverse()).linear();
  Eigen::Vector3d t_body_gps = T_body_gps.translation();
  Eigen::Vector3d gps_body_velocity;
  tf::vectorMsgToEigen(msg->twist.twist.linear,gps_body_velocity);
  Eigen::Vector3d gps_body_angularVelocity;
  tf::vectorMsgToEigen(msg->twist.twist.angular,gps_body_angularVelocity);
  Eigen::Vector3d body_velocity = R_gps_body * gps_body_velocity + R_gps_body * skewSymmetric(gps_body_angularVelocity) * t_body_gps;
  Eigen::Vector3d body_angularVelocity = R_gps_body * gps_body_angularVelocity;
  tf::vectorEigenToMsg(body_velocity, odom_vehicle_W.twist.twist.linear);
  tf::vectorEigenToMsg(body_angularVelocity, odom_vehicle_W.twist.twist.angular);
  
  tf::vectorEigenToMsg(body_velocity, odom_vehicle_GBi.twist.twist.linear);
  tf::vectorEigenToMsg(body_angularVelocity, odom_vehicle_GBi.twist.twist.angular);

  Eigen::Matrix3d R_gps_body_SBi = T_odom0_gps0.linear();
  Eigen::Vector3d t_body_gps_SBi= (T_odom0_gps0.inverse()).translation();
  Eigen::Vector3d body_velocity_SBi= R_gps_body_SBi * gps_body_velocity + R_gps_body_SBi * skewSymmetric(gps_body_angularVelocity) * t_body_gps_SBi;
  Eigen::Vector3d body_angularVelocity_SBi = R_gps_body_SBi * gps_body_angularVelocity;
  tf::vectorEigenToMsg(body_velocity_SBi, odom_vehicle_GSi.twist.twist.linear);
  tf::vectorEigenToMsg(body_angularVelocity_SBi, odom_vehicle_GSi.twist.twist.angular);

 odom_vehicle_W.header.stamp = msg->header.stamp;
 odom_vehicle_W.header.frame_id = odom_frame_id;
 odom_vehicle_W.child_frame_id = base_frame_id_W;
 odom_vehicle_W.pose.pose.position.x = T_odom0_base_tf_W.transform.translation.x;
 odom_vehicle_W.pose.pose.position.y = T_odom0_base_tf_W.transform.translation.y;
 odom_vehicle_W.pose.pose.position.z = T_odom0_base_tf_W.transform.translation.z;
 odom_vehicle_W.pose.pose.orientation = T_odom0_base_tf_W.transform.rotation;

  // {GSi}
  T_odom0_base_tf_GSi = tf2::eigenToTransform(T_odom0_base_GSB_Si);
  T_odom0_base_tf_GSi.header.stamp = msg->header.stamp;
  T_odom0_base_tf_GSi.header.frame_id = odom_frame_id;
  T_odom0_base_tf_GSi.child_frame_id = base_frame_id_GS;

 odom_vehicle_GSi.header.stamp = msg->header.stamp;
 odom_vehicle_GSi.header.frame_id = odom_frame_id;
 odom_vehicle_GSi.child_frame_id = base_frame_id_GS;
 odom_vehicle_GSi.pose.pose.position.x = T_odom0_base_tf_GSi.transform.translation.x;
 odom_vehicle_GSi.pose.pose.position.y = T_odom0_base_tf_GSi.transform.translation.y;
 odom_vehicle_GSi.pose.pose.position.z = T_odom0_base_tf_GSi.transform.translation.z;
 odom_vehicle_GSi.pose.pose.orientation = T_odom0_base_tf_GSi.transform.rotation;

  // {GBi}
  T_odom0_base_tf_GBi = tf2::eigenToTransform(T_odom0_base_GSB_Bi);
  T_odom0_base_tf_GBi.header.stamp = msg->header.stamp;
  T_odom0_base_tf_GBi.header.frame_id = odom_frame_id;
  T_odom0_base_tf_GBi.child_frame_id = base_frame_id_GB;

  odom_vehicle_GBi.header.stamp = msg->header.stamp;
  odom_vehicle_GBi.header.frame_id = odom_frame_id;
  odom_vehicle_GBi.child_frame_id = base_frame_id_GB;
  odom_vehicle_GBi.pose.pose.position.x = T_odom0_base_tf_GBi.transform.translation.x;
  odom_vehicle_GBi.pose.pose.position.y = T_odom0_base_tf_GBi.transform.translation.y;
  odom_vehicle_GBi.pose.pose.position.z = T_odom0_base_tf_GBi.transform.translation.z;
  odom_vehicle_GBi.pose.pose.orientation = T_odom0_base_tf_GBi.transform.rotation;

 //  50Hz
 if(delta_time > 0.02 - 0.003){

    odom_vehicle_pub_W.publish(odom_vehicle_W);
    odom_vehicle_pub_GSi.publish(odom_vehicle_GSi);
    odom_vehicle_pub_GBi.publish(odom_vehicle_GBi);

    double dStamp_W = odom_vehicle_W.header.stamp.toSec();
    Eigen::Vector3d p_wi_W;
    Eigen::Quaterniond q_wi_W;
    p_wi_W = tf2::transformToEigen(T_odom0_base_tf_W).translation();
    q_wi_W = Eigen::Quaterniond(tf2::transformToEigen(T_odom0_base_tf_W).linear());
    // 保存TUM格式 #timestamp(sec) x y z q_x q_y q_z q_w
    pose_file_W.precision(16);
    pose_file_W << fixed << dStamp_W << " " << p_wi_W(0) << " " << p_wi_W(1) << " " << p_wi_W(2) << " "
            << q_wi_W.x() << " " << q_wi_W.y() << " " << q_wi_W.z() << " " << q_wi_W.w() << endl;

    double dStamp_G = odom_vehicle_GBi.header.stamp.toSec();
    Eigen::Vector3d p_wi_G;
    Eigen::Quaterniond q_wi_G;
    p_wi_G = tf2::transformToEigen(T_odom0_base_tf_GBi).translation();
    q_wi_G = Eigen::Quaterniond(tf2::transformToEigen(T_odom0_base_tf_GBi).linear());
    // 保存TUM格式 #timestamp(sec) x y z q_x q_y q_z q_w
    pose_file_G.precision(16);
    pose_file_G << fixed << dStamp_G << " " << p_wi_G(0) << " " << p_wi_G(1) << " " << p_wi_G(2) << " "
            << q_wi_G.x() << " " << q_wi_G.y() << " " << q_wi_G.z() << " " << q_wi_G.w() << endl;

    // 保存odom
    Eigen::Vector3d b_v;
    Eigen::Vector3d b_w;
    tf::vectorMsgToEigen(odom_vehicle_GBi.twist.twist.linear,b_v);
    tf::vectorMsgToEigen(odom_vehicle_GBi.twist.twist.angular,b_w);
    double roll_wi_G, pitch_wi_G, yaw_wi_G;
    tf::Matrix3x3(tf::Quaternion(q_wi_G.x(),q_wi_G.y(),q_wi_G.z(),q_wi_G.w())).getRPY(roll_wi_G, pitch_wi_G, yaw_wi_G, 1);  
    roll_wi_G = roll_wi_G * 180 / M_PI;  
    pitch_wi_G = pitch_wi_G * 180 / M_PI; 
    yaw_wi_G = yaw_wi_G * 180 / M_PI; 
    std::string delim = ",";
    odom_file_G.precision(16);
    odom_file_G << fixed << dStamp_G << delim;
    odom_file_G << p_wi_G(0) << delim << p_wi_G(1) << delim << p_wi_G(2) << delim;
    odom_file_G << q_wi_G.x() << delim <<  q_wi_G.y() << delim << q_wi_G.z() << delim << q_wi_G.w() << delim;
    odom_file_G << roll_wi_G << delim << pitch_wi_G << delim << yaw_wi_G << delim;
    odom_file_G << b_v(0) << delim << b_v(1) << delim << b_v(2) << delim;
    odom_file_G << b_w(0) << delim << b_w(1) << delim << b_w(2) << delim;
    odom_file_G << std::endl;

    callback_last_time = callback_curr_time;

 }

}

int main(int argc, char** argv)
{	
 ros::init(argc, argv, "gt_tf_broadcaster");
 ros::NodeHandle nh;
 ros::NodeHandle priv_nh("~");

 priv_nh.param<bool>("use_debug", use_debug, false);
 priv_nh.param<std::string>("world_frame_id", world_frame_id, "world_UTM_");
 priv_nh.param<std::string>("gpsInit_frame_id", gpsInit_frame_id, "world_UTM_init");
 priv_nh.param<std::string>("odom_frame_id", odom_frame_id, "odom");
 priv_nh.param<std::string>("odom_gps_frame_id", odom_gps_frame_id, "odom_gps");
 priv_nh.param<std::string>("gps_base_frame_id", gps_base_frame_id, "gps_base_link");
 priv_nh.param<std::string>("base_frame_id_W", base_frame_id_W, "gt_base_link_W");
 priv_nh.param<std::string>("base_frame_id_GS", base_frame_id_GS, "gt_base_link_GS");
 priv_nh.param<std::string>("base_frame_id_GB", base_frame_id_GB, "gt_base_link_GB");

//  JPL
 T_body_gps = utils::getTransformEigen(priv_nh, "T_gps_body");

output_path = priv_nh.param<string>("output_path","");
ROS_INFO_STREAM("Loaded ex_output_file: " << output_path);

pose_file_W.open((output_path + "pose_gps_W.txt"), std::ios::out);
if(!pose_file_W.is_open())
{
    cerr << "pose_gps_W is not open" << endl;
}
pose_file_G.open((output_path + "pose_gps_G.txt"), std::ios::out);
if(!pose_file_G.is_open())
{
    cerr << "pose_gps_G is not open" << endl;
}
odom_file_G.open((output_path + "odom_gps_G.csv"), std::ios::out);
if(!odom_file_G.is_open())
{
    cerr << "odom_gps_G is not open" << endl;
}
odom_file_G << "#Time(sec),";
odom_file_G << "x_G(m),y_G(m),z_G(m),";
odom_file_G << "qx_G,qy_G,qz_G,qw_G,";
odom_file_G << "roll_x_G(deg),pitch_y_G(deg),yaw_z_G(deg),";
odom_file_G << "vx(m/s),vy(m/s),vz(m/s),";
odom_file_G << "wx(rad/s),wy(rad/s),wz(rad/s),";
odom_file_G << std::endl; 

 ros::Subscriber sub = nh.subscribe("odom", 200, odometryCallback);

 // 发布里程计数据 pub
 odom_vehicle_pub_W = nh.advertise<nav_msgs::Odometry>("/gps_rtk_msgs/gt_odom_W", 100,true);
 odom_vehicle_pub_GBi = nh.advertise<nav_msgs::Odometry>("/gps_rtk_msgs/gt_odom_G", 100,true);
 odom_vehicle_pub_GSi = nh.advertise<nav_msgs::Odometry>("/gps_rtk_msgs/gt_odom_G_SBi", 100,true);

 ros::spin();
 pose_file_W.close();
 pose_file_G.close();
 odom_file_G.close();

}
