#ifndef GPS_WORKER_H
#define GPS_WORKER_H

// C++ includes
#include <memory>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>

 #include <string>
 #include <time.h>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>

#include "rs232.h"
#include "an_packet_protocol.h"
#include "spatial_packets.h"

// #include <bfl/wrappers/matrix/matrix_wrapper.h>
#define pi 3.1415926

using namespace std;

#ifdef __cplusplus
extern "C"
{
#endif

/**
单位：
位置: 	   m
速度：		  m/s
欧拉角：　	rad
角速度:     rad/s
**/

/**
全局位置方向：　采用北东地导航坐标系，地球北向north坐标，地球东向east坐标
安装方向：
x 车辆前进为正
y 车辆右侧为正
z 车辆向下为正
**/

namespace gps_rtk_msgs{


class GpsWorker
{
public:
	GpsWorker(string gps_serialport,int gps_baudrate,string record_or_read,string if_read_then_csv_name,string write_csv_path_name);
	virtual ~GpsWorker();

	std::vector<std::string> spilt_string(const std::string &str, const std::string &delim);
	void write_csv_title();
	void open_port();
	void gps_go(ros::Publisher &odom_pub,ros::Publisher &standard_pub,ros::Publisher &pose_pub,ros::Publisher &fix_pub,ros::Publisher &fix_raw_pub,ros::Publisher &velocity_raw_pub, ros::Publisher &imu_pub,ros::Publisher &imu_raw_pub, ros::Publisher &satellites_pub, ros::Publisher &euler_pub, ros::Publisher &odom_vehicle_pub, ros::Publisher &pose_vehicle_pub, ros::Publisher &euler_vehicle_pub, ros::Publisher &imu_vehicle_raw_pub);
	

private:
	string gps_serialport;
	int gps_baudrate;
	int gps_rate;
	// Record_or_read
	string record_or_read;
	string if_read_then_csv_name;
	string write_csv_path_name;

	vector<double> utc_csv_time;
	vector<double> utm_csv_north_x;
	vector<double> utm_csv_east_y;
	vector<double> utm_csv_height_z;
	vector<double> quaternions_csv_qx;
	vector<double> quaternions_csv_qy;
	vector<double> quaternions_csv_qz;
	vector<double> quaternions_csv_qw;
	vector<float> velocity_csv_Vx;
	vector<float> velocity_csv_Vy;
	vector<float> velocity_csv_Vz;
	unsigned long long int csv_arrays_length;

	// ID20 System State Packet
	double hard_time = 0;
	double latitude = 0;    //纬度(rad)
	double longitude = 0;	//经度(rad)
	double height = 0;
	double latitude_std;
	double longitude_std;
	double height_std;
	double v_north = 0;
	double v_east = 0;
	double v_down = 0;
	double a_x = 0;
	double a_y = 0;
	double a_z = 0;
	double roll_x = 0;	// deg
	double pitch_y = 0;
	double heading_z = 0;
	double roll_x_vehicle = 0;	// deg
	double pitch_y_vehicle = 0;
	double heading_z_vehicle = 0;
	double w_x = 0;		// rad/s
	double w_y = 0;
	double w_z = 0;
	// ID30 Satellites Packet
	double hdop = 0;
	double vdop = 0;
	double gps_satellites = 0;
	double glonass_satellites = 0;
	double beidou_satellites = 0;
	double galileo_satellites = 0;
	double sbas_satellites = 0;
	// ID34 UTM Position Packet /UTM 位置数据包
	double utm_north_x = 0;
	double utm_east_y = 0;
	double utm_height_z = 0;
	double utm_zone = 0;
	double utm_north_x_vehicle = 0;				//车辆坐标系　x朝前，y朝，z朝上
	double utm_east_y_vehicle  = 0;
	double utm_height_z_vehicle = 0;
	// ID36 Body Velocity Packet /载体速度数据包
	double v_x = 0;
	double v_y = 0;
	double v_z = 0;
	double v_x_vehicle = 0;
	double v_y_vehicle = 0;
	double v_z_vehicle = 0;
	// ID40 Quaternion Orientation Packet /四元数角度数据包
    double qw_0 = 1.0;
    double qx_1 = 0.0;
    double qy_2 = 0.0;
    double qz_3 = 0.0;
	// ID28 Raw Sensors Packet /传感器原始数据数据包
	double a_x_raw = 0;		
	double a_y_raw = 0;
	double a_z_raw = 0;
	double w_x_raw = 0;		// rad/s
	double w_y_raw = 0;
	double w_z_raw = 0;
	double a_x_vehicle_raw = 0;		
	double a_y_vehicle_raw = 0;
	double a_z_vehicle_raw = 0;
	double w_x_vehicle_raw = 0;		// rad/s
	double w_y_vehicle_raw = 0;
	double w_z_vehicle_raw = 0;
	// ID29 Raw GNSS Packet /原始 GNSS 数据包
	double latitude_raw = 0;    //纬度(rad)
	double longitude_raw = 0;	//经度(rad)
	double height_raw = 0;
	double latitude_raw_std;
	double longitude_raw_std;
	double height_raw_std;

	an_decoder_t an_decoder;
	an_packet_t *an_packet;
	
	system_state_packet_t system_state_packet;
	satellites_packet_t satellites_packet;
	utm_position_packet_t utm_position_packet;
	body_velocity_packet_t body_velocity_packet;
	quaternion_orientation_packet_t quaternion_orientation_packet;
	raw_sensors_packet_t raw_sensors_packet;
	raw_gnss_packet_t raw_gnss_packet;

	int bytes_received;

	ros::Time current_time;
	unsigned long long int sequenceNum = 0;
	unsigned long long int i = 0;

	sensor_msgs::NavSatFix fix;
	sensor_msgs::NavSatFix fix_raw;
	nav_msgs::Odometry odom;
	geometry_msgs::Vector3Stamped standard;
	geometry_msgs::Vector3Stamped satellites_status;
	geometry_msgs::PoseStamped this_pose_stamped;
	sensor_msgs::Imu imu;
	sensor_msgs::Imu imu_raw;
	geometry_msgs::Vector3Stamped euler;

	nav_msgs::Odometry odom_vehicle;
	geometry_msgs::PoseStamped this_pose_stamped_vehicle;
	sensor_msgs::Imu imu_vehicle_raw;
	geometry_msgs::Vector3Stamped euler_vehicle;

	geometry_msgs::Vector3Stamped velocity_raw;

	geometry_msgs::Quaternion q_vehicle;

	std::fstream myfile;
	std::string delim = ",";

};

}





#ifdef __cplusplus
}
#endif



#endif 
