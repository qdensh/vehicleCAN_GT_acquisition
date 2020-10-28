#include "gps_rtk_worker.h"

/**
单位：
时间：       s
位置: 	   m
速度：		  m/s
欧拉角：　	rad
角速度:     rad/s
**/

/**
全局位置方向：　采用北东地导航坐标系NEU，地球北向north坐标，地球东向east坐标
安装方向：
x 车辆前进为正
y 车辆右侧为正
z 车辆向下为正
**/

/**
四元数<->欧拉角：ZYX旋转方向（偏航、俯仰、横滚）
**/

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "gps_rtk_msgs");
    ros::NodeHandle nh;

    string gps_rtk_serialport;
	string gps_rtk_record_or_read;
	string gps_rtk_write_csv_path_name;
	int gps_rtk_baudrate;
	int gps_rtk_rate;

    nh.getParam("gps_rtk_serialport",gps_rtk_serialport);
    nh.getParam("gps_rtk_baudrate",gps_rtk_baudrate);
    nh.getParam("gps_rtk_rate",gps_rtk_rate);
 	nh.getParam("gps_rtk_record_or_read",gps_rtk_record_or_read);
	nh.getParam("gps_rtk_write_csv_path_name",gps_rtk_write_csv_path_name);

    ros::Time current_time;
	current_time = ros::Time::now();
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("gps_rtk_msgs/odom", 1,true);
	ros::Publisher std_pub = nh.advertise<geometry_msgs::Vector3Stamped>("gps_rtk_msgs/standard", 1,true);
	ros::Publisher satellites_pub = nh.advertise<geometry_msgs::Vector3Stamped>("gps_rtk_msgs/satellites_status", 1,true);
	
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("gps_rtk_msgs/pose", 1,true);
    ros::Publisher fix_pub = nh.advertise<sensor_msgs::NavSatFix>("gps_rtk_msgs/fix", 1,true);
	ros::Publisher fix_raw_pub = nh.advertise<sensor_msgs::NavSatFix>("gps_rtk_msgs/fix_raw", 1,true);
	ros::Publisher velocity_raw_gnss_pub = nh.advertise<geometry_msgs::Vector3Stamped>("gps_rtk_msgs/velocity_raw_gnss", 1,true);

	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("gps_rtk_msgs/imu", 1,true);
    ros::Publisher imu_raw_pub = nh.advertise<sensor_msgs::Imu>("gps_rtk_msgs/imu_raw", 1,true);
	
	ros::Publisher euler_pub = nh.advertise<geometry_msgs::Vector3Stamped>("gps_rtk_msgs/euler", 1,true);

	ros::Publisher odom_vehicle_pub = nh.advertise<nav_msgs::Odometry>("gps_rtk_msgs/odom_vehicle", 1,true);
	ros::Publisher pose_vehicle_pub = nh.advertise<geometry_msgs::PoseStamped>("gps_rtk_msgs/pose_vehicle", 1,true);
	ros::Publisher imu_vehicle_raw_pub = nh.advertise<sensor_msgs::Imu>("gps_rtk_msgs/imu_vehicle_raw", 1,true);
	ros::Publisher euler_vehicle_pub = nh.advertise<geometry_msgs::Vector3Stamped>("gps_rtk_msgs/euler_vehicle", 1,true);

	gps_rtk_msgs::GpsWorker gps_rtk_worker(gps_rtk_serialport,gps_rtk_baudrate,gps_rtk_record_or_read, "", gps_rtk_write_csv_path_name);

	gps_rtk_worker.open_port();
	
	gps_rtk_worker.write_csv_title();

	cout.precision(18);
	while (ros::ok())
	{
		
		gps_rtk_worker.gps_go(odom_pub,std_pub,pose_pub,fix_pub,fix_raw_pub,velocity_raw_gnss_pub,imu_pub,imu_raw_pub,satellites_pub, euler_pub,odom_vehicle_pub,pose_vehicle_pub,euler_vehicle_pub,imu_vehicle_raw_pub);
		
	}

	return 0;
}



