#include <string>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <sstream>
#include <fstream>
#include <sys/types.h>  
#include <sys/stat.h> 
#include <cmath>
#include <chrono>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/TwistStamped.h>

#include "lib/wiican/wiican_driver.h"
#include "lib/dbc/a27/a27.h"


#include "vehicle_msgs_canpal/VehicleStamped.h"
#include <ackermann_msgs/AckermannDriveStamped.h>

using namespace std;

#define M_PI 3.14159265358979323846
#define WHEEL_RADIUS 0.25		       // a27轮胎半径 m
#define WHEEL_BASE   1.9               //轴距 m
#define Tread_BASE   0.98	           //轮距	m  1.1m  主销距离0.98m
#define Motor_Ratio 1                  //驱动电机转速与轮胎转速传动比
#define Steer_Ratio_f (360/21)         //前轮转向电机与前轮转向角传动比  1/Steer_Ratio_f           		          

//AckermannDriveStamped
// m/s
// rad
// rad/s

// 车速与轮胎转向角
// km/h		前进+　后退- 
double vehicle_speed_vcu = 0;
double vehicle_speed = 0;
double vehicle_last_speed = 0;
double vehicle_acceleration = 0;
double vehicle_speed_l = 0;
double vehicle_speed_r = 0;
// deg		左转+　右转-
double vehicle_steerAngle_f = 0;
// deg/s
double vehicle_steerAngle_sp_f = 0;

// 驱动电机转速、转向电机转向角、转向电机转向角速度
// Rpm
double speed_measured_mcul = 0;
// Rpm
double speed_measured_mcur = 0;
// deg
double sasf_steering_wheel_angle = 0;
// deg/s
double sasf_steering_wheel_angle_speed = 0;

//　tbox数据
//tbox
double tbox_shift_level_request = 0;
double tbox_target_speed  = 0;
double tbox_st_wheel_angle_req = 0;

a27_vcu_status_1_t vcu_status_1_t;
a27_mcul_torque_feedback_t  mcul_torque_feedback_t;
a27_mcur_torque_feedback_t  mcur_torque_feedback_t;
a27_sas_status_t sasf_status_t;
a27_tbox_vehicle_request_status_t tbox_vehicle_request_status_t;

std::vector<std::string> spilt_string(const std::string &str, const std::string &delim)
{
    std::vector<std::string> spiltCollection;
    if(str.size()==0)
        return spiltCollection;
    int start = 0;
    int idx = str.find(delim, start);
    while( idx != std::string::npos )
    {
        spiltCollection.push_back(str.substr(start, idx-start));
        start = idx+delim.size();
        idx = str.find(delim, start);
    }
    spiltCollection.push_back(str.substr(start));
    return spiltCollection;
}

int a27_unpack(unsigned int can_id, const uint8_t *src_p, size_t size){
    cout.precision(18);
	
    switch(can_id){
		case A27_VCU_STATUS_1_FRAME_ID:
			a27_vcu_status_1_unpack(&vcu_status_1_t,src_p,size);
			vehicle_speed_vcu = a27_vcu_status_1_vcu_vehicle_speed_decode(vcu_status_1_t.vcu_vehicle_speed);
			break;
        case A27_MCUL_TORQUE_FEEDBACK_FRAME_ID:
            a27_mcul_torque_feedback_unpack(&mcul_torque_feedback_t,src_p,size);
			speed_measured_mcul = a27_mcul_torque_feedback_speed_measured_decode(mcul_torque_feedback_t.speed_measured);
			break;
		case A27_MCUR_TORQUE_FEEDBACK_FRAME_ID:
			a27_mcur_torque_feedback_unpack(&mcur_torque_feedback_t,src_p,size);
			speed_measured_mcur = a27_mcur_torque_feedback_speed_measured_decode(mcur_torque_feedback_t.speed_measured);
			break;
		case A27_SAS_STATUS_FRAME_ID:
			a27_sas_status_unpack(&sasf_status_t,src_p,size);
			sasf_steering_wheel_angle = a27_sas_status_sas_steering_wheel_angle_decode(sasf_status_t.sas_steering_wheel_angle);
			sasf_steering_wheel_angle_speed = a27_sas_status_sas_steering_wheel_angle_speed_decode(sasf_status_t.sas_steering_wheel_angle_speed);
			break;
        case A27_TBOX_VEHICLE_REQUEST_STATUS_FRAME_ID:
            a27_tbox_vehicle_request_status_unpack(&tbox_vehicle_request_status_t,src_p,size);
            tbox_shift_level_request = a27_tbox_vehicle_request_status_tbox_shift_level_request_decode(tbox_vehicle_request_status_t.tbox_shift_level_request);
            tbox_target_speed = a27_tbox_vehicle_request_status_tbox_target_speed_decode(tbox_vehicle_request_status_t.tbox_target_speed);
            tbox_st_wheel_angle_req = a27_tbox_vehicle_request_status_tbox_steer_wheel_angle_request_decode(tbox_vehicle_request_status_t.tbox_steer_wheel_angle_request);
            // tbox_st_wheel_angle_req_rear = a27_tbox_vehicle_request_status_tbox_st_wheel_angle_req_rear_decode(tbox_vehicle_request_status_t.tbox_st_wheel_angle_req_rear);
            break;	
    }

	vehicle_speed_l = -((speed_measured_mcul+1) * 2 * M_PI * WHEEL_RADIUS / 60 / Motor_Ratio * 3.6);
	vehicle_speed_r = -(speed_measured_mcur * 2 * M_PI * WHEEL_RADIUS / 60 / Motor_Ratio * 3.6);
    vehicle_speed = (vehicle_speed_l + vehicle_speed_r)/2;

    vehicle_steerAngle_f = (-sasf_steering_wheel_angle) / Steer_Ratio_f;
	vehicle_steerAngle_sp_f = sasf_steering_wheel_angle_speed / Steer_Ratio_f;

	return 0;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "vehicle_msgs_canpal");
    ros::NodeHandle nh;

	string vehicle_record_or_read;
	string vehilce_write_csv_path_name;

 	nh.getParam("vehicle_record_or_read",vehicle_record_or_read);
	nh.getParam("vehilce_write_csv_path_name",vehilce_write_csv_path_name);

    ros::Time last_time, current_time;
	current_time = ros::Time::now();
    double begin_time = current_time.toSec();

    ros::Publisher vehicle_pub = nh.advertise<vehicle_msgs_canpal::VehicleStamped>("vehicle_msgs_canpal/VehicleStamped", 1,true);
	vehicle_msgs_canpal::VehicleStamped vehicleStamped;

    ros::Publisher AckermannDriveStamped_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("vehicle_msgs_canpal/AckermannDriveStamped", 1,true);
    ackermann_msgs::AckermannDriveStamped AckermannDriveStamped;

    unsigned long long int sequence = 0;

    //1.open canpal.
	CAN_HANDLE wc = wiican_open();
	tCOMM_PARAMETER  para;
	para.comm_type = tCOMM_TYPE::COMM_TYPE_USBCOM;
	para.local_port = 1;
	para.run_mode = COMM_PARA_RUN_MODE_ANALYZER;

	bool bRet = false;
	bRet = wiican_connect(wc, &para);

	//2.wait for connection.
	unsigned int nErrNo = ERROR_NONE;
	bool isConn = false;
	while (!wiican_connected(wc) && wiican_error(wc) == ERROR_NONE)
	{
		usleep(100000);
	}
	isConn = wiican_connected(wc);
	nErrNo = wiican_error(wc);
	if (!isConn)
	{
		std::cout << "failed to connect Wiican device!\n" << std::endl;
		return EXIT_FAILURE;
	}
	//3.before tx/rx message, working CAN channel(s) MUST be set enable.
	tCAN_CHANNEL_CONTROL channel_ctrl;
	channel_ctrl.CAN1_channel = CHANNEL_ENABLE;
	channel_ctrl.CAN2_channel = CHANNEL_DISABLE;
	int data_size = sizeof(tCAN_CHANNEL_CONTROL);
	if (wiican_ioctl(wc, tIOCTL_ACT::IOCTL_SET_CHANNEL_A, &channel_ctrl, data_size) != IOCTL_ERR_NONE)
	{
		std::cout << "failed to enable CAN channels!\n" << std::endl;
		return -1;
	}

	//4. send some CAN mesaages to CAN1 channel in every a few seconds and receive them back from CAN2 channel.
	tDEVICE_STATUS dev_status;
	tCAN_MESSAGE rx_buf_value[100] = {0};
	std::cout  << "Prepare to reading data!!!" << std::endl;

    std::fstream myfile;
	std::string delim = ",";
    myfile.precision(18);
	if(vehicle_record_or_read == "record"){

		string file_time;
		[&]() { 
			 time_t timep;
		     time (&timep);
		     char tmp[64];
		     strftime(tmp, sizeof(tmp), "%Y-%m-%d-%H-%M-%S",localtime(&timep) );
		     file_time = tmp;
			 }();

		string wholedatasets = vehilce_write_csv_path_name + "vehicle_" + file_time;
		cout << wholedatasets << endl;


		myfile.open((wholedatasets + ".csv"), std::ios::out | std::ios::app);
		myfile << "#Timestamp(nsec),";
		myfile << "speed(m/s),steering_angle(rad),steering_angle_velocity(rad/s),";
		myfile <<std::endl;
	}

	unsigned long long int  sequenceNum = 0;
	unsigned int receivedNum;
	unsigned int index = 0;
    while(ros::ok()) 
    {  	
		if(!wiican_connected(wc)){
			continue;
		}
		
		if (wiican_ioctl(wc, tIOCTL_ACT::IOCTL_GET_DEVICE_STATUS, &dev_status, sizeof(tDEVICE_STATUS)) != IOCTL_ERR_NONE)
		{
			std::cout << "Failed to check device status, maybe it is not ready yet!\n" << std::endl;
			continue;
		}

		if (!dev_status.CAN1_enabled){
			continue;
		}
		
        receivedNum = wiican_receive(wc, rx_buf_value, 100, 0, 0xFF);
		if (receivedNum <= 0){
            continue;
        }

		for (index = 0; index < receivedNum; index++)
		{
			current_time = ros::Time::now();
			
			if(rx_buf_value[index].can_id == A27_VCU_STATUS_1_FRAME_ID || rx_buf_value[index].can_id == A27_MCUL_TORQUE_FEEDBACK_FRAME_ID
            || rx_buf_value[index].can_id == A27_MCUR_TORQUE_FEEDBACK_FRAME_ID || rx_buf_value[index].can_id == A27_SAS_STATUS_FRAME_ID
            || rx_buf_value[index].can_id == A27_TBOX_VEHICLE_REQUEST_STATUS_FRAME_ID){
				if (!rx_buf_value[index].rtr){
					++sequenceNum;
                    if(sequenceNum % 200 == 0){
                        cout << "%%%vehicle_msgs_canpal%%%" << current_time << "%%%" << endl;
                    }
					a27_unpack(rx_buf_value[index].can_id,rx_buf_value[index].data,rx_buf_value[index].len);

                    // 过滤噪声 noise
                    if(abs(vehicle_speed) >= 30 || abs(vehicle_steerAngle_f) >= 30){
                        continue;
                    }
                    if(abs(vehicle_steerAngle_f) <= 0.3){
                        vehicle_steerAngle_f = 0;
                    }

                    AckermannDriveStamped.header.stamp = current_time;
                    AckermannDriveStamped.header.seq = sequenceNum;
                    AckermannDriveStamped.header.frame_id = "AckermannDriveStamped";
                    AckermannDriveStamped.drive.steering_angle = float(vehicle_steerAngle_f * M_PI / 180);  // rad/s
                    AckermannDriveStamped.drive.steering_angle_velocity = float(vehicle_steerAngle_sp_f * M_PI / 180);  // rad/s
                    AckermannDriveStamped.drive.speed = float(vehicle_speed / 3.6);         //　m/s
                    if(sequenceNum <= 1){
                        AckermannDriveStamped.drive.acceleration = 0;
                    }else{
                        vehicle_acceleration = (vehicle_speed - vehicle_last_speed) / 3.6 /(current_time.toSec() - last_time.toSec());
                        AckermannDriveStamped.drive.acceleration = float(vehicle_acceleration);
                    }
                    vehicle_last_speed = vehicle_speed;
                    last_time = current_time;
                    AckermannDriveStamped.drive.jerk = 0;

					vehicleStamped.header.stamp = current_time;
                    vehicleStamped.header.seq = sequenceNum;
                    vehicleStamped.header.frame_id = "VehicleStamped";
					vehicleStamped.vehicle_speed_vcu = vehicle_speed_vcu;
                    vehicleStamped.vehicle_speed = vehicle_speed;
                    vehicleStamped.vehicle_acceleration = vehicle_acceleration;
                    vehicleStamped.vehicle_speed_f = vehicle_speed_l;
                    vehicleStamped.vehicle_speed_r = vehicle_speed_r;
                    vehicleStamped.vehicle_steerAngle_f = vehicle_steerAngle_f;
                    vehicleStamped.vehicle_steerAngle_sp_f = vehicle_steerAngle_sp_f;

                    vehicleStamped.speed_measured_mcuf = speed_measured_mcul;
                    vehicleStamped.speed_measured_mcur = speed_measured_mcur;
                    vehicleStamped.sasf_steering_wheel_angle = sasf_steering_wheel_angle;
                    vehicleStamped.sasf_steering_wheel_angle_speed = sasf_steering_wheel_angle_speed;

                    vehicleStamped.tbox_shift_level_request = tbox_shift_level_request;
                    vehicleStamped.tbox_target_speed = tbox_target_speed;
                    vehicleStamped.tbox_st_wheel_angle_req = tbox_st_wheel_angle_req;

                    if(rx_buf_value[index].can_id != A27_SAS_STATUS_FRAME_ID && 
                    rx_buf_value[index].can_id != A27_MCUR_TORQUE_FEEDBACK_FRAME_ID
                    ){
                        continue;
                    }

                    AckermannDriveStamped_pub.publish(AckermannDriveStamped);                    
                    vehicle_pub.publish(vehicleStamped);
                    
                    if(vehicle_record_or_read == "record"){
                        if(sequenceNum >= 10){
                            myfile <<  current_time.toSec() * 1e9 << delim;
                            myfile << AckermannDriveStamped.drive.speed << delim << AckermannDriveStamped.drive.steering_angle << delim << AckermannDriveStamped.drive.steering_angle_velocity << delim;
                            myfile <<std::endl;	
                        }
					}

                    std::cout.precision(18);
                    cout << " AckermannDriveStamped.drive.speed: " <<  AckermannDriveStamped.drive.speed * 3.6 << " km/h" <<endl;
                    cout << "AckermannDriveStamped.drive.steering_angle: " <<AckermannDriveStamped.drive.steering_angle * 180 / M_PI << " deg" << endl;
                    cout << endl;
				}
			}

		}
		
    }

	wiican_close(wc);

    return 0;
}