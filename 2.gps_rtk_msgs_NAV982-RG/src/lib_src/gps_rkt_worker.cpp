#include "gps_rtk_worker.h"

namespace gps_rtk_msgs{

GpsWorker::GpsWorker(string gps_serialport,int gps_baudrate,string record_or_read,string if_read_then_csv_name,string write_csv_path_name)
{
    this->gps_serialport = gps_serialport;
    this->gps_baudrate = gps_baudrate;
    this->record_or_read = record_or_read;
    this->if_read_then_csv_name = if_read_then_csv_name;
    this->write_csv_path_name = write_csv_path_name;

}

GpsWorker::~GpsWorker()
{
    if(record_or_read == "record"){
		myfile.close();
	}

	CloseComport();
}

std::vector<std::string> GpsWorker::spilt_string(const std::string &str, const std::string &delim)
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

void GpsWorker::open_port(){
    /* open the com port */
	if (OpenComport((char *)gps_serialport.c_str(), gps_baudrate))
	{
		printf("Could not open serial port\n");
		exit(EXIT_FAILURE);
	}
	an_decoder_initialise(&an_decoder);    
}

void GpsWorker::write_csv_title(){
	myfile.precision(18);
	if(record_or_read == "record"){

		string file_time;
		[&]() { 
			 time_t timep;
		     time (&timep);
		     char tmp[64];
		     strftime(tmp, sizeof(tmp), "%Y-%m-%d-%H-%M-%S",localtime(&timep) );
		     file_time = tmp;
			 }();

		string wholedatasets = write_csv_path_name + "gps_rtk_" + file_time;
		cout << wholedatasets << endl;

		myfile.open((wholedatasets + ".csv"), std::ios::out | std::ios::app);
		myfile << "#Timestamp(nsec),";
		myfile << "UTM_North_X(m),UTM_East_Y(m),UTM_Height_Z(m),";
		myfile << "Orientation_qw,Orientation_qx,Orientation_qy,Orientation_qz,";
		myfile << "Velocity_X(m/s),Velocity_Y(m/s),Velocity_Z(m/s),";
		myfile << "Acceleration_X(m/s^2),Acceleration_Y(m/s^2),Acceleration_Z(m/s^2),";
		myfile << "W_X(rad/s),W_Y(rad/s),W_Z(rad/s),";

		myfile << "Acceleration_X_Raw(m/s^2),Acceleration_Y_Raw(m/s^2),Acceleration_Z_Raw(m/s^2),";
		myfile << "W_X_Raw(rad/s),W_Y_Raw(rad/s),W_Z_Raw(rad/s),";

		myfile << "Latitude(rad),Longitude(rad),Height(m),";
		myfile << "Latitude_std(m),Longitude_std(m),Height_std(m),GPS_satellites,";
		myfile << "Latitude_raw(rad),Longitude_raw(rad),Height_raw(m),";
		myfile << "Latitude_raw_std(m),Longitude_raw_std(m),Height_raw_std(m),";
		myfile << "Roll_X(deg),Pitch_Y(deg),Heading_Z(deg),";

  		myfile << "UTM_North_X_vehicle(m),UTM_East_Y_vehicle(m),UTM_Height_Z_vehicle(m),";
		myfile << "Orientation_qw_vehicle,Orientation_qx_vehicle,Orientation_qy_vehicle,Orientation_qz_vehicle,";
		myfile << "Velocity_X_vehicle(m/s),Velocity_Y_vehicle(m/s),Velocity_Z_vehicle(m/s),";
		myfile << "Acceleration_X_vehicle(m/s^2),Acceleration_Y_vehicle(m/s^2),Acceleration_Z_vehicle(m/s^2),";
		myfile << "W_X_vehicle(rad/s),W_Y_vehicle(rad/s),W_Z_vehicle(rad/s),";

		myfile << "Acceleration_X_vehicle_Raw(m/s^2),Acceleration_Y_vehicle_Raw(m/s^2),Acceleration_Z_vehicle_Raw(m/s^2),";
		myfile << "W_X_vehicle_Raw(rad/s),W_vehicle_Y_Raw(rad/s),W_vehicle_Z_Raw(rad/s),";

		myfile << "Roll_X_vehicle(deg),Pitch_Y_vehicle(deg),Heading_Z_vehicle(deg),";

		myfile <<std::endl;
	}
}

void GpsWorker::gps_go(ros::Publisher &odom_pub,ros::Publisher &standard_pub,ros::Publisher &pose_pub,ros::Publisher &fix_pub,ros::Publisher &fix_raw_pub, ros::Publisher &velocity_raw_pub,ros::Publisher &imu_pub,ros::Publisher &imu_raw_pub, ros::Publisher &satellites_pub, ros::Publisher &euler_pub, ros::Publisher &odom_vehicle_pub, ros::Publisher &pose_vehicle_pub, ros::Publisher &euler_vehicle_pub, ros::Publisher &imu_vehicle_raw_pub){
    cout.precision(18);
    if((bytes_received = PollComport(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) <= 0){
			return;
		}
		/* increment the decode buffer length by the number of bytes received */
		an_decoder_increment(&an_decoder, bytes_received);

		/* decode all the packets in the buffer */
		while(1){
			current_time = ros::Time::now();
			if((an_packet = an_packet_decode(&an_decoder)) == NULL){
				break;
			}
			if (an_packet->id == packet_id_system_state) /* system state packet ID20 */
			{
				/* copy all the binary data into the typedef struct for the packet */
				/* this allows easy access to all the different values             */
				if(decode_system_state_packet(&system_state_packet, an_packet) == 0)
				{	
					++sequenceNum;
                	cout << "~~~gps_rtk_msgs~~~" << current_time << "~~~" << endl;
					latitude = (double)system_state_packet.latitude;			//rad
					longitude = (double)system_state_packet.longitude;			//rad
					height = (double)system_state_packet.height;

					latitude_std = (double)system_state_packet.standard_deviation[0];
					longitude_std = (double)system_state_packet.standard_deviation[1];
					height_std = (double)system_state_packet.standard_deviation[2];

					v_north = (double)system_state_packet.velocity[0];
					v_east = (double)system_state_packet.velocity[1];
					v_down = (double)system_state_packet.velocity[2];
					a_x = (double)system_state_packet.body_acceleration[0];
					a_y = (double)system_state_packet.body_acceleration[1];
					a_z = (double)system_state_packet.body_acceleration[2];

					roll_x = ((double)system_state_packet.orientation[0]); 		// rad
					pitch_y = ((double)system_state_packet.orientation[1]);
					heading_z = ((double)system_state_packet.orientation[2]);
					roll_x_vehicle = roll_x;		// rad
					pitch_y_vehicle = -pitch_y;
					heading_z_vehicle = 2 * pi - heading_z;

  					q_vehicle=tf::createQuaternionMsgFromRollPitchYaw(roll_x_vehicle, pitch_y_vehicle, heading_z_vehicle);

					w_x = (double)system_state_packet.angular_velocity[0];		// rad/s
					w_y = (double)system_state_packet.angular_velocity[1];		// rad/s
					w_z = (double)system_state_packet.angular_velocity[2];		// rad/s
					
					fix.header.seq = sequenceNum;
					fix.header.stamp = current_time;
					fix.header.frame_id = ("gps_rtk_msgs");
					fix.latitude = latitude;
					fix.longitude = longitude;
					fix.altitude = height;
					fix.position_covariance[0] = latitude_std * latitude_std;
					fix.position_covariance[1] = 0;
					fix.position_covariance[2] = 0;
					fix.position_covariance[3] = 0;
					fix.position_covariance[4] = longitude_std * longitude_std;
					fix.position_covariance[5] = 0;
					fix.position_covariance[6] = 0;
					fix.position_covariance[7] = 0;
					fix.position_covariance[8] = height_std * height_std;
					fix_pub.publish(fix);

					odom.header.seq = sequenceNum;
					odom.header.stamp = current_time;
					odom.header.frame_id = ("gps_rtk_msgs");
					odom.child_frame_id = ("gps_rtk_msgs");
					odom.pose.pose.position.x = utm_north_x;
					odom.pose.pose.position.y = utm_east_y;
					odom.pose.pose.position.z = utm_height_z;
					odom.pose.pose.orientation.x = qx_1;
					odom.pose.pose.orientation.y = qy_2;
					odom.pose.pose.orientation.z = qz_3;
					odom.pose.pose.orientation.w = qw_0;
					odom.twist.twist.linear.x = v_x;
					odom.twist.twist.linear.y = v_y;
					odom.twist.twist.linear.z = v_z;
					odom.twist.twist.angular.x = w_x;
					odom.twist.twist.angular.y = w_y;
					odom.twist.twist.angular.z = w_z;
					odom_pub.publish(odom);
					
					odom_vehicle.header.seq = sequenceNum;
					odom_vehicle.header.stamp = current_time;
					odom_vehicle.header.frame_id = ("gps_rtk_vehicle_msgs");
					odom_vehicle.child_frame_id = ("gps_rtk_vehicle_msgs");
					odom_vehicle.pose.pose.position.x = utm_north_x_vehicle;
					odom_vehicle.pose.pose.position.y = utm_east_y_vehicle;
					odom_vehicle.pose.pose.position.z = utm_height_z_vehicle;
					odom_vehicle.pose.pose.orientation = q_vehicle;
					odom_vehicle.twist.twist.linear.x = v_x_vehicle;
					odom_vehicle.twist.twist.linear.y = v_y_vehicle;
					odom_vehicle.twist.twist.linear.z = v_z_vehicle;
					odom_vehicle.twist.twist.angular.x = w_x;
					odom_vehicle.twist.twist.angular.y = -w_y;
					odom_vehicle.twist.twist.angular.z = -w_z;
					odom_vehicle_pub.publish(odom_vehicle);

					standard.header.seq = sequenceNum;
					standard.header.stamp = current_time;
					standard.header.frame_id = ("gps_rtk_msgs");
					standard.vector.x = latitude_std;
					standard.vector.y = longitude_std;
					standard.vector.z = height_std;
					standard_pub.publish(standard);

					satellites_status.header.seq = sequenceNum;
					satellites_status.header.stamp = current_time;
					satellites_status.header.frame_id = ("gps_rtk_msgs");
					satellites_status.vector.x = gps_satellites;
					satellites_pub.publish(satellites_status);

					euler.header.seq = sequenceNum;
					euler.header.stamp = current_time;
					euler.header.frame_id = ("gps_rtk_msgs");
					euler.vector.x = roll_x;
					euler.vector.y = pitch_y;
					euler.vector.z = heading_z;
					euler_pub.publish(euler);

					euler_vehicle.header.seq = sequenceNum;
					euler_vehicle.header.stamp = current_time;
					euler_vehicle.header.frame_id = ("gps_rtk_vehicle_msgs");
					euler_vehicle.vector.x = roll_x_vehicle;
					euler_vehicle.vector.y = pitch_y_vehicle;
					euler_vehicle.vector.z = heading_z_vehicle;
					euler_vehicle_pub.publish(euler_vehicle);
					
					this_pose_stamped.header.seq = sequenceNum;
					this_pose_stamped.header.stamp = current_time;
					this_pose_stamped.header.frame_id = ("gps_rtk_msgs");
					this_pose_stamped.pose.position.x = utm_north_x;
					this_pose_stamped.pose.position.y = utm_east_y;
					this_pose_stamped.pose.position.z = utm_height_z;
					this_pose_stamped.pose.orientation.x = qx_1;
					this_pose_stamped.pose.orientation.y = qy_2;
					this_pose_stamped.pose.orientation.z = qz_3;
					this_pose_stamped.pose.orientation.w = qw_0;
					pose_pub.publish(this_pose_stamped);

					this_pose_stamped_vehicle.header.seq = sequenceNum;
					this_pose_stamped_vehicle.header.stamp = current_time;
					this_pose_stamped_vehicle.header.frame_id = ("gps_rtk_vehicle_msgs");
					this_pose_stamped_vehicle.pose.position.x = utm_north_x_vehicle;
					this_pose_stamped_vehicle.pose.position.y = utm_east_y_vehicle;
					this_pose_stamped_vehicle.pose.position.z = utm_height_z_vehicle;
					this_pose_stamped_vehicle.pose.orientation = q_vehicle;
					pose_vehicle_pub.publish(this_pose_stamped_vehicle);

					imu.header.seq = sequenceNum;
					imu.header.stamp = current_time;
					imu.header.frame_id = ("gps_rtk_msgs");
					imu.orientation.x = qx_1;
					imu.orientation.y = qy_2;
					imu.orientation.z = qz_3;
					imu.orientation.w = qw_0;
					imu.angular_velocity.x = w_x;
					imu.angular_velocity.y = w_y;
					imu.angular_velocity.z = w_z;
					imu.linear_acceleration.x = a_x;
					imu.linear_acceleration.y = a_y;
					imu.linear_acceleration.z = a_z;
					imu_pub.publish(imu);

					imu_raw.header.seq = sequenceNum;
					imu_raw.header.stamp = current_time;
					imu_raw.header.frame_id = ("gps_rtk_msgs");
					imu_raw.orientation.x = qx_1;
					imu_raw.orientation.y = qy_2;
					imu_raw.orientation.z = qz_3;
					imu_raw.orientation.w = qw_0;
					imu_raw.angular_velocity.x = w_x_raw;
					imu_raw.angular_velocity.y = w_y_raw;
					imu_raw.angular_velocity.z = w_z_raw;
					imu_raw.linear_acceleration.x = a_x_raw;
					imu_raw.linear_acceleration.y = a_y_raw;
					imu_raw.linear_acceleration.z = a_z_raw;
					imu_raw_pub.publish(imu_raw);

					imu_vehicle_raw.header.seq = sequenceNum;
					imu_vehicle_raw.header.stamp = current_time;
					imu_vehicle_raw.header.frame_id = ("gps_rtk_vehicle_msgs");
					imu_vehicle_raw.orientation = q_vehicle;
					imu_vehicle_raw.angular_velocity.x = w_x_vehicle_raw;
					imu_vehicle_raw.angular_velocity.y = w_y_vehicle_raw;
					imu_vehicle_raw.angular_velocity.z = w_z_vehicle_raw;
					imu_vehicle_raw.linear_acceleration.x = a_x_vehicle_raw;
					imu_vehicle_raw.linear_acceleration.y = a_y_vehicle_raw;
					imu_vehicle_raw.linear_acceleration.z = a_z_vehicle_raw;
					imu_vehicle_raw_pub.publish(imu_vehicle_raw);

					if(record_or_read == "record"){
						if(sequenceNum >= 10){
							myfile <<  current_time.toSec() * 1e9 << delim;
							myfile << utm_north_x << delim << utm_east_y << delim << utm_height_z << delim;
							myfile << qw_0 << delim << qx_1 << delim << qy_2 << delim << qz_3 << delim;
							myfile << v_x << delim << v_y << delim << v_z << delim;
							myfile << a_x << delim << a_y << delim << a_z << delim;
							myfile << w_x << delim << w_y << delim << w_z << delim;
							
							myfile << a_x_raw << delim << a_y_raw << delim << a_z_raw << delim;
							myfile << w_x_raw << delim << w_y_raw << delim << w_z_raw << delim;

							myfile << latitude << delim << longitude << delim << height << delim;
							myfile << latitude_std << delim << longitude_std << delim << height_std << delim  << gps_satellites << delim;
							myfile << latitude_raw << delim << longitude_raw << delim << height_raw << delim;
							myfile << latitude_raw_std << delim << longitude_raw_std << delim << height_raw_std << delim;
							myfile << roll_x * 180 / pi << delim << pitch_y * 180 / pi << delim << heading_z * 180 / pi << delim;
							
							myfile << utm_north_x_vehicle << delim << utm_east_y_vehicle << delim << utm_height_z_vehicle << delim;
                            myfile << q_vehicle.w << delim << q_vehicle.x << delim << q_vehicle.y << delim << q_vehicle.z << delim;
                            myfile << v_x_vehicle << delim << v_y_vehicle << delim << v_z_vehicle << delim;
							myfile << a_x << delim << -a_y << delim << -a_z << delim;
							myfile << w_x << delim << -w_y << delim << -w_z << delim;
							myfile << a_x_vehicle_raw << delim << a_y_vehicle_raw << delim << a_z_vehicle_raw << delim;
							myfile << w_x_vehicle_raw << delim << w_y_vehicle_raw << delim << w_z_vehicle_raw << delim;
						    myfile << roll_x_vehicle * 180 / pi << delim << pitch_y_vehicle * 180 / pi << delim << heading_z_vehicle * 180 / pi << delim;	

							myfile <<std::endl;	
						}
						
					}

					printf("\tRoll_x = %lf deg, Pitch_y = %lf deg, Heading_z = %lf deg\n", roll_x * 180 / pi, pitch_y * 180 / pi, heading_z * 180 / pi);
					printf("\tlatitude_std = %lf m, longitude_std = %lf m, height_std = %lf m\n", latitude_std, longitude_std, height_std);
				}
			}
			else if (an_packet->id == packet_id_satellites) /* satellites_packet ID30 */
			{
				/* copy all the binary data into the typedef struct for the packet */
				/* this allows easy access to all the different values             */
				if(decode_satellites_packet(&satellites_packet, an_packet) == 0)
				{
					current_time = ros::Time::now();
					hdop = (double)satellites_packet.hdop;
					vdop = (double)satellites_packet.vdop;
					gps_satellites = (double)satellites_packet.gps_satellites;
					glonass_satellites = (double)satellites_packet.glonass_satellites;
					beidou_satellites = (double)satellites_packet.beidou_satellites;
					galileo_satellites = (double)satellites_packet.galileo_satellites;
					sbas_satellites = (double)satellites_packet.sbas_satellites;
				}
			}
			else if (an_packet->id == packet_id_utm_position) /* rutm_position_packet ID34 */
			{
				/* copy all the binary data into the typedef struct for the packet */
				/* this allows easy access to all the different values    */ 
				if(decode_utm_position_packet(&utm_position_packet, an_packet) == 0)
				{
					utm_north_x = (double)utm_position_packet.position[0];
					utm_east_y = (double)utm_position_packet.position[1];
					utm_height_z = (double)utm_position_packet.position[2];
					utm_zone = (double)utm_position_packet.zone;
					utm_north_x_vehicle = utm_north_x;				//车辆坐标系　x朝前，y朝，z朝上
					utm_east_y_vehicle  = -utm_east_y;
					utm_height_z_vehicle = utm_height_z;
				}
			}
			else if (an_packet->id == packet_id_body_velocity) /* body_velocity_packet ID36 */
			{
				/* copy all the binary data into the typedef struct for the packet */
				/* this allows easy access to all the different values             */
				if(decode_body_velocity_packet(&body_velocity_packet, an_packet) == 0)
				{
					v_x = (double)body_velocity_packet.velocity[0];
					v_y = (double)body_velocity_packet.velocity[1];
					v_z = (double)body_velocity_packet.velocity[2];
					v_x_vehicle = v_x;
					v_y_vehicle = -v_y;
					v_z_vehicle = -v_z;
				}
			}
			else if (an_packet->id == packet_id_quaternion_orientation) /* quaternion_orientation_packet ID40 */
			{
				/* copy all the binary data into the typedef struct for the packet */
				/* this allows easy access to all the different values             */
				if(decode_quaternion_orientation_packet(&quaternion_orientation_packet, an_packet) == 0)
				{
					qw_0 = (double)quaternion_orientation_packet.orientation[0];
					qx_1 = (double)quaternion_orientation_packet.orientation[1];
					qy_2 = (double)quaternion_orientation_packet.orientation[2];
					qz_3 = (double)quaternion_orientation_packet.orientation[3];
				}
			}
			
			else if (an_packet->id == packet_id_raw_sensors) /* Raw Sensors Packet ID28 */
			{
				/* copy all the binary data into the typedef struct for the packet */
				/* this allows easy access to all the different values             */
				if(decode_raw_sensors_packet(&raw_sensors_packet, an_packet) == 0)
				{
					a_x_raw = (double)raw_sensors_packet.accelerometers[0];
					a_y_raw = (double)raw_sensors_packet.accelerometers[1];
					a_z_raw = (double)raw_sensors_packet.accelerometers[2];
					w_x_raw = (double)raw_sensors_packet.gyroscopes[0];
					w_y_raw = (double)raw_sensors_packet.gyroscopes[1];
					w_z_raw = (double)raw_sensors_packet.gyroscopes[2];

					a_x_vehicle_raw = a_x_raw;		
					a_y_vehicle_raw = -a_y_raw;
					a_z_vehicle_raw = -a_z_raw;
					w_x_vehicle_raw = w_x_raw;		// rad/s
					w_y_vehicle_raw = -w_y_raw;
					w_z_vehicle_raw = -w_z_raw;
				}
			}
			else if (an_packet->id == packet_id_raw_gnss) /* Raw GNSS Packet ID29 */
			{
				/* copy all the binary data into the typedef struct for the packet */
				/* this allows easy access to all the different values             */
				if(decode_raw_gnss_packet(&raw_gnss_packet, an_packet) == 0)
				{

					latitude_raw = (double)raw_gnss_packet.position[0];
					longitude_raw = (double)raw_gnss_packet.position[1];
					height_raw = (double)raw_gnss_packet.position[2];
					latitude_raw_std = (double)raw_gnss_packet.position_standard_deviation[0];
					longitude_raw_std = (double)raw_gnss_packet.position_standard_deviation[1];
					height_raw_std = (double)raw_gnss_packet.position_standard_deviation[2];

					fix_raw.header.seq = sequenceNum;
					fix_raw.header.stamp = current_time;
					fix_raw.header.frame_id = ("gps_rtk_msgs");
					fix_raw.latitude = latitude_raw;
					fix_raw.longitude = longitude_raw;
					fix_raw.altitude = height_raw;
					fix_raw.position_covariance[0] = latitude_raw_std * latitude_raw_std;
					fix_raw.position_covariance[1] = 0;
					fix_raw.position_covariance[2] = 0;
					fix_raw.position_covariance[3] = 0;
					fix_raw.position_covariance[4] = longitude_raw_std * longitude_raw_std;
					fix_raw.position_covariance[5] = 0;
					fix_raw.position_covariance[6] = 0;
					fix_raw.position_covariance[7] = 0;
					fix_raw.position_covariance[8] = height_raw_std * height_raw_std;
					fix_raw_pub.publish(fix_raw);

					velocity_raw.header.seq = sequenceNum;
					velocity_raw.header.stamp = current_time;
					velocity_raw.header.frame_id = ("gps_rtk_msgs");
					velocity_raw.vector.x = (double)raw_gnss_packet.velocity[0];
					velocity_raw.vector.y = (double)raw_gnss_packet.velocity[1];
					velocity_raw.vector.z = (double)raw_gnss_packet.velocity[2];
					velocity_raw_pub.publish(velocity_raw);
				}
			}
			else
			{
				// printf("Packet ID %u of Length %u\n", an_packet->id, an_packet->length);
			}
			/* Ensure that you free the an_packet when your done with it or you will leak memory */
			an_packet_free(&an_packet);
		}

}

}