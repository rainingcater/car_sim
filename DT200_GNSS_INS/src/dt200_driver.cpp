/****************************************************************/
/*                                                              */
/*          SNAV_BIN Packet Protocol Library                    */
/*        ROS Driver, Packet to Published Message Example       */
/*          Copyright 2021, Beijing Nuogeng Technology Ltd      */
/*                                                              */
/****************************************************************/
/*
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

// enable this if use ROS
//#define HAVE_ROS 1

// enable this if use UDP
//#define HAVE_UDP 1

#ifdef HAVE_ROS
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#endif // HAVE_ROS

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string>
#include <thread>
#include <chrono>

#include "snav_bin_packet_protocol.h"
#include "snav_bin_records.h"


#define RADIANS_TO_DEGREES (180.0/M_PI)
#define DEGREES_TO_RADIANS (M_PI/180.0)
 
 // The start of GPS time in a time_t style. In this version it is a constant, but this constant assumes that
// the local machine uses 00:00:00 01/01/1970 as its Epoch time. If your machine is different then you need to
// convert 00:00:00 06/01/1980 in to the local machine's time_t time.
#define GPS_TIME_START_TIME_MSEC (315964800000)

#define MINUTES_IN_WEEK (10080)       //!< Number of minutes in a week.
#define G2MS 9.80144145



#ifdef HAVE_UDP
// Ethernet
#include <arpa/inet.h>

static bool openSocket(const std::string &interface, const std::string &ip_addr, uint16_t port, int *fd_ptr, sockaddr_in *sock_ptr)
{
	// Create UDP socket
	int fd;
	fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (fd != -1) {
		if (interface.length()) {
			if (!setsockopt(fd, SOL_SOCKET, SO_BINDTODEVICE, interface.c_str(), interface.length()) == 0) {
				close(fd);
				return false;
			}
		}
		memset(sock_ptr, 0, sizeof(sockaddr_in));
		sock_ptr->sin_family = AF_INET;
		sock_ptr->sin_port = htons(port);
		if (!inet_aton(ip_addr.c_str(), &sock_ptr->sin_addr)) {
			sock_ptr->sin_addr.s_addr = INADDR_ANY; // Invalid address, use ANY
		}
		if (bind(fd, (sockaddr*)sock_ptr, sizeof(sockaddr)) == 0) {
			*fd_ptr = fd;
			return true;
		}
	}
	return false;
}

static int readSocket(int fd, unsigned int timeout, void *data, size_t size, sockaddr *source_ptr = NULL)
{
	if (fd >= 0) {
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(fd, &fds);

		// Set up timeout
		struct timeval tv;
		tv.tv_sec = timeout / 1000;
		tv.tv_usec = (timeout * 1000) % 1000000;

		if (select(fd + 1, &fds, NULL, NULL, &tv) > 0) {
			socklen_t socklen = source_ptr ? sizeof(*source_ptr) : 0;
			socklen_t *socklen_ptr = source_ptr ? &socklen : NULL;
			return recvfrom(fd, data, size, 0, source_ptr, socklen_ptr);
		}

		// Timeout
		return 0;
	}
	return -1;
}

#else
#include "rs232/rs232.h"
#endif // HAVE_UDP


int64_t gps_time_to_utc(time_record_t* time_record)
{
	int64_t ms_gps_utc = 0;
	if (time_record->flags.val.time_valid && time_record->flags.val.offset_valid)
	{
		ms_gps_utc = int64_t(time_record->gps_week) * MINUTES_IN_WEEK * 60000;
		ms_gps_utc += time_record->gps_time_ms;
		ms_gps_utc += GPS_TIME_START_TIME_MSEC + int64_t(time_record->utc_offset) * 1000;
	}
	return ms_gps_utc;
}


int main(int argc, char *argv[])
{

#ifdef HAVE_ROS
	// Set up ROS node //
	ros::init(argc, argv, "dt200_device_node");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
#endif // HAVE_ROS

	printf("\nYour DT200 ROS driver is currently running\nPress Ctrl-C to interrupt\n");

	// Set up the Communication port
#ifdef HAVE_UDP
	std::string interface = "";
	std::string ip_addr = "192.168.0.200";
	int port = 3000;
	if (argc >= 3) {
		ip_addr = std::string(argv[1]);
		port = atoi(argv[2]);
	}
#else
	std::string com_port = "COM8";
	int baud_rate = 115200;

	if (argc >= 3) {
		com_port = std::string(argv[1]);
		baud_rate = atoi(argv[2]);
	}
#endif // HAVE_UDP


#ifdef HAVE_ROS
	else {
#ifdef HAVE_UDP
		pnh.getParam("interface", interface);
		pnh.getParam("ip_address", ip_addr);
		pnh.getParam("port", port);
#else
		pnh.param("port", com_port, std::string("/dev/ttyUSB0"));
		pnh.param("baud_rate", baud_rate, 115200);
#endif // HAVE_UDP
	}

	std::string imu_frame_id;
	std::string nav_sat_frame_id;
	std::string topic_prefix;
	pnh.param("imu_frame_id", imu_frame_id, std::string("imu"));
	pnh.param("nav_sat_frame_id", nav_sat_frame_id, std::string("gps"));
	pnh.param("topic_prefix", topic_prefix, std::string("dt200_device"));
#endif // HAVE_ROS


#ifdef HAVE_ROS
	// Initialise Publishers and Topics //
	ros::Publisher nav_sat_fix_pub=nh.advertise<sensor_msgs::NavSatFix>(topic_prefix + "/NavSatFix",10);
	ros::Publisher twist_pub=nh.advertise<geometry_msgs::Twist>(topic_prefix + "/Twist",10);
	ros::Publisher imu_pub=nh.advertise<sensor_msgs::Imu>(topic_prefix + "/Imu",10);
	ros::Publisher system_status_pub=nh.advertise<diagnostic_msgs::DiagnosticStatus>(topic_prefix + "/SystemStatus",10);
	ros::Publisher filter_status_pub=nh.advertise<diagnostic_msgs::DiagnosticStatus>(topic_prefix + "/FilterStatus",10);

	// Initialise messages
	sensor_msgs::NavSatFix nav_sat_fix_msg;
	nav_sat_fix_msg.header.stamp.sec=0;
	nav_sat_fix_msg.header.stamp.nsec=0;
	nav_sat_fix_msg.header.frame_id='0';
	nav_sat_fix_msg.status.status=0;
	nav_sat_fix_msg.status.service=1; // fixed to GPS
	nav_sat_fix_msg.latitude=0.0;
	nav_sat_fix_msg.longitude=0.0;
	nav_sat_fix_msg.altitude=0.0;
	nav_sat_fix_msg.position_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	nav_sat_fix_msg.position_covariance_type=2; // fixed to variance on the diagonal

	geometry_msgs::Twist twist_msg;
	twist_msg.linear.x=0.0;
	twist_msg.linear.y=0.0;
	twist_msg.linear.z=0.0;
	twist_msg.angular.x=0.0;
	twist_msg.angular.y=0.0;
	twist_msg.angular.z=0.0;

	sensor_msgs::Imu imu_msg;
	imu_msg.header.stamp.sec=0;
	imu_msg.header.stamp.nsec=0;
	imu_msg.header.frame_id='0';
	imu_msg.orientation.x=0.0;
	imu_msg.orientation.y=0.0;
	imu_msg.orientation.z=0.0;
	imu_msg.orientation.w=0.0;
	imu_msg.orientation_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	imu_msg.angular_velocity.x=0.0;
	imu_msg.angular_velocity.y=0.0;
	imu_msg.angular_velocity.z=0.0;
	imu_msg.angular_velocity_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // fixed
	imu_msg.linear_acceleration.x=0.0;
	imu_msg.linear_acceleration.y=0.0;
	imu_msg.linear_acceleration.z=0.0;
	imu_msg.linear_acceleration_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // fixed

	diagnostic_msgs::DiagnosticStatus system_status_msg;
	system_status_msg.level = 0; // default OK state
	system_status_msg.name = "System Status";
	system_status_msg.message = "";

	diagnostic_msgs::DiagnosticStatus filter_status_msg;
	filter_status_msg.level = 0; // default OK state
	filter_status_msg.name = "Filter Status";
	filter_status_msg.message = "";
#endif // HAVE_ROS

	// get data from com port //
	snav_bin_decoder_t* snav_bin_decoder = (snav_bin_decoder_t*)malloc(sizeof(snav_bin_decoder_t));
	memset(snav_bin_decoder, 0, sizeof(snav_bin_decoder_t));
	snav_bin_record_t* snav_bin_record = (snav_bin_record_t*)malloc(sizeof(snav_bin_record_t));
	memset(snav_bin_record, 0, sizeof(snav_bin_record_t));
	gpfpd_record_t* gpfpd_record = (gpfpd_record_t*)malloc(sizeof(gpfpd_record_t));
	memset(gpfpd_record, 0, sizeof(gpfpd_record_t));
	gtimu_record_t* gtimu_record = (gtimu_record_t*)malloc(sizeof(gtimu_record_t));
	memset(gtimu_record, 0, sizeof(gtimu_record_t));
	nvstd_record_t* nvstd_record = (nvstd_record_t*)malloc(sizeof(nvstd_record_t));
	memset(nvstd_record, 0, sizeof(nvstd_record_t));
	time_record_t* time_record = (time_record_t*)malloc(sizeof(time_record_t));
	memset(time_record, 0, sizeof(time_record_t));
	int bytes_received;

#ifdef HAVE_UDP
	int fd;
	sockaddr_in sock;
	sockaddr source;
	if (!openSocket(interface, ip_addr, port, &fd, &sock))
	{
		printf("Failed to open socket on %s:%u\n", ip_addr.c_str(), port);
		exit(EXIT_FAILURE);
	}
	printf("Open socket on %s:%u\n", ip_addr.c_str(), port);
#else
	if (OpenComport(const_cast<char*>(com_port.c_str()), baud_rate))
	{
		printf("Could not open serial port: %s \n", com_port.c_str());
		exit(EXIT_FAILURE);
	}
	printf("Open com port: %s, baud rate: %d\n", com_port.c_str(), baud_rate);
#endif // HAVE_UDP
	
	// Loop continuously, polling for packets
#ifdef HAVE_ROS
	printf("Run with ROS\n");
	while (ros::ok())
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 10ms=100Hz
		ros::spinOnce();

#ifdef HAVE_UDP
		if (bytes_received = readSocket(fd, 10, snav_bin_decoder_pointer(snav_bin_decoder), snav_bin_decoder_size(snav_bin_decoder), &source) > 0)
#else
		if ((bytes_received = PollComport(snav_bin_decoder_pointer(snav_bin_decoder), snav_bin_decoder_size(snav_bin_decoder))) > 0)
#endif // HAVE_UDP
		{
			printf("Received bytes: %d\n", bytes_received);
			// increment the decode buffer length by the number of bytes received //
			snav_bin_decoder_increment(snav_bin_decoder, bytes_received);

			// decode all the packets in the buffer //
			while (0 == snav_bin_record_decode(snav_bin_decoder, snav_bin_record))
			{
				printf("DT200 SNAV_BIN packet decoded.\n");
				switch (snav_bin_record->record_type)
				{
				case record_type_gpfpd:
				{
					if (decode_gpfpd_record(gpfpd_record, snav_bin_record) == 0)
					{
						printf("GPFPD record decoded.\n");
						// NavSatFix
						nav_sat_fix_msg.header.frame_id = nav_sat_frame_id;
						if (gpfpd_record->status.val.rtk == RTK_Integer || gpfpd_record->status.val.rtk == RTK_Float)
						{
							nav_sat_fix_msg.status.status = 2; // STATUS_GBAS_FIX
						}
						else if (gpfpd_record->nsv2 > 4)
						{
						  nav_sat_fix_msg.status.status = 0; // STATUS_FIX
						}
						else
						{
							nav_sat_fix_msg.status.status = -1; // STATUS_NO_FIX
						}
						nav_sat_fix_msg.latitude = gpfpd_record->latitude * 1E-7;
						nav_sat_fix_msg.longitude = gpfpd_record->longitude * 1E-7;
						nav_sat_fix_msg.altitude = gpfpd_record->altitude * 1E-3;

						// Twist
						twist_msg.linear.x = gpfpd_record->vel_north;
						twist_msg.linear.y = gpfpd_record->vel_east;
						twist_msg.linear.z = gpfpd_record->vel_up;

						// IMU
						imu_msg.header.frame_id = imu_frame_id;
						// Convert roll, pitch, yaw from radians to quaternion format //
						float phi = gpfpd_record->roll * DEGREES_TO_RADIANS / 2.0f;
						float theta = gpfpd_record->pitch * DEGREES_TO_RADIANS / 2.0f;
						float psi = gpfpd_record->heading * DEGREES_TO_RADIANS / 2.0f;
						float sin_phi = sinf(phi);
						float cos_phi = cosf(phi);
						float sin_theta = sinf(theta);
						float cos_theta = cosf(theta);
						float sin_psi = sinf(psi);
						float cos_psi = cosf(psi);
						imu_msg.orientation.x = -cos_phi * sin_theta * sin_psi + sin_phi * cos_theta * cos_psi;
						imu_msg.orientation.y = cos_phi * sin_theta * cos_psi + sin_phi * cos_theta * sin_psi;
						imu_msg.orientation.z = cos_phi * cos_theta * sin_psi - sin_phi * sin_theta * cos_psi;
						imu_msg.orientation.w = cos_phi * cos_theta * cos_psi + sin_phi * sin_theta * sin_psi;

						// System Status
						system_status_msg.message = "";
						system_status_msg.level = 0; // default OK state

						// Filter Status
						filter_status_msg.message = "";
						filter_status_msg.level = 0; // default OK state
						if (gpfpd_record->status.val.system == Align_GnssHeading ||
							gpfpd_record->status.val.system == Align_DyncmicAlign) {
							filter_status_msg.message = filter_status_msg.message + "0. Aligned. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "1. Not aligned. ";
						}

						//namespace sc = std::chrono;
						//static sc::duration<double> start_time = sc::system_clock::now().time_since_epoch();
						//sc::duration<double> now_time = sc::system_clock::now().time_since_epoch();
						//double fps = 0;
						//double diff = (now_time - start_time).count();
						//if (diff > 0) {
						//  fps = 1 / diff;
						//  start_time = now_time;
						//}

						//printf("GPFPD rate:%dHz, Lat:%.8f, Lon:%.8f, Alt:%.2f\n\n",
						//  (int)fps, gpfpd_record->latitude * 1E-7, gpfpd_record->longitude * 1E-7, gpfpd_record->altitude * 1E-3);
					}
					break;
				}
				case record_type_gtimu:
				{
					if (decode_gtimu_record(gtimu_record, snav_bin_record) == 0)
					{
						printf("GTIMU record decoded.\n");
						// Twist
						twist_msg.angular.x = gtimu_record->gyro_x * DEGREES_TO_RADIANS;
						twist_msg.angular.y = gtimu_record->gyro_y * DEGREES_TO_RADIANS;
						twist_msg.angular.z = gtimu_record->gyro_z * DEGREES_TO_RADIANS;

						// IMU
						imu_msg.angular_velocity.x = gtimu_record->gyro_x * DEGREES_TO_RADIANS; // These the same as the TWIST msg values
						imu_msg.angular_velocity.y = gtimu_record->gyro_y * DEGREES_TO_RADIANS;
						imu_msg.angular_velocity.z = gtimu_record->gyro_z * DEGREES_TO_RADIANS;
						imu_msg.linear_acceleration.x = gtimu_record->acc_x * G2MS;
						imu_msg.linear_acceleration.y = gtimu_record->acc_y * G2MS;
						imu_msg.linear_acceleration.z = gtimu_record->acc_z * G2MS;
					}
					break;
				}
				case record_type_nvstd:
				{
				  if (decode_nvstd_record(nvstd_record, snav_bin_record) == 0)
				  {
					printf("NVSTD record decoded.\n");
					// NavSatFix
					nav_sat_fix_msg.position_covariance = {
					  pow(nvstd_record->lat_std,2), 0.0, 0.0,
					  0.0, pow(nvstd_record->lon_std,2), 0.0,
					  0.0, 0.0, pow(nvstd_record->alt_std,2)
					};

					// IMU
					imu_msg.orientation_covariance[0] = nvstd_record->roll_std * DEGREES_TO_RADIANS;
					imu_msg.orientation_covariance[4] = nvstd_record->pitch_std * DEGREES_TO_RADIANS;
					imu_msg.orientation_covariance[8] = nvstd_record->heading_std * DEGREES_TO_RADIANS;

					//printf("GTIMU, gps_ms:%d\n", nvstd_record->gps_time_ms);
				  }
				  break;
				}
				case record_type_time:
				{
				  if (decode_time_record(time_record, snav_bin_record) == 0)
				  {
					printf("TIME record decoded.\n");
					int64_t ms_gps_utc = gps_time_to_utc(time_record);
					if (ms_gps_utc != 0)
					{
					  // NavSatFix
					  nav_sat_fix_msg.header.stamp.sec = static_cast<uint32_t>(ms_gps_utc / 1000);
					  nav_sat_fix_msg.header.stamp.nsec = static_cast<uint32_t>((ms_gps_utc - nav_sat_fix_msg.header.stamp.sec) * 1000000);
					  // IMU
					  imu_msg.header.stamp.sec = nav_sat_fix_msg.header.stamp.sec;
					  imu_msg.header.stamp.nsec = nav_sat_fix_msg.header.stamp.nsec;
					}
				  }
				  break;
				}
				default:
					break;
				}
				
				// Publish messages //
				if (time_record->flags.val.time_valid && time_record->flags.val.offset_valid)
				{
					printf("Publish messages.\n");
					nav_sat_fix_pub.publish(nav_sat_fix_msg);
					twist_pub.publish(twist_msg);
					imu_pub.publish(imu_msg);
					system_status_pub.publish(system_status_msg);
					filter_status_pub.publish(filter_status_msg);
				}
				else {
					printf("UTC time invalid, messages not published.\n");
				}
			}
		}
	}

#else // NOT HAVE_ROS
	while (1)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 10ms=100Hz

#ifdef HAVE_UDP
		if (bytes_received = readSocket(fd, 10, snav_bin_decoder_pointer(snav_bin_decoder), snav_bin_decoder_size(snav_bin_decoder), &source) > 0)
#else
		if ((bytes_received = PollComport(snav_bin_decoder_pointer(snav_bin_decoder), snav_bin_decoder_size(snav_bin_decoder))) > 0)
#endif // HAVE_UDP
		{
			// increment the decode buffer length by the number of bytes received //
			snav_bin_decoder_increment(snav_bin_decoder, bytes_received);

			printf("Received bytes:%d\n", bytes_received);
			// decode all the packets in the buffer //
			while (0 == snav_bin_record_decode(snav_bin_decoder, snav_bin_record))
			{
				switch (snav_bin_record->record_type)
				{
				case record_type_gpfpd:
				{
					if (decode_gpfpd_record(gpfpd_record, snav_bin_record) == 0)
					{
					  namespace sc = std::chrono;
					  static sc::duration<double> start_time = sc::system_clock::now().time_since_epoch();
					  sc::duration<double> now_time = sc::system_clock::now().time_since_epoch();
					  double fps = 0;
					  double diff = (now_time - start_time).count();
					  if (diff > 0) {
						fps = 1 / diff;
						start_time = now_time;
					  }

					  printf("GPFPD rate:%dHz, Lat:%.8f, Lon:%.8f, Alt:%.2f\n\n",
						(int)fps, gpfpd_record->latitude * 1E-7, gpfpd_record->longitude * 1E-7, gpfpd_record->altitude * 1E-3);
					}
					break;
				}
				case record_type_gtimu:
				{
					if (decode_gtimu_record(gtimu_record, snav_bin_record) == 0)
					{
					  namespace sc = std::chrono;
					  static sc::duration<double> start_time = sc::system_clock::now().time_since_epoch();
					  sc::duration<double> now_time = sc::system_clock::now().time_since_epoch();
					  double fps = 0;
					  double diff = (now_time - start_time).count();
					  if (diff > 0) {
						fps = 1 / diff;
						start_time = now_time;
					  }

					  printf("GTIMU rate:%dHz, AccX:%.2f, AccY:%.2f, AccZ:%.2f, GyroX:%.2f, GyroY:%.2f, GyroZ:%.2f\n\n",
						(int)fps, gtimu_record->acc_x * G2MS, gtimu_record->acc_y * G2MS, gtimu_record->acc_z * G2MS,
						gtimu_record->gyro_x, gtimu_record->gyro_y, gtimu_record->gyro_z);
					}
					break;
				}
				case record_type_nvstd:
				{
				  if (decode_nvstd_record(nvstd_record, snav_bin_record) == 0)
				  {
					namespace sc = std::chrono;
					static sc::duration<double> start_time = sc::system_clock::now().time_since_epoch();
					sc::duration<double> now_time = sc::system_clock::now().time_since_epoch();
					double fps = 0;
					double diff = (now_time - start_time).count();
					if (diff > 0) {
					  fps = 1 / diff;
					  start_time = now_time;
					}

					printf("NVSTD rate:%dHz, Pos:%.2f, Vel:%.2f, Heading:%.2f, Altitude:%.2f\n\n",
					  (int)fps, nvstd_record->lat_std, nvstd_record->ve_std, nvstd_record->heading_std, nvstd_record->pitch_std);
				  }
				  break;
				}
				case record_type_time:
				{
				  if (decode_time_record(time_record, snav_bin_record) == 0)
				  {
					namespace sc = std::chrono;
					static sc::duration<double> start_time = sc::system_clock::now().time_since_epoch();
					sc::duration<double> now_time = sc::system_clock::now().time_since_epoch();
					double fps = 0;
					double diff = (now_time - start_time).count();
					if (diff > 0) {
					  fps = 1 / diff;
					  start_time = now_time;
					}
					
					int64_t ms_gps_to_utc = gps_time_to_utc(time_record);
					printf("TIME rate:%dHz, UTC offset:%d, gps_ms:%d\n",
					  (int)fps, time_record->utc_offset, time_record->gps_time_ms);
				  }
				  break;
				}
				default:
					break;
				}
			}
		}
	}

#endif

	// realese resource
	free(snav_bin_decoder);
	free(snav_bin_record);
	free(gpfpd_record);
	free(gtimu_record);
	free(nvstd_record);
	free(time_record);

#ifdef HAVE_UDP	
	close(fd); // Close socket
#endif // HAVE_UDP

} // end of main()