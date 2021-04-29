#ifndef AUTOPILOT2_XPLANE_H
#define AUTOPILOT2_XPLANE_H

//#include "main.h"

//#include <Arduino.h>


#include <iostream>
#include <chrono>
#include <cstdint>
#include <cmath> 

#include <unistd.h>
#include <string.h> //For memset()

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>


//#include "Control.h"
#include "Location.h"





class XPlane
{
	

	public:

		

		/*
		Data received from XPlane

		*/
		
		struct XPLANE_DATA
		{

			float flight_time;

			//Attitude angles in degrees
			float roll;
			float pitch;
			float heading;

			float latitude;
			float longitude;
			float elevation;

			float airspeed_kias;
			float dist_agl;
			float static_pressure_inHg;

			float OpenGL_acc_x;
			float OpenGL_acc_y;
			float OpenGL_acc_z;

			float OpenGL_roll;
			float OpenGL_pitch;
			float OpenGL_hdg;

			//Rotational velocities in radians/sec
			float roll_rot_vel;
			float pitch_rot_vel;
			float heading_rot_vel;

			float mag_heading;

			float ambient_temp; //Ambient temperature in Celsius


			//Calculated values
			float elapsed_time; //Time elapsed between current time and last batch of data from XPlane in seconds

			float roll_rad; //Attitude values in radians
			float pitch_rad;
			float heading_rad;

			float roll_rot_vel_calcd; //Roll rotation rate based on roll delta in calculated elapsed time in radians/sec
			float pitch_rot_vel_calcd;
			float heading_rot_vel_calcd;

			float longitudinal_Q; //Dynamic pressure along longitudinal axis derived from indicated airspeed, in pascals.

		} ;

		XPLANE_DATA Current_XPlane_data;
		XPLANE_DATA Last_XPlane_data;

		int sock_send;
		struct sockaddr_in myaddr_send;

		int sock_recv;
		struct sockaddr_in addr_recv;


		int UDP_Recv(char * UDP_Received);
		void get_data_from_XPlane();
		int UDP_Send(int array[12]);
		int UDP_Send_IP(char IP_addr[20]);
		void send_output_to_XPlane(float force_and_moment_array[6], float servo_angle_array[4], float loop_time);

		int UDP_Setup_Send();


};



#endif