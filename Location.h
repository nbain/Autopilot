#ifndef AUTOPILOT2_LOCATION_H
#define AUTOPILOT2_LOCATION_H

#include <iostream>
#include <fstream>
#include <cstdio>
#include <string.h>

#include <sys/ioctl.h>
#include <linux/serial.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>


#include "XPlane.h"

/*
Seems to be a circular dependency which causes include errors/not recognizing something has been included.
https://stackoverflow.com/questions/625799/resolve-build-errors-due-to-circular-dependency-amongst-classes

Instead of doing #include "XPlane.h", use class XPlane (not super clear )
*/
//class XPlane;



class Location
{
	


	public:

		void init(std::string file_path);
		void estimate();
		void Log();

		void readData();
		void setup_port();
		void convertMessage();

		std::string LOG_PATH;
		std::string readPath = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Due_Prog._Port_5573030313735171E0F0-if00"; // reading IMU and Receiver data from the programming port of the Arduino
		int serial_port_read;
		struct termios options;

		// Location data consists of 6 flaots, each 8 bytes long (+/- 000.000) and separated by a comma
		static const int numBytes = 54;
		char loc_startMarker = '&';
		//char endMarker = '>';
		bool dataAvailable;
		char incomingByte[1];
		char location_msg[numBytes];
		char* location_msg_ptr;
		float location_vals[6];


		struct LOCATION
		{

			float pitch; //In radians 
			float roll;
			float heading; //From 0 to 2*PI with true north = 0

			float pitch_rate; //In rad/sec. 
			float roll_rate;
			float heading_rate;

			float pitch_acc;
			float roll_acc;
			float yaw_acc;


			float longitudinal_true_airspeed; //Airspeed parallel to longitudinal axis, m/s
			float longitudinal_Q; //Dynamic pressure along longitudinal axis.  Can be negative.
			float vertical_speed; //Velocity in vertical direction, m/s (not airspeed - though should be very close to vertical true airspeed)

			float latitude; //Latitude in degrees
			float longitude; //Longitude in degrees
			float altitude_MSL; //Altitude above sea-level, meters

			float altitude_AGL; //Altitude above ground-level, meters

			float AOA; //Angle of attack, in degrees

			float air_density; //Air density in kg/m^3
			float air_density_fraction; //Fraction of sea-level, STP air density (1.2247 kg/m^3)

			float time; //Seconds after program start 

		};

		LOCATION Current_Location; //Most recent location data


};


#endif
