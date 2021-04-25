#ifndef AUTOPILOT2_LOCATION_H
#define AUTOPILOT2_LOCATION_H

#include <iostream>
#include <fstream>
#include <cstdio>
#include <string.h>

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

//#include "main.h"

/*
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
*/




class Location
{
	


	public:

		void init();
		void estimate();
		void Log();

		void readData();
		void setup_port();
		void convertMessage();

		std::string readPath = "/dev/ttyACM0"; // reading IMU and Receiver data from the programming port of the Arduino
		int serial_port_read;
		struct termios tty;

		// Location data consists of 6 flaots, each 8 bytes long (+/- 000.000) and separated by a comma
		static const int numBytes = 54;
		char startMarker = '&';
		//char endMarker = '>';
		bool dataAvailable;
		char incomingByte[1];
		char location_msg[numBytes];
		char* location_msg_ptr;
		float location_vals[6];


		//imu::Vector<3> euler;
		//imu::Vector<3> euler_rates;
		//sensors_event_t orientationEvent, ratesEvent;


		int timeSinceReset;
		int lastReset = 7000;
		int reset_count;

		float yaw_correction;


		float last_roll;
		float last_pitch;
		float last_yaw;
		float last_roll_rate;
		float last_pitch_rate;
		float last_yaw_rate;
		int last_time;

		float last_roll2;
		float last_pitch2;
		float last_yaw2;
		float last_roll_rate2;
		float last_pitch_rate2;
		float last_yaw_rate2;
		int last_time2;


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


			//Will fill in position - lat, long, height MSL - when ready

			float time; //In seconds. 

		};

		LOCATION Current_Location; //Most recent location data



		


};


#endif