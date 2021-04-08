#ifndef AUTOPILOT2_LOCATION_H
#define AUTOPILOT2_LOCATION_H

#include <iostream>
#include <cstdio>

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