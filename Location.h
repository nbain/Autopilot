#ifndef AUTOPILOT2_LOCATION_H
#define AUTOPILOT2_LOCATION_H

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


class Location
{
	


	public:

		void init();
		void estimate();
		void Log();

		imu::Vector<3> euler;
		imu::Vector<3> euler_rates;

		sensors_event_t orientationEvent, ratesEvent;
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

			float pitch; //In degrees (for now - might want degrees in float later)
			float roll;
			float yaw;

			float pitch_rate; //In dps.Found from looking at pitch delta and delta t - no new information
			float roll_rate;
			float yaw_rate;

			float pitch_acc;
			float roll_acc;
			float yaw_acc;

			//Will fill in position - lat, long, height MSL - when ready

			unsigned int timestamp; //In microseconds.  Limit for each of these is 4.3 billion, so weird after an hour

		};

		LOCATION Location; //Most recent location data


};


#endif