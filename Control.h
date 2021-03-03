#ifndef AUTOPILOT2_CONTROL_H
#define AUTOPILOT2_CONTROL_H

#include <Arduino.h>

#include "Location.h"
#include "Receiver.h"

class Control
{
	

	public:

		struct COMMANDS
		{

			//Should be a value from 0 to 1 from zero to maximum thrust
			float front_right;
			float front_left;
			float back_right;
			float back_left;
			float back_mid_right;
			float back_mid_left;
			float back_far_right;
			float back_far_left;

			//0 means tilted straight up, 90 is tilted straight forward
			float front_right_servo;
			float front_left_servo;
			float back_right_servo;
			float back_left_servo;

		} ;

		COMMANDS Commands;

	public:

		float Pitch_Rate_P; //How much to accelerate towards target rate.  More than about 0.03 on Deathtrap and starts to shake.
		float Roll_Rate_P; //0.1, with sqrt p = 6, -> 1.8 for 10 deg with no rate (3 and 1 for 10 deg, no rate PID pitch, roll)
		float Yaw_Rate_P;

		bool Aux1_been_up = false;
		bool Aux1_been_down = false;

		void run(Location::LOCATION *, Receiver::RECEIVER *);
		void Log();
		void MotorTest(Receiver::RECEIVER *);

};



#endif