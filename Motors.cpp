#include <Arduino.h>
#include <math.h> //Is this the best place to include this?  Used for rounding.
#include "Motors.h"



/*
Should be able to access the most recent input array found by control law and send to the motors and servos.

You should just be able to send an array of values from 0 to 1 for how much thrust you want from each motor
and the motors should just give that amount of thrust, from 0% to 100% of what they are capable of

**Might want to attach a load cell to a motor with propeller and get a regression/table for what PWM
corresponds with what % of total thrust 

*/

/*
1500 kv motor at 11.1V:
Thrust (lbs) = [ (PWM - 1020) / 610 ] ^ 1.55

1500 kv motor at 12.6V:
Thrust (lbs) = [ (PWM - 1020) / 530 ] ^ 1.5



1400 kv motor at 11.1V:
Thrust (lbs) = [ (PWM - 1050) / 660 ] ^ 1.65
	-Max at 1.68 lbs, so max PWM is prob 1954 (where curve hits 1.68 lbs [max] - then would plateau after that)
	-174W max power


1400 kv motor at 12.6V:
Thrust (lbs) = [ (PWM - 1050) / 590 ] ^ 1.61
	-Curve hits 1.99 [max] at 1955 (same as above) - prob not a coincidence
	-230W max power

For thrust at higher freestream velocities, assume max power is same
	-Won't be same:



*/





void Motors::init()
{

	
	//arduinoMaxPulseLength = 1000000.0f/MOTORS_ESC_PWM_FREQ;
	
	
	// define pins as output
	/*
	pinMode(FRONT_RIGHT_MOT_PIN, OUTPUT);
	pinMode(FRONT_LEFT_MOT_PIN, OUTPUT);
	pinMode(BACK_RIGHT_MOT_PIN, OUTPUT);
	pinMode(BACK_LEFT_MOT_PIN, OUTPUT);
	pinMode(BACK_FAR_RIGHT_MOT_PIN, OUTPUT);
	pinMode(BACK_FAR_LEFT_MOT_PIN, OUTPUT);
	*/

	Serial.println("Sending 1000us to all motors and servos");


	sendMicroseconds(FRONT_RIGHT_SERVO_PIN, FR_SERVO_PWM_0);
	sendMicroseconds(FRONT_LEFT_SERVO_PIN, FL_SERVO_PWM_0);
	sendMicroseconds(BACK_RIGHT_SERVO_PIN, BR_SERVO_PWM_0);
	sendMicroseconds(BACK_LEFT_SERVO_PIN, BL_SERVO_PWM_0);

	sendMicroseconds(FRONT_RIGHT_MOT_PIN, 1000);
	sendMicroseconds(FRONT_LEFT_MOT_PIN, 1000);
	sendMicroseconds(BACK_RIGHT_MOT_PIN, 1000);
	sendMicroseconds(BACK_LEFT_MOT_PIN, 1000);
	sendMicroseconds(BACK_MID_RIGHT_MOT_PIN, 1000);
	sendMicroseconds(BACK_MID_LEFT_MOT_PIN, 1000);
	sendMicroseconds(BACK_FAR_RIGHT_MOT_PIN, 1000);
	sendMicroseconds(BACK_FAR_LEFT_MOT_PIN, 1000);


	delay(5000); //3 seconds is just barely long enough for ESC to do startup sequence

}

/*
Send pulse train of certain number of microseconds to a certain pin, at whatever frequency is being used
until rewritten by a new analogWrite

Nearest_bit_value must be between 0 and 255 inclusive, so microseconds must be from 0 to 2490
(technically a few microseconds beyond that because of rounding, but no reason to do that) 

At 400Hz, pulses are actually 2490us apart (2490us between starts, ends)
Each one goes up by 9.765 us (confirmed to the thosandths place by oscilloscope, and equals 2500 / 256)
Value of 254 is 2,480.0 us long, value of 1 is 9.765 long
0 is always off, 255 is always on
*/
void Motors::sendMicroseconds(int motor_pin, int microseconds)
{

	//float analog_write_bit_us = 1000000.0f / (float)MOTORS_PWM_FREQ / 255;

	float precise_bit_value = (float)microseconds / 10;

	int nearest_bit_value = round(precise_bit_value);


	analogWrite(motor_pin, nearest_bit_value);


}


/*
This should get a value from 0 to 1 for each motor and send that fraction of maximum thrust to each motor



Right now, just linearly interpolates between motor's max and min pwm, which is set in motors2.h

In the future, this micros value should correspond to
measured max thrust percent at that micros value.  So, must have function to convert 0-1 fraction 
of max thrust into a microsecond value.
Probably define the 0 and 1 point, say 1030 and 1940, and do a slight exponential or polynomial to match in between
But if flies ok with just min and max, just go straight to machine learning
*/
void Motors::sendMotorSignals(Control::COMMANDS * Commands)
{ 


	front_right_micros = MOT_PWM_MIN + Commands->front_right * (FR_MOT_PWM_MAX - MOT_PWM_MIN);
	sendMicroseconds(FRONT_RIGHT_MOT_PIN, front_right_micros);

	front_left_micros = MOT_PWM_MIN + Commands->front_left * (FL_MOT_PWM_MAX - MOT_PWM_MIN);
	sendMicroseconds(FRONT_LEFT_MOT_PIN, front_left_micros);

	back_right_micros = MOT_PWM_MIN + Commands->back_right * (BR_MOT_PWM_MAX - MOT_PWM_MIN);
	sendMicroseconds(BACK_RIGHT_MOT_PIN, back_right_micros);

	back_left_micros = MOT_PWM_MIN + Commands->back_left * (BL_MOT_PWM_MAX - MOT_PWM_MIN);
	sendMicroseconds(BACK_LEFT_MOT_PIN, back_left_micros);

	back_mid_right_micros = MOT_PWM_MIN + Commands->back_mid_right * (BMR_MOT_PWM_MAX - MOT_PWM_MIN);
	sendMicroseconds(BACK_MID_RIGHT_MOT_PIN, back_mid_right_micros);

	back_mid_left_micros = MOT_PWM_MIN + Commands->back_mid_left * (BML_MOT_PWM_MAX - MOT_PWM_MIN);
	sendMicroseconds(BACK_MID_LEFT_MOT_PIN, back_mid_left_micros);

	back_far_right_micros = MOT_PWM_MIN + Commands->back_far_right * (BFR_MOT_PWM_MAX - MOT_PWM_MIN);
	sendMicroseconds(BACK_FAR_RIGHT_MOT_PIN, back_far_right_micros);

	back_far_left_micros = MOT_PWM_MIN + Commands->back_far_left * (BFL_MOT_PWM_MAX - MOT_PWM_MIN);
	sendMicroseconds(BACK_FAR_LEFT_MOT_PIN, back_far_left_micros);




	front_right_servo_micros = FR_SERVO_PWM_0 + Commands->front_right_servo / 90 * (FR_SERVO_PWM_90 - FR_SERVO_PWM_0);
	sendMicroseconds(FRONT_RIGHT_SERVO_PIN, front_right_servo_micros);

	front_left_servo_micros = FL_SERVO_PWM_0 + Commands->front_left_servo / 90 * (FL_SERVO_PWM_90 - FL_SERVO_PWM_0);
	sendMicroseconds(FRONT_LEFT_SERVO_PIN, front_left_servo_micros);


	back_right_servo_micros = BR_SERVO_PWM_0 + Commands->back_right_servo / 90 * (BR_SERVO_PWM_90 - BR_SERVO_PWM_0);
	sendMicroseconds(BACK_RIGHT_SERVO_PIN, back_right_servo_micros);

	back_left_servo_micros = BL_SERVO_PWM_0 + Commands->back_left_servo / 90 * (BL_SERVO_PWM_90 - BL_SERVO_PWM_0);
	sendMicroseconds(BACK_LEFT_SERVO_PIN, back_left_servo_micros);


	
}


void Motors::Log()
{ 

	//PWM Microseconds checks
	Serial.print("  PWMs: ");

	Serial.print(front_right_micros); Serial.print(" ");
	Serial.print(front_left_micros); Serial.print(" ");
	Serial.print(back_right_micros); Serial.print(" ");
	Serial.print(back_left_micros); Serial.print(" ");
	Serial.print(back_mid_right_micros); Serial.print(" ");
	Serial.print(back_mid_left_micros); Serial.print(" ");
	Serial.print(back_far_right_micros); Serial.print(" ");
	Serial.print(back_far_left_micros); Serial.print(" ");

	Serial.print(front_right_servo_micros); Serial.print(" ");
	Serial.print(front_left_servo_micros); Serial.print(" ");
	Serial.print(back_right_servo_micros); Serial.print(" ");
	Serial.print(back_left_servo_micros); Serial.print(" ");

	//Serial.println(micros());

}




