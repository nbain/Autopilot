#ifndef AUTOPILOT2_MOTORS
#define AUTOPILOT2_MOTORS

#include "Control.h"



//PWM range where have some effect on the motor
#define MOT_PWM_MIN 1000


//For full 60k rpm
#define FR_MOT_PWM_MAX 2016
#define FL_MOT_PWM_MAX 2016
#define BR_MOT_PWM_MAX 2016
#define BL_MOT_PWM_MAX 2016
#define BMR_MOT_PWM_MAX 2016
#define BML_MOT_PWM_MAX 2016
#define BFR_MOT_PWM_MAX 2016
#define BFL_MOT_PWM_MAX 2016

/*
//For Batt Voltage = 22.5V
#define FR_MOT_PWM_MAX 1647
#define FL_MOT_PWM_MAX 1680
#define BR_MOT_PWM_MAX 1573
#define BL_MOT_PWM_MAX 1612
#define BMR_MOT_PWM_MAX 1566
#define BML_MOT_PWM_MAX 1548
#define BFR_MOT_PWM_MAX 1629
#define BFL_MOT_PWM_MAX 1623
*/


/*
Zero degree points should be so that there is no yaw and no front-back force 
A lot of combinations that give these (front tilted forward, back tilted backwards in varying degrees),
so should set front servos to known 90 degree angle (easier to measure) and then set back servos to meet
no front-back force, no yaw moment constraint (fully constrained - only one possible combination)
*/
#define FR_SERVO_PWM_0 950
#define FR_SERVO_PWM_90 1900

#define FL_SERVO_PWM_0 850
#define FL_SERVO_PWM_90 2000

#define BR_SERVO_PWM_0 900
#define BR_SERVO_PWM_90 2000

#define BL_SERVO_PWM_0 1000
#define BL_SERVO_PWM_90 2000


#define FRONT_RIGHT_SERVO_PIN 2
#define FRONT_LEFT_SERVO_PIN 3
#define BACK_RIGHT_SERVO_PIN 4
#define BACK_LEFT_SERVO_PIN 5

#define FRONT_RIGHT_MOT_PIN 6
#define FRONT_LEFT_MOT_PIN 7
#define BACK_RIGHT_MOT_PIN 8 //Blue
#define BACK_LEFT_MOT_PIN 9 //Blue
#define BACK_MID_RIGHT_MOT_PIN 10  //Orange
#define BACK_MID_LEFT_MOT_PIN 11  //Orange
#define BACK_FAR_RIGHT_MOT_PIN 12  //Green
#define BACK_FAR_LEFT_MOT_PIN 13  //Green





class Motors
{
	
	public:


		int front_right_micros;
		int front_left_micros;
		int back_right_micros;
		int back_left_micros;
		int back_mid_right_micros;
		int back_mid_left_micros;
		int back_far_right_micros;
		int back_far_left_micros;

		int front_right_servo_micros;
		int front_left_servo_micros;
		int back_right_servo_micros;
		int back_left_servo_micros;


		

	public:

		
		void init();
		void sendMicroseconds(int motor_pin, int microseconds);

		void sendMotorSignals(Control::COMMANDS *);
		void Log();
		

};



#endif