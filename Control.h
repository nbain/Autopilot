#ifndef AUTOPILOT2_CONTROL_H
#define AUTOPILOT2_CONTROL_H

#include <Arduino.h>

#include "Location.h"
#include "Receiver.h"

//https://github.com/moritzmaier/Eigen
//Notes on porting Eigen for Arduino: https://forum.pjrc.com/threads/41929-How-to-install-use-Eigen-template-library
#include <Eigen.h>     // Calls main Eigen matrix class library
#include <Eigen/LU>    // Calls inverse, determinant, LU decomp., etc.
#include <Eigen/QR>
using namespace Eigen;



/*This doesn't actually change the PWM frequency sent by analogWrite()
To change the PWM frequency sent out, must change the Arduino library by rewriting PWM_FREQUENCY and TC_FREQUENCY in Variant.h
To get to Variant.h, go to Finder and click Go (at top), then click Library.  Then Arduino15 folder is near the bottom
In Arduino 15: packages/arduino/hardware/sam/1.6.11/variants/arduino_due_x/variant.h
Note: PWM_FREQUENCY only affects pins 6,7,8,9, and TC_FREQUENCY affects the rest.
*/
#define MOTORS_PWM_FREQ 1000 //Hz


#define FRONT_RIGHT_SERVO_PIN 2
#define FRONT_LEFT_SERVO_PIN 3
#define BACK_RIGHT_SERVO_PIN 4
#define BACK_LEFT_SERVO_PIN 5

#define FRONT_RIGHT_MOT_PIN 6
#define FRONT_LEFT_MOT_PIN 7
#define BACK_RIGHT_MOT_PIN 8
#define BACK_LEFT_MOT_PIN 9
#define BACK_MID_RIGHT_MOT_PIN 10
#define BACK_MID_LEFT_MOT_PIN 11
#define BACK_FAR_RIGHT_MOT_PIN 12
#define BACK_FAR_LEFT_MOT_PIN 13




class Control
{
	

	public:

		float vehicle_mass = 4; //Vehicle mass in kg.
		float roll_moment_of_inertia = 0.295; //Roll moment of inertia in kg*m^2
		float pitch_moment_of_inertia = 0.872; //Pitch moment of inertia in kg*m^2
		float yaw_moment_of_inertia = 1.165; //Yaw moment of inertia in kg*m^2

		float motor_directions[8] = {1, -1, 1, -1, -1, 1, -1, 1}; //CW is 1, CCW is -1
		float motor_x_locs[8] = {0.23, -0.23, 0.18, -0.18, 0.4, -0.4, 0.62, -0.62}; //In meters.  Left-right location.
		float motor_y_locs[8] = {0.99, 0.99, -0.35, -0.35, -0.35, -0.35, -0.35, -0.35}; //In meters.  Front-back location.

		float motor_torque_per_thrust = 0.01378; //Nm of propeller torque per N thrust





		/*
		Parameters of fan unit

		*/
		struct FAN_PARAMETERS
		{

			float voltage1; //First voltage at which test measurements taken
			float voltage1_min_pwm; //PWM at which motor just starts spinning 
			float voltage1_hover_thrust_exponent; //Exponent at which thrust increases with pwm
			float voltage1_hover_thrust_curve_coefficient; //Coefficent to scale exponent by

			float voltage2; //Second voltage at which test measurements taken
			float voltage2_min_pwm; //PWM at which motor just starts spinning 
			float voltage2_hover_thrust_exponent; //Exponent at which thrust increases with pwm
			float voltage2_hover_thrust_curve_coefficient; //Coefficent to scale exponent by

			float voltage_fraction; //Fraction between voltages 1 and 2.  Calculated and set in control code.
			float operating_voltage_min_pwm; //Interpolation between voltages 1 and 2.
			float operating_voltage_hover_thrust_exponent; //Interpolation between voltages 1 and 2.
			float operating_voltage_hover_thrust_curve_coefficient; //Interpolation between voltages 1 and 2.

		} ;

		FAN_PARAMETERS motor_1400kv{};
		FAN_PARAMETERS motor_1500kv{};


		/*
		Parameters of servo unit

		*/
		struct SERVO_PARAMETERS
		{

			float zero_deg_pwm; //PWM at which servo is directly vertical
			float ninety_deg_pwm; //PWM at which servo is horizontal vertical

		} ;

		SERVO_PARAMETERS front_right_servo;
		SERVO_PARAMETERS front_left_servo;
		SERVO_PARAMETERS back_right_servo;
		SERVO_PARAMETERS back_left_servo;


		/*
		Current desired accelerations in rad/s^2 and m/s^2

		*/
		struct ACCELERATIONS
		{

			//Desired rotational accelerations in rad/s^2
			float roll_acceleration;
			float pitch_acceleration;
			float yaw_acceleration;

			//Desired translational accelerations in m/s^2
			float x_acceleration;
			float z_acceleration;

		} ;

		ACCELERATIONS Accelerations;


		/*
		Forces and moments in Newtons and Newton-meters.

		For:
		1. Current target
		2. Storage location for results of calculations on the theoretical forces and moments due to a set of fan forces and servo angles

		*/
		struct FORCES_AND_MOMENTS
		{

			//Rotational accelerations in rad/s^2
			float roll_moment;
			float pitch_moment;
			float yaw_moment;

			//Translational accelerations in m/s^2
			float x_force;
			float z_force;

		} ;

		FORCES_AND_MOMENTS Target_Forces_and_Moments;
		FORCES_AND_MOMENTS Theoretical_Fan_Forces_and_Moments;



		/*
		Current desired thrusts and angles from each motor and servo in Newtons and Radians

		*/
		struct THRUSTS_AND_ANGLES
		{

			//Desired fan thrusts in Newtons
			float front_right_fan_thrust;
			float front_left_fan_thrust;
			float back_right_fan_thrust;
			float back_left_fan_thrust;
			float back_mid_right_fan_thrust;
			float back_mid_left_fan_thrust;
			float back_far_right_fan_thrust;
			float back_far_left_fan_thrust;

			//Desired servo angles in Radians
			float front_right_servo_angle;
			float front_left_servo_angle;
			float back_right_servo_angle;
			float back_left_servo_angle;

		} ;

		THRUSTS_AND_ANGLES Thrusts_and_Angles;


		/*
		Final PWM output

		*/
		struct PWM_OUTPUT
		{

			//Desired fan thrusts in Newtons
			int front_right_fan_micros;
			int front_left_fan_micros;
			int back_right_fan_micros;
			int back_left_fan_micros;
			int back_mid_right_fan_micros;
			int back_mid_left_fan_micros;
			int back_far_right_fan_micros;
			int back_far_left_fan_micros;

			//Desired servo angles in Radians
			int front_right_servo_micros;
			int front_left_servo_micros;
			int back_right_servo_micros;
			int back_left_servo_micros;

		} ;

		PWM_OUTPUT PWM_Output;




		/*
		Not used in new control system

		*/
		struct COMMANDS
		{

			//Desired Thrust in
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

		int motor_thrust_to_pwm(Control::FAN_PARAMETERS *, float motor_thrust, float batt_voltage);
		int servo_angle_to_pwm(Control::SERVO_PARAMETERS *, float servo_angle);

		MatrixXf calculate_theoretical_fan_forces_and_moments(MatrixXf control_guess);
		MatrixXf calculate_kinematics_derivatives(MatrixXf control_guess);
		MatrixXf calculate_gradient_descended_control_guess(MatrixXf control_guess, MatrixXf target_forces_and_moments);

		void acceleration_controller(Location::LOCATION *, Receiver::RECEIVER *);
		void accelerations_to_forces_and_moments(Control::ACCELERATIONS *);
		void forces_and_moments_to_thrusts_and_angles(Control::FORCES_AND_MOMENTS *);
		void thrusts_and_angles_to_pwms(Control::THRUSTS_AND_ANGLES *);
		void send_pwms(Control::PWM_OUTPUT *);
		void send_microseconds(int motor_pin, int microseconds);
		void run(Location::LOCATION *, Receiver::RECEIVER *);

		void print_mtxf(const Eigen::MatrixXf& K);

		float Pitch_Rate_P; //How much to accelerate towards target rate.  More than about 0.03 on Deathtrap and starts to shake.
		float Roll_Rate_P; //0.1, with sqrt p = 6, -> 1.8 for 10 deg with no rate (3 and 1 for 10 deg, no rate PID pitch, roll)
		float Yaw_Rate_P;

		bool Aux1_been_up = false;
		bool Aux1_been_down = false;



		void run2(Location::LOCATION *, Receiver::RECEIVER *);
		void Log();
		void MotorTest(Receiver::RECEIVER *);

};



#endif