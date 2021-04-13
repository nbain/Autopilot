#ifndef AUTOPILOT2_CONTROL_H
#define AUTOPILOT2_CONTROL_H

//#include "main.h"

//#include <Arduino.h>


#include <iostream>
#include <cstdio>
#include <chrono>
#include <cstdint>

#include <Eigen/LU> 
#include <Eigen/QR>

#include "XPlane.h"
#include "Location.h"
#include "Receiver.h"





#define e 2.718281828






class Control
{
	

	public:

		void setup_port();
		void writeData();

		std::string path = "/dev/ttyACM0";
		int serial_port;
		struct termios tty;

		char startMarker = '<';
		char endMarker = '>';
		static const int numBytes = 60;
		char msg_buffer[numBytes];
		char pwm[32];

		std::chrono::steady_clock::time_point program_start_time;

		std::chrono::steady_clock::time_point loop_end_time;

		std::chrono::steady_clock::time_point last_loop_end_time;


		//Make pointers to location and receiver structs available to all functions
		Location::LOCATION * Current_Location_ptr;

		Receiver::RECEIVER * Current_Receiver_Values_ptr;


		unsigned int loop_time_us;
		unsigned int loop_time2;

		unsigned int loop_time3;

		float loop_time = 0.001;

		//unsigned int last_loop_end_time;


		float vehicle_mass = 4; //Vehicle mass in kg.
		float roll_moment_of_inertia = 0.295; //Roll moment of inertia in kg*m^2
		float pitch_moment_of_inertia = 0.872; //Pitch moment of inertia in kg*m^2
		float yaw_moment_of_inertia = 1.165; //Yaw moment of inertia in kg*m^2

		float commands_sent_time; //Time of start of control loop in microseconds
		//float loop_time = 0.015; //Loop time in seconds.  Updated as loop runs.


		float servo_supply_voltage = 6.21;
		float batt_voltage = 12.6;
		float inflow_velocity = 0; //Inflow velocity in m/s

		float test_input = 0;
		float test_pwm = 1040;
		

		Eigen::MatrixXf end_of_loop_motor_vels_and_servo_angles; //Rotational velocities and servo angles at end of current loop packaged as Eigen::MatrixXf

		Eigen::MatrixXf target_accelerations;
		Eigen::MatrixXf target_forces_and_moments;
		Eigen::MatrixXf theoretical_rot_vel_and_servo_angle_deltas;//(12,1).  Delta rotational velocities and servo angles needed to reach desired forces and moments.  Not likely possible in next timestep
		Eigen::MatrixXf next_loop_rot_vel_and_servo_angle_deltas;//(12,1);  Delta rotational velocities and servo angles to apply in next loop time

		Eigen::MatrixXf theoretical_fan_forces_and_moments;

		Eigen::MatrixXf end_of_loop_theoretical_fan_forces_and_moments;

		Eigen::MatrixXf targets;

		/*
		Parameters of motor controller

		*/
		struct FAN
		{

			//From APC data at design point.  Not exactly square because of Reynolds and Mach.
			float fan_thrust_coefficient; //Coefficient in Thrust (N) = k * (fan rad per sec)^2 in 10-12k rpm range
			float fan_aero_torque_coefficient; //Coefficient in Torque (Nm) = k * (fan rad per sec)^2 in 10-12k rpm range

			float fan_assembly_moment_of_inertia; //Moment of inertia of fan, coupler, and rotor of motor in kg*m^2


		} ;

		FAN APC_8_45MR;



 		/*
		Parameters of motor controller

		*/
		struct MOTOR
		{

			float effective_impedance; //Hack to determine current through motor.  Determined by input voltage / total current at full throttle
			float torque_constant; //Torque in Nm per net Volt through coils.  Determines slope of torque curve.
			float coil_resistance; //Coil resistance at operating temperature (Ohms)
			float motor_kv; //Motor kv in RPM/Volt.  RPM/Volt used for ease of entry/verification, but always converted to [rad/sec] / volt  (divide by 9.5492966)

		} ;

		MOTOR Turnigy_1400kv;
		MOTOR Turnigy_1500kv;



		/*
		Parameters of motor controller

		*/
		struct MOTOR_CONTROLLER
		{

			float motor_controller_input_exponent; //Motor controller current exponent.  Current = Max current * (pulse fraction) ^ (input exponent)
			float min_pwm; //Minimum PWM that sends current to motor
			float max_pwm; //Maximum PWM, no current restriction
			float P_gain; //P gain applied to rotational velocity error.  Torque = P_gain * rotational velocity error (rad/sec) + Integral term
			float I_gain; //I gain applied to rotational velocity error.  Torque = P term + I_gain + rotational velocity error (radians)
			float integral_discount_per_sec; //Integral term continuously discounted to weight older errors less.  0.01x (per sec) -> 63.1% of initial value after 0.1 sec (0.01 ^ 0.1), 95.5% after 0.01 sec
			float max_pulse_power_fraction; //Fraction of maximum possible motor power applied in steady state at max pulse width 

		} ;

		MOTOR_CONTROLLER BLHeli_35A;
		MOTOR_CONTROLLER Multistar_40A;





		/*
		Parameters of propulsion unit

		*/
		struct PROPULSION_UNIT
		{

			FAN fan; //Parameters of fan
			MOTOR motor; //Parameters of motor hardware
			MOTOR_CONTROLLER motor_controller; //Parameters of motor controller hardware


			float x_loc; //In meters.  Left-right location.  Right is +.
			float y_loc; //In meters.  Front-back location.  Front is +.
			float motor_direction; //CW viewed from the top is 1, CCW is -1
			int gpio_pin; //GPIO pin servicing motor controller
			int servo_num; //Index of servo servicing propulsion unit

			float rotational_velocity; //Fan rotational velocity in rad/sec at the end of the current loop
			float fan_thrust; //Fan thrust at the end of the current loop in Newtons
			float fan_torque; //Fan thrust and beginning of current loop in Nm

			//Calculated at beginning of each loop, after updating motor rotational velocity, with calculate_motor_operating_limits(int fan_number)
			float max_torque_at_current_rot_vel; //Max torque theoretically possible at current rotational velocity (may not be possible to reach because of controller PI loop)
			float max_rotational_velocity; //Rotational velocity reached once in steady state at max pulse length
			float max_rot_vel_thrust; //Maximum steady state thrust in current flight condition
			float max_rot_vel_output_power; //Output power reached once in steady state at max pulse length
			float max_rot_vel_input_power; //Input power reached once in steady state at max pulse length
			float max_rot_vel_efficiency; //Efficiency (output power / input power) reached once in steady state at max pulse length

			float max_positive_torque;  //Max positive torque in Nm possible to send at end of current loop.
			float max_negative_torque;  //Max negative torque in Nm possible to send at end of current loop.

			float max_positive_rot_vel_delta; //Max delta rotational velocity between end of current loop and end of next loop.
			float max_negative_rot_vel_delta; //Note: Should be a negative number.


			float rot_vel_error_integral; //Only update when commands actually sent to motors
			float steady_state_rot_vel_command; //

			float rotational_velocity_delta_required; //Commanded rotational velocity delta in rad/sec between end of current loop and end of next loop
			float motor_torque_required; //Motor torque in Nm required to achieve rotational velocity delta in next loop time
			float PWM_micros; //PWM microseconds sent at end of last control loop

		} ;

		PROPULSION_UNIT motor_1400kv{};
		PROPULSION_UNIT motor_1500kv{};

		PROPULSION_UNIT fr_fan{};
		PROPULSION_UNIT fl_fan{};
		PROPULSION_UNIT br_fan{};
		PROPULSION_UNIT bl_fan{};
		PROPULSION_UNIT bmr_fan{};
		PROPULSION_UNIT bml_fan{};
		PROPULSION_UNIT bfr_fan{};
		PROPULSION_UNIT bfl_fan{};

		PROPULSION_UNIT propulsion_units[8];



		/*
		Powertrain model output data

		*/
		struct POWERTRAIN_MODEL_OUTPUT
		{

			float initial_motor_torque; //Motor torque in Nm generated by pulse width
			float rot_vel_after_loop_time; //Fan rotational velocity after loop time at pulse width

			float steady_state_rotational_velocity; //Steady state rotational velocity that would be achieved with given pulse width



		} ;




		/*
		Parameters of servo unit

		*/
		struct SERVO
		{

			float P_gain; //P gain applied to angle error.  Motor duty cycle = P_gain * angle_error (rad/sec)
			float dc_motor_effective_kv; //DC motor kv when including gearbox [volts per rad/sec].  Supply voltage - dc_motor_effective_kv * max rotational speed at supply voltage = 0
			float acceleration_per_net_volt; //Servo acceleration in rad/sec^2 per net volt when including back EMF

			float min_net_voltage_for_movement; //Minimum net voltage that results in movement.
			float min_abs_velocity_for_movement; //Rad/sec. If the minimum net voltage for movement is not met, some minimum rotational velocity needed to maintain motion.

		} ;

		SERVO Quimat_17kgcm;



		/*
		Operating conditions of servo unit

		*/
		struct SERVO_UNIT
		{

			SERVO servo; //Parameters of servo hardware

			int gpio_pin; //GPIO pin servicing motor controller

			float zero_deg_pwm; //PWM at which servo is directly vertical
			float ninety_deg_pwm; //PWM at which servo is horizontal vertical

			float min_angle_pwm; //PWM at max backwards/upwards angle (no interference).
			float max_angle_pwm; //PWM at max forwards/downwards angle (no interference).  Not necessarily greater than min_angle_pwm (if servo min pwm corresponds to downwards)

			float servo_angle; //Servo angle in rads at time last commands sent.  0 is directly vertical, 90 deg is directly horizontal.
			float servo_rotational_velocity; //Rotational velocity in rad/sec at time last commands sent.


			float max_positive_angle_delta; //Max delta servo angle between end of current loop and end of next loop.
			float max_negative_angle_delta; //Note: Should be a negative number.


			float min_positive_angle_delta; //Min delta servo angle between end of current loop and end of next loop.
			float min_negative_angle_delta; //Note: Should be a negative number.


			float angle_delta_required; //Servo angle delta target between end of current timestep and end of next timestep
			float commanded_servo_acc; //Commanded rotational acceleration in rad/s^2 at time last commands sent
			float PWM_micros = 1040; //PWM microseconds sent at end of last control loop

		} ;

		SERVO_UNIT front_right_servo;
		SERVO_UNIT front_left_servo;
		SERVO_UNIT back_right_servo;
		SERVO_UNIT back_left_servo;

		SERVO_UNIT servo_units[4];



		/*
		Servo model output data

		*/
		struct SERVO_MODEL_OUTPUT
		{

			float initial_acceleration;

			float pos_after_loop_time;
			float rot_vel_after_loop_time; 
			float acc_after_loop_time;


		} ;



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

			//Rotational moments in Nm
			float roll_moment;
			float pitch_moment;
			float yaw_moment;

			//Translational force in Newtons
			float x_force; //Forward is positive
			float z_force; //Upwards is positive

		} ;

		FORCES_AND_MOMENTS Target_Forces_and_Moments;
		FORCES_AND_MOMENTS Theoretical_End_of_Loop_Fan_Forces_and_Moments;



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


		//Location::LOCATION * Current_location_ptr;


	public:

		void set_motor_and_servo_parameters();

		float get_fan_aero_torque(int fan_number, float rotational_velocity);
		float get_fan_thrust(int fan_number, float rotational_velocity);

		void get_end_of_loop_rot_vels_and_servo_angles();
		void get_end_of_loop_fan_rotational_velocity(int fan_number);
		void get_end_of_loop_servo_angle_and_rotational_velocity(int servo_num);

		Eigen::MatrixXf get_end_of_loop_theoretical_fan_forces_and_moments();

		void get_motor_operating_limits();
		void calculate_motor_operating_limits(int fan_number);

		void get_max_rot_vel_and_servo_angle_deltas();
		void get_available_fan_motor_torques(int fan_number);
		void get_max_fan_rotational_velocity_deltas(int fan_number);

		void get_servo_movement_limits(int servo_num);
		void get_min_and_max_servo_angle_deltas(int servo_num);


		POWERTRAIN_MODEL_OUTPUT run_powertrain_model(int fan_number, float PWM_micros);
		SERVO_MODEL_OUTPUT run_servo_model(int servo_num, float PWM_micros);

		Eigen::MatrixXf get_kinematics_derivatives(Eigen::MatrixXf motor_vels_and_servo_angles);
		

		Eigen::MatrixXf calculate_theoretical_fan_forces_and_moments(Eigen::MatrixXf motor_vels_and_servo_angles);
		Eigen::MatrixXf calculate_kinematics_derivatives(Eigen::MatrixXf control_guess);
		//MatrixXf calculate_gradient_descended_control_guess(MatrixXf control_guess, MatrixXf target_forces_and_moments);

		Eigen::MatrixXf get_theoretical_rot_vel_and_servo_angle_deltas(Eigen::MatrixXf target_forces_and_moments);
		void get_next_loop_rot_vel_and_servo_angle_deltas();


		void rot_vel_and_servo_angle_deltas_to_pwms();

		void motor_rot_vel_delta_to_pwm(int fan_number);

		void motor_rot_vel_delta_to_torque_required(int fan_number);
		void motor_torque_to_pwm(int fan_number);


		void servo_angle_delta_to_pwm_required(int servo_num);
		void servo_angle_delta_to_acc_required(int servo_num);
		void servo_acc_required_to_pwm(int servo_num);


		//void acceleration_controller(Location::LOCATION *, Receiver::RECEIVER *);
		Eigen::MatrixXf acceleration_controller();

		Eigen::MatrixXf accelerations_to_forces_and_moments(Eigen::MatrixXf target_accelerations);
		void forces_and_moments_to_thrusts_and_angles(Control::FORCES_AND_MOMENTS *);
		void thrusts_and_angles_to_pwms(Control::THRUSTS_AND_ANGLES *);

		void update_motor_controller_error_integral(int fan_number, float PWM_micros);
		void send_pwms();
		void send_microseconds(int motor_pin, float microseconds);
		void send_XPlane_inputs();

		//void run();
		void run(Location::LOCATION *, Receiver::RECEIVER *);


		//Location::LOCATION test;


//		void MotorTest(Receiver::RECEIVER *);

		//void print_mtxf(const Eigen::MatrixXf& K);

		float Pitch_Rate_P; //How much to accelerate towards target rate.  More than about 0.03 on Deathtrap and starts to shake.
		float Roll_Rate_P; //0.1, with sqrt p = 6, -> 1.8 for 10 deg with no rate (3 and 1 for 10 deg, no rate PID pitch, roll)
		float Yaw_Rate_P;

		bool Aux1_been_up = false;
		bool Aux1_been_down = false;



//		void run2(Location::LOCATION *, Receiver::RECEIVER *);
//		void Log();
//		void MotorTest2(Receiver::RECEIVER *);

};



#endif