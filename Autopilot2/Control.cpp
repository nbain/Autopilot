#include "Control.h"
#include <math.h> //For sqrt() control

float dt;
float l_time = 0;
float desired_yaw = 0;
float yaw_trim = 0;
float master_tilt = 0; //So master_tilt used to be -Receiver->pitch * 90 with no slew limiter 
float desired_tilt = 0;
float max_tilt_rate;

float roll_rate_error_3 = 0;
float roll_rate_error_2 = 0;
float roll_rate_error_1 = 0;

float pitch_rate_error_3 = 0;
float pitch_rate_error_2 = 0;
float pitch_rate_error_1 = 0;

float yaw_rate_error_3 = 0;
float yaw_rate_error_2 = 0;
float yaw_rate_error_1 = 0;

float roll_rate_3 = 0;
float roll_rate_2 = 0;
float roll_rate_1 = 0;

float pitch_rate_3 = 0;
float pitch_rate_2 = 0;
float pitch_rate_1 = 0;

float yaw_rate_3 = 0;
float yaw_rate_2 = 0;
float yaw_rate_1 = 0;

float pitch_rate_error_int = 0;
float roll_rate_error_int = 0;





/*

Command a jerk up to certain alpha limits (prob something like -150 to 150 alpha/sec)
	-Will be state-based
		-Should generate first jerk in series that will work from current a, v, p (will prob always be max)



Physically possible to command -50 to 50 deg/s^2 (alpha)
	-Depending on state, alpha limits will change
	-Can only ramp to alpha at certain rate (also state dependent)

Say can ramp instantly and alpha limits are constant:
	-If at 10 deg right roll and rolling 10 deg/s right, apply alpha = -50
		-0.1 sec: 5 deg/s right, 10.75 deg right
		-0.2 sec: 0 deg/s right, 11 deg right
		-0.3 sec: 5 deg/s left, 10.75 deg right
		-0.4 sec: 10 deg/s left, 10 deg right
		-0.5 sec: 15 deg/s left, 8.75 deg right
		-0.6 sec: 20 deg/s left, 7 deg right
		-0.7 sec: 25 deg/s left, 4.75 deg right
		-0.8 sec: 20 deg/s left, 2.5 deg right
		-0.9 sec: 15 deg/s left, 0.75 deg right
		-1.0 sec: 10 deg/s left, 0.5 deg left
		-1.1 sec: 


Maximum velocity at given position where can still hit zero velocity at zero angle when going max accel:
	Max roll velocity = -2 * 0.5*Max Roll Accel *sqrt(roll angle / (0.5 * Max Roll Accel))
		-Intuition is that this is a just a stretched/scaled sqrt function where max ok velocity increases with root of distance

1.  If below this velocity, command max left roll acceleration until: Actual roll velocity = Max_roll_velocity(max_accel, roll_angle)
		-I.e. any time rolled right and rolling right, or rolled right and rolling slowly enough left

2.  When condition met, max right roll acceleration until at zero angle



*/


/*

Right front moments:

Roll moment = - Right Front Thrust (N) * 0.23m (y dist to CG) * cos(servo angle) + 0.0133 * Right Front Thrust (N) * sin(servo angle)
		[Prop Torque term is positive if spinning CW as viewed from top - will roll right if increase thrust]

Pitch moment = Right Front Thrust (N) * 0.95m (x dist to CG) * cos(servo angle)

Yaw moment = - Right Front Thrust (N) * 0.23m (y dist to CG) * sin(servo angle) - 0.0133 * Right Front Thrust (N) * cos(servo angle)
		[Prop torque term is negative if spinning CW as viewed from top - will yaw CCW if increase thrust]

Z_thrust = Right Front Thrust (N) * cos(servo angle)

X_thrust = Right Front Thrust (N) * sin(servo angle)


Right front fan thrust derivatives (at given servo angle):

	-Roll moment derivative (Nm delta per N thrust): -0.23 Nm * cos(servo angle) + 0.0133 Nm * sin(servo angle)
	-Pitch moment derivative (Nm delta per N thrust): 0.95 Nm * cos(servo angle)
	-Yaw moment derivative (Nm delta per N thrust): - 0.23 Nm * sin(servo angle) - 0.0133 Nm * cos(servo angle)
		-1.003 In-lb /39.3 in. *4.5 N /(1.914 Lbf*4.5N ) = 0.0133 Nm torque per N thrust at 11k rpm for 8*45MR(very close to linear across rpm)


Right front servo angle derivatives (at current angle and given thrust level):
	-Roll moment derivative: (Nm delta per rad servo movement): Thrust (N) * 0.23m * sin(servo angle) + 0.0133 * Thrust(N) * cos(servo angle)
	-Pitch moment derivative: -Thrust (N) * 0.95m * sin(servo angle)
	-Yaw moment derivative: -Thrust (N) * 0.23m * cos(servo angle) + 0.0133 * Thrust (N) * sin(servo angle)

Actual_actuations = [Right_front_thrust   Right_front_angle  Left_front_thrust  etc.]

Actual_moments = [Actual_roll_moment   Actual_pitch_moment   Actual_yaw_moment]
Target_moments = [Target_roll_moment   Target_pitch_moment   Target_yaw_moment]

Deltas_from_target_moment = [ Target_roll_moment - Actual_roll_moment    etc. ]


Moment derivatives (function of all servo angles, thrust levels.  In Nm per N thrust or Nm per rad servo angle) = 

[Front right fan roll derivative    Front right fan pitch derivative   Front right fan yaw derivative
Front right servo roll derivative    Front right servo pitch derivative   Front right servo yaw derivative
Front left fan roll derivative       etc.
etc.
]

=
In hover:
[-0.23  0.95  -0.0133;  (front right fan)
 0.133  0     -2.3;     (front right servo)
 0.23   0.95  0.0133;   (front left fan)
 -0.133 0     2.3;      (front left servo)
 23
 23
 23
 23
 23
 23
 23 
 0.35   0     6.1;     (back left servo)
 ]

 Control_deltas = [Right_front_delta_thrust  Right_front_delta_angle  Left_front_delta_thrust  etc.]


Moment_derivatives * Control_deltas = Deltas_from_target_moment

->

Control_deltas = Deltas_from_target_moment / Moment derivatives

Control = Actual_actuations + Control_deltas



-Transmitter controls z_thrust/x_thrust diagonal
-Dial 1 controls direction of z_thrust/x_thrust
-Others control angle targets

Max accelerations hard-coded (to maybe ~2 rad/s^2 for all axes to start)

	Always apply max acceleration in direction need to go until going too fast back towards target
	Max roll velocity = -2 * 0.5*Max Roll Accel *sqrt(roll angle / (0.5 * Max Roll Accel))


*/


/*

Fit to APC Prop data (and scaled from In-Lbs to N-m):
Torque (Nm) = (RPM / 32130)^2
https://www.desmos.com/calculator/l8om6idpub

1 rad/s = 9.549 RPM -> RPM = rad/s * 9.549
Aero Torque (Nm) = ([Rad/s] / 3365)^2

Thrust (Lbs) = (RPM / 7950)^2 (on same Desmos graph)
Thrust (N) = (sqrt(4.448) * [Rad/s] * 9.549 / 7950)^2 = ([rad/s]/395)^2
	With inflow correction:
	Thrust(N) = ([rad/s]/395)^2 - (1/94.56) * [rad/s]^2


8 * 4.5 MR Propeller weight (without shaft adapter/coupler ring): 8.78 grams
	-If weight uniformly distributed across 8 inches (0.203 m.): (may slightly less since seems like more weight towards center, but probably close)
		->0.00003015 kg*m^2 (1/12 * m * L^2) (30.15 kg*mm^2)

Collet and adapter ring: 3.78 grams, 10mm diameter
	-If weight uniformly distributed as a 10mm diameter cylinder (seems ok since cut out in middle, but also tapering in at top)
		-> 0.00000004725 kg*m^2 (1/2 * m * r^2) (0.04725 kg*mm^2)

D2826-1400kv: 57.40 grams (with wires and male bullet connectors and bolted-in cross mount on  bottom)
	Rotor on taken apart motor (same diameter - 27.8mm): 33.46 grams
		-Cylindrical section length: 22.8 mm
	-15.0mm cylindrical section length
		-> Likely 15.0/22.8 * larger rotor (or slightly more since some overhead from aluminum going to center)
				-> would be 22.01 grams
		-If 23 grams and modeled as a shell with radius = 12mm (about at inside surface of magnets):
			-> 0.000003312 kg*m^2 (m * r^2) (3.312 kg*mm^2)

		Prob ~34 kg*mm^2 total inertia


	In-Lbs = (RPM / 11000) ^2
	Rad/sec = RPM / 9.5493
	Nm = In-Lbs / 8.8507

	Nm * 8.8507 = (rads * 9.5493 / 11000) ^ 2

	Nm = 1/8.8507 * (rads / 1,151.91689) ^ 2

	Nm = (rads / 3,426.9672) ^ 2

Allows calculation of motor velocity at every time step if motor torque known:

	Motor acceleration (Rad/s^2) = (Motor torque [Nm] - Aero torque [Nm]) / Motor and prop moment of inertia [kg*m^2]

	motor_acceleration [rad/s^2] = (motor_torque [Nm] - ((motor_velocity [rad/s] / 3365)^2 ))  /  Motor and prop moment of inertia [kg*m^2]


Steady_state_commanded_rad_per_sec  = 4.448 * ([PWM - 1050] / 590) ^ 1.61

Commanded_thrust_increase_per_sec [N/sec] = 

Excess_motor_torque [Nm] = 

14 magnets -> 7 rotor poles/12 stator teeth -> 6 stator poles



Steady-state motor torque as a function of pwm (may not just be steady state):
	Thrust (N) = 4.448 * [((PWM - 1020)/ 590) ^ 1.61]
	Aero Torque (Nm) = 0.01378 * 4.448 * [((PWM - 1020)/ 590) ^ 1.61]
		-PWM = 1950 us -> 0.1275 Nm
			-3,750 rad/s^2 at 34 kg*mm^2 inertia (35,800 RPM / sec)

Net Voltage (V) = Batt_voltage - Motor_kv * RPM
	-1928 us at 2.00 lbs at 12.6 V (1400 kv)
	-11,240 RPM from APC data (at 2.00 lbs)
	-8.028 V back EMF at 11,240 RPM (1400 kv)
	-4.572 V net (12.6 V batt)
		-> 59.37 Amps at 77 mOhm if let get to steady state
		-> 304,800 Amps/sec at 15 uH
	-0.1224 Nm (11,240 RPM or 2.00 lbs)
		-> 0.0267 Nm/net V is max PWM (property of motor/motor controller?)

	If true:
		-1500 us -> 0.717 lbs (experimental from load cell at 12.6V) (52.8% of full pulse width)
		-6,730 RPM (APC data at 0.717 lbs)
		-4.807 back EMF at 6,730 RPM
		-7.793 V net (12.6 V batt) 
		-0.0439 Nm (0.717 lbs)
			-> 0.00563 Nm/net V (0.2108x 1928 us)



Good documentation on hobby-grade motor controllers:
https://www.ti.com/lit/an/slyt692/slyt692.pdf?ts=1615218851514

Inductance meter shows:
	-14-15 uH on each coil
	-77-78 mOhm on each coil -> 151.2 amps if left in steady state at 11.8 V
	-> 15/1,000,000 = Batt_voltage / (Coil current / s)
		-> 786,666 Amps/sec -> 0.048 ms to go from -19A to 19A


1400 kv motor, VESC calibration:
	-12.6 volts on power supply, on load cell:
		-Motor current: 17.00 A
		-Motor R: 34.60 mOhms
		-Motor L: 3.36 uH
		-Motor Flux Linkage: 0.53 mWb

	-11.83 volts on batt on plane, motor P2:
		-Motor current: 17.22 A
		-Motor R: 33.70 mOhms
		-Motor L: 3.01 uH
		-Motor flux linkage: 0.54 mWb

		-> 1400 kv -> 0.011798 Nm per Amp ideal -> 10.1 amps ideal at max rot vel, 20.2 amps in reality (motor efficiency * motor controller efficiency = 0.50)
			kv / 118,664 = kt (Nm per Amp)

1500 kv motor, VESC calibration:
	-11.83 volts on batt on plane, front right motor:
		-Motor current: 20.73 A
		-Motor R: 23.30 mOhms
		-Motor L: 2.15 uH
		-Motor Flux Linkage: 0.48 mWb

		-> 1500 kv -> 0.011012 Nm per Amp ideal

-Confirmed on oscilloscope: 1400 rpm -> 1 V peaks for line-line voltage 
	-Rotor poles = Number of rotor magnets / 2
	-Electrical RPM = Mechanical RPM * rotor poles

	For delta-wired windings:
		Line voltage = phase voltage
		Line current = 1.732 * phase current

		0.00185 Nm per Amp for 7,000 kv
		0.4 amps -> 0 Nm (= 0.00074 Nm)

		3700 kv motor:
			7V: (9500 rpm, 0.004 Nm), (25000 rpm, 0.00022 Nm)
				-> 0.24 mNm per k rpm
				-25,900 rpm theoretical at zero-torque
				-25,920 actual (just bc estimating end points) - probably right at 25,900 n reality

				-6.28 mNm at zero rpm at 7V

			5 V: (18,500 rpm, 0 Nm), (10,000 rpm, 0.0021 Nm)
				-> 0.247 mNm per k rpm
				-> 4.57 mNm at 0 rpm


			4V: (6200 rpm, 0.002 Nm), (12500 rpm, 0.0004 Nm)
				-> 0.254 mNm per k rpm, 14,075 rpm est. zero torque
				-14,800 rpm zero torque theoretical (would be 0.2325 mNm per k rpm) -> 3.441 mNm at zero rpm

			Conclusions:
			-> Bottom point of torque-curve determined by kv and voltage only
			-> Max Nm per k rpm slope is constant for given motor
				-> Start of torque curve determined by some Nm per volt constant for given motor

*/








/*
Fill in the values for the motor and servo structs.  Do once at the beginning.  As far as understand, must be done in a function rather
than initialized in the .h file (ok if only one variable that has specific struct form, but not if multiple - bc doesn't make sense to
 initialize two values for a struct)

*/
void Control::set_motor_and_servo_parameters()
	{


	APC_8_45MR.fan_thrust_coefficient = 430;
	APC_8_45MR.fan_aero_torque_coefficient = 3757;
	APC_8_45MR.fan_assembly_moment_of_inertia = 0.000034;


	Turnigy_1400kv.effective_impedance = 0.7;
	Turnigy_1400kv.torque_constant = 0.03143;
	Turnigy_1400kv.coil_resistance = 0.083;
	Turnigy_1400kv.motor_kv = 1400;


	Turnigy_1500kv.effective_impedance = 0.44;
	Turnigy_1500kv.torque_constant = 0.045;
	Turnigy_1500kv.coil_resistance = 0.0485; //44.9-45.1 mOhms at 20 deg C
	Turnigy_1500kv.motor_kv = 1500;


	BLHeli_35A.motor_controller_input_exponent = 2.35;
	BLHeli_35A.P_gain = 0.0015;
	BLHeli_35A.I_gain = 0.017;
	BLHeli_35A.min_pwm = 1040;
	BLHeli_35A.max_pwm = 1960;
	BLHeli_35A.integral_discount_per_sec = 0.01;
	BLHeli_35A.max_pulse_power_fraction = 0.94;


	Multistar_40A.motor_controller_input_exponent = 2.35;
	Multistar_40A.P_gain = 0.001;
	Multistar_40A.I_gain = 0.015;
	Multistar_40A.min_pwm = 1040;
	Multistar_40A.max_pwm = 1960;
	Multistar_40A.integral_discount_per_sec = 0.01;
	Multistar_40A.max_pulse_power_fraction = 0.94;


	Quimat_17kgcm.P_gain = 2.5;
	Quimat_17kgcm.dc_motor_effective_kv = 1.48;
	Quimat_17kgcm.acceleration_per_net_volt = 17;
	Quimat_17kgcm.min_net_voltage_for_movement = 0.81;
	Quimat_17kgcm.min_abs_velocity_for_movement = 0.08;



	//Fan hardware for all propulsion units
	fr_fan.fan = APC_8_45MR;
	fl_fan.fan = APC_8_45MR;
	br_fan.fan = APC_8_45MR;
	bl_fan.fan = APC_8_45MR;
	bmr_fan.fan = APC_8_45MR;
	bml_fan.fan = APC_8_45MR;
	bfr_fan.fan = APC_8_45MR;
	bfl_fan.fan = APC_8_45MR;


	//Motor hardware for all propulsion units
	fr_fan.motor = Turnigy_1500kv;
	fl_fan.motor = Turnigy_1500kv;
	br_fan.motor = Turnigy_1400kv;
	bl_fan.motor = Turnigy_1400kv;
	bmr_fan.motor = Turnigy_1400kv;
	bml_fan.motor = Turnigy_1400kv;
	bfr_fan.motor = Turnigy_1500kv;
	bfl_fan.motor = Turnigy_1500kv;


	//Motor controller for all propulsion units
	fr_fan.motor_controller = Multistar_40A;
	fl_fan.motor_controller = Multistar_40A;
	br_fan.motor_controller = BLHeli_35A;
	bl_fan.motor_controller = BLHeli_35A;
	bmr_fan.motor_controller = BLHeli_35A;
	bml_fan.motor_controller = BLHeli_35A;
	bfr_fan.motor_controller = Multistar_40A;
	bfl_fan.motor_controller = Multistar_40A;



	//In meters.  Left-right location.  Right is +.
	fr_fan.x_loc = 0.23;
	fl_fan.x_loc = -0.23;
	br_fan.x_loc = 0.18;
	bl_fan.x_loc = -0.18;
	bmr_fan.x_loc = 0.4;
	bml_fan.x_loc = -0.4;
	bfr_fan.x_loc = 0.62;
	bfl_fan.x_loc = -0.62;


	 //In meters.  Front-back location.  Front is +.
	fr_fan.y_loc = 0.99;
	fl_fan.y_loc = 0.99;
	br_fan.y_loc = -0.35;
	bl_fan.y_loc = -0.35;
	bmr_fan.y_loc = -0.35;
	bml_fan.y_loc = -0.35;
	bfr_fan.y_loc = -0.35;
	bfl_fan.y_loc = -0.35;


	//In meters.  Front-back location.  Front is +.
	fr_fan.motor_direction = 1;
	fl_fan.motor_direction = -1;
	br_fan.motor_direction = 1;
	bl_fan.motor_direction = -1;
	bmr_fan.motor_direction = -1;
	bml_fan.motor_direction = 1;
	bfr_fan.motor_direction = -1;
	bfl_fan.motor_direction = 1;


	//GPIO pin servicing propulsion unit
	fr_fan.gpio_pin = 6;
	fl_fan.gpio_pin = 7;
	br_fan.gpio_pin = 8;
	bl_fan.gpio_pin = 9;
	bmr_fan.gpio_pin = 10;
	bml_fan.gpio_pin = 11;
	bfr_fan.gpio_pin = 12;
	bfl_fan.gpio_pin = 13;

	//Servo servicing propulsion unit
	fr_fan.servo_num = 1;
	fl_fan.servo_num = 2;
	br_fan.servo_num = 3;
	bl_fan.servo_num = 4;
	bmr_fan.servo_num = 3;
	bml_fan.servo_num = 4;
	bfr_fan.servo_num = 3;
	bfl_fan.servo_num = 4;


	//0.4166 sec 0-90 deg from first movement starting from stop -> 2,920 deg/sec^2 = 50.96 rad/sec^2 (0.082 sec to max, moves 9.8 degrees during this acceleration)
	//0.375 sec 0-90 deg starting from behind and going full speed through -> 4.19 rad/sec (5.05 rad/sec if 83.05% of zero net voltage kv -> 0.813 rad/sec effective per volt)
	//0.021 sec from command sent to first movement (extremely slight)
	//0.208 from signal sent to movement stop after 800 us->850 us movement -> 430 deg/s^2 = 7.51 rad/sec^2 (0.078 rads moved, 0.375 rad/sec avg, 0.75 rad/sec max)
	//800us->834 us delta is smallest that gets all servos to move (about 0.055 rads)
	//Servo taken apart (no mounting point for bar on other side) audibly decelerates when within 45 degrees of target
	//Motor supply voltage = 6.21 V
	front_right_servo.servo = Quimat_17kgcm;
	front_left_servo.servo = Quimat_17kgcm;
	back_right_servo.servo = Quimat_17kgcm;
	back_left_servo.servo = Quimat_17kgcm;

	front_right_servo.gpio_pin = 2;	
	front_left_servo.gpio_pin = 3;
	back_right_servo.gpio_pin = 4;
	back_left_servo.gpio_pin = 5;

	front_right_servo.zero_deg_pwm = 850;
	front_left_servo.zero_deg_pwm = 810;
	back_right_servo.zero_deg_pwm = 850;
	back_left_servo.zero_deg_pwm = 950;

	front_right_servo.ninety_deg_pwm = 1830;	
	front_left_servo.ninety_deg_pwm = 1860;
	back_right_servo.ninety_deg_pwm = 1820;
	back_left_servo.ninety_deg_pwm = 1930;

	front_right_servo.min_angle_pwm = 400;	
	front_left_servo.min_angle_pwm = 400;
	back_right_servo.min_angle_pwm = 400;
	back_left_servo.min_angle_pwm = 400;

	front_right_servo.max_angle_pwm = 2400;	
	front_left_servo.max_angle_pwm = 2400;
	back_right_servo.max_angle_pwm = 2400;
	back_left_servo.max_angle_pwm = 2400;


	propulsion_units[0] = fr_fan;
	propulsion_units[1] = fl_fan;
	propulsion_units[2] = br_fan;
	propulsion_units[3] = bl_fan;
	propulsion_units[4] = bmr_fan;
	propulsion_units[5] = bml_fan;
	propulsion_units[6] = bfr_fan;
	propulsion_units[7] = bfl_fan;

	servo_units[0] = front_right_servo;
	servo_units[1] = front_left_servo;
	servo_units[2] = back_right_servo;
	servo_units[3] = back_left_servo;

	Serial.println("MOTOR AND SERVO PARAMETERS SET");


	}






/*
	Input: fan parameters, current rotational velocity (rad/sec), inflow velocity (m/s), inflow angle (radians from direct inflow)
	Output: aero torque from fan in Nm.

*/
float Control::get_fan_aero_torque(int fan_number, float rotational_velocity)
	{

		float fan_torque_coeff = propulsion_units[fan_number].fan.fan_aero_torque_coefficient;

		//Incompressible aero torque in Nm
		float incompressible_fan_aero_torque = 1 / (std::pow(fan_torque_coeff, 2)) * std::pow(rotational_velocity, 2);

		float compressible_fan_aero_torque = 0; //0 for now

		float total_fan_aero_torque = incompressible_fan_aero_torque + compressible_fan_aero_torque;

		return total_fan_aero_torque;
	}





/*
	Input: fan parameters, current rotational velocity (rad/sec), inflow velocity (m/s), inflow angle (radians from direct inflow)
	Output: Thrust from fan in Newtons

*/
float Control::get_fan_thrust(int fan_number, float rotational_velocity)
	{

		//Current Thrust (N) = ([current rad/s] / 395)^2 - (1/94.56 = 0.010575)*(m/s inflow)^2

		float fan_thrust_coeff = propulsion_units[fan_number].fan.fan_thrust_coefficient;

		//Zero-inflow velocity thrust (N)
		float zero_inflow_fan_thrust = 1 / (std::pow(fan_thrust_coeff, 2)) * std::pow(rotational_velocity, 2);

		float inflow_thrust_delta = - 0.010575 * std::pow(inflow_velocity,2);

		float total_fan_thrust = zero_inflow_fan_thrust + inflow_thrust_delta;

		return total_fan_thrust;

}




/*
	Input: current fan rotational velocities, last commanded fan torques sent, last servo angles, last servo velocities and commanded servo accelerations
	Output: None.  Updated fan rotational velocities and servo angles and rotational velocities at the end of the current loop (not changeable)

	No real-world feedback.  Entirely solved by integrating commanded fan torques and commanded pulse widths at end of last loop
	Starting at 0 position and velocity.

*/
void Control::get_end_of_loop_rot_vels_and_servo_angles()
	{

		for (int i=0; i<8; i++)
    	{

    		/*
    	
    		POWERTRAIN_MODEL_OUTPUT powertrain_model_output = run_powertrain_model(i, propulsion_units[i].PWM_micros);

    		
			//Step 1: Update error integral in simulated motor controller
			//Update variable to make accessible throughout loop
			propulsion_units[i].steady_state_rot_vel_command = powertrain_model_output.steady_state_rotational_velocity;

			//Calculate rotational velocity error.  Should be done before updating rotational velocity
			float rot_vel_error = propulsion_units[i].steady_state_rot_vel_command - propulsion_units[i].rotational_velocity;

			//Update error integral
			//Integral gets discounted by certain percentage per time (99% every 0.1 seconds -> 0.904x per second.  0.904 ^ (loop_time))
			float integral_discount = std::pow(propulsion_units[i].motor_controller.integral_discount_per_sec, loop_time);
			propulsion_units[i].rot_vel_error_integral = propulsion_units[i].rot_vel_error_integral * integral_discount + rot_vel_error * loop_time;

			//Prevent integral from dragging down target when already below
			if (rot_vel_error > 0 && propulsion_units[i].rot_vel_error_integral < 0){
				propulsion_units[i].rot_vel_error_integral = 0;
			}

			//Step 2: Update rotational velocity
			propulsion_units[i].rotational_velocity = powertrain_model_output.rot_vel_after_loop_time;

*/

///*

    		if (i==2){


    			test_pwm = 1040 + 920 * test_input;

    			//Serial.print(" OK ");
    			Serial.println(test_pwm);


    			POWERTRAIN_MODEL_OUTPUT powertrain_model_output = run_powertrain_model(i, test_pwm);

    			//Step 1: Update error integral in simulated motor controller
    			//Update variable to make accessible throughout loop
    			propulsion_units[i].steady_state_rot_vel_command = powertrain_model_output.steady_state_rotational_velocity;

    			//Calculate rotational velocity error.  Should be done before updating rotational velocity
    			float rot_vel_error = propulsion_units[i].steady_state_rot_vel_command - propulsion_units[i].rotational_velocity;

    			//Update error integral
    			//Integral gets discounted by certain percentage per time (99% every 0.1 seconds -> 0.904x per second.  0.904 ^ (loop_time))
    			float integral_discount = std::pow(propulsion_units[i].motor_controller.integral_discount_per_sec, loop_time);
				propulsion_units[i].rot_vel_error_integral = propulsion_units[i].rot_vel_error_integral * integral_discount + rot_vel_error * loop_time;

				//Prevent integral from dragging down target when already below
				if (rot_vel_error > 0 && propulsion_units[i].rot_vel_error_integral < 0){
					propulsion_units[i].rot_vel_error_integral = 0;
				}


				//Step 2: Update rotational velocity
    			propulsion_units[i].rotational_velocity = powertrain_model_output.rot_vel_after_loop_time;


    			float thrust = get_fan_thrust(i, propulsion_units[i].rotational_velocity) / 4.448;
    			//Serial.println(" "); 
    			Serial.println(thrust);
    			//Serial.print(" ");
    			//Serial.print(millis() % 10000); Serial.print(" ");

    			POWERTRAIN_MODEL_OUTPUT powertrain_model_output2 = run_powertrain_model(i, 1040);
    			float rot_vel_delta2 = powertrain_model_output2.rot_vel_after_loop_time - propulsion_units[i].rotational_velocity;

    			POWERTRAIN_MODEL_OUTPUT powertrain_model_output3 = run_powertrain_model(i, 1250);
    			float rot_vel_delta3 = powertrain_model_output3.rot_vel_after_loop_time - propulsion_units[i].rotational_velocity;


    			POWERTRAIN_MODEL_OUTPUT powertrain_model_output4 = run_powertrain_model(i, 1500);
    			float rot_vel_delta4 = powertrain_model_output4.rot_vel_after_loop_time - propulsion_units[i].rotational_velocity;


    			POWERTRAIN_MODEL_OUTPUT powertrain_model_output5 = run_powertrain_model(i, 1750);
    			float rot_vel_delta5 = powertrain_model_output5.rot_vel_after_loop_time - propulsion_units[i].rotational_velocity;

    			POWERTRAIN_MODEL_OUTPUT powertrain_model_output6 = run_powertrain_model(i, 1960);
    			float rot_vel_delta6 = powertrain_model_output6.rot_vel_after_loop_time - propulsion_units[i].rotational_velocity;


    			Serial.print(propulsion_units[i].rotational_velocity); Serial.print(" ");
    			Serial.print(rot_vel_delta2); Serial.print(" ");
    			Serial.print(rot_vel_delta3); Serial.print(" ");
    			Serial.print(rot_vel_delta4); Serial.print(" ");
    			Serial.print(rot_vel_delta5); Serial.print(" ");
    			Serial.print(rot_vel_delta6); Serial.print(" ");


    		}
//*/


		}

		for (int i=0; i<4; i++)
    	{

    		SERVO_MODEL_OUTPUT servo_model_output = run_servo_model(i, servo_units[i].PWM_micros);

			servo_units[i].servo_angle = servo_model_output.pos_after_loop_time;
			servo_units[i].servo_rotational_velocity = servo_model_output.rot_vel_after_loop_time;

		}

/*
		Serial.println("");
		Serial.println("End of loop rotational velocities:");

		for (int i=0; i<8; i++)
    	{
    		Serial.print(i + 1); Serial.print(" ");
    		Serial.print(propulsion_units[i].rotational_velocity); Serial.println(" ");
		}


		Serial.println("");
		Serial.println("End of loop servo angles, velocities, last commanded acc, next timestep target delta:");

		for (int i=0; i<4; i++)
    	{
    		Serial.print(i + 1); Serial.print(" ");
    		Serial.print(servo_units[i].servo_angle); Serial.print(" ");
    		Serial.print(servo_units[i].servo_rotational_velocity); Serial.print(" ");
    		Serial.print(servo_units[i].commanded_servo_acc); Serial.print(" ");
    		Serial.print(servo_units[i].servo_angle_delta); Serial.println(" ");
    		
		}

		Serial.println("");
*/

	}





/*
	Input: fan number -> fan rotational velocity at end of last timestep, last fan torque commanded, last loop time elapsed (as proxy for coming loop time)
	Output: None.

	Gets rotational velocity that should be reached by the fan at the end of this loop, at the time the commands are sent.  Not possible to change.
	Initialized at 0 rotational velocity.

	BLHeli Motor Controller:
		-Seems to command a rotor excess torque (i.e. acceleration rather than a torque)
			-Thrust transient looks the same going down as going up - torque command cannot be only dependent on rot vel error

		-Seems to have integral term
			-Continues increasing thrust even when thrust command is negative after a high thrust command

		-Integral term seems to be added to differently during positive deltas than negative deltas
			-Overshoot going to higher thrust, never overshoots going to lower thrust -> integral term may have floor at zero

		-Acceleration probably a fraction of available acceleration rather than absolute
			-Same Newtons/sec thrust increase at low and high RPMs

		-Max power is probably not max duty cycle on MOSFETs
			-Some overshoot going to max thrust

		-Acceleration is a function of error magnitude
			-Constant transient time


		Works when:
			-Just always gets to new set point in 0.25 seconds

*/
Control::POWERTRAIN_MODEL_OUTPUT Control::run_powertrain_model(int fan_number, float PWM_micros)
	{


		//Makes values readable
		float min_pwm = propulsion_units[fan_number].motor_controller.min_pwm;
		float max_pwm = propulsion_units[fan_number].motor_controller.max_pwm;
		float fan_torque_coeff = propulsion_units[fan_number].fan.fan_aero_torque_coefficient;
		float current_rot_vel = propulsion_units[fan_number].rotational_velocity;

		float max_rot_vel_efficiency = propulsion_units[fan_number].max_rot_vel_efficiency;
		float max_rot_vel_input_power = propulsion_units[fan_number].max_rot_vel_input_power;

		float P_gain = propulsion_units[fan_number].motor_controller.P_gain;
		float I_gain = propulsion_units[fan_number].motor_controller.I_gain;
		float rot_vel_error_integral = propulsion_units[fan_number].rot_vel_error_integral;



		//Step 1: Find steady state conditions that would be generated by pulse width

		//Pulse fraction of total
		float pulse_fraction = (PWM_micros - min_pwm) / (max_pwm - min_pwm);

		//Max current or power fraction = (pulse fraction) ^ 2.35
		float input_exponent = propulsion_units[fan_number].motor_controller.motor_controller_input_exponent;

		//Apply input exponent and scale down to represent less than full current at max cmd
		float max_power_input_fraction = std::pow(pulse_fraction, input_exponent) * propulsion_units[fan_number].motor_controller.max_pulse_power_fraction;

		float steady_state_input_power = max_power_input_fraction * max_rot_vel_input_power;
		float steady_state_output_power = steady_state_input_power * max_rot_vel_efficiency;

		//If Torque = (1/k)^2 * (rot_vel)^2, Power = (1/k)^2 * (rot_vel)^3 -> rot_vel = (Power * k^2) ^ (1/3)
		//As torque model becomes higher fidelity, possible to use as initial guess
		float steady_state_rot_vel = std::pow(steady_state_output_power * fan_torque_coeff * fan_torque_coeff, (1.0/3.0));

		float rot_vel_error = steady_state_rot_vel - current_rot_vel;



		//Step 2: Find available acceleration at current RPM (when accelerating against aero torque)
		//Would correspond to duty cycle = 100%
		float initial_aero_torque = get_fan_aero_torque(fan_number, current_rot_vel);

		float max_pos_excess_torque = propulsion_units[fan_number].max_torque_at_current_rot_vel - initial_aero_torque;

		float max_pos_acceleration = max_pos_excess_torque / propulsion_units[fan_number].fan.fan_assembly_moment_of_inertia;


		//Step 3: Find acceleration as function of P,I gains and rotational velocity error

		//Update error integral
		//Integral gets discounted by certain percentage per time (1% every second -> 63% after 0.1 sec, 95.5% after 0.01 sec)
		float integral_discount = std::pow(propulsion_units[fan_number].motor_controller.integral_discount_per_sec, loop_time);
		rot_vel_error_integral = propulsion_units[fan_number].rot_vel_error_integral * integral_discount + rot_vel_error * loop_time;

		//Prevent integral from dragging down target when already below
		if (rot_vel_error > 0 && rot_vel_error_integral < 0){
			rot_vel_error_integral = 0;
		}

		

		//Not exactly duty cycle because acceleration doesn't correspond to torque (aero torque)
		float acc_fraction_cmd = P_gain * rot_vel_error + I_gain * rot_vel_error_integral;


		if (std::abs(acc_fraction_cmd) > 1){
			acc_fraction_cmd = copysignf(1, acc_fraction_cmd);
		}

		float initial_rot_acc = acc_fraction_cmd * max_pos_acceleration;

		//Possible to use differential equation to model evolution
		float rot_vel_after_loop_time = current_rot_vel + initial_rot_acc * loop_time;

		//Prevent from going to negative velocity
		if (rot_vel_after_loop_time < 0){
			rot_vel_after_loop_time = 0;
		}

		
		POWERTRAIN_MODEL_OUTPUT powertrain_model_output;

		//powertrain_model_output.initial_motor_torque = initial_motor_torque;
		powertrain_model_output.rot_vel_after_loop_time = rot_vel_after_loop_time;
		powertrain_model_output.steady_state_rotational_velocity = steady_state_rot_vel;


		//Serial.print(pulse_fraction); Serial.print(" ");
		//Serial.print(max_pos_acceleration); Serial.print(" ");
		//Serial.print(acc_fraction_cmd); Serial.print(" ");
		//Serial.print(max_acc_fraction); Serial.print(" ");

/*
		Serial.print("Steady State: ");
		Serial.print(steady_state_rot_vel); Serial.print(" ");
		Serial.print(current_rot_vel); Serial.print(" ");
		Serial.print(steady_state_input_power); Serial.print(" ");
		Serial.print(steady_state_output_power); Serial.print(" ");
		Serial.print(max_rot_vel_efficiency); Serial.print(" ");

		//Serial.print(initial_motor_torque_fraction); Serial.print(" ");
		Serial.print(P_gain * rot_vel_error); Serial.print(" ");
		Serial.print(I_gain * rot_vel_error_integral); Serial.print(" ");

		float thrust = get_fan_thrust(fan_number,steady_state_rot_vel)/4.448;
		Serial.print(thrust); Serial.print(" ");
		Serial.print(rot_vel_error_integral); Serial.print(" ");

		Serial.print(" Powertrain model: ");
		Serial.print(pulse_fraction); Serial.print(" ");
		Serial.print(max_power_input_fraction); Serial.print(" ");

		Serial.print(" Torque: ");
		//Serial.print(initial_motor_torque, 4); Serial.print(" ");
		Serial.print(propulsion_units[fan_number].max_torque_at_current_rot_vel, 4); Serial.print(" ");

		
		Serial.print(rot_vel_error); Serial.print(" ");
		Serial.print(rot_vel_error_integral); Serial.print(" ");

		Serial.print(initial_aero_torque, 4); Serial.print(" ");
		Serial.print(initial_rot_acc); Serial.print(" ");
		Serial.print(rot_vel_after_loop_time); Serial.print(" ");
		Serial.print(" ");
*/

/*
		Serial.println("Powertrain model constants:");
		Serial.print(current_rot_vel);  Serial.print(" ");
		Serial.print(P_gain); Serial.print(" ");
		Serial.print(I_gain);  Serial.print(" ");
		Serial.print(rot_vel_error_integral);  Serial.print(" ");
		Serial.print(acc_per_net_volt);  Serial.print(" ");
*/

		return powertrain_model_output;


	}







/*
	Input: motor parameters, current rad/sec, current torque, battery voltage
	Output: None.

	Calculate:
		-Max theoretical input torque (for motor only - motor controller might not allow)
		-Max steady state rotational velocity
		-Max steady state output power
		-Max steady state input power
		-Efficiency at max steady state output power

*/
void Control::calculate_motor_operating_limits(int fan_number)
	{

		float current_rot_vel = propulsion_units[fan_number].rotational_velocity;
		float torque_constant = propulsion_units[fan_number].motor.torque_constant;
		float effective_impedance = propulsion_units[fan_number].motor.effective_impedance;
		float motor_kv_rad_per_sec = propulsion_units[fan_number].motor.motor_kv / 9.5492966;

		//Calculate operational limits at zero rotational velocity
		float zero_speed_max_current = batt_voltage / propulsion_units[fan_number].motor.coil_resistance;
		//float zero_speed_torque = propulsion_units[fan_number].motor.torque_constant * zero_speed_max_current;

		float zero_speed_torque = batt_voltage * propulsion_units[fan_number].motor.torque_constant;

		//Calculate theoretical zero torque rotational velocity
		//float zero_net_voltage_rot_vel = batt_voltage * motor_kv_rad_per_sec;
		//float theoretical_zero_torque_rot_vel = zero_net_voltage_rot_vel * propulsion_units[fan_number].motor.motor_zero_torque_rot_vel_fraction;

		//
		float zero_torque_rot_vel = batt_voltage * motor_kv_rad_per_sec;

		//Calculate max torque theoretically possible at current rotational velocity (may not be possible to reach because of controller PI loop)
		//float max_torque_at_current_rot_vel = zero_speed_torque * (1 - current_rot_vel / theoretical_zero_torque_rot_vel);


		float net_voltage_at_current_rot_vel = batt_voltage - current_rot_vel / motor_kv_rad_per_sec;

		float max_torque_at_current_rot_vel = net_voltage_at_current_rot_vel * torque_constant;

		/*
		Find at motor's max rotational velocity at given operating point.

		Max RPM occurs where:
		Aero torque(RPM) = Max motor output torque(RPM)
		Can do analytically when aero torque is a quadratic, but prob want to gradient descend and converge on equal point
		([rad/sec] / 3427)^2  = 0.582 - 0.582 / 1,534.1 * [rad/sec] -> need quadratic equation to solve
		 a = (1/3427^2),  b = 0.582 / 1,534.1,  c = -0.582
		For 1400kv motor, 0 m/s inflow: Finds 1,207 rad/sec -> 2.0994 lbs thrust max, exactly right
		 [rad/sec] = [-b + sqrt(b^2 - 4*a*c) ] / s
		*/

		//a = 0.000000085147
		//b = 0.00037937
		//c = -0.582
		float a = 1 / std::pow(propulsion_units[fan_number].fan.fan_aero_torque_coefficient, 2);
		float b = zero_speed_torque / zero_torque_rot_vel;
		float c = -zero_speed_torque;



		//Max rotational velocity, where max output torque at speed equal to aero torque at speed
		float max_rot_vel = (-b + sqrt(b*b - 4*a*c)) / (2*a);



		float max_rot_vel_torque = get_fan_aero_torque(fan_number, max_rot_vel);

		float max_rot_vel_output_power = max_rot_vel * max_rot_vel_torque;

		//Net voltage through coils at maximum rotational velocity
		float max_rot_vel_net_voltage = batt_voltage - max_rot_vel / motor_kv_rad_per_sec;

		/*
		Note on pseudocurrent:
		Current through coil at max rotational velocity when unrestricted including back EMF.  Pseudo because not equal to current drawn on power supply.
		Way to think about this is:
		net_voltage * pseudocurrent = batt_voltage * actual current draw
		Same power draw, but first has lower voltage, higher current because back EMF subtracted out
		With back EMF, seems like a current in the reverse direction is added and a positive voltage is added
		*/
		//float max_rot_vel_coil_pseudocurrent = max_rot_vel_net_voltage / propulsion_units[fan_number].motor.coil_resistance;

		//Input power = net_voltage * pseudocurrent = batt_voltage * actual current draw - shown to be wrong, gives efficiency over 100% at high RPM
		//float max_rot_vel_input_power = max_rot_vel_net_voltage * max_rot_vel_coil_pseudocurrent;


		float max_rot_vel_total_current_draw = batt_voltage / effective_impedance;

		float max_rot_vel_input_power = batt_voltage * max_rot_vel_total_current_draw;




		//Actual (not pseudo) current draw at max rotational velocity
		//float max_rot_vel_actual_current_draw = max_rot_vel_input_power / batt_voltage;

		float max_rot_vel_efficiency = max_rot_vel_output_power / max_rot_vel_input_power;



		propulsion_units[fan_number].max_torque_at_current_rot_vel = max_torque_at_current_rot_vel;
		propulsion_units[fan_number].max_rotational_velocity = max_rot_vel;
		propulsion_units[fan_number].max_rot_vel_output_power = max_rot_vel_output_power;
		propulsion_units[fan_number].max_rot_vel_input_power = max_rot_vel_input_power;
		propulsion_units[fan_number].max_rot_vel_efficiency = max_rot_vel_efficiency;

/*
		if (fan_number == 2){

			Serial.print("Max eff: ");
			Serial.print(max_rot_vel_efficiency); Serial.print(" ");
			Serial.print(max_torque_at_current_rot_vel); Serial.print(" ");
			Serial.print(max_rot_vel); Serial.print(" ");
			Serial.print(max_rot_vel_torque); Serial.print(" ");
			Serial.print(max_rot_vel_output_power); Serial.print(" ");
			Serial.print(max_rot_vel_net_voltage); Serial.print(" ");

			Serial.print(max_rot_vel_input_power); Serial.print(" ");
			Serial.print(max_rot_vel_total_current_draw); Serial.print(" ");

			Serial.print("end ");

		}
		*/

		/*
		Serial.println("");
		Serial.println("Motor operating limits:");

		Serial.println("Max torque  Max Rot Vel   Max Output Power  Max Input Power  Efficiency at Max");

		for (int i=0; i<8; i++)
    	{
    		Serial.print(i + 1); Serial.print(" ");
    		Serial.print(propulsion_units[i].max_torque_at_current_rot_vel); Serial.print(" ");
    		Serial.print(propulsion_units[i].max_rotational_velocity); Serial.print(" ");
    		Serial.print(propulsion_units[i].max_rot_vel_output_power); Serial.print(" ");
    		Serial.print(propulsion_units[i].max_rot_vel_input_power); Serial.print(" ");
    		Serial.print(propulsion_units[i].max_rot_vel_efficiency); Serial.print(" ");

    		Serial.println(" ");
		}
		Serial.println("");

		*/

	}






/*
	Input: 
	Output: None.  Update two arrays (+/-) of fan rotational velocity deltas and servo angle deltas possible to reach 
		at end of next loop after sending commands this loop, based on vehicle state at end of current loop


*/
void Control::get_max_rot_vel_and_servo_angle_deltas()
	{


		for (int i=0; i<8; i++)
    	{
    		//get_max_fan_rotational_velocity_deltas(i);

    		float min_pwm = propulsion_units[i].motor_controller.min_pwm;
    		float max_pwm = propulsion_units[i].motor_controller.max_pwm;

    		POWERTRAIN_MODEL_OUTPUT model_output_min_pwm = run_powertrain_model(i, min_pwm);
    		POWERTRAIN_MODEL_OUTPUT model_output_max_pwm = run_powertrain_model(i, max_pwm);

			propulsion_units[i].max_positive_rot_vel_delta = model_output_max_pwm.rot_vel_after_loop_time - propulsion_units[i].rotational_velocity;
			propulsion_units[i].max_negative_rot_vel_delta = model_output_min_pwm.rot_vel_after_loop_time - propulsion_units[i].rotational_velocity;

		}

		for (int servo_num=0; servo_num<4; servo_num++)
    	{

    		float min_angle_pwm = servo_units[servo_num].min_angle_pwm;
			float max_angle_pwm = servo_units[servo_num].max_angle_pwm;

			SERVO_MODEL_OUTPUT model_output_min_pwm = run_servo_model(servo_num, min_angle_pwm);
			SERVO_MODEL_OUTPUT model_output_max_pwm = run_servo_model(servo_num, max_angle_pwm);

			servo_units[servo_num].max_positive_angle_delta = model_output_max_pwm.pos_after_loop_time - servo_units[servo_num].servo_angle;
			servo_units[servo_num].max_negative_angle_delta = model_output_min_pwm.pos_after_loop_time - servo_units[servo_num].servo_angle;



		}


		/*
		Serial.println("");
		Serial.println("Max pos/neg motor torques, max pos/neg servo acc, min pos/neg servo acc: ");

		for (int i=0; i<8; i++)
    	{
    		Serial.print(i + 1); Serial.print(" ");
    		Serial.print(propulsion_units[i].max_positive_torque); Serial.print(" ");
    		Serial.print(propulsion_units[i].max_negative_torque); Serial.println(" ");


		}

		for (int i=0; i<4; i++)
    	{
    		Serial.print(i + 1); Serial.print(" ");
    		Serial.print(servo_units[i].max_positive_acceleration); Serial.print(" ");
    		Serial.print(servo_units[i].max_negative_acceleration); Serial.print(" ");
    		Serial.print(servo_units[i].min_positive_acceleration); Serial.print(" ");
    		Serial.print(servo_units[i].min_negative_acceleration); Serial.println(" ");
		}

		Serial.println("");
		*/




		/*
		Serial.println("");
		Serial.println("Max pos/neg rot vel deltas, max pos/neg servo angle deltas, min pos/neg servo angle deltas: ");

		for (int i=0; i<8; i++)
    	{
    		Serial.print(i + 1); Serial.print(" ");
    		Serial.print(propulsion_units[i].max_positive_rot_vel_delta); Serial.print(" ");
    		Serial.print(propulsion_units[i].max_negative_rot_vel_delta); Serial.println(" ");


		}

		for (int i=0; i<4; i++)
    	{
    		Serial.print(i + 1); Serial.print(" ");
    		Serial.print(servo_units[i].max_positive_angle_delta); Serial.print(" ");
    		Serial.print(servo_units[i].max_negative_angle_delta); Serial.print(" ");
    		Serial.print(servo_units[i].min_positive_angle_delta); Serial.print(" ");
    		Serial.print(servo_units[i].min_negative_angle_delta); Serial.println(" ");
		}

		Serial.println("");
		*/


	}







/*
	Input: servo_num (includes hardware parameters and current state), PWM microseconds
	Output: servo angle after loop time, servo velocity after loop time, initial acceleration, acceleration after loop time

	Should model all dynamics of a particular servo, given a PWM command.


*/
Control::SERVO_MODEL_OUTPUT Control::run_servo_model(int servo_num, float PWM_micros)
	{

		//Make values readable
		float P_gain = servo_units[servo_num].servo.P_gain;
		float dc_motor_effective_kv = servo_units[servo_num].servo.dc_motor_effective_kv;
		float acc_per_net_volt = servo_units[servo_num].servo.acceleration_per_net_volt;
		float min_net_voltage_for_movement = servo_units[servo_num].servo.min_net_voltage_for_movement;
		float min_abs_velocity_for_movement = servo_units[servo_num].servo.min_abs_velocity_for_movement;

		float zero_deg_pwm = servo_units[servo_num].zero_deg_pwm;
		float ninety_deg_pwm = servo_units[servo_num].ninety_deg_pwm;

		float current_angle = servo_units[servo_num].servo_angle;
		float current_rot_vel = servo_units[servo_num].servo_rotational_velocity;


		//PWM microseconds per radian of servo movement
		float microseconds_per_rad = (ninety_deg_pwm - zero_deg_pwm) / (M_PI / 2);


		//Calculate steady state angle represented by microsecond command
		float steady_state_angle_command = (PWM_micros - zero_deg_pwm) / microseconds_per_rad;

		//Angle error in radians
		float angle_error = steady_state_angle_command - current_angle;

		//Apply P gain to angle error
		float dc_motor_duty_cycle = P_gain * angle_error;

		//Constrain duty cycle to between -1 and 1
		if (std::abs(dc_motor_duty_cycle) > 1){
			dc_motor_duty_cycle = dc_motor_duty_cycle / std::abs(dc_motor_duty_cycle);
		}


		/*
		Problem is when spinning in one direction, can apply batt voltage + back emf and get to super high net voltages
		Allows huge accelerations to be reached but doesn't take into account new back EMF from new velocities

		Need differential equation:
		D(Vel)/dT = Servo acceleration = net voltage * acc per volt = (max_acc_input_voltage - dc_motor_effective_kv * (Vel)) * acc per volt

		D(Vel)/dT = max_acc_input_voltage * acc per volt - acc per volt * dc_motor_effective_kv * (Vel)

		Say:
		A = max_acc_input_voltage * acc per volt
		B = dc_motor_effective_kv * acc per volt

		DV/dT = A - B * V

		Solution to differential equation:
		V(t) = C * e ^ (B * t) + A/B
		V(0) = current_rot_vel = C + A/B
			-> C = current_rot_vel - A/B

		Velocity at end of loop time:
		V(loop_time) = [current_rot_vel - A/B] * e ^ (B * loop_time) + A/B

		Position vs. time (V(t) integration):
		P(time) = initial position + (C/B) * (1 - e^(-B * time)) + (A/B) * (time)

		Acceleration vs. time (V(t) differentiation):
		A(time) = - B * C * e ^(-B * time) 

		*/


		float input_voltage = dc_motor_duty_cycle * servo_supply_voltage;

		float initial_net_voltage = input_voltage - current_rot_vel * dc_motor_effective_kv;
		float initial_acceleration = initial_net_voltage * acc_per_net_volt;

		float A = input_voltage * acc_per_net_volt;
		float B = dc_motor_effective_kv * acc_per_net_volt;
		float C = current_rot_vel - A/B;

		float rot_vel_after_loop_time = C * std::pow(e, (- B * loop_time)) + A/B;

		float pos_after_loop_time = current_angle + (C/B) * (1 - std::pow(e, (-B * loop_time))) + (A/B) * loop_time;

		float acc_after_loop_time = - C * B * std::pow(e, (- B * loop_time));



		if (std::abs(initial_net_voltage) < min_net_voltage_for_movement && std::abs(current_rot_vel) < min_abs_velocity_for_movement){
			
			//If servo is going to stick, model the dynamics of sticking over the loop as the deceleration that would slow to zero by the end of the loop time
			initial_acceleration = -current_rot_vel / loop_time;

			pos_after_loop_time = current_angle + (current_rot_vel/2) * loop_time;
			rot_vel_after_loop_time = 0;
			acc_after_loop_time = 0;
		}



		SERVO_MODEL_OUTPUT servo_model_output;

		servo_model_output.initial_acceleration = initial_acceleration;
		servo_model_output.pos_after_loop_time = pos_after_loop_time;
		servo_model_output.rot_vel_after_loop_time = rot_vel_after_loop_time;
		servo_model_output.acc_after_loop_time = acc_after_loop_time;


/*
		Serial.println("Servo model:");
		Serial.print(steady_state_angle_command); Serial.print(" ");
		Serial.print(current_angle); Serial.print(" ");
		Serial.print(current_rot_vel); Serial.print(" ");
		Serial.print(dc_motor_duty_cycle); Serial.print(" ");
		Serial.print(initial_net_voltage); Serial.print(" ");
		Serial.print(pos_after_loop_time); Serial.print(" ");
		Serial.print(rot_vel_after_loop_time); Serial.print(" ");
		Serial.print(acc_after_loop_time); Serial.print(" ");
		Serial.println(" ");
*/

/*
		Serial.println("Servo model constants:");
		Serial.print(microseconds_per_rad); Serial.print(" ");
		Serial.print(P_gain);  Serial.print(" ");
		Serial.print(dc_motor_effective_kv);  Serial.print(" ");
		Serial.print(servo_supply_voltage);  Serial.print(" ");
		Serial.print(acc_per_net_volt);  Serial.print(" ");
*/

		return servo_model_output;

	}









/*
	Input: current location information, receiver commands
	Output: series of accelerations that will result in control around target points

	At first, hard code in rotational acceleration limits and apply the max acceleration in direction needed

*/
void Control::acceleration_controller(Location::LOCATION * Location, Receiver::RECEIVER * Receiver)
	{

		//For testing only
		test_input = Receiver->thrust;


		//Translational Accelerations
		float total_translational_acceleration = Receiver->thrust * 17; //Max total acceleration set to 17 m/s^2

		if (Receiver->thrust < 0.1){
			total_translational_acceleration = 0;
		}


		//0 for accelerating straight up in body frame, 1.57 (rads) for directly forwards
		 //Dial goes from -1 to 1.  Set to 1.745 (100 degrees) so can multiply by 100 to get actual angle
		float translational_acceleration_vector_angle = 0;//Receiver->dial1 * 1.745; //CHANGE!!!!!!!


		//Z and X acceleration from total desired and desired vector
		Accelerations.x_acceleration = total_translational_acceleration * sin(translational_acceleration_vector_angle);

		Accelerations.z_acceleration = total_translational_acceleration * cos(translational_acceleration_vector_angle);



		//Angular Accelerations
		float max_roll_angle = 0.3; //Radians
		float roll_target = Receiver->roll * max_roll_angle;

		float max_pitch_angle = 0.3; //Radians
		float pitch_target = Receiver->pitch * max_pitch_angle;

		float max_yaw_angle = 0.3; //Radians
		float yaw_target = Receiver->yaw * max_yaw_angle;


		//Implementatation of max(0, total_translational_acceleration - 2) bc max() not working (may be Eigen library)
		float clipped_translational_acceleration = total_translational_acceleration - 2;
		if (total_translational_acceleration < 2){
			clipped_translational_acceleration = 0;
		}

		float max_hover_roll_acceleration = 1 * sqrt(clipped_translational_acceleration);
 		float max_cruise_roll_acceleration = 1 * sqrt(total_translational_acceleration);
 		float max_roll_acceleration = max_hover_roll_acceleration + Receiver->dial1 * (max_cruise_roll_acceleration - max_hover_roll_acceleration);


		//Helps to graph in Desmos.  Zero until 2 m/s^2 translational, then shifted (by 2) square root from there and scaled down to be at 2 rad/s^2 when at 10 m/s^2
 		float max_hover_pitch_acceleration = 0.7 * sqrt(clipped_translational_acceleration);
 		float max_cruise_pitch_acceleration = 0.75 * sqrt(total_translational_acceleration);
 		float max_pitch_acceleration = max_hover_pitch_acceleration + Receiver->dial1 * (max_cruise_pitch_acceleration - max_hover_pitch_acceleration);


 		float max_hover_yaw_acceleration = 0.15 * sqrt(clipped_translational_acceleration);
 		float max_cruise_yaw_acceleration = 0.15 * sqrt(total_translational_acceleration);
 		float max_yaw_acceleration = max_hover_yaw_acceleration + Receiver->dial1 * (max_cruise_yaw_acceleration - max_hover_yaw_acceleration);



		float radians_to_roll_target = roll_target - Location->roll; //Positive if need to roll right more to hit target
		float radians_to_pitch_target = pitch_target - Location->pitch; //Positive if need to pitch right more to hit target
		//IMPORTANT NOTE: fix before yawing more than 180 degrees
		float radians_to_yaw_target = yaw_target - Location->yaw; //Positive if need to yaw right more to hit target


		float transient_time = 0.5 * Receiver->dial1;

		//Max rate possible to decelerate down from at current distance from target
		//NOTE: Always positive.  Treat as absolute value of maximum angular rate.
		float max_roll_rate_abs_value = max_roll_acceleration * sqrt(2 * std::abs(radians_to_roll_target) / max_roll_acceleration);
		float max_pitch_rate_abs_value = max_pitch_acceleration * sqrt(2 * std::abs(radians_to_roll_target) / max_pitch_acceleration);
		float max_yaw_rate_abs_value = max_yaw_acceleration * sqrt(2 * std::abs(radians_to_roll_target) / max_yaw_acceleration);

		float abs_value_radians_from_target_to_transient_start_roll = 0.5 * std::pow((Location->roll_rate / max_roll_acceleration),2) * max_roll_acceleration
			+ max_roll_acceleration * std::pow(transient_time,2) / 6 + copysignf(Location->roll_rate, 1) * transient_time;


		float abs_value_radians_from_target_to_transient_start_pitch = 0.5 * std::pow((Location->pitch_rate / max_pitch_acceleration),2) * max_pitch_acceleration
			+ max_pitch_acceleration * std::pow(transient_time,2) / 6 + copysignf(Location->pitch_rate, 1) * transient_time;


		float abs_value_radians_from_target_to_transient_start_yaw = 0.5 * std::pow((Location->yaw_rate / max_yaw_acceleration),2) * max_yaw_acceleration
			+ max_yaw_acceleration * std::pow(transient_time,2) / 6 + copysignf(Location->yaw_rate, 1) * transient_time;


		//Roll Control
		//If rolling left
		if (Location->roll_rate < 0){
			
			//If rolling left but far away from the point where need to slow down
			if (Location->roll > abs_value_radians_from_target_to_transient_start_roll){
				//Copysignf(x,y) takes two floats (hence the f) and outputs a value with the magnitude of x and sign of y
				Accelerations.roll_acceleration = -max_roll_acceleration;
			}
			else{
				//If within deceleration range, start decelerating
				Accelerations.roll_acceleration = max_roll_acceleration;
			}

		}
		//If rolling right
		else{
			//If rolling right but far away from the point where need to slow down
			if (Location->roll < -abs_value_radians_from_target_to_transient_start_roll){
				//Copysignf(x,y) takes two floats (hence the f) and outputs a value with the magnitude of x and sign of y
				Accelerations.roll_acceleration = max_roll_acceleration;
			}
			else{
				//If within deceleration range, start decelerating
				Accelerations.roll_acceleration = - max_roll_acceleration;
			}

		}


		//Pitch Control
		//If pitching down
		if (Location->pitch_rate < 0){
			
			//If pitching down but far away from the point where need to slow down
			if (Location->pitch > abs_value_radians_from_target_to_transient_start_pitch){
				//Copysignf(x,y) takes two floats (hence the f) and outputs a value with the magnitude of x and sign of y
				Accelerations.pitch_acceleration = -max_pitch_acceleration;
			}
			else{
				//If within deceleration range, start decelerating
				Accelerations.pitch_acceleration = max_pitch_acceleration;
			}

		}
		//If pitching up
		else{
			//If pitching up but far away from the point where need to slow down
			if (Location->pitch < -abs_value_radians_from_target_to_transient_start_pitch){
				//Copysignf(x,y) takes two floats (hence the f) and outputs a value with the magnitude of x and sign of y
				Accelerations.pitch_acceleration = max_pitch_acceleration;
			}
			else{
				//If within deceleration range, start decelerating
				Accelerations.pitch_acceleration = - max_pitch_acceleration;
			}

		}

		//Yaw Control
		//If yawing CCW
		if (Location->yaw_rate < 0){
			
			//If yawing CCW but far away from the point where need to slow down
			if (Location->yaw > abs_value_radians_from_target_to_transient_start_yaw){
				//Copysignf(x,y) takes two floats (hence the f) and outputs a value with the magnitude of x and sign of y
				Accelerations.yaw_acceleration = -max_yaw_acceleration;
			}
			else{
				//If within deceleration range, start decelerating
				Accelerations.yaw_acceleration = max_yaw_acceleration;
			}

		}
		//If yawing CW
		else{
			//If pitching up but far away from the point where need to slow down
			if (Location->yaw < -abs_value_radians_from_target_to_transient_start_yaw){
				//Copysignf(x,y) takes two floats (hence the f) and outputs a value with the magnitude of x and sign of y
				Accelerations.yaw_acceleration = max_yaw_acceleration;
			}
			else{
				//If within deceleration range, start decelerating
				Accelerations.yaw_acceleration = - max_yaw_acceleration;
			}

		}

		/*
		//Input fake accelerations for testing
		Accelerations.roll_acceleration = -2.66;
		Accelerations.pitch_acceleration = 2.66;
		Accelerations.yaw_acceleration = 0.55;
		Accelerations.x_acceleration = 0;
		Accelerations.z_acceleration = 14.5;
		*/

		/*
		Serial.println("Cmd Acc, rpyxz: ");
		Serial.print(Accelerations.roll_acceleration); Serial.print(" ");
		Serial.print(Accelerations.pitch_acceleration); Serial.print(" ");
		Serial.print(Accelerations.yaw_acceleration); Serial.print(" ");
		Serial.print(Accelerations.x_acceleration); Serial.print(" ");
		Serial.print(Accelerations.z_acceleration); Serial.print(" ");
		//Serial.println("");
		*/

		target_accelerations(0,0) = Accelerations.roll_acceleration;
		target_accelerations(0,1) = Accelerations.pitch_acceleration;
		target_accelerations(0,2) = Accelerations.yaw_acceleration;
		target_accelerations(0,3) = Accelerations.x_acceleration;
		target_accelerations(0,4) = Accelerations.z_acceleration;

		//Serial.print("Target Acc: ");
		//print_mtxf(target_accelerations);

	}





/*
	Input: roll, pitch, yaw angular acceleration (rad/s^2), z acceleration (m/s^2), x acceleration (m/s^2)
	Output: series of forces (N) and moments (Nm) that will result in input accelerations (from fans only - nothing from wing, body aero)

*/
void Control::accelerations_to_forces_and_moments()
	{

		//Torque = Moment of Inertia * Angular Acceleration
		//Target_Forces_and_Moments.roll_moment = roll_moment_of_inertia * Accelerations->roll_acceleration;
		//Target_Forces_and_Moments.pitch_moment = pitch_moment_of_inertia * Accelerations->pitch_acceleration;
		//Target_Forces_and_Moments.yaw_moment = yaw_moment_of_inertia * Accelerations->yaw_acceleration;

		//Force = Mass * Acceleration
		//Target_Forces_and_Moments.x_force = vehicle_mass * Accelerations->x_acceleration;
		//Target_Forces_and_Moments.z_force = vehicle_mass * Accelerations->z_acceleration;


		//Torque = Moment of Inertia * Angular Acceleration
		target_forces_and_moments(0,0) = roll_moment_of_inertia * target_accelerations(0,0);
		target_forces_and_moments(0,1) = pitch_moment_of_inertia * target_accelerations(0,1);
		target_forces_and_moments(0,2) = yaw_moment_of_inertia * target_accelerations(0,2);

		//Force = Mass * Acceleration
		target_forces_and_moments(0,3) = vehicle_mass * target_accelerations(0,3);
		target_forces_and_moments(0,4) = vehicle_mass * target_accelerations(0,4);


		//Serial.print("Target Forces and Moments: ");
		//print_mtxf(target_forces_and_moments);

		/*
		Serial.println("");
		Serial.print("Commanded Forces and Moments [roll moment, pitch, yaw, x force, z force]: ");
		Serial.print(Target_Forces_and_Moments.roll_moment); Serial.print(" ");
		Serial.print(Target_Forces_and_Moments.pitch_moment); Serial.print(" ");
		Serial.print(Target_Forces_and_Moments.yaw_moment); Serial.print(" ");
		Serial.print(Target_Forces_and_Moments.x_force); Serial.print(" ");
		Serial.print(Target_Forces_and_Moments.z_force); Serial.print(" ");
		Serial.println("");
		*/

	}


/*
	Input: Series of rotational velocities from all fans, series of fan angles (rads), motor directions, motor x/y locations, motor torque per thrust
	Output: None.  Update Theoretical_Fan_Forces_and_Moments struct.

*/
MatrixXf Control::calculate_theoretical_fan_forces_and_moments(MatrixXf motor_vels_and_servo_angles)
	{


		float fan_i_roll_moment;
		float fan_i_pitch_moment;
		float fan_i_yaw_moment;
		float fan_i_x_thrust;
		float fan_i_z_thrust;

		float total_roll_moment = 0;
		float total_pitch_moment = 0;
		float total_yaw_moment = 0;
		float total_x_force = 0;
		float total_z_force = 0;

		for (int i=0; i<8; i++)
    	{

    		int servo_num = propulsion_units[i].servo_num;
    		float x_loc = propulsion_units[i].x_loc;
    		float y_loc = propulsion_units[i].y_loc;
    		float rot_vel = motor_vels_and_servo_angles(i,0);

    		float motor_torque = get_fan_aero_torque(i, rot_vel);
    		float motor_thrust = get_fan_thrust(i, rot_vel);

    		int servo_index = propulsion_units[i].servo_num + 8;
    		float servo_angle = motor_vels_and_servo_angles(servo_index,0);
    		int motor_dir = propulsion_units[i].motor_direction;


			fan_i_roll_moment = -motor_thrust * x_loc * cos(servo_angle) + motor_dir * motor_torque * sin(servo_angle);
			total_roll_moment = total_roll_moment + fan_i_roll_moment;

			fan_i_pitch_moment = motor_thrust * y_loc * cos(servo_angle);
			total_pitch_moment = total_pitch_moment + fan_i_pitch_moment;

			fan_i_yaw_moment = -motor_thrust * x_loc * sin(servo_angle) - motor_dir * motor_torque * cos(servo_angle);
			total_yaw_moment = total_yaw_moment + fan_i_yaw_moment;

			fan_i_x_thrust = motor_thrust * sin(servo_angle);
			total_x_force = total_x_force + fan_i_x_thrust;

			fan_i_z_thrust = motor_thrust * cos(servo_angle);
			total_z_force = total_z_force + fan_i_z_thrust;

		}

		//Set Theoretical_Fan_Forces_and_Moments to latest values.
		Theoretical_Fan_Forces_and_Moments.roll_moment = total_roll_moment;
		Theoretical_Fan_Forces_and_Moments.pitch_moment = total_pitch_moment;
		Theoretical_Fan_Forces_and_Moments.yaw_moment = total_yaw_moment;

		Theoretical_Fan_Forces_and_Moments.x_force = total_x_force;
		Theoretical_Fan_Forces_and_Moments.z_force = total_z_force;


		MatrixXf theoretical_forces_and_moments(1,5);
		theoretical_forces_and_moments(0,0) = Theoretical_Fan_Forces_and_Moments.roll_moment;
		theoretical_forces_and_moments(0,1) = Theoretical_Fan_Forces_and_Moments.pitch_moment;
		theoretical_forces_and_moments(0,2) = Theoretical_Fan_Forces_and_Moments.yaw_moment;
		theoretical_forces_and_moments(0,3) = Theoretical_Fan_Forces_and_Moments.x_force;
		theoretical_forces_and_moments(0,4) = Theoretical_Fan_Forces_and_Moments.z_force;


		//Serial.print("Theoretical Forces: ");
		//print_mtxf(theoretical_forces_and_moments_from_control_guess);

		return theoretical_forces_and_moments;


	}






/*
	Input: Series of rotational velocities for all fans (rads/sec), series of servo angles (rads)
	Output: Kinematics derivatives matrix at current state

*/
MatrixXf Control::get_kinematics_derivatives(MatrixXf motor_vels_and_servo_angles)
	{

		//Derivatives of 3 moments and 2 forces with respect to thrust and servo angle
		MatrixXf kinematics_derivatives(12,5);

		//Derivatives of moments and forces with respect to servo angle for each motor
		MatrixXf servo_kinematics_derivatives(8,5);


		//Roll Moment (Nm) = -motor_x_locs[i] * fan_thrust_coefficient * fan_rad_per_sec^2 * cos(servo_angles_mapped[i]) + motor_directions[i] * fan_aero_torque_coefficient * fan_rad_per_sec^2 * sin(servo_angles_mapped[i]);
		//Roll Moment deriv = - 2 * motor_x_locs[i] * fan_thrust_coefficient * fan_rad_per_sec * cos(servo_angles_mapped[i]) + 2 * motor_directions[i] * fan_aero_torque_coefficient * fan_rad_per_sec * sin(servo_angles_mapped[i]);

		//Get derivatives of roll, pitch, and yaw moments, x and z thrusts with respect to motor thrust at given thrust point and servo angle point for each motor
		for (int i=0; i<8; i++)
    	{

    		int servo_num = propulsion_units[i].servo_num;
    		float x_loc = propulsion_units[i].x_loc;
    		float y_loc = propulsion_units[i].y_loc;
    		float rot_vel = motor_vels_and_servo_angles(i,0);
    		float fan_torque_coeff = 1 / std::pow(propulsion_units[i].fan.fan_aero_torque_coefficient,2);
    		float fan_thrust_coeff = 1 / std::pow(propulsion_units[i].fan.fan_thrust_coefficient,2);

    		float motor_torque = get_fan_aero_torque(i, rot_vel);
    		float motor_thrust = get_fan_thrust(i, rot_vel);

    		int servo_index = propulsion_units[i].servo_num + 8;
    		float servo_angle = motor_vels_and_servo_angles(servo_index,0);
    		int motor_dir = propulsion_units[i].motor_direction;


    		//Roll moment derivative
			kinematics_derivatives(i,0) = -2 * x_loc * fan_thrust_coeff * rot_vel * cos(servo_angle) + 2 * motor_dir * fan_torque_coeff * rot_vel * sin(servo_angle);

			//Pitch moment derivative
			kinematics_derivatives(i,1) = 2 * y_loc * fan_thrust_coeff * rot_vel * cos(servo_angle);

			//Yaw moment derivative
			kinematics_derivatives(i,2) = -2 * x_loc * fan_thrust_coeff * rot_vel * sin(servo_angle) - 2 * motor_dir * fan_torque_coeff * rot_vel * cos(servo_angle);

			//X thrust derivative
			kinematics_derivatives(i,3) = sin(servo_angle);

			//Y thrust derivative
			kinematics_derivatives(i,4) = cos(servo_angle);



			//Get derivatives of roll, pitch, and yaw moments, z and x thrusts with respect to servo angle at given fan rotational velocity and servo angle for each motor
			servo_kinematics_derivatives(i,0) = motor_thrust * x_loc * sin(servo_angle) + motor_dir * fan_torque_coeff * std::pow(rot_vel,2) * cos(servo_angle);

			servo_kinematics_derivatives(i,1) = -motor_thrust * y_loc * sin(servo_angle);

			servo_kinematics_derivatives(i,2) = -motor_thrust * x_loc * cos(servo_angle) + motor_dir * fan_torque_coeff * std::pow(rot_vel,2) * sin(servo_angle);

			servo_kinematics_derivatives(i,3) = motor_thrust * cos(servo_angle);

			servo_kinematics_derivatives(i,4) = -motor_thrust * sin(servo_angle);


		}



		//Put servo derivatives on the bottom four rows of the derivatives matrix.  Addition due to effects of multiple motors on one servo.
		kinematics_derivatives.row(8) = servo_kinematics_derivatives.row(0);
		kinematics_derivatives.row(9) = servo_kinematics_derivatives.row(1);
		kinematics_derivatives.row(10) = servo_kinematics_derivatives.row(2) + servo_kinematics_derivatives.row(4) + servo_kinematics_derivatives.row(6);
		kinematics_derivatives.row(11) = servo_kinematics_derivatives.row(3) + servo_kinematics_derivatives.row(5) + servo_kinematics_derivatives.row(7);


		//Serial.println("Kinematics Derivatives: ");
		//print_mtxf(kinematics_derivatives);

		return kinematics_derivatives;


	}





/*
	Inputs:
		-Current rotational velocities and servo angles
		-Target moments and forces
		-Motor and servo parameters

	Output: Delta fan rotational velocities and servo angles needed to achieve target forces and moments - likely not possible in next timestep

*/
void Control::get_theoretical_rot_vel_and_servo_angle_deltas()
	{

		MatrixXf current_motor_vels_and_servo_angles(12,1);
		for (int i=0; i<8; i++)
    	{
			current_motor_vels_and_servo_angles(i,0) = propulsion_units[i].rotational_velocity;
		}

		for (int i=0; i<4; i++)
    	{
			current_motor_vels_and_servo_angles(i+8,0) = servo_units[i].servo_angle;
		}

		//Estimated forces and moments being generated by commands sent last timestep
		MatrixXf current_forces_and_moments = calculate_theoretical_fan_forces_and_moments(current_motor_vels_and_servo_angles);
		//Serial.print("Current forces and moments: ");
		//print_mtxf(current_forces_and_moments);


		MatrixXf deltas_to_target_forces_and_moments(1,5);
		deltas_to_target_forces_and_moments = target_forces_and_moments - current_forces_and_moments;
		//Serial.print("Deltas to Target Forces: ");
		//print_mtxf(deltas_to_target_forces_and_moments);

		//360 micros on first iteration, 960 on second iteration (more as get closer?)
		//Kinematics derivatives at the current state
		MatrixXf kinematics_derivatives = get_kinematics_derivatives(current_motor_vels_and_servo_angles);
		//Serial.println("Kinematics derivatives: ");
		//print_mtxf(kinematics_derivatives);


		//Takes around 800-1000 microseconds
		MatrixXf estimated_delta_vels_and_angles = kinematics_derivatives.transpose().completeOrthogonalDecomposition().solve(deltas_to_target_forces_and_moments.transpose());
		//Serial.print("Estimated input deltas: ");
		//print_mtxf(estimated_delta_vels_and_angles);


		MatrixXf estimated_vels_and_angles(12,1);
		estimated_vels_and_angles = current_motor_vels_and_servo_angles + estimated_delta_vels_and_angles;
		//Serial.print("Estimated inputs: ");
		//print_mtxf(estimated_control_input);


		//Estimated forces and moments generated by estimated vels and angles calculated after estimated vels and angles
		MatrixXf estimated_forces_and_moments = calculate_theoretical_fan_forces_and_moments(estimated_vels_and_angles);

		deltas_to_target_forces_and_moments = target_forces_and_moments - estimated_forces_and_moments;

		kinematics_derivatives = get_kinematics_derivatives(estimated_vels_and_angles);

		estimated_delta_vels_and_angles = kinematics_derivatives.transpose().completeOrthogonalDecomposition().solve(deltas_to_target_forces_and_moments.transpose());

		//Updated final guess at rotational velocities and servo angles
		estimated_vels_and_angles = estimated_vels_and_angles + estimated_delta_vels_and_angles;

		///*
		//estimated_vels_and_angles(0,0) = test_input * 1200;

		//Serial.print("Long term vel target: ");
		//Serial.println(test_input);
		//Serial.print(estimated_vels_and_angles(0,0));
		//Serial.print(" ");
		//print_mtxf(estimated_vels_and_angles);

		//*/



		//completeOrthogonalDecomposition() - 19,091 micros entire control loop
			//Solution deltas for location all zeroes, commanded moments -0.78, 2.32, 0.64, 0.00, 58.00:
			//1.33, 1.05, -0.29, -0.51, -0.15, -0.64, -0.02, -0.78, -0.08, 0.08, -0.02, 0.02 (Probably gold standard - same as Octave)
		//fullPivLu() - 9,970 micros entire loop
			//Solution deltas for location all zeroes, commanded moments -0.78, 2.32, 0.64, 0.00, 58.00:
			//-2.38, 0.00, -1.54, 0.00, 0.00, 0.00, 0.00, -0.84, 0.00, 0.00, -0.04, 0.04
		//colPivHouseholderQr() - 14,513 micros entire loop
			//Solution deltas for location all zeroes, commanded moments -0.78, 2.32, 0.64, 0.00, 58.00:
			//2.38, 0.00, 0.00, 0.00, 0.00, 0.00, -0.99, -1.39, 0.00, 0.00, -0.04, 0.04
		//fullPivHouseholderQr() - 11783 micros entire loop
			//Solution deltas for location all zeroes, commanded moments -0.78, 2.32, 0.64, 0.00, 58.00:
			//2.38, 0.00, -1.54, 0.00, 0.00, 0.00, 0.00, -0.84, 0.00, 0.00, -0.04, 0.04
		//partialPivLu() - 6,252 micros entire loop. !!Doesn't work. Matrix must be invertible, can't find solution for sample initial condition.
		//householderQr() - 9762 micros entire loop.  !!Doesn't work.  Some matrix values are NaN.
		//llt() - doesn't run.  Matrix must be positive definite.
		//ldlt() - doesn't run.  Matrix must be positive or negative semidefinite.
		


		theoretical_rot_vel_and_servo_angle_deltas = estimated_vels_and_angles - current_motor_vels_and_servo_angles;


		/*

		Serial.print("Delta to target: ");
		Serial.print(theoretical_rot_vel_and_servo_angle_deltas(0,0));
		Serial.print(" ");
		//print_mtxf(theoretical_rot_vel_and_servo_angle_deltas);

		*/


	}




/*
	Inputs: Max positive and negative fan velocity and servo angle deltas, theoretical target deltas
	Output: scaled down rotational velocity and servo angle deltas to values achievable in next timestep

*/
void Control::get_next_loop_rot_vel_and_servo_angle_deltas()
	{


		//Maximum multiple over the allowable rotational velocity
		//Default to 1 so that if none over the allowable delta, only divided by 1
		float max_delta_multiple_over_allowable = 1;

		//For each motor, get desired delta and compare with delta currently possible (for either spinning up or spinning down)
		for (int i=0; i<8; i++)
    	{

    		if (theoretical_rot_vel_and_servo_angle_deltas(i,0) > 0){

    			float multiple = theoretical_rot_vel_and_servo_angle_deltas(i,0) / propulsion_units[i].max_positive_rot_vel_delta;

    			if (multiple > 1 && multiple > max_delta_multiple_over_allowable) {
    				max_delta_multiple_over_allowable = multiple;
    			}

    		}

    		else{

    			float multiple = theoretical_rot_vel_and_servo_angle_deltas(i,0) / propulsion_units[i].max_negative_rot_vel_delta;

    			if (multiple > 1 && multiple > max_delta_multiple_over_allowable) {
    				max_delta_multiple_over_allowable = multiple;
    			}

    		}

		}




		//For each motor, get desired delta and compare with delta currently possible (for either spinning up or spinning down)
		for (int i=0; i<4; i++)
    	{

    		if (theoretical_rot_vel_and_servo_angle_deltas(i+8,0) > 0){

    			float multiple = theoretical_rot_vel_and_servo_angle_deltas(i+8,0) / servo_units[i].max_positive_angle_delta;

    			if (multiple > 1 && multiple > max_delta_multiple_over_allowable) {
    				max_delta_multiple_over_allowable = multiple;
    			}
    		}

    		else{

    			float multiple = theoretical_rot_vel_and_servo_angle_deltas(i+8,0) / servo_units[i].max_negative_angle_delta;

    			if (multiple > 1 && multiple > max_delta_multiple_over_allowable) {
    				max_delta_multiple_over_allowable = multiple;
    			}
    		}
		}



		next_loop_rot_vel_and_servo_angle_deltas = theoretical_rot_vel_and_servo_angle_deltas / max_delta_multiple_over_allowable;


		//Set fan rotational velocity deltas for next loop
		for (int i=0; i<8; i++)
    	{

			propulsion_units[i].rotational_velocity_delta_required = next_loop_rot_vel_and_servo_angle_deltas(i,0);

/*
			if (i == 0){
				Serial.print("Next loop: ");
				Serial.print(propulsion_units[i].max_positive_rot_vel_delta);
				Serial.print(" ");
				Serial.print(propulsion_units[i].max_negative_rot_vel_delta);
				Serial.print(" ");
				Serial.print(max_delta_multiple_over_allowable);
				Serial.print(" ");
				Serial.print(propulsion_units[i].rotational_velocity_delta_required);
				Serial.print(" ");
			}
			*/

		}

		//Set servo angle deltas for next loop
		for (int i=0; i<4; i++)
    	{

    		servo_units[i].angle_delta_required = next_loop_rot_vel_and_servo_angle_deltas(i+8,0);

		}


		
	/*
		Serial.println("Next loop vel and angle deltas: ");
		Serial.println(theoretical_rot_vel_and_servo_angle_deltas(8,0));
		Serial.println(max_delta_multiple_over_allowable);
		Serial.println(servo_units[0].servo_angle_delta);
		print_mtxf(next_loop_rot_vel_and_servo_angle_deltas);
	*/

	}




/*
	Input: Fan rotational velocity and servo angle deltas
	Output: None.  Runs functions to calculate and set pulse lengths.


*/
void Control::rot_vel_and_servo_angle_deltas_to_pwms()
	{

		//For each motor, get and set PWMs
		for (int i=0; i<8; i++)
    	{
			motor_rot_vel_delta_to_pwm(i);

		}

		for (int i=0; i<4; i++)
    	{
    		servo_angle_delta_to_pwm_required(i);
		}


		/*
		Serial.println("");
		Serial.println("Required motor torque, required PWM: ");

		for (int i=0; i<8; i++)
    	{
    		Serial.print(i + 1); Serial.print(" ");
    		Serial.print(propulsion_units[i].motor_torque_required); Serial.print(" ");
    		Serial.print(propulsion_units[i].PWM_micros); Serial.println(" ");

		}
		*/

		/*
		Serial.println("");
		Serial.println("Required servo acc, required PWM: ");
		for (int i=0; i<4; i++)
    	{
    		Serial.print(i + 1); Serial.print(" ");
    		Serial.print(servo_units[i].commanded_servo_acc); Serial.print(" ");
    		Serial.print(servo_units[i].PWM_micros); Serial.println(" ");
		}

		Serial.println("");
		*/




	}





/*
	Input: 
	Output: 


*/
void Control::motor_rot_vel_delta_to_torque_required(int fan_number)
	{

		//Make readable
		float current_rot_vel = propulsion_units[fan_number].rotational_velocity;

		float acc_required = propulsion_units[fan_number].rotational_velocity_delta_required / loop_time;
		float excess_torque_required = propulsion_units[fan_number].fan.fan_assembly_moment_of_inertia * acc_required;

		float torque_required = get_fan_aero_torque(fan_number, current_rot_vel) + excess_torque_required;

		propulsion_units[fan_number].motor_torque_required = torque_required;

	}








/*
	Input: 
	Output: 

*/
void Control::motor_rot_vel_delta_to_pwm(int fan_number)
	{

		//Make readable values
		float min_pwm = propulsion_units[fan_number].motor_controller.min_pwm;
		float max_pwm = propulsion_units[fan_number].motor_controller.max_pwm;
		float max_positive_rot_vel_delta = propulsion_units[fan_number].max_positive_rot_vel_delta;
		float max_negative_rot_vel_delta = propulsion_units[fan_number].max_negative_rot_vel_delta;

		float rot_vel_delta_req = propulsion_units[fan_number].rotational_velocity_delta_required;
		float current_rot_vel = propulsion_units[fan_number].rotational_velocity;



		/*
		Step 1:
		Linearly interpolate between pwm values - use dedicated function

		*/

		float estimated_fraction = (rot_vel_delta_req - max_negative_rot_vel_delta) / (max_positive_rot_vel_delta - max_negative_rot_vel_delta);

		float PWM_micros_guess = min_pwm + estimated_fraction * (max_pwm - min_pwm);


		float upper_bound_rot_vel_delta;
		float lower_bound_rot_vel_delta;

		float upper_bound_pwm;
		float lower_bound_pwm;




		POWERTRAIN_MODEL_OUTPUT powertrain_model_output = run_powertrain_model(fan_number, PWM_micros_guess);

		float rot_vel_delta_guess = powertrain_model_output.rot_vel_after_loop_time - current_rot_vel;


		if (rot_vel_delta_req > rot_vel_delta_guess){

			upper_bound_rot_vel_delta = max_positive_rot_vel_delta;
			lower_bound_rot_vel_delta = rot_vel_delta_guess;

			upper_bound_pwm = max_pwm;
			lower_bound_pwm = PWM_micros_guess;

		}

		else {

			upper_bound_rot_vel_delta = rot_vel_delta_guess;
			lower_bound_rot_vel_delta = max_negative_rot_vel_delta;

			upper_bound_pwm = PWM_micros_guess;
			lower_bound_pwm = min_pwm;

		}



		estimated_fraction = (rot_vel_delta_req - lower_bound_rot_vel_delta) / (upper_bound_rot_vel_delta - lower_bound_rot_vel_delta);

		PWM_micros_guess = lower_bound_pwm + estimated_fraction * (upper_bound_pwm - lower_bound_pwm);







		powertrain_model_output = run_powertrain_model(fan_number, PWM_micros_guess);

		rot_vel_delta_guess = powertrain_model_output.rot_vel_after_loop_time - current_rot_vel;


		if (rot_vel_delta_req > rot_vel_delta_guess){

			upper_bound_rot_vel_delta = upper_bound_rot_vel_delta;
			lower_bound_rot_vel_delta = rot_vel_delta_guess;

			upper_bound_pwm = upper_bound_pwm;
			lower_bound_pwm = PWM_micros_guess;

		}

		else {

			upper_bound_rot_vel_delta = rot_vel_delta_guess;
			lower_bound_rot_vel_delta = lower_bound_rot_vel_delta;

			upper_bound_pwm = PWM_micros_guess;
			lower_bound_pwm = lower_bound_pwm;

		}


		estimated_fraction = (rot_vel_delta_req - lower_bound_rot_vel_delta) / (upper_bound_rot_vel_delta - lower_bound_rot_vel_delta);

		PWM_micros_guess = lower_bound_pwm + estimated_fraction * (upper_bound_pwm - lower_bound_pwm);





		powertrain_model_output = run_powertrain_model(fan_number, PWM_micros_guess);

		rot_vel_delta_guess = powertrain_model_output.rot_vel_after_loop_time - current_rot_vel;

		if (rot_vel_delta_req > rot_vel_delta_guess){

			upper_bound_rot_vel_delta = upper_bound_rot_vel_delta;
			lower_bound_rot_vel_delta = rot_vel_delta_guess;

			upper_bound_pwm = upper_bound_pwm;
			lower_bound_pwm = PWM_micros_guess;

		}

		else {

			upper_bound_rot_vel_delta = rot_vel_delta_guess;
			lower_bound_rot_vel_delta = lower_bound_rot_vel_delta;

			upper_bound_pwm = PWM_micros_guess;
			lower_bound_pwm = lower_bound_pwm;

		}


		estimated_fraction = (rot_vel_delta_req - lower_bound_rot_vel_delta) / (upper_bound_rot_vel_delta - lower_bound_rot_vel_delta);

		PWM_micros_guess = lower_bound_pwm + estimated_fraction * (upper_bound_pwm - lower_bound_pwm);



		powertrain_model_output = run_powertrain_model(fan_number, PWM_micros_guess);

		rot_vel_delta_guess = powertrain_model_output.rot_vel_after_loop_time - current_rot_vel;

		Serial.print("rot_vel_delta final guess:  ");
		Serial.println(rot_vel_delta_guess);



		//Pulse width output
		propulsion_units[fan_number].PWM_micros = PWM_micros_guess;

	}











/*
	Input: 
	Output: 

	Essential model should illustrate that:
		-Steady state current sent by controller seems to be Max Current * (PWM fraction) ^ 2.35
		-Motor controller seems to be a PI loop commanding a torque from:
			-Error between actual rotational velocity and rotational velocity associated with commanded PWM
			-Matching transients by simulating in Octave, may be approximately:
				Torque sent (Nm) = 0.0004 * [rotational velocity error in rad/sec] + 0.0015 * [rotational velocity error integral wrt time in seconds]
					If Torque sent > max allowable:
						-Send max allowable in correct direction
						-Don't increase error integral

*/
void Control::motor_torque_to_pwm(int fan_number)
	{



		/*
		Step 1:
		Find steady state rotational velocity required to command to produce desired torque

		*/

		//Motor torque required (Nm)
		float required_torque = propulsion_units[fan_number].motor_torque_required;

		//Get current fan rotational velocity
		float current_rot_vel = propulsion_units[fan_number].rotational_velocity;

		
		//Calculate motor parameters to determine max motor torque at given rotational velocity
		float zero_speed_max_current = batt_voltage / propulsion_units[fan_number].motor.coil_resistance;
		float zero_speed_torque = propulsion_units[fan_number].motor.torque_constant * zero_speed_max_current;

		float motor_kv_rad_per_sec = propulsion_units[fan_number].motor.motor_kv / 9.5492966;
		float theoretical_zero_torque_rot_vel = batt_voltage * motor_kv_rad_per_sec;

		//Calculate max torque theoretically possible at current rotational velocity (may not be possible to reach because of controller PI loop)
		float max_theoretical_input_torque = zero_speed_torque * (1 - current_rot_vel / theoretical_zero_torque_rot_vel);


		//Make readable variables for motor controller torque model
		float P_gain = propulsion_units[fan_number].motor_controller.P_gain;
		float I_gain = propulsion_units[fan_number].motor_controller.I_gain;
		float rot_vel_error_integral = propulsion_units[fan_number].rot_vel_error_integral;

		/*
		Rotational velocity error required to generate given torque derived from motor's PI control of torque:
		Torque = P_gain * rot_vel_error + I_gain * rot_vel_error_integral
		*/
		float required_rot_vel_error = (required_torque - I_gain * rot_vel_error_integral) / (P_gain + loop_time * I_gain);


		//Rotational velocity error for timestep should be error found to generate required torque
		float rot_vel_error = required_rot_vel_error;
		rot_vel_error_integral = rot_vel_error_integral + rot_vel_error * loop_time;


		//Steady state rotational velocity command required to generate desired torque
		float steady_state_rot_vel_required = current_rot_vel + required_rot_vel_error;



		/*
		Step 2:
		Find what PWM would result in the target steady state rotational velocity

		*/

		//Readable min and max motor controller PWM
		float min_pwm = propulsion_units[fan_number].motor_controller.min_pwm;
		float max_pwm = propulsion_units[fan_number].motor_controller.max_pwm;
		float fan_torque_coeff = propulsion_units[fan_number].fan.fan_aero_torque_coefficient;

		float max_rot_vel_efficiency = propulsion_units[fan_number].max_rot_vel_efficiency;
		float max_rot_vel_input_power = propulsion_units[fan_number].max_rot_vel_input_power;


		float steady_state_torque = get_fan_aero_torque(fan_number, steady_state_rot_vel_required);
		float steady_state_output_power = steady_state_rot_vel_required * steady_state_torque;


		//Efficiency assumed constant across thrust input level.
		//Seems to be the case by experiment and both input and output power should increase with rot_vel ^ 3
		float steady_state_input_power = steady_state_output_power / max_rot_vel_efficiency;

		float steady_state_target_max_power_fraction = steady_state_input_power / max_rot_vel_input_power;

		//Input pulse fraction = Max power fraction ^ 1/2.35
		float inverse_input_exponent = 1 / propulsion_units[fan_number].motor_controller.motor_controller_input_exponent;
		float steady_state_target_input_fraction = std::pow(steady_state_target_max_power_fraction, inverse_input_exponent);


		float PWM_micros = min_pwm + steady_state_target_input_fraction * (max_pwm - min_pwm);


		//Pulse width output
		propulsion_units[fan_number].PWM_micros = PWM_micros;

	}





/*
	Input: 
	Output: 

	Calculate acceleration required over next timestep to reach given angle delta

*/
void Control::servo_angle_delta_to_pwm_required(int servo_num)
	{

		//Make values readable
		float P_gain = servo_units[servo_num].servo.P_gain;
		float dc_motor_effective_kv = servo_units[servo_num].servo.dc_motor_effective_kv;
		float acc_per_net_volt = servo_units[servo_num].servo.acceleration_per_net_volt;
		float min_net_voltage_for_movement = servo_units[servo_num].servo.min_net_voltage_for_movement;
		float min_abs_velocity_for_movement = servo_units[servo_num].servo.min_abs_velocity_for_movement;

		float zero_deg_pwm = servo_units[servo_num].zero_deg_pwm;
		float ninety_deg_pwm = servo_units[servo_num].ninety_deg_pwm;

		float current_angle = servo_units[servo_num].servo_angle;
		float current_rot_vel = servo_units[servo_num].servo_rotational_velocity;
		float delta_required = servo_units[servo_num].angle_delta_required;


		//PWM microseconds per radian of servo movement
		float microseconds_per_rad = (ninety_deg_pwm - zero_deg_pwm) / (M_PI / 2);


		/*
		Problem is when spinning in one direction, can apply batt voltage + back emf and get to super high net voltages
		Allows huge accelerations to be reached but doesn't take into account new back EMF from new velocities

		Need differential equation:
		D(Vel)/dT = Servo acceleration = net voltage * acc per volt = (max_acc_input_voltage - dc_motor_effective_kv * (Vel)) * acc per volt

		D(Vel)/dT = max_acc_input_voltage * acc per volt - acc per volt * dc_motor_effective_kv * (Vel)

		Say:
		A = max_acc_input_voltage * acc per volt
		B = dc_motor_effective_kv * acc per volt

		DV/dT = A - B * V

		Solution to differential equation:
		V(t) = C * e ^ (B * t) + A/B
		V(0) = current_rot_vel = C + A/B
			-> C = current_rot_vel - A/B

		Velocity at end of loop time:
		V(loop_time) = [current_rot_vel - A/B] * e ^ (B * loop_time) + A/B

		Position vs. time (V(t) integration):
		P(time) = initial position + (C/B) * (1 - e^(-B * time)) + (A/B) * (time)

		Acceleration vs. time (V(t) differentiation):
		A(time) = - B * C * e ^(-B * time) 



		Rearranging solution for position as a function of time:

		delta_required = (C/B) * (1 - std::pow(e, (-B * loop_time))) + (A/B) * loop_time;

		delta_required = C * e_term / B   +   A * loop_time / B;

		delta_required * B = C * e_term + A * loop_time;

		Say:
		D = Delta required
		k = dc_motor_effective_kv
		n = acc_per_net_volt
		r = current_rot_vel
		v = input_voltage

		A = v * n
		B = k * n
		C = r - (v / k)

		Solving for v.

		D * k * n = (r - (v / k)) * e_term +  v * n * loop_time;

		D * k * n = r * e_term - (v / k) * e_term +  v * n * loop_time;

		D * k * n - r * e_term  =  - (v / k) * e_term +  v * n * loop_time;

		D * k * n - r * e_term  =  v * (- (1 / k) * e_term +  n * loop_time);

		[D * k * n - r * e_term]  / (- (1 / k) * e_term +  n * loop_time) = v

		input_voltage_required = [D * k * n - r * e_term]  / (- (1 / k) * e_term +  n * loop_time)


		*/
		float D = delta_required;
		float k = dc_motor_effective_kv;
		float n = acc_per_net_volt;
		float r = current_rot_vel;
		float e_term = (1 - std::pow(e, (-k * n * loop_time)));

		float input_voltage_required = (D * k * n - r * e_term)  / (- (1 / k) * e_term +  n * loop_time);

		float dc_motor_duty_cycle_required = input_voltage_required / servo_supply_voltage;

		float angle_error_required = dc_motor_duty_cycle_required / P_gain;

		float steady_state_angle_command_required = current_angle + angle_error_required;

		float PWM_micros = steady_state_angle_command_required * microseconds_per_rad + zero_deg_pwm;


		//Set pulse width
		servo_units[servo_num].PWM_micros = PWM_micros;

/*

		if (servo_num == 0){

			//Serial.println("Servo angle delta to pwm:");
			Serial.print(current_angle); Serial.print(" ");
			Serial.print(delta_required); Serial.print(" ");
			Serial.print(input_voltage_required); Serial.print(" ");
			Serial.print(angle_error_required); Serial.print(" ");
			Serial.print(PWM_micros); Serial.print(" ");
			Serial.println("");

		}


*/


	}





/*
	Input: series of pwms to all motors and servos
	Output: None

	Sets PWMs for each motor and servo on GPIO pins

*/
void Control::send_pwms()
{

	//Send motor PWMs to GPIOs
	for (int i=0; i<8; i++)
    	{

    		//Send pulse to GPIO pin
			send_microseconds(propulsion_units[i].gpio_pin, propulsion_units[i].PWM_micros);

		}
	

	/*
	//Send servo PWMs to GPIOs
	for (int i=0; i<4; i++)
    	{
			send_microseconds(servo_units[i].gpio_pin, servo_units[i].PWM_micros);

		}
*/

}



/*
Send pulse train of certain number of microseconds to a certain pin, at whatever frequency is being used
until rewritten by a new analogWrite.


To change the PWM frequency sent out, must change the Arduino library by rewriting PWM_FREQUENCY and TC_FREQUENCY in Variant.h
To get to Variant.h, go to Finder and click Go (at top), then click Library.  Then Arduino15 folder is near the bottom
In Arduino 15: packages/arduino/hardware/sam/1.6.11/variants/arduino_due_x/variant.h
Note: PWM_FREQUENCY only affects pins 6,7,8,9, and TC_FREQUENCY affects the rest.

To set frequency:
PWM_FREQUENCY and TC_FREQUENCY sets PWM frequency in Hz (confirmed with oscilloscope correct to within ~0.02 Hz sigma)

To set resolution:
8 bit is default.  Due hardware supports up to 12 bit.  To use 12 bit, set PWM_RESOLUTION and TC_RESOLUTION to 12.
Set PWM_MAX_DUTY_CYCLE and TC_MAX_DUTY_CYCLE to 1023.

In code, use:

analogWriteResolution(12);
analogWrite(13,1000) is 2247 us with frequency set to 435 Hz (sets max pulse width 2,299 us. * 1000/1023 = 2,247.16 us)

No need to do pinMode(x, OUTPUT); on any pins

So, resolution is 2.247 us, where analogWrite(x,1) sends a 2.247 us pulse at 435 Hz.  Possible values are all multiples of 2.24716 us (at 435 Hz).
	Set to 435 Hz bc estimated max pulse length would ever need to send would be ~2300 us (1,000,000/435 = 2,298.85 us)

*/

void Control::send_microseconds(int motor_pin, float microseconds)
{


	float PWM_frequency = 435; //PWM frequency 

	float PWM_period_micros = 1000000 / PWM_frequency;

	float precise_bit_value = microseconds / PWM_period_micros * 1023;

	int nearest_bit_value = int(precise_bit_value); //Changed from round() because Eigen library seems to redefine round().  Always "rounds" down.

	//Prevent from sending an always-on signal
	if (microseconds > PWM_period_micros){
		nearest_bit_value = 1022;
	}

	analogWriteResolution(12);
	analogWrite(motor_pin, nearest_bit_value);

}




/*
	Run the control system functions and send the output directly to the motors

*/
void Control::run(Location::LOCATION * Location, Receiver::RECEIVER * Receiver)
	{

		/*
		Get fan rotational velocities and servo angles that will be reached by the end of this control loop, at time commands are sent
		Note: Not possible to change.  Predetermined by existing accelerations and velocities at the beginning of this control loop.
		*/
		get_end_of_loop_rot_vels_and_servo_angles();


		/*
		Get motor operational limits given rotational velocity, inflow velocity, battery voltage, etc. at end of current loop
		*/
		for (int i=0; i<8; i++)
    	{
    		calculate_motor_operating_limits(i);
		}


		/*
		Update window on fan rotational velocity and servo angle deltas possible to 
		reach at end of next loop, after sending commands at the end of this loop
		*/
		get_max_rot_vel_and_servo_angle_deltas();

		
		/*
		Get desired accelerations based on location and receiver commands.  Not possible to reach in next timestep, but desired.
		*/
		acceleration_controller(Location, Receiver); //32 microseconds

	
		//Forces and moments required to generate accelerations from controller
		accelerations_to_forces_and_moments(); //6 microseconds


		//Get delta rotational velocities and delta servo angles that would be needed to generate desired moments (likely not possible in next timestep)
		get_theoretical_rot_vel_and_servo_angle_deltas();


		//Get commanded rotational velocity or servo angle delta, scaled down to desired deltas achievable in next timestep
		get_next_loop_rot_vel_and_servo_angle_deltas();


		//Get PWMs required to reach rotational velocity delta and servo angle delta
		rot_vel_and_servo_angle_deltas_to_pwms();


		//Send PWMs directly to GPIO pins
		send_pwms(); //87 microseconds



		int current_time = micros();
		loop_time = (current_time - commands_sent_time) / 1000000;
		commands_sent_time = current_time;

		//Serial.print("Loop: ");
		//Serial.println(loop_time, 6);



	}






void Control::MotorTest(Receiver::RECEIVER * Receiver)
	{

	int pulse = 10 + Receiver->thrust * 2400;

	//Send motor PWMs to GPIOs
	send_microseconds(FRONT_RIGHT_MOT_PIN, 1000); //25 - 1.5A, 50 - 5.7A, 75 - 13.9A, 100 - 25.0A
	send_microseconds(FRONT_LEFT_MOT_PIN, 1000); //25 - 1.5, 50 - 5.8, 75 - 14, 100 - 25.2

	send_microseconds(BACK_RIGHT_MOT_PIN, 1000); //25 - 0.9A, 50 - 4A, 75 - 10.3A, 100 - 19.1A
	send_microseconds(BACK_LEFT_MOT_PIN, 1000); //18.8A

	send_microseconds(BACK_MID_RIGHT_MOT_PIN, 1000); //18.3 A
	send_microseconds(BACK_MID_LEFT_MOT_PIN, 1000); //18.7A

	send_microseconds(BACK_FAR_RIGHT_MOT_PIN, 1000); //27.8 A
	send_microseconds(BACK_FAR_LEFT_MOT_PIN, 1000); //28.9 A


	if (Receiver->thrust > 0.5){

		digitalWrite(LED_BUILTIN, HIGH);
			//Send servo PWMs to GPIOs
	send_microseconds(FRONT_RIGHT_SERVO_PIN, 832); //2400 us sends so far below that almost hits itself.
	send_microseconds(FRONT_LEFT_SERVO_PIN, 832);

	send_microseconds(BACK_RIGHT_SERVO_PIN, 832);
	send_microseconds(BACK_LEFT_SERVO_PIN, 832);
	}

	else{
		digitalWrite(LED_BUILTIN, LOW);
		send_microseconds(FRONT_RIGHT_SERVO_PIN, 800); //2400 us sends so far below that almost hits itself.
		send_microseconds(FRONT_LEFT_SERVO_PIN, 800);

		send_microseconds(BACK_RIGHT_SERVO_PIN, 800);
		send_microseconds(BACK_LEFT_SERVO_PIN, 800);
	}


	Serial.print("Pulse sent: ");
	Serial.println(pulse);


}






void Control::print_mtxf(const Eigen::MatrixXf& X)  
{
   int i, j, nrow, ncol;
   
   nrow = X.rows();
   ncol = X.cols();

   //Serial.print("nrow: "); Serial.println(nrow);
   //Serial.print("ncol: "); Serial.println(ncol);       
   //Serial.println();
   
   for (i=0; i<nrow; i++)
   {
       for (j=0; j<ncol; j++)
       {
           Serial.print(X(i,j));   // print 6 decimal places
           Serial.print(", ");
       }
       Serial.println();
   }
   Serial.println();
}



/*
	Input: fan number -> fan rotational velocity at end of last timestep, last fan torque commanded, last loop time elapsed (as proxy for coming loop time)
	Output: None.

	Gets rotational velocity that should be reached by the fan at the end of this loop, at the time the commands are sent.  Not possible to change.
	Initialized at 0 rotational velocity.

*/
	/*
void Control::get_end_of_loop_fan_rotational_velocity(int fan_number)
	{

		float last_rot_vel = propulsion_units[fan_number].rotational_velocity;
		float excess_torque = propulsion_units[fan_number].motor_torque_required - get_fan_aero_torque(fan_number, last_rot_vel);
		float rot_acc = excess_torque / propulsion_units[fan_number].fan.fan_assembly_moment_of_inertia;

		float rot_vel = last_rot_vel + rot_acc * loop_time;

		propulsion_units[fan_number].rotational_velocity = rot_vel;

	}
*/



/*
	Input: motor parameters, current rad/sec, current torque, battery voltage
	Output: positive (spinning faster) torque available from given motor in Nm

*/
	/*
void Control::get_available_fan_motor_torques(int fan_number)
	{

		//Make readable variables for motor controller torque model
		float current_rot_vel = propulsion_units[fan_number].rotational_velocity;
		float P_gain = propulsion_units[fan_number].motor_controller.P_gain;
		float I_gain = propulsion_units[fan_number].motor_controller.I_gain;
		float rot_vel_error_integral = propulsion_units[fan_number].rot_vel_error_integral;
		float max_rot_vel = propulsion_units[fan_number].max_rotational_velocity;


		//Rotational velocity reached once in steady state at max pulse length
		float max_pos_rot_vel_error = max_rot_vel - current_rot_vel;
		float max_pos_next_rot_vel_error_integral = rot_vel_error_integral + max_pos_rot_vel_error * loop_time;

		//Rotational velocity reached once in steady state at max pulse length
		float max_neg_rot_vel_error = 0 - current_rot_vel;
		float max_neg_next_rot_vel_error_integral = rot_vel_error_integral + max_neg_rot_vel_error * loop_time;

		
		//Torque derived from motor's PI control of torque:
		//Torque = P_gain * rot_vel_error + I_gain * rot_vel_error_integral
		
		float max_positive_torque = P_gain * max_pos_rot_vel_error + I_gain * max_pos_next_rot_vel_error_integral;
		float max_negative_torque = P_gain * max_neg_rot_vel_error + I_gain * max_neg_next_rot_vel_error_integral;


		float max_torque_possible = propulsion_units[fan_number].max_torque_at_current_rot_vel;

		if (max_positive_torque > max_torque_possible){
			max_positive_torque = max_torque_possible;
		}

		if (max_negative_torque < -max_torque_possible){
			max_negative_torque = -max_torque_possible;
		}

		propulsion_units[fan_number].max_positive_torque = max_positive_torque;
		propulsion_units[fan_number].max_negative_torque = max_negative_torque;


	}

*/



/*
	Input: fan parameters, current rotational velocity (rad/sec), inflow velocity (m/s), inflow angle (radians from direct inflow), battery voltage, loop time (seconds)
	Output: positive rads per sec delta possible to command within the next loop time

*/
	/*
void Control::get_max_fan_rotational_velocity_deltas(int fan_number)
	{

		//Get available torque and make readable
		get_available_fan_motor_torques(fan_number);
		float max_positive_torque = propulsion_units[fan_number].max_positive_torque;
		float max_negative_torque = propulsion_units[fan_number].max_negative_torque;

		//Make readable rotational velocity at end of loop
		float rot_vel = propulsion_units[fan_number].rotational_velocity;

		//Calculate aero torque on motor at end of current loop
		float aero_torque = get_fan_aero_torque(fan_number, rot_vel);

		float max_pos_net_torque_available = max_positive_torque - aero_torque;
		float max_neg_net_torque_available = max_negative_torque - aero_torque;

		float max_positive_rot_vel_delta = max_pos_net_torque_available / propulsion_units[fan_number].fan.fan_assembly_moment_of_inertia * loop_time;
		float max_negative_rot_vel_delta = max_neg_net_torque_available / propulsion_units[fan_number].fan.fan_assembly_moment_of_inertia * loop_time;

		propulsion_units[fan_number].max_positive_rot_vel_delta = max_positive_rot_vel_delta;
		propulsion_units[fan_number].max_negative_rot_vel_delta = max_negative_rot_vel_delta;


	}
*/



/*
	//Aero thrust coefficients for all fans 
	fr_fan.fan_thrust_coefficient = 395;
	fl_fan.fan_thrust_coefficient = 395;
	br_fan.fan_thrust_coefficient = 395;
	bl_fan.fan_thrust_coefficient = 395;
	bmr_fan.fan_thrust_coefficient = 395;
	bml_fan.fan_thrust_coefficient = 395;
	bfr_fan.fan_thrust_coefficient = 395;
	bfl_fan.fan_thrust_coefficient =  395;


	//Aero torque coefficients for all fans
	fr_fan.fan_aero_torque_coefficient = 3427;
	fl_fan.fan_aero_torque_coefficient = 3427;
	br_fan.fan_aero_torque_coefficient = 3427;
	bl_fan.fan_aero_torque_coefficient = 3427;
	bmr_fan.fan_aero_torque_coefficient = 3427;
	bml_fan.fan_aero_torque_coefficient = 3427;
	bfr_fan.fan_aero_torque_coefficient = 3427;
	bfl_fan.fan_aero_torque_coefficient =  3427;


	//Fan and motor rotor moment of inertia in kg*m^2
	fr_fan.fan_assembly_moment_of_inertia = 0.000034;
	fl_fan.fan_assembly_moment_of_inertia = 0.000034;
	br_fan.fan_assembly_moment_of_inertia = 0.000034;
	bl_fan.fan_assembly_moment_of_inertia = 0.000034;
	bmr_fan.fan_assembly_moment_of_inertia = 0.000034;
	bml_fan.fan_assembly_moment_of_inertia = 0.000034;
	bfr_fan.fan_assembly_moment_of_inertia = 0.000034;
	bfl_fan.fan_assembly_moment_of_inertia = 0.000034;


	//Zero torque rotational velocity as fraction of zero net voltage rotational velocity
	fr_fan.motor_zero_torque_rot_vel_fraction = 0.8305;
	fl_fan.motor_zero_torque_rot_vel_fraction = 0.8305;
	br_fan.motor_zero_torque_rot_vel_fraction = 0.8305;
	bl_fan.motor_zero_torque_rot_vel_fraction = 0.8305;
	bmr_fan.motor_zero_torque_rot_vel_fraction = 0.8305;
	bml_fan.motor_zero_torque_rot_vel_fraction = 0.8305;
	bfr_fan.motor_zero_torque_rot_vel_fraction = 0.8305;
	bfl_fan.motor_zero_torque_rot_vel_fraction = 0.8305;


	//Theoretical torque per coil current at zero rotational velocity.  Coil current at zero rot vel = Batt_voltage (V) / coil resistance (Ohms)
	fr_fan.motor_zero_speed_torque_constant = 0.0037876;
	fl_fan.motor_zero_speed_torque_constant = 0.0037876;
	br_fan.motor_zero_speed_torque_constant = 0.0037876;
	bl_fan.motor_zero_speed_torque_constant = 0.0037876;
	bmr_fan.motor_zero_speed_torque_constant = 0.0037876;
	bml_fan.motor_zero_speed_torque_constant = 0.0037876;
	bfr_fan.motor_zero_speed_torque_constant = 0.0037876;
	bfl_fan.motor_zero_speed_torque_constant = 0.0037876;


	//Coil resistance at operating temperature (Ohms)
	fr_fan.coil_resistance = 0.083;
	fl_fan.coil_resistance = 0.083;
	br_fan.coil_resistance = 0.083;
	bl_fan.coil_resistance = 0.083;
	bmr_fan.coil_resistance = 0.083;
	bml_fan.coil_resistance = 0.083;
	bfr_fan.coil_resistance = 0.083;
	bfl_fan.coil_resistance = 0.083;


	//Motor kv (RPM/Volt). RPM/Volt used for ease of entry/verification, but always converted to [rad/sec] / volt  (divide by 9.5492966)
	fr_fan.motor_kv = 1500;
	fl_fan.motor_kv = 1500;
	br_fan.motor_kv = 1400;
	bl_fan.motor_kv = 1400;
	bmr_fan.motor_kv = 1400;
	bml_fan.motor_kv = 1400;
	bfr_fan.motor_kv = 1500;
	bfl_fan.motor_kv = 1500;


	//Motor controller input exponent. Current = Max current * (pulse fraction) ^ (input exponent)
	fr_fan.motor_controller_input_exponent = 2.35;
	fl_fan.motor_controller_input_exponent = 2.35;
	br_fan.motor_controller_input_exponent = 2.35;
	bl_fan.motor_controller_input_exponent = 2.35;
	bmr_fan.motor_controller_input_exponent = 2.35;
	bml_fan.motor_controller_input_exponent = 2.35;
	bfr_fan.motor_controller_input_exponent = 2.35;
	bfl_fan.motor_controller_input_exponent = 2.35;


	//Minimum PWM that sends current to motor - 1040 shown as default for BLHeli app for Mac
	fr_fan.min_pwm = 1050;
	fl_fan.min_pwm = 1050;
	br_fan.min_pwm = 1020;
	bl_fan.min_pwm = 1020;
	bmr_fan.min_pwm = 1020;
	bml_fan.min_pwm = 1020;
	bfr_fan.min_pwm = 1050;
	bfl_fan.min_pwm = 1050;


	//Maximum PWM, no current restriction - 1960 also shown as default on BLHeli app for Mac
	fr_fan.max_pwm = 1960;
	fl_fan.max_pwm = 1960;
	br_fan.max_pwm = 1960;
	bl_fan.max_pwm = 1960;
	bmr_fan.max_pwm = 1960;
	bml_fan.max_pwm = 1960;
	bfr_fan.max_pwm = 1960;
	bfl_fan.max_pwm = 1960;

	//Using 1960 us as max PWM input

	//Front right fan parameters.  Likely ~2.29 lbs. max thrust if 25.0A max (^(2/3) from 29.1A -> 2.53 lbs).  Would be 2.104 lbs. at 11.1V.
	fr_fan.voltage1 = 11.1;
	fr_fan.voltage1_min_pwm = 1050;
	fr_fan.voltage1_hover_thrust_exponent = 1.55;
	fr_fan.voltage1_hover_thrust_curve_coefficient = 565;
	fr_fan.voltage2 = 12.6;
	fr_fan.voltage2_min_pwm = 1050;
	fr_fan.voltage2_hover_thrust_exponent = 1.5;
	fr_fan.voltage2_hover_thrust_curve_coefficient = 525;


	//Front left fan parameters - 25.0A
	fl_fan.voltage1 = 11.1;
	fl_fan.voltage1_min_pwm = 1050;
	fl_fan.voltage1_hover_thrust_exponent = 1.55;
	fl_fan.voltage1_hover_thrust_curve_coefficient = 565;
	fl_fan.voltage2 = 12.6;
	fl_fan.voltage2_min_pwm = 1050;
	fl_fan.voltage2_hover_thrust_exponent = 1.5;
	fl_fan.voltage2_hover_thrust_curve_coefficient = 525;


	//Back right fan parameters - 19.1A max on power supply (~0.3A less - bc sensors, etc.).  2.00 lbs. max theoret. 1.68 lbs. max at 11.1V 
	br_fan.voltage1 = 11.1;
	br_fan.voltage1_min_pwm = 1020;
	br_fan.voltage1_hover_thrust_exponent = 1.65;
	br_fan.voltage1_hover_thrust_curve_coefficient = 685;
	br_fan.voltage2 = 12.6;
	br_fan.voltage2_min_pwm = 1020;
	br_fan.voltage2_hover_thrust_exponent = 1.61;
	br_fan.voltage2_hover_thrust_curve_coefficient = 610;


	//Back left fan parameters - 18.8A max on power supply.  1.98 lbs. max at 12.6V.  1.67 lbs. at 11.1V.
	bl_fan.voltage1 = 11.1;
	bl_fan.voltage1_min_pwm = 1020;
	bl_fan.voltage1_hover_thrust_exponent = 1.65;
	bl_fan.voltage1_hover_thrust_curve_coefficient = 660;
	bl_fan.voltage2 = 12.6;
	bl_fan.voltage2_min_pwm = 1020;
	bl_fan.voltage2_hover_thrust_exponent = 1.61;
	bl_fan.voltage2_hover_thrust_curve_coefficient = 690;


	//Back mid right fan parameters - 18.3A max (-> 18.1A) on power supply.  1.96 lbs. max at 12.6V.  1.66 lbs. max at 11.1V.
	bmr_fan.voltage1 = 11.1;
	bmr_fan.voltage1_min_pwm = 1020;
	bmr_fan.voltage1_hover_thrust_exponent = 1.65;
	bmr_fan.voltage1_hover_thrust_curve_coefficient = 692;
	bmr_fan.voltage2 = 12.6;
	bmr_fan.voltage2_min_pwm = 1020;
	bmr_fan.voltage2_hover_thrust_exponent = 1.61;
	bmr_fan.voltage2_hover_thrust_curve_coefficient = 618;


	//Back mid left fan parameters - 18.7A (-> 18.5A) max on power supply. 1.99 lbs. max at 12.6V.  1.68 lbs. max at 11.1V.
	bml_fan.voltage1 = 11.1;
	bml_fan.voltage1_min_pwm = 1020;
	bml_fan.voltage1_hover_thrust_exponent = 1.65;
	bml_fan.voltage1_hover_thrust_curve_coefficient = 686;
	bml_fan.voltage2 = 12.6;
	bml_fan.voltage2_min_pwm = 1020;
	bml_fan.voltage2_hover_thrust_exponent = 1.61;
	bml_fan.voltage2_hover_thrust_curve_coefficient = 613;


	//Back far right fan parameters - 27.8A (-> 27.6A) max on power supply.  2.44 lbs. max at 12.6V.  2.06 lbs. max at 11.1V.
	bfr_fan.voltage1 = 11.1;
	bfr_fan.voltage1_min_pwm = 1050;
	bfr_fan.voltage1_hover_thrust_exponent = 1.55;
	bfr_fan.voltage1_hover_thrust_curve_coefficient = 572;
	bfr_fan.voltage2 = 12.6;
	bfr_fan.voltage2_min_pwm = 1050;
	bfr_fan.voltage2_hover_thrust_exponent = 1.5;
	bfr_fan.voltage2_hover_thrust_curve_coefficient = 502; 


	//Back far left fan parameters - 28.9A max (-> 28.7A) on power supply.  2.51 lbs. max at 12.6V.  2.12 lbs. max at 11.1V.
	bfl_fan.voltage1 = 11.1;
	bfl_fan.voltage1_min_pwm = 1050;
	bfl_fan.voltage1_hover_thrust_exponent = 1.55;
	bfl_fan.voltage1_hover_thrust_curve_coefficient = 560;
	bfl_fan.voltage2 = 12.6;
	bfl_fan.voltage2_min_pwm = 1050;
	bfl_fan.voltage2_hover_thrust_exponent = 1.5;
	bfl_fan.voltage2_hover_thrust_curve_coefficient = 492;
*/




/*
	Input: Servo parameters, servo angle in radians
	Output: PWM signal length in exact microseconds that will result in input angle after transient

*/
	/*
float Control::servo_angle_to_pwm(Control::SERVO_UNIT * Servo_Unit, float servo_angle)
	{

	//Fraction from vertical angle (0) to horizontal (1).  Could be negative to indicate backwards tilt or >1 when past horizontal.
	float angle_fraction = servo_angle / (M_PI / 2);

	//Microsecond command interpolated/extrapolated from zero and ninety degree values
	float PWM_micros = Servo_Unit->zero_deg_pwm + (Servo_Unit->ninety_deg_pwm - Servo_Unit->zero_deg_pwm) * angle_fraction;

	return PWM_micros;

	}
*/



/*
	Input: Series of forces (N) from all fans, series of fan angles (rads), motor directions, motor x/y locations, motor torque per thrust
	Output: None.  Update kinematics_derivatives variable.

*/
	/*
MatrixXf Control::calculate_kinematics_derivatives(MatrixXf control_guess)
	{

		float motor_thrusts[8] = {control_guess(0,0), control_guess(1,0), control_guess(2,0), control_guess(3,0), control_guess(4,0), control_guess(5,0), control_guess(6,0), control_guess(7,0)};

		//Servo angles mapped to match with array of motors
		float servo_angles_mapped[8] = {control_guess(8,0), control_guess(9,0), control_guess(10,0), control_guess(11,0), control_guess(10,0), control_guess(11,0), control_guess(10,0), control_guess(11,0)};

		//Derivatives of 3 moments and 2 forces with respect to thrust and servo angle
		MatrixXf kinematics_derivatives(12,5);

		//Derivatives of moments and forces with respect to servo angle for each motor
		MatrixXf servo_kinematics_derivatives(8,5);

		//Get derivatives of roll, pitch, and yaw moments, x and z thrusts with respect to motor thrust at given thrust point and servo angle point for each motor
		for (int i=0; i<8; i++)
    	{

			kinematics_derivatives(i,0) = -motor_x_locs[i] * cos(servo_angles_mapped[i]) + motor_directions[i] * motor_torque_per_thrust * sin(servo_angles_mapped[i]);

			kinematics_derivatives(i,1) = motor_y_locs[i] * cos(servo_angles_mapped[i]);

			kinematics_derivatives(i,2) = -motor_x_locs[i] * sin(servo_angles_mapped[i]) - motor_directions[i] * motor_torque_per_thrust * cos(servo_angles_mapped[i]);

			kinematics_derivatives(i,3) = sin(servo_angles_mapped[i]);

			kinematics_derivatives(i,4) = cos(servo_angles_mapped[i]);

		}


		//Get derivatives of roll, pitch, and yaw moments, z and x thrusts with respect to servo angle at given thrust point and servo angle for each motor
		for (int i=0; i<8; i++)
    	{

			servo_kinematics_derivatives(i,0) = motor_thrusts[i] * motor_x_locs[i] * sin(servo_angles_mapped[i]) + motor_directions[i] * motor_torque_per_thrust * motor_thrusts[i] * cos(servo_angles_mapped[i]);

			servo_kinematics_derivatives(i,1) = -motor_thrusts[i] * motor_y_locs[i] * sin(servo_angles_mapped[i]);

			servo_kinematics_derivatives(i,2) = -motor_thrusts[i] * motor_x_locs[i] * cos(servo_angles_mapped[i]) + motor_directions[i] * motor_torque_per_thrust * motor_thrusts[i] * sin(servo_angles_mapped[i]);

			servo_kinematics_derivatives(i,3) = motor_thrusts[i] * cos(servo_angles_mapped[i]);

			servo_kinematics_derivatives(i,4) = -motor_thrusts[i] * sin(servo_angles_mapped[i]);

		}


		kinematics_derivatives.row(8) = servo_kinematics_derivatives.row(0);
		kinematics_derivatives.row(9) = servo_kinematics_derivatives.row(1);
		kinematics_derivatives.row(10) = servo_kinematics_derivatives.row(2) + servo_kinematics_derivatives.row(4) + servo_kinematics_derivatives.row(6);
		kinematics_derivatives.row(11) = servo_kinematics_derivatives.row(3) + servo_kinematics_derivatives.row(5) + servo_kinematics_derivatives.row(7);


		//Serial.println("Kinematics Derivatives: ");
		//print_mtxf(kinematics_derivatives);

		return kinematics_derivatives;

	}
*/



/*
	Inputs:
		-Series of target moments and forces from fans
		-Estimated fan thrust and servo angle required to generate given target moments and forces
		-Motor directions, motor x/y locations, motor torque per thrust

	Output: None.  Update estimated_control_input variable.

*/
	/*
MatrixXf Control::calculate_gradient_descended_control_guess(MatrixXf control_guess, MatrixXf target_forces_and_moments)
	{


		//200 micros on first iteration, 520 micros on second iteration
		MatrixXf theoretical_forces_and_moments_from_control_guess = calculate_theoretical_fan_forces_and_moments(control_guess);
		//Serial.print("Theoretical Forces: ");
		//print_mtxf(theoretical_forces_and_moments_from_control_guess);




		MatrixXf deltas_to_target_forces_and_moments(1,5);
		deltas_to_target_forces_and_moments = target_forces_and_moments - theoretical_forces_and_moments_from_control_guess;
		//Serial.print("Deltas to Target Forces: ");
		//print_mtxf(deltas_to_target_forces_and_moments);


		//Serial.print("Control guess: ");
		//print_mtxf(control_guess);



		//360 micros on first iteration, 960 on second iteration (more as get closer?)
		MatrixXf kinematics_derivatives = get_kinematics_derivatives();
		//Serial.println("Kinematics derivatives: ");
		//print_mtxf(kinematics_derivatives);


		//Takes around 800-1000 microseconds
		MatrixXf estimated_control_input_deltas_from_control_guess = kinematics_derivatives.transpose().completeOrthogonalDecomposition().solve(deltas_to_target_forces_and_moments.transpose());
		//Serial.print("Estimated input deltas: ");
		//print_mtxf(estimated_control_input_deltas_from_control_guess);



		//completeOrthogonalDecomposition() - 19,091 micros entire control loop
			//Solution deltas for location all zeroes, commanded moments -0.78, 2.32, 0.64, 0.00, 58.00:
			//1.33, 1.05, -0.29, -0.51, -0.15, -0.64, -0.02, -0.78, -0.08, 0.08, -0.02, 0.02 (Probably gold standard - same as Octave)
		//fullPivLu() - 9,970 micros entire loop
			//Solution deltas for location all zeroes, commanded moments -0.78, 2.32, 0.64, 0.00, 58.00:
			//-2.38, 0.00, -1.54, 0.00, 0.00, 0.00, 0.00, -0.84, 0.00, 0.00, -0.04, 0.04
		//colPivHouseholderQr() - 14,513 micros entire loop
			//Solution deltas for location all zeroes, commanded moments -0.78, 2.32, 0.64, 0.00, 58.00:
			//2.38, 0.00, 0.00, 0.00, 0.00, 0.00, -0.99, -1.39, 0.00, 0.00, -0.04, 0.04
		//fullPivHouseholderQr() - 11783 micros entire loop
			//Solution deltas for location all zeroes, commanded moments -0.78, 2.32, 0.64, 0.00, 58.00:
			//2.38, 0.00, -1.54, 0.00, 0.00, 0.00, 0.00, -0.84, 0.00, 0.00, -0.04, 0.04
		//partialPivLu() - 6,252 micros entire loop. !!Doesn't work. Matrix must be invertible, can't find solution for sample initial condition.
		//householderQr() - 9762 micros entire loop.  !!Doesn't work.  Some matrix values are NaN.
		//llt() - doesn't run.  Matrix must be positive definite.
		//ldlt() - doesn't run.  Matrix must be positive or negative semidefinite.
		


		MatrixXf estimated_control_input(12,1);
		estimated_control_input = control_guess + estimated_control_input_deltas_from_control_guess;
		//Serial.print("Estimated inputs: ");
		//print_mtxf(estimated_control_input);


		return estimated_control_input;


	}

*/



/*
	Input: roll, pitch, yaw moments (Nm), z thrust (N), x thrust (N)
	Output: series of motor thrusts (N) and servo angles (radians) that will result in input forces and moments after transient

*/
	/*
void Control::forces_and_moments_to_thrusts_and_angles(Control::FORCES_AND_MOMENTS * Target_Forces_and_Moments)
	{

		//Initial guess for servo angles in radians from forces required
		float avg_servo_angle = atan2(Target_Forces_and_Moments->x_force, Target_Forces_and_Moments->z_force);
		float avg_thrust = sqrt(pow(Target_Forces_and_Moments->x_force, 2) + pow(Target_Forces_and_Moments->z_force,2)) / 8;

		//Initial guess at motor thrusts and servo angles that will generate the target moments and forces
		MatrixXf control_guess(1,12);

		for (int i=0; i<8; i++)
    	{
    		//Thrust (Nm) = 1 / (coeff ^ 2) * rot_vel ^ 2
    		//sqrt(Thrust * coeff^2) = rot_vel
    		control_guess(0,i) = sqrt(avg_thrust * pow(fr_fan.fan_thrust_coefficient,2));
		}
		for (int i=8; i<12; i++)
    	{
    		control_guess(0,i) = avg_servo_angle;
		}


		//Serial.println("START: ");


		//Serial.print("Control guess: ");
		//print_mtxf(control_guess);
		
		

		//Derivatives of moments and forces with respect to servo angle for each motor
		MatrixXf target_forces_and_moments(1,5);
		target_forces_and_moments(0,0) = Target_Forces_and_Moments->roll_moment;
		target_forces_and_moments(0,1) = Target_Forces_and_Moments->pitch_moment;
		target_forces_and_moments(0,2) = Target_Forces_and_Moments->yaw_moment;
		target_forces_and_moments(0,3) = Target_Forces_and_Moments->x_force;
		target_forces_and_moments(0,4) = Target_Forces_and_Moments->z_force;
		//Serial.print("Target forces: ");
		//print_mtxf(target_forces_and_moments);


		//1460 micros for first one, 2600 micros for second one (harder as gets closer to answer?)
		MatrixXf new_guess = calculate_gradient_descended_control_guess(control_guess, target_forces_and_moments);
		//Serial.print("Guess #1: ");
		//print_mtxf(new_guess);




		MatrixXf estimated = calculate_theoretical_fan_forces_and_moments(new_guess);
		//Serial.print("Estimated forces #1: ");
		//print_mtxf(estimated);



		MatrixXf estimated_control_input = calculate_gradient_descended_control_guess(new_guess, target_forces_and_moments);
		//Serial.print("Guess #2: ");
		//print_mtxf(estimated_control_input);



		MatrixXf estimated2 = calculate_theoretical_fan_forces_and_moments(estimated_control_input);
		Serial.print("Estimated forces #2: ");
		print_mtxf(estimated2);



		//estimated_control_input = calculate_gradient_descended_control_guess(estimated_control_input, target_forces_and_moments);
		//Serial.print("Guess #3: ");
		//print_mtxf(estimated_control_input);


		//estimated = calculate_theoretical_fan_forces_and_moments(estimated_control_input);
		//Serial.print("Estimated forces #3: ");
		//print_mtxf(estimated);




		//Output thrusts from matrix in Newtons
		Thrusts_and_Angles.front_right_fan_thrust = estimated_control_input(0,0);
		Thrusts_and_Angles.front_left_fan_thrust = estimated_control_input(1,0);
		Thrusts_and_Angles.back_right_fan_thrust = estimated_control_input(2,0);
		Thrusts_and_Angles.back_left_fan_thrust = estimated_control_input(3,0);
		Thrusts_and_Angles.back_mid_right_fan_thrust = estimated_control_input(4,0);
		Thrusts_and_Angles.back_mid_left_fan_thrust = estimated_control_input(5,0);
		Thrusts_and_Angles.back_far_right_fan_thrust = estimated_control_input(6,0);
		Thrusts_and_Angles.back_far_left_fan_thrust = estimated_control_input(7,0);

		//Output servo angles from matrix in Radians
		Thrusts_and_Angles.front_right_servo_angle = estimated_control_input(8,0);
		Thrusts_and_Angles.front_left_servo_angle = estimated_control_input(9,0);
		Thrusts_and_Angles.back_right_servo_angle = estimated_control_input(10,0);
		Thrusts_and_Angles.back_left_servo_angle = estimated_control_input(11,0);

*/

/*
		Serial.println("");
		Serial.print("Newtons and Rads: ");
		Serial.print(Thrusts_and_Angles.front_right_fan_thrust); Serial.print(" ");
		Serial.print(Thrusts_and_Angles.front_left_fan_thrust); Serial.print(" ");
		Serial.print(Thrusts_and_Angles.back_right_fan_thrust); Serial.print(" ");
		Serial.print(Thrusts_and_Angles.back_left_fan_thrust); Serial.print(" ");
		Serial.print(Thrusts_and_Angles.back_mid_right_fan_thrust); Serial.print(" ");
		Serial.print(Thrusts_and_Angles.back_mid_left_fan_thrust); Serial.print(" ");
		Serial.print(Thrusts_and_Angles.back_far_right_fan_thrust); Serial.print(" ");
		Serial.print(Thrusts_and_Angles.back_far_left_fan_thrust); Serial.print(" ");

		Serial.print(Thrusts_and_Angles.front_right_servo_angle); Serial.print(" ");
		Serial.print(Thrusts_and_Angles.front_left_servo_angle); Serial.print(" ");
		Serial.print(Thrusts_and_Angles.back_right_servo_angle); Serial.print(" ");
		Serial.print(Thrusts_and_Angles.back_left_servo_angle); Serial.print(" ");
		Serial.println("");
	*/
		

//	}





		//Probably not needed - sections from thrust per second model of controller no longer needed:




		//Function tracks trajectory to max thrust
		//float max_acc_fraction = 0.45 + std::pow(current_rot_vel, 2) / 2800000; //0.4 at 0 to 1 at 1200, but scale with torque (square) -> 
		//if (std::abs(max_acc_fraction) > 1){
		//	max_acc_fraction = 1;
		//}

		//max_acc_fraction = 1;

		/*
		PI control of torque through rotational velocity feedback:
		Torque = P_gain * rot_vel_error + I_gain * rot_vel_error_integral
		*/
		
		/*
		rot_vel_error_integral = rot_vel_error_integral + rot_vel_error * loop_time;
		float initial_motor_torque = P_gain * rot_vel_error + I_gain * rot_vel_error_integral;

		//Constrain torque command to max possible torque at current rotational velocity
		if (std::abs(initial_motor_torque) > std::abs(propulsion_units[fan_number].max_torque_at_current_rot_vel)){
			initial_motor_torque = copysignf(propulsion_units[fan_number].max_torque_at_current_rot_vel, initial_motor_torque);
		}
		*/
	
		/*
		rot_vel_error_integral = rot_vel_error_integral + rot_vel_error * loop_time;
		float initial_motor_torque_fraction = P_gain * rot_vel_error + I_gain * rot_vel_error_integral;
		if (std::abs(initial_motor_torque_fraction) > 1){
			initial_motor_torque_fraction = copysignf(1, initial_motor_torque_fraction);
		}
		
		float initial_motor_torque = initial_motor_torque_fraction * propulsion_units[fan_number].max_torque_at_current_rot_vel;
		*/


		//float initial_excess_torque = initial_motor_torque - initial_aero_torque;

		//float initial_rot_acc = initial_excess_torque / propulsion_units[fan_number].fan.fan_assembly_moment_of_inertia;


		//Pulse fraction commmanded = (PWM - 1020) / (1960 - 1020)
		//Steady state current fraction commanded = (Pulse fraction commmanded ^ 2.35)
		//Steady state current commanded (A) = Steady state current fraction commanded * Max steady state current (A)



		//Input current at max rotational velocity:
		//Net voltage = Batt_voltage - [max rad/sec] / 146.61 //1400 kv = 146.61 rad/sec per volt
		//Max steady state pseudo-current (A) = Net voltage (V) / Motor coil resistance (ohms)
		//Max steady state pseudo-current (A) = [Batt_voltage - [max rad/sec] / 146.61 ] / Motor coil resistance (ohms)

		//Max steady state thrust (N) = ([max rad/s] / 395)^2 - (1/94.56)*(m/s inflow)^2
	
		//Max steady state power (W) = Net_voltage([rad/sec]) * Max steady state current (A)

		//Torque at max rpm (Nm) = 
		//Max output power (W) = Torque at max rpm (Nm) * [rad/sec]

		//Efficiency change vs. PWM seems to be too small to measure accurately - should probably assume calculated efficiency applies at all points
		//Efficiency = Max output power (W) / Max input power (W)

		//Steady state commanded output power (W) = Steady state current fraction commanded * Max output power (W)
		//




		//Max input power (W) = [Batt_voltage - [rad/sec] / 146.61 ] ^2  / Motor coil resistance (ohms)

		//Max theoretical torque doesn't take into account motor controller - only max current possible given coil resistance
		//Max theoretical torque (Nm) =  Max input power (W) / [rad/sec]
		//Max theoretical torque (Nm) =  [[Batt_voltage - [rad/sec] / 146.61 ] ^2]   /    [ Motor coil resistance (ohms) * [rad/sec] ]


		// ([rad/sec] / 3427)^2  =  [[Batt_voltage - [rad/sec] / 146.61 ] ^2]   /    [ Motor coil resistance (ohms) * [rad/sec] ]
		// Aero torque should eventually be a more sophisticated function of RPM and inflow velocity



		//Last N/sec commanded
		//If copysignf(last N/sec commanded, N/sec theo) > N/sec theo:
		//N/sec = last N/sec commanded
		//Else:
		//N/sec = N/sec theoretical


		//Excess torque (Nm) = 0.000034 * [rad / sec^2]
		//Aero torque (Nm) = ([rad/sec] / 3427)^2 (looks like within +/- 10% of correct value up to 40 mph in 8-13k rpm range)
		//Motor torque (Nm) = Excess torque + aero torque = 0.000034 * [rad / sec^2] + ([rad/sec] / 3427)^2

		//Motor torque (Nm) as function of current rotational vel and Newtons/sec command (to meet given Newtons/sec):
		//Motor torque (Nm) = 0.000034 * (Newtons/sec) * (395^2 / 2* [rad/sec]) + ([rad/sec] / 3427)^2

		//Can get Newtons/sec command required to generate given torque:
		// Newtons/sec = [Motor torque (Nm) - ([rad/sec] / 3427)^2]   /   [0.000034 * (395^2 / 2* [rad/sec])]

		//...



		//float aero_torque = get_fan_aero_torque(fan_number);

		//Excess torque required to achieve given total torque.  Hard to think about because command ultimately sent to motor is an excess torque
		//float excess_torque_required = motor_torque_required - aero_torque;

		//Rotational acceleration required to command to generate given torque
		//float rotational_acc_required = excess_torque_required / propulsion_units[fan_number].fan_assembly_moment_of_inertia;

		/*
		Calculate a Newtons/sec thrust increase/decrease that has the effect of generating the required rotational acceleration

		Newtons/sec can be mapped to rotational acceleration at a given rotational velocity:
		
		Thrust (N) = ([rad/s] / 395)^2 - (1/94.56)*(m/s inflow)^2

		dThrust/dRotVel = (2 * [rad/sec]) / (395^2)
		dRotVel / dTime = (rad/sec^2)
		dThrust/dTime = dThrust/dRotVel * dRotVel / dTime = (2 * [rad/sec]) / (395^2)) * (rad/sec^2)
		Rad/sec^2 needed = (Newtons/sec) * (395^2 / (2*[rad/sec]) )

		*/
		//float dThrust_wrt_RotVel = 2 * current_rot_vel / pow(propulsion_units[fan_number].fan_thrust_coefficient, 2);

		//Newtons per second required to command to generate given total motor torque
		//float dThrust_wrt_time_required = dThrust_wrt_RotVel * rotational_acc_required;



		//PWM maps to N/sec thrust, as:
		//N/sec theo = (Thrust (N) at steady state associated with PWM - current Thrust (N)) / 0.25 seconds
		//Current Thrust (N) = ([current rad/s] / 395)^2 - (1/94.56)*(m/s inflow)^2
		//Max steady state thrust (N) = ([max rad/s] / 395)^2 - (1/94.56)*(m/s inflow)^2

		//Current fan thrust (Newtons)
		//float current_thrust = get_fan_thrust(fan_number);


		//Steady-state thrust required to command to set motor torque to given value
		//float steady_state_thrust_required = dThrust_wrt_time_required * motor_controller_transient + current_thrust;


		//Thrust (N) = ([rad/s] / 395)^2 - (1/94.56)*(m/s inflow)^2
		//[Thrust (N) + (1/94.56)*(m/s inflow)^2] * 395^2 = [rad/s]^2
		//[rad/sec] = sqrt( [Thrust (N) + (1/94.56 = 0.010575)*(m/s inflow)^2] * 395^2  )
		//Steady state rotational velocity resulting in given steady state thrust target
		//float steady_state_rot_vel_required = sqrt((steady_state_thrust_required + 0.010575 * pow(inflow_velocity,2)) * pow(propulsion_units[fan_number].fan_thrust_coefficient,2));






/*
	Input: series of thrusts to all motors in Newtons
	Output: series of pwms that will result in input thrusts after transient

	Mapping of pwm to thrust as a function of:
		-Fan unit type
		-Battery voltage
		-Inflow velocity
		-Inflow angle

*/
/*
void Control::thrusts_and_angles_to_pwms(Control::THRUSTS_AND_ANGLES * Thrusts_and_Angles)
	{


	float batt_voltage = 12.0; //Set to actual value in future

		

	//1589 micros for all 8 to execute
	propulsion_units[0].PWM_micros = motor_thrust_to_pwm(&fr_fan, Thrusts_and_Angles->front_right_fan_thrust, batt_voltage);
	propulsion_units[1].PWM_micros = motor_thrust_to_pwm(&fl_fan, Thrusts_and_Angles->front_left_fan_thrust, batt_voltage);

	propulsion_units[2].PWM_micros = motor_thrust_to_pwm(&br_fan, Thrusts_and_Angles->back_right_fan_thrust, batt_voltage);
	propulsion_units[3].PWM_micros = motor_thrust_to_pwm(&bl_fan, Thrusts_and_Angles->back_left_fan_thrust, batt_voltage);

	propulsion_units[4].PWM_micros = motor_thrust_to_pwm(&bmr_fan, Thrusts_and_Angles->back_mid_right_fan_thrust, batt_voltage);
	propulsion_units[5].PWM_micros = motor_thrust_to_pwm(&bml_fan, Thrusts_and_Angles->back_mid_left_fan_thrust, batt_voltage);

	propulsion_units[6].PWM_micros = motor_thrust_to_pwm(&bfr_fan, Thrusts_and_Angles->back_far_right_fan_thrust, batt_voltage);
	propulsion_units[7].PWM_micros = motor_thrust_to_pwm(&bfl_fan, Thrusts_and_Angles->back_far_left_fan_thrust, batt_voltage);
	

	servo_units[0].PWM_micros = servo_angle_to_pwm(&front_right_servo, Thrusts_and_Angles->front_right_servo_angle);
	servo_units[1].PWM_micros = servo_angle_to_pwm(&front_left_servo, Thrusts_and_Angles->front_left_servo_angle);
	servo_units[2].PWM_micros = servo_angle_to_pwm(&back_right_servo, Thrusts_and_Angles->back_right_servo_angle);
	servo_units[3].PWM_micros = servo_angle_to_pwm(&back_left_servo, Thrusts_and_Angles->back_left_servo_angle);


	PWM_Output.front_right_servo_micros = servo_angle_to_pwm(&front_right_servo, 0);
	PWM_Output.front_left_servo_micros = servo_angle_to_pwm(&front_left_servo, 0);
	PWM_Output.back_right_servo_micros = servo_angle_to_pwm(&back_right_servo, 0);
	PWM_Output.back_left_servo_micros = servo_angle_to_pwm(&back_left_servo, 0);




	Serial.println("");
	Serial.print("PWM: ");
	Serial.print(PWM_Output.front_right_fan_micros); Serial.print(" ");
	Serial.print(PWM_Output.front_left_fan_micros); Serial.print(" ");
	Serial.print(PWM_Output.back_right_fan_micros); Serial.print(" ");
	Serial.print(PWM_Output.back_left_fan_micros); Serial.print(" ");

	Serial.print(PWM_Output.back_mid_right_fan_micros); Serial.print(" ");
	Serial.print(PWM_Output.back_mid_left_fan_micros); Serial.print(" ");
	Serial.print(PWM_Output.back_far_right_fan_micros); Serial.print(" ");
	Serial.print(PWM_Output.back_far_left_fan_micros); Serial.print(" ");

	Serial.print(PWM_Output.front_right_servo_micros); Serial.print(" ");
	Serial.print(PWM_Output.front_left_servo_micros); Serial.print(" ");
	Serial.print(PWM_Output.back_right_servo_micros); Serial.print(" ");
	Serial.print(PWM_Output.back_left_servo_micros); Serial.print(" ");
	Serial.println("");


	}
*/



/*
	Input: Motor parameters, thrust in Newtons, battery voltage (probably airspeed, inflow angle, etc in future)
	Output: PWM signal length in exact microseconds that will result in input thrust after transient

	Time: 196 microseconds (timed from inside function, stopping right before "return").  Calculating pow() takes >90% of the time in this function.

*/
/*
float Control::motor_thrust_to_pwm(Control::PROPULSION_UNIT * Propulsion_Unit, float motor_thrust, float batt_voltage)
	{


	//Fraction of from min to max voltage
	float voltage_fraction = (batt_voltage - Propulsion_Unit->voltage1) / (Propulsion_Unit->voltage2 - Propulsion_Unit->voltage1);

	//Interpolate between voltages 1 and 2 for all values
	float min_pwm_thrust = Propulsion_Unit->voltage1_min_pwm + (Propulsion_Unit->voltage2_min_pwm - Propulsion_Unit->voltage1_min_pwm) * Propulsion_Unit->voltage_fraction;

	float operating_voltage_hover_thrust_exponent = Propulsion_Unit->voltage1_hover_thrust_exponent + (Propulsion_Unit->voltage2_hover_thrust_exponent - Propulsion_Unit->voltage1_hover_thrust_exponent) * Propulsion_Unit->voltage_fraction;

	float operating_voltage_hover_thrust_curve_coefficient = Propulsion_Unit->voltage1_hover_thrust_curve_coefficient + (Propulsion_Unit->voltage2_hover_thrust_curve_coefficient - Propulsion_Unit->voltage1_hover_thrust_curve_coefficient) * Propulsion_Unit->voltage_fraction;

	
	//Changing to a function that doesn't have pow() reduces total function time from 196 micros to 12 micros
	float PWM_micros = pow((motor_thrust / 4.448), (1/operating_voltage_hover_thrust_exponent)) * operating_voltage_hover_thrust_curve_coefficient + min_pwm_thrust;

	if (motor_thrust == 0){
		PWM_micros = 1000;
	}

	return PWM_micros;

	}

*/



void Control::run2(Location::LOCATION * Location, Receiver::RECEIVER * Receiver)
	{




	float Pitch_P = 0.32;
	float Roll_P = 0.08;

	float Pitch_D = 0.015;
	float Roll_D = 0.007;

	float fr_x = 37.5; //X is front-back axis, front is +
	float fr_y = 9; //Y is left-right axis, right is +

	float fl_x = 37.5;
	float fl_y = -9;

	float brc_x = -12.5;
	float brc_y = 15;

	float blc_x = -12.5;
	float blc_y = -15;


	float max_roll_angle = 15; //Degrees
	float roll_target = Receiver->roll * max_roll_angle;
	//roll_target = 0; //CHANGE!!!

	float max_pitch_angle = 25; //Degrees
	float pitch_target = Receiver->pitch * max_pitch_angle;
	//pitch_target = 0; //CHANGE!!!



	//Something quick to stabilize yaw
	float yaw_slew = 0; //Max rate of change in desired yaw - degrees per second
	dt = micros() - l_time;
	//Handle problem where very first dt is huge because l_time starts at 0.
	if (l_time == 0){
		dt = 10000;
	}
	l_time = micros();
	float desired_yaw_delta = yaw_slew * Receiver->yaw * dt / 1000000;
	//desired_yaw_delta = 0; //CHANGE!!!

	//desired_yaw = desired_yaw + desired_yaw_delta; //Desired yaw in degrees from 0 to 360.  For slewing desired yaw angle.  Hard to control.

	float max_yaw_delta = 20; //Max desired deviation from starting yaw.  Yaw in center always desired yaw = starting yaw.
	desired_yaw = Receiver->yaw * max_yaw_delta;
	desired_yaw = 0; //CHANGE!!!

	if (desired_yaw > 360){
		desired_yaw = desired_yaw - 360;
	}
	if(desired_yaw < 0){
		desired_yaw = desired_yaw + 360;
	}

	//Yaw error from -360 to 360.  So if at 20 deg and desired yaw is 10 deg, should be 10.  At 0, should be -10.
	float yaw_error = Location->yaw - desired_yaw;
	if (yaw_error > 180){
		yaw_error = yaw_error - 360;
	}
	if(yaw_error < -180){
		yaw_error = yaw_error + 360;
	}



	//Rate control method - try to get to a desired roll, pitch, yaw rate.  Probably should be some function of error (Ardupilot does sqrt)
	//Linear around 0, then goes to square root (or else very fast rate change around 0)
	//Even without any D gain, this is super stable - disturbances have a little shake but not much
	float pitch_rate_target;
	if (std::abs(pitch_target - Location->pitch) > 2){
		//copysignf (f for float) makes value of first argument the sign (+ or -) of second argument
		pitch_rate_target = 6 * copysignf(sqrt(std::abs(pitch_target - Location->pitch)-1), (pitch_target - Location->pitch));//5 deg->12dps, 10dps>18deg/s
	}
	else{
		pitch_rate_target = (pitch_target - Location->pitch) * 3;
	}

	float roll_rate_target;
	if (std::abs(roll_target - Location->roll) > 2){
		//copysignf (f for float) makes value of first argument the sign (+ or -) of second argument
		roll_rate_target = 6 * copysignf(sqrt(std::abs(roll_target - Location->roll)), (roll_target - Location->roll)); //If rolled right, should be neg
	}
	else{
		roll_rate_target = (roll_target - Location->roll) * 3;
	}
	

	//roll_rate_target = roll_rate_target * 3 * Receiver->dial1;


	float yaw_rate_target;
	if (std::abs(yaw_error) > 2){
		//copysignf (f for float) makes value of first argument the sign (+ or -) of second argument
		yaw_rate_target = 6 * copysignf(sqrt(std::abs(yaw_error)), (-yaw_error)); //Negative if yawed right
	}
	else{
		yaw_rate_target = (-yaw_error) * 3;
	}
	yaw_rate_target = constrain(yaw_rate_target, -45, 45);


	Pitch_Rate_P = 0.03; //How much to accelerate towards target rate.  More than about 0.03 on Deathtrap and starts to shake.
	Roll_Rate_P = 0.03; //0.1, with sqrt p = 6, -> 1.8 for 10 deg with no rate (3 and 1 for 10 deg, no rate PID pitch, roll)
	Yaw_Rate_P = 0.09;



	//Serial.println(" ");
	//Serial.print("Roll_Rate_P = ");  Serial.print(Roll_Rate_P);

	float pitch_rate_error = (Location->pitch_rate - pitch_rate_target);
	float roll_rate_error = (Location->roll_rate - roll_rate_target); //If rolled right at no rate, should be positive
	float yaw_rate_error = (Location->yaw_rate - yaw_rate_target); //If yawed right and no rate, should be positive



	float pitch_rate_error_deriv = (pitch_rate_error - pitch_rate_error_3) / (3*dt / 1000000); //

	float roll_rate_error_deriv = (roll_rate_error - roll_rate_error_3) / (3*dt / 1000000);



	//Adding in D term for Location->pitch_rate, etc. always makes more shaky - tried adjusting with dial from 0-0.04.  0 best.
	//For rate_error_derivs, starts to shudder around 0.0015.  0.0005 no real shudder but makes control feel tighter (oscillations immediately damped)
	//pitch_rate_error_int * 0.0015, above 0.0015 it starts oscillations
	float Pitch_Thrust = pitch_rate_error * Pitch_Rate_P + pitch_rate_error_int * 0 + pitch_rate_error_deriv * 0.0005;
	//Pitch_Thrust = 0;

	float Roll_Thrust = roll_rate_error * Roll_Rate_P + roll_rate_error_deriv * 0.0005;
	//Roll_Thrust = 0;

	float Yaw_Thrust = yaw_rate_error * Yaw_Rate_P; //If yawed right and no rate, should be positive
	//Yaw_Thrust = 0;

	//Record most recent roll, pitch rates.  Super stable with just P rate gain
	roll_rate_error_3 = roll_rate_error_2;
	roll_rate_error_2 = roll_rate_error_1;
	roll_rate_error_1 = roll_rate_error;

	pitch_rate_error_3 = pitch_rate_error_2;
	pitch_rate_error_2 = pitch_rate_error_1;
	pitch_rate_error_1 = pitch_rate_error;

	yaw_rate_error_3 = yaw_rate_error_2;
	yaw_rate_error_2 = yaw_rate_error_1;
	yaw_rate_error_1 = yaw_rate_error;

	roll_rate_3 = roll_rate_2;
	roll_rate_2 = roll_rate_1;
	roll_rate_1 = Location->roll_rate;

	pitch_rate_3 = pitch_rate_2;
	pitch_rate_2 = pitch_rate_1;
	pitch_rate_1 = Location->pitch_rate;

	yaw_rate_3 = yaw_rate_2;
	yaw_rate_2 = yaw_rate_1;
	yaw_rate_1 = Location->yaw_rate;



	//PD method
	/*
	float Pitch_Thrust = (Location->pitch - pitch_target) * Pitch_P + Location->pitch_rate * Pitch_D;

	float Roll_Thrust = (Location->roll - roll_target) * Roll_P + Location->roll_rate * Roll_D; //If rolled right at no rate, should be positive

	float Yaw_Thrust = yaw_error * 0.08 + Location->yaw_rate * 0.015;
	*/


	//Pitch thrust should go from full value at 0 deg, half at 45 deg, none at 90 deg (try sinusoid later)
	float pitch_thrust_fr = Pitch_Thrust * (90 - Commands.front_right_servo) / 90;//0 degrees, use 0 roll tilt, at 45 deg, half, 90 deg, full tilt
	float pitch_thrust_fl = Pitch_Thrust * (90 - Commands.front_left_servo) / 90;
	float pitch_thrust_br = Pitch_Thrust * (90 - Commands.back_right_servo) / 90;
	float pitch_thrust_bl = Pitch_Thrust * (90 - Commands.back_left_servo) / 90;


	//Roll thrust should go from full value at 0 deg, half at 45 deg, none at 90 deg (try sinusoid later)
	float roll_thrust_fr = Roll_Thrust * (90 - Commands.front_right_servo) / 90;//0 degrees, use 0 roll tilt, at 45 deg, half, 90 deg, full tilt
	float roll_thrust_fl = Roll_Thrust * (90 - Commands.front_left_servo) / 90;
	float roll_thrust_br = Roll_Thrust * (90 - Commands.back_right_servo) / 90;
	float roll_thrust_bl = Roll_Thrust * (90 - Commands.back_left_servo) / 90;


	//Yaw thrust should go from nothing at 0 deg, half at 45 deg, full at 90 deg (try sinusoid later)
	float yaw_thrust_fr = Yaw_Thrust * (Commands.front_right_servo) / 90;
	float yaw_thrust_fl = Yaw_Thrust * (Commands.front_left_servo) / 90;
	float yaw_thrust_br = Yaw_Thrust * (Commands.back_right_servo) / 90;
	float yaw_thrust_bl = Yaw_Thrust * (Commands.back_left_servo) / 90;


	Commands.front_right = constrain(Receiver->thrust -pitch_thrust_fr / fr_x + roll_thrust_fr / fr_y + yaw_thrust_fr / fr_y, 0, 1);
	Commands.front_left = constrain(Receiver->thrust -pitch_thrust_fl / fl_x + roll_thrust_fl / fl_y + yaw_thrust_fl / fl_y, 0, 1);

	Commands.back_right = constrain(Receiver->thrust -pitch_thrust_br / brc_x + roll_thrust_br / brc_y + yaw_thrust_br / brc_y, 0, 1);
	Commands.back_left = constrain(Receiver->thrust -pitch_thrust_bl / blc_x + roll_thrust_bl / blc_y + yaw_thrust_bl / blc_y, 0, 1);

	Commands.back_mid_right = constrain(Receiver->thrust -pitch_thrust_br / brc_x + roll_thrust_br / brc_y + yaw_thrust_br / brc_y, 0, 1);
	Commands.back_mid_left = constrain(Receiver->thrust -pitch_thrust_bl / blc_x + roll_thrust_bl / blc_y + yaw_thrust_bl / blc_y, 0, 1);

	Commands.back_far_right = constrain(Receiver->thrust -pitch_thrust_br / brc_x + roll_thrust_br / brc_y + yaw_thrust_br / brc_y, 0, 1);
	Commands.back_far_left = constrain(Receiver->thrust -pitch_thrust_bl / blc_x + roll_thrust_bl / blc_y + yaw_thrust_bl / blc_y, 0, 1);

/*
	Serial.println(Receiver->thrust);
	Serial.println(Pitch_Thrust);
	Serial.println(Roll_Thrust);
	Serial.println(Yaw_Thrust);
*/


	//Arming: Aux1 (top far left) must go all the way down (can start anywhere) and all the way up to arm
	//If Aux1 starts down (1) or goes down (to 1), set Aux1_down true (always starts false)
	if (Receiver->aux1 < 0.3){
		Aux1_been_up = true;
	}
	
	if (Receiver->aux1 > 0.7){
		Aux1_been_down = true;
	}

	/*
	Arming off for now

	if (not Aux1_been_up || not Aux1_been_down){
		Commands.front_right = 0;
		Commands.front_left = 0;
		Commands.back_right = 0;
		Commands.back_left = 0;
		Commands.back_mid_right = 0;
		Commands.back_mid_left = 0;
		Commands.back_far_right = 0;
		Commands.back_far_left = 0;
	}
	*/


	if (Receiver->thrust < 0.1){

		Commands.front_right = 0;
		Commands.front_left = 0;
		Commands.back_right = 0;
		Commands.back_left = 0;
		Commands.back_mid_right = 0;
		Commands.back_mid_left = 0;
		Commands.back_far_right = 0;
		Commands.back_far_left = 0;

	}




	float PD_Yaw = yaw_rate_error * 0.2 + roll_rate_error_deriv * 0; //0.2
	float yaw_tilt_fr = PD_Yaw * (90 - Commands.front_right_servo) / 90;//0 degrees, use full yaw tilt, at 45 deg, half as much, 90 deg, none
	float yaw_tilt_fl = PD_Yaw * (90 - Commands.front_left_servo) / 90;
	float yaw_tilt_br = PD_Yaw * (90 - Commands.back_right_servo) / 90;
	float yaw_tilt_bl = PD_Yaw * (90 - Commands.back_left_servo) / 90;

	//Intuitively, if rolled 5 deg, might want 2-3 deg arm tilt (just guess based on yaw effectiveness), 45 deg/sec might need 4-5 deg tilt
	float PD_Roll_Tilt = roll_rate_error * 0.2 + roll_rate_error_deriv * 0; //0.2
	float roll_tilt_fr = PD_Roll_Tilt * (Commands.front_right_servo) / 90;//0 degrees, use 0 roll tilt, at 45 deg, half, 90 deg, full tilt
	float roll_tilt_fl = PD_Roll_Tilt * (Commands.front_left_servo) / 90;
	float roll_tilt_br = PD_Roll_Tilt * (Commands.back_right_servo) / 90;
	float roll_tilt_bl = PD_Roll_Tilt * (Commands.back_left_servo) / 90;

	//Just guess, if pitched up 5 deg, might want 2-3 deg arm tilt, 45 deg/sec might need 4-5 deg tilt
	float PD_Pitch_Tilt = pitch_rate_error * 0.6 + pitch_rate_error_deriv * 0; //0.3
	float pitch_tilt_fr = PD_Pitch_Tilt * (Commands.front_right_servo) / 90;//0 degrees, use 0 pitch tilt, at 45 deg, half, 90 deg, full tilt
	float pitch_tilt_fl = PD_Pitch_Tilt * (Commands.front_left_servo) / 90;
	float pitch_tilt_br = PD_Pitch_Tilt * (Commands.back_right_servo) / 90;
	float pitch_tilt_bl = PD_Pitch_Tilt * (Commands.back_left_servo) / 90;


	//Quick thing to limit slew rate on master tilt (controls still move at max rate)
	float max_tilt_rate_slow = 15; //Deg/sec max tilt rate of master_tilt
	float max_tilt_rate_fast = 30; //Deg/sec max tilt rate of master_tilt


	//If flipped up (0), move to forward position at slow speed (no matter what)
	//If flipped to the middle (0.5), move to hover position at slow speed (no matter what)
	//If flipped down (1), move to reverse thrust at fast speed (no matter what)
	if (Receiver->aux2 < 0.25){
		max_tilt_rate = max_tilt_rate_slow;
		desired_tilt = Receiver->dial1 * 100;
	}

	if (Receiver->aux2 >= 0.25 && Receiver->aux2 <= 0.75){
		max_tilt_rate = max_tilt_rate_slow;
		desired_tilt = 25;
	}

	if (Receiver->aux2 > 0.75){
		max_tilt_rate = max_tilt_rate_fast;
		desired_tilt = 0;
	}


	//Flipping aux1 up makes it go to hover position fast no matter what
	if (Receiver->aux1 < 0.7){
		max_tilt_rate = max_tilt_rate_fast;
		desired_tilt = 0;
	}


	if (dt > 20000){
		dt = 10000;
	}


	if (master_tilt < desired_tilt){
		master_tilt = master_tilt + max_tilt_rate * dt / 1000000; //(dt from above)
	}
	else{
		master_tilt = master_tilt - max_tilt_rate * dt / 1000000;
	}

	//Limit backwards tilt to avoid servo crushing into itself
	if (master_tilt < -32){
		master_tilt = -32;
	}

/*
	Serial.println(" ");
	Serial.print("master_tilt: "); Serial.print(master_tilt);
	Serial.println(" ");
	Serial.print("des tilt: "); Serial.print(desired_tilt);
	Serial.println(" ");
	Serial.print("yaw: "); Serial.print(yaw_tilt_fr);
*/

	Commands.front_right_servo = master_tilt + constrain(yaw_tilt_fr, -10, 10) - constrain(roll_tilt_fr, -18, 18) + constrain(pitch_tilt_fr, -18, 18);
	Commands.front_left_servo = master_tilt - constrain(yaw_tilt_fl, -10, 10) + constrain(roll_tilt_fl, -18, 18) + constrain(pitch_tilt_fl, -18, 18);

	Commands.back_right_servo = master_tilt + constrain(yaw_tilt_br, -10, 10) - constrain(roll_tilt_br, -18, 18) - constrain(pitch_tilt_br, -18, 18);
	Commands.back_left_servo = master_tilt  - constrain(yaw_tilt_bl, -10, 10) + constrain(roll_tilt_bl, -18, 18) - constrain(pitch_tilt_br, -18, 18);




/*
	Serial.println(Commands.front_right_servo);
	Serial.println(Commands.front_left_servo);
	Serial.println(Commands.back_right_servo);
	Serial.println(Commands.back_far_right_servo);
	Serial.println(Commands.back_left_servo);
	Serial.println(Commands.back_far_left_servo);

	Serial.println(" ");
	Serial.print("Loop: "); Serial.print(dt); Serial.println(" ");
*/

}


void Control::Log()
	{

		//Command checks
		Serial.print("Cmds: ");
	
		Serial.print(Commands.front_right); Serial.print(" ");
		Serial.print(Commands.front_left); Serial.print(" ");
		Serial.print(Commands.back_right); Serial.print(" ");
		Serial.print(Commands.back_left); Serial.print(" ");
		Serial.print(Commands.back_mid_right); Serial.print(" ");
		Serial.print(Commands.back_mid_left); Serial.print(" ");
		Serial.print(Commands.back_far_right); Serial.print(" ");
		Serial.print(Commands.back_far_left); Serial.print(" ");

		Serial.print(Commands.front_right_servo); Serial.print(" ");
		Serial.print(Commands.front_left_servo); Serial.print(" ");
		Serial.print(Commands.back_right_servo); Serial.print(" ");
		Serial.print(Commands.back_left_servo); Serial.print(" ");

		//Serial.print(Pitch_Rate_P); Serial.print(" ");
		//Serial.print(Roll_Rate_P); Serial.print(" ");

		//Serial.println(micros());

		//Serial.print(dt); Serial.print(" "); //Put in commands instead so reports every time

	

	}


void Control::MotorTest2(Receiver::RECEIVER * Receiver)
	{

	//Individual motor testing.  Motor speed is throttle, set motor by combination of aux switches and pitch forward/backward for front/back motor

	if (Receiver->pitch < 0){

		if (Receiver->aux2 > 0.5){
			Commands.front_right = constrain(Receiver->thrust, 0, 1);
			Commands.front_left = constrain(0, 0, 1);

			Commands.back_right = constrain(0, 0, 1);
			Commands.back_left = constrain(0, 0, 1);
			Commands.back_far_right = constrain(0, 0, 1);
			Commands.back_far_left = constrain(0, 0, 1);
		}
		else{
			Commands.front_right = constrain(0, 0, 1);
			Commands.front_left = constrain(Receiver->thrust, 0, 1);

			Commands.back_right = constrain(0, 0, 1);
			Commands.back_left = constrain(0, 0, 1);
			Commands.back_far_right = constrain(0, 0, 1);
			Commands.back_far_left = constrain(0, 0, 1);
		}

	}

	else{

		if (Receiver->aux2 > 0.5){

			if (Receiver->aux1 > 0.5){

				Commands.front_right = constrain(0, 0, 1);
				Commands.front_left = constrain(0, 0, 1);

				Commands.back_right = constrain(Receiver->thrust, 0, 1);
				Commands.back_left = constrain(0, 0, 1);
				Commands.back_far_right = constrain(0, 0, 1);
				Commands.back_far_left = constrain(0, 0, 1);
			}
			else{

				Commands.front_right = constrain(0, 0, 1);
				Commands.front_left = constrain(0, 0, 1);

				Commands.back_right = constrain(0, 0, 1);
				Commands.back_left = constrain(0, 0, 1);
				Commands.back_far_right = constrain(Receiver->thrust, 0, 1);
				Commands.back_far_left = constrain(0, 0, 1);
			}

		}

		else{

			if (Receiver->aux1 > 0.5){

				Commands.front_right = constrain(0, 0, 1);
				Commands.front_left = constrain(0, 0, 1);

				Commands.back_right = constrain(0, 0, 1);
				Commands.back_left = constrain(Receiver->thrust, 0, 1);
				Commands.back_far_right = constrain(0, 0, 1);
				Commands.back_far_left = constrain(0, 0, 1);
			}
			else{
				Commands.front_right = constrain(0, 0, 1);
				Commands.front_left = constrain(0, 0, 1);

				Commands.back_right = constrain(0, 0, 1);
				Commands.back_left = constrain(0, 0, 1);
				Commands.back_far_right = constrain(0, 0, 1);
				Commands.back_far_left = constrain(Receiver->thrust, 0, 1);
			}

		}


	}


	Commands.front_right_servo = Receiver->dial1 * 90;
	Commands.front_left_servo = Receiver->dial1 * 90;
	Commands.back_right_servo = Receiver->dial1 * 90;
	Commands.back_left_servo = Receiver->dial1 * 90;


}
