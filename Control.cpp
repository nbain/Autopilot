#include "Control.h"
#include <math.h> //For sqrt() control

cout << "TESTING" << endl;

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


void Control::run(Location::LOCATION * Location, Receiver::RECEIVER * Receiver)
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
	if (abs(pitch_target - Location->pitch) > 2){
		//copysignf (f for float) makes value of first argument the sign (+ or -) of second argument
		pitch_rate_target = 6 * copysignf(sqrt(abs(pitch_target - Location->pitch)-1), (pitch_target - Location->pitch));//5 deg->12dps, 10dps>18deg/s
	}
	else{
		pitch_rate_target = (pitch_target - Location->pitch) * 3;
	}

	float roll_rate_target;
	if (abs(roll_target - Location->roll) > 2){
		//copysignf (f for float) makes value of first argument the sign (+ or -) of second argument
		roll_rate_target = 6 * copysignf(sqrt(abs(roll_target - Location->roll)), (roll_target - Location->roll)); //If rolled right, should be neg
	}
	else{
		roll_rate_target = (roll_target - Location->roll) * 3;
	}
	

	//roll_rate_target = roll_rate_target * 3 * Receiver->dial1;


	float yaw_rate_target;
	if (abs(yaw_error) > 2){
		//copysignf (f for float) makes value of first argument the sign (+ or -) of second argument
		yaw_rate_target = 6 * copysignf(sqrt(abs(yaw_error)), (-yaw_error)); //Negative if yawed right
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


void Control::MotorTest(Receiver::RECEIVER * Receiver)
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
