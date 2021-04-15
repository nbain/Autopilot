#ifndef AUTOPILOT2_RECEIVER_H
#define AUTOPILOT2_RECEIVER_H

#include <iostream>
#include <fstream>
#include <cstdio>
#include <string.h>
#include <vector>
#include <chrono>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>


#include "Location.h"



//#include "main.h"

//#include <Arduino.h>

#define RECV_CHAN0PIN 	53 //Use this only for thrust (to avoid complexity)
#define RECV_CHAN1PIN 	51 //Only for pitch
#define RECV_CHAN2PIN 	49 //Only for roll
#define RECV_CHAN3PIN 	47 //Only for yaw
#define RECV_CHAN4PIN 	45 //Only for AUX1 - switch on the top far left, zero when flipped up
#define RECV_CHAN5PIN 	43 //Only for AUX2 - switch on the top far right, zero when flipped up
#define RECV_CHAN6PIN 	41 //Only for Dial1 (I made up Dial1 - called S1 on QX7, the dial on the top left).  Goes from -1 to 1
//Note: 41 used for MPU9250 interrupt, so should prob change

#define MIN_PWM_THRUST 988
#define MAX_PWM_THRUST 2008

#define MIN_PWM_PITCH 1000
#define MAX_PWM_PITCH 2012
#define PITCH_PWM_ZERO 1502

#define MIN_PWM_ROLL 988
#define MAX_PWM_ROLL 2008
#define ROLL_PWM_ZERO 1497

#define MIN_PWM_YAW 1006 //Reading 96.5 (not sure why, not super easy to see how to change to -100.0)
#define MAX_PWM_YAW 2012 //Reading 100.0
#define YAW_PWM_ZERO 1500 //Reading 0.0 on QX7 display

#define MIN_PWM_AUX1 988
#define MAX_PWM_AUX1 2012

#define MIN_PWM_AUX2 988
#define MAX_PWM_AUX2 2012

#define MIN_PWM_DIAL1 988
#define MAX_PWM_DIAL1 2012

/*

For input voltage = 6.2V, still 3.30V PWM output

Pulses every 18ms -> 55.55 Hz (still 1,000 us - 2,000 us)


*/


//Pitch, roll, and yaw should be -1 to 1, others should be 0-1

class Receiver //used to be receiverModule
{
	
	public:

		void setGPIO(int state);
		void turnon_GPIO();
		void readData();
		void setup_port();
		void convertMessage();
		void requestData();

		std::string path = "/dev/ttyACM1";
		int serial_port;
		struct termios tty;
		std::ofstream gpio;

		static const int numBytes = 60;
		char startMarker = '<';
		char endMarker = '>';
		char writeMarker[2] = "&";
		char incomingByte[1];
		char receivedMsg[numBytes];
		char* chars_array;
		std::vector<float> convertedMsg;
		bool dataAvailable;

		struct RECEIVER
		{
			float thrust = 0;
			float pitch;
			float roll;
			float yaw;
			float aux1;
			float aux2;
			float dial1;

		};

		RECEIVER Current_Receiver_Values;


		void init();
		void read_intent();
		void Log();


};


#endif
