#include "Receiver.h"

/*
You should just be able to get 0 to 1 or -1 to 1 values that tell the current intent of the receiver
for thrust, pitch, etc.

There's so much repetition here because the Interrupt Service Routine (ISR) is needed to detect a change
in a pin value corresponding to the start or end of a pwm pulse - an interrupt is needed so it only takes
this action when a pin actually changes from high-low/low-high

Reference: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/

ISRs cannot have any parameters passed into them and return nothing (though obv can save value to variable)
So, a separate ISR must be used for each receiver channel

When a pin goes low, the function subtracts the most recent start time for that pin
from the time now, when it just went low.

So, separate but identical functions are used for each channel.  The alternative is to have separate ISRs that
each call a single function (this is done in the other quadcopter library), but the conceptual complexity of
ensuring that start/end times for different channels aren't confused with eachother makes separate ones
probably worth it (only 6 channels anyway)

**WARNING: If PWM turns off, there will be no more "CHANGE"'s, so it will just stay at the most recent value
**Should detect if no update after a bit (obv must be outside ISR), or may need to output time of most recent change
**from ISR, and if there are no more pulses, then do something like zero controls and turn throttle off

Binding X8R to Taranis QX7:
https://www.openrcforums.com/forum/viewtopic.php?f=96&t=5345

*/


/*
int ch0_start;//micros at which a certain pin went to high
int ch0_end; //micros at which a certain pin went to low
int ch0_us; //delta between start and finish micros when pin goes low
//ch0_us will always, automatically (because of the ISR), be the most recent pulse from the receiver

int ch1_start;
int ch1_end;
int ch1_us;

int ch2_start;
int ch2_end;
int ch2_us;

int ch3_start;
int ch3_end;
int ch3_us;

int ch4_start;
int ch4_end;
int ch4_us;

int ch5_start;
int ch5_end;
int ch5_us;

int ch6_start;
int ch6_end;
int ch6_us;


static void ch0_PWM();
static void ch1_PWM();
static void ch2_PWM();
static void ch3_PWM();
static void ch4_PWM();
static void ch5_PWM();
static void ch6_PWM();
*/


void Receiver::init()
	{

		std::cout << "Initializing receiver" << std::endl;
		setup_port();
		std::cout << "Finished setting up serial port...\n";
		//setGPIO(0);
		//std::cout << "GPIO pin set to logic LOW...\n";
		//turnon_GPIO();
		//std::cout << "GPIO pin is enabled to OUT...\n";
		/*
		pinMode(RECV_CHAN0PIN, INPUT);
  		pinMode(RECV_CHAN1PIN, INPUT);
  		pinMode(RECV_CHAN2PIN, INPUT);
  		pinMode(RECV_CHAN3PIN, INPUT);
  		pinMode(RECV_CHAN4PIN, INPUT);
  		pinMode(RECV_CHAN5PIN, INPUT);
  		pinMode(RECV_CHAN6PIN, INPUT);

  		attachInterrupt(RECV_CHAN0PIN, &ch0_PWM, CHANGE);
  		attachInterrupt(RECV_CHAN1PIN, &ch1_PWM, CHANGE);
  		attachInterrupt(RECV_CHAN2PIN, &ch2_PWM, CHANGE);
  		attachInterrupt(RECV_CHAN3PIN, &ch3_PWM, CHANGE);
  		attachInterrupt(RECV_CHAN4PIN, &ch4_PWM, CHANGE);
  		attachInterrupt(RECV_CHAN5PIN, &ch5_PWM, CHANGE);
  		attachInterrupt(RECV_CHAN6PIN, &ch6_PWM, CHANGE);

  		Serial.println("CHANGE interrupts on receiver pins are set");
  		*/

	}

void Receiver::turnon_GPIO() {
	gpio.open("/gpio/pin43/direction", std::ios::out | std::ios::trunc);
	gpio << "out";
	gpio.close();
}

void Receiver::setGPIO(int state) {
	if(!gpio.is_open()) {
		//cout << "Opening gpio pin...\n";
		gpio.open("/gpio/pin43/value", std::ios::out | std::ios::trunc);
	}	

	gpio << state;
	//std::cout << (state == 1 ? "GPIO pin set to logic HIGH...\n" :
	//		"GPIO pin set to logic LOW...\n");
	gpio.close();
}

void Receiver::requestData() {
	int n = write(serial_port, &writeMarker, sizeof(writeMarker));
}

void Receiver::readData() {
	//setGPIO(1); // set GPIO pin to logic high, telling Arduino that we want data
	requestData();

	dataAvailable = true;
	static bool recvInProgress = false;
	static int indx = 0;
	
	// keep reading data until we read an end marker (indicates end of message)
	while(dataAvailable) {
		memset(&incomingByte, '\0', sizeof(incomingByte)); // clear the buffer holding the incoming byte
		//std::cout << "Reading serial port...\n";
		int n = read(serial_port, &incomingByte, sizeof(incomingByte)); // read a single byte
		
		//std::cout << "Read byte: " << incomingByte[0] << "\n";
		
		// error checks
		if(n < 0) {
			printf("Error reading from serial port: %s\n", strerror(errno));
		}
		else if(n == 0) {
			std::cout << "Nothing received on serial port." << std::endl;
		}
		else {
			//cout << "Reading data..." << incomingByte[0] << endl;
			if (recvInProgress) {
				// keep filling the messagebuffer until we reach the end marker
	  			if (incomingByte[0] != endMarker) {
					receivedMsg[indx] = incomingByte[0];
					indx++;

					if (indx >= numBytes)
		  				indx = numBytes - 1;
	  			}
	  			else {
					//cout << "Received message: " << receivedMsg << endl;
					convertMessage();			
					receivedMsg[0] = '\0'; // clear the message buffer
	   			 	recvInProgress = false;
					indx = 0;
					dataAvailable = false;
					//setGPIO(0); // finished reading data, set GPIO pin to logic LOW
	  			}
			}
			else if (incomingByte[0] == startMarker) {
				//cout << "Received start marker...\n";
	  			recvInProgress = true; // start to fill the message buffer if we received the start marker
			}
		}
	}
}


// Converts char array received message to a vector of floats
void Receiver::convertMessage() {
	chars_array = strtok(receivedMsg, ",");
	while(chars_array) {
        convertedMsg.push_back(atof(chars_array));
        chars_array = strtok(NULL, ",");
    }
}

void Receiver::setup_port() {
	serial_port = open(path.c_str(), O_RDWR | O_NOCTTY);

	if(serial_port < 0) {
		printf("Error %i from open: %s\n", errno, strerror(errno));			
	}

	// Read in existing settings, and handle any error
	if(tcgetattr(serial_port, &tty) != 0) {
		printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
	}

	tty.c_lflag |= ICANON; // run in canonical mode (receive data line by line)
	tty.c_cflag &= ~CRTSCTS; // disable hardware flow control	

	//tty.c_cc[VTIME] = 0; // no timeout
	//tty.c_cc[VMIN] = 1; // always wait for 1 byte 

	//tty.c_cflag |= CS8; // 8 bits per byte
	cfsetispeed(&tty, B460800);
	cfsetospeed(&tty, B460800);

	// Save tty settings, also checking for error
  	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
  		printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  	}
}


void Receiver::read_intent()
	{
		
		//std::cout << "Reading receiver" << std::endl;

		readData();

		///*
		//For testing without receiver connected
	  	Current_Receiver_Values.thrust = convertedMsg[0];
	  	Current_Receiver_Values.pitch = convertedMsg[1];
	  	Current_Receiver_Values.roll = convertedMsg[2];
	  	Current_Receiver_Values.yaw = convertedMsg[3];
	  	Current_Receiver_Values.aux1 = convertedMsg[4];
	  	Current_Receiver_Values.aux2 = convertedMsg[5];
	  	Current_Receiver_Values.dial1 = convertedMsg[6];

	  	convertedMsg.clear(); // clear vector containing the data from Arduino

	  	// Print for debugging
		std::cout << "TPRYAAD: ";
		std::cout << Current_Receiver_Values.thrust << " ";
		std::cout << Current_Receiver_Values.pitch << " ";
		std::cout << Current_Receiver_Values.roll << " ";
		std::cout << Current_Receiver_Values.yaw << " ";
		std::cout << Current_Receiver_Values.aux1 << " ";
		std::cout << Current_Receiver_Values.aux2 << " ";
		std::cout << Current_Receiver_Values.dial1 << " " << std::endl;


	  	return;
	  	//*/

/*

		Receiver.thrust = ((float)ch0_us - MIN_PWM_THRUST) / (float)(MAX_PWM_THRUST - MIN_PWM_THRUST);
		if (Receiver.thrust > 1.1){
			Receiver.thrust = 0; //Problem where erroneous startup throttle caused flip
		}
		if (millis() < 4000){
			Receiver.thrust = 0; //Never start up with motors on no matter what
		}
		

		//Pitch, roll, and yaw have a little more math to make -1 to 1.  Linear between zero point (resting) and edges.
		//If don't do between resting point, zero point will always be off by about 0.03 or so (since doesn't rest
		//exactly in the middle)
		if (ch1_us < PITCH_PWM_ZERO){ //For when below the zero point, (Delta from zero)/(Range from that edge to mid)
			Receiver.pitch = ((float)(ch1_us - PITCH_PWM_ZERO) / (float)(MIN_PWM_PITCH - PITCH_PWM_ZERO));}
		else{ //For when above the zero point
			Receiver.pitch = ((float)(ch1_us - PITCH_PWM_ZERO) / (float)(PITCH_PWM_ZERO - MAX_PWM_PITCH));}

		if (ch2_us < ROLL_PWM_ZERO){
			Receiver.roll = ((float)(ch2_us - ROLL_PWM_ZERO) / (float)(ROLL_PWM_ZERO - MIN_PWM_ROLL));}
		else{
			Receiver.roll = ((float)(ch2_us - ROLL_PWM_ZERO) / (float)(MAX_PWM_ROLL - ROLL_PWM_ZERO));}

		if (ch3_us < YAW_PWM_ZERO){
			Receiver.yaw = ((float)(ch3_us - YAW_PWM_ZERO) / (float)(YAW_PWM_ZERO - MIN_PWM_YAW));}
		else{
			Receiver.yaw = ((float)(ch3_us - YAW_PWM_ZERO) / (float)(MAX_PWM_YAW - YAW_PWM_ZERO));}
		if (abs(Receiver.yaw) < 0.03){ //If yaw close to zero, just make zero to avoid unintentional slew rate
			Receiver.yaw = 0;}


		Receiver.aux1 = ((float)ch4_us - MIN_PWM_AUX1) / (float)(MAX_PWM_AUX1- MIN_PWM_AUX1);
		Receiver.aux2 = ((float)ch5_us - MIN_PWM_AUX2) / (float)(MAX_PWM_AUX2 - MIN_PWM_AUX2);

		Receiver.dial1 = ((float)ch6_us - (MIN_PWM_DIAL1 + MAX_PWM_DIAL1)/2) / (float)(MAX_PWM_DIAL1 - (MIN_PWM_DIAL1 + MAX_PWM_DIAL1)/2);
*/


	}



/*
float Receiver::get_receiver_values()
	{

		printf("Recv Thrust: %f\n", Receiver.thrust);

		return Receiver.thrust;


	}
	*/



void Receiver::Log()
	{
		

		//Receiver checks
		std::cout << "Logging receiver" << std::endl;

		/*
		Serial.print("  Rx: ");

	  	Serial.print(Receiver.thrust); Serial.print(" ");
	  	Serial.print(Receiver.pitch); Serial.print(" ");
	  	Serial.print(Receiver.roll);  Serial.print(" ");
	  	Serial.print(Receiver.yaw);  Serial.print(" ");
	  	Serial.print(Receiver.aux1);  Serial.print(" ");
	  	Serial.print(Receiver.aux2);  Serial.print(" ");
	  	Serial.print(Receiver.dial1);  Serial.print(" ");

	  	//Serial.println(micros());
	  	*/


	}


/*
void ch0_PWM()
	{ 
	  int ch0_pin = digitalRead(RECV_CHAN0PIN);

	  if (ch0_pin == 1){ //Pin just turned on
	    ch0_start = micros();
	  }
	  if (ch0_pin == 0){ //Pin just turned off
	    ch0_end = micros();
	    ch0_us = ch0_end - ch0_start; //Only calculate 'us' when it turns off.
	    //Otherwise might get an end time from the last pulse if this one's not done, so would get a 
	    //value near 18ms, or 18,000, and negative.
	  }

	}


void ch1_PWM()
	{ 
	  int ch1_pin = digitalRead(RECV_CHAN1PIN);

	  if (ch1_pin == 1){ //Pin just turned on
	    ch1_start = micros();
	  }
	  if (ch1_pin == 0){ //Pin just turned off
	    ch1_end = micros();
	    ch1_us = ch1_end - ch1_start; //Only calculate us when it turns off.
	  }
	}

void ch2_PWM()
	{ 
	  int ch2_pin = digitalRead(RECV_CHAN2PIN);

	  if (ch2_pin == 1){ //Pin just turned on
	    ch2_start = micros();
	  }
	  if (ch2_pin == 0){ //Pin just turned off
	    ch2_end = micros();
	    ch2_us = ch2_end - ch2_start; //Only calculate us when it turns off.
	  }
	}

void ch3_PWM()
	{ 
	  int ch3_pin = digitalRead(RECV_CHAN3PIN);

	  if (ch3_pin == 1){ //Pin just turned on
	    ch3_start = micros();
	  }
	  if (ch3_pin == 0){ //Pin just turned off
	    ch3_end = micros();
	    ch3_us = ch3_end - ch3_start; //Only calculate us when it turns off.
	  }
	}

void ch4_PWM()
	{ 
	  int ch4_pin = digitalRead(RECV_CHAN4PIN);

	  if (ch4_pin == 1){ //Pin just turned on
	    ch4_start = micros();
	  }
	  if (ch4_pin == 0){ //Pin just turned off
	    ch4_end = micros();
	    ch4_us = ch4_end - ch4_start; //Only calculate us when it turns off.
	  }
	}

void ch5_PWM()
	{ 
	  int ch5_pin = digitalRead(RECV_CHAN5PIN);

	  if (ch5_pin == 1){ //Pin just turned on
	    ch5_start = micros();
	  }
	  if (ch5_pin == 0){ //Pin just turned off
	    ch5_end = micros();
	    ch5_us = ch5_end - ch5_start; //Only calculate us when it turns off.
	  }
	}

void ch6_PWM()
	{ 
	  int ch6_pin = digitalRead(RECV_CHAN6PIN);

	  if (ch6_pin == 1){ //Pin just turned on
	    ch6_start = micros();
	  }
	  if (ch6_pin == 0){ //Pin just turned off
	    ch6_end = micros();
	    ch6_us = ch6_end - ch6_start; //Only calculate us when it turns off.
	  }
	}

	*/