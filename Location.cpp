#include "Location.h"
/*
#include <Wire.h>
#define SDAPIN 20
#define CLKPIN 21
*/
/*
Key thing about the BNO055 is fusion data is produced at 100 Hz - reading significantly faster than this (around 200 Hz) causes
it to return only zeroes.

//I2C Bus Recovery code/discussion: https://github.com/esp8266/Arduino/issues/1025


I2C on Due:
	-Make sure powered on 3.3V, not 5V
	-I2C tied to 3.3V
	-Confirmed on oscilloscope that on 3.3V

I2C on Udoo:

Need PC9306 to level shift.  People running into problems with others.
Same chip used for onboard level shifter on Udoo (to brick connector) - not clear if still there on Udoo X86 Advanced
https://www.udoo.org/forum/threads/how-can-i-use-the-braswell-i2c-interface.8937/

Udoo schematic:
https://udoo.org/download/files/UDOO_X86/schematics/UDOOX86_revH_schematics.pdf

Braswell chip has ~1,100 pins.  7 I2C channels (14 pins).
	-1 not connected at all one Udoo
	-2 are dual mode (I2C and another) and both used for other mode
	-1 used to reset Curie chip
	-1 connected to PCA9306 level shifter and out to CN27 (doesn't seem to exist on X86 Advanced)

	-I2C0_SCL and I2C0_SDA go directly from chip to pins 10 and 12 on CN14
	-I2C5_SCL and I2C0_SDA go directly from chip to pins 2 and 4 on CN14
	-400 pF max bus capacitance
	-1.8V output from Braswell pin
		-1.26V is min. logic high, 0.54V is max logic low input
		-0.36V is max logic low sent as output
		-2-5 pF pin capacitance
	-Never provide more than 1.8V on pin



*/

XPlane XPlane_for_Location;
// array of 12 pulses (8 motors + 4 servos) to send to Arduino
char send_msg[] = "<50,100,200,300,400,500,600,700,800,900,950,999>";

//Adafruit_BNO055 bno = Adafruit_BNO055(55);

void Location::init()
	{


		std::cout << "Starting IMU" << std::endl;

		// buffer array that holds the PWMs to send to the Arduino over Serial
		setup_port();
		std::cout << "Finished setting up serial port...\n";
		setGPIO(0);
		std::cout << "GPIO pin set to logic LOW...\n";
		turnon_GPIO();
		std::cout << "GPIO pin is enabled...\n";

/*
	  //BNO055 reset - (Due pin 39 connected to BNO055 RST pin).  Need to do this after restarting after a bus loss or else have to try a couple times.
	  
	  pinMode(39, OUTPUT);
	  digitalWrite(39, LOW);
	  delay(1);
	  digitalWrite(39, HIGH);
	  delay(650);
	   

	   while(!bno.begin())
	  {

	      Serial.println("Starting I2C bus recovery - init");

		  //try i2c bus recovery at 100kHz = 5uS high, 5uS low
		  pinMode(SDAPIN, OUTPUT);//keeping SDA high during recovery
		  digitalWrite(SDAPIN, HIGH);
		  pinMode(CLKPIN, OUTPUT);
		  for (int i = 0; i < 10; i++) { //9nth cycle acts as NACK
		    digitalWrite(CLKPIN, HIGH);
		    delayMicroseconds(5);
		    digitalWrite(CLKPIN, LOW);
		    delayMicroseconds(5);
		  }

		  //a STOP signal (SDA from low to high while CLK is high)
		  digitalWrite(SDAPIN, LOW);
		  delayMicroseconds(5);
		  digitalWrite(CLKPIN, HIGH);
		  delayMicroseconds(2);
		  digitalWrite(SDAPIN, HIGH);
		  delayMicroseconds(2);
		  //bus status is now : FREE

		  Serial.println("Bus recovery done - init");
		  //return to power up mode
		  pinMode(SDAPIN, INPUT);
		  pinMode(CLKPIN, INPUT);

	  }

	  Serial.println("I2C bus up, BNO055 detected");


		bno.setExtCrystalUse(true);

*/

	}


void Location::turnon_GPIO() {
	gpio.open("/gpio/pin43/direction", std::ios::out | std::ios::trunc);
	gpio << "out";
	gpio.close();
}

void Location::setGPIO(int state) {
	if(!gpio.is_open()) {
		//cout << "Opening gpio pin...\n";
		gpio.open("/gpio/pin43/value", std::ios::out | std::ios::trunc);
	}	

	gpio << state;
	//std::cout << (state == 1 ? "GPIO pin set to logic HIGH...\n" :
	//		"GPIO pin set to logic LOW...\n");
	gpio.close();
}

void Location::readData() {
	setGPIO(1); // set GPIO pin to logic high, telling Arduino that we want data
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
					//estimate();				
					receivedMsg[0] = '\0'; // clear the message buffer
	   			 	recvInProgress = false;
					indx = 0;
					dataAvailable = false;
					setGPIO(0); // finished reading data, set GPIO pin to logic LOW
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
void Location::convertMessage() {
	chars_array = strtok(receivedMsg, ",");
	while(chars_array) {
        convertedMsg.push_back(atof(chars_array));
        chars_array = strtok(NULL, ",");
    }
}

void Location::setup_port() {
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

void Location::estimate()
	{

	//int start = micros();

///*


	/*XPlane_for_Location.get_data_from_XPlane();	

	Current_Location.pitch = XPlane_for_Location.Current_XPlane_data.pitch_rad;
	Current_Location.roll = XPlane_for_Location.Current_XPlane_data.roll_rad;
	Current_Location.heading = XPlane_for_Location.Current_XPlane_data.heading_rad;
	Current_Location.pitch_rate = XPlane_for_Location.Current_XPlane_data.pitch_rot_vel_calcd;
	Current_Location.roll_rate = XPlane_for_Location.Current_XPlane_data.roll_rot_vel_calcd;
	Current_Location.heading_rate = XPlane_for_Location.Current_XPlane_data.heading_rot_vel_calcd;
	Current_Location.time = XPlane_for_Location.Current_XPlane_data.flight_time;
	*/



	//For testing without IMU connected, reading dummy data from Arduino
	readData();


	Current_Location.pitch = convertedMsg[0];
	Current_Location.roll = convertedMsg[1];
	Current_Location.heading = convertedMsg[2];
	Current_Location.pitch_rate = convertedMsg[3];
	Current_Location.roll_rate = convertedMsg[4];
	Current_Location.heading_rate = convertedMsg[5];

	convertedMsg.clear(); // clear vector containing the data from Arduino

	// Print for debugging
	std::cout << "PRY: ";
	std::cout << Current_Location.pitch << " ";
	std::cout << Current_Location.roll << " ";
	std::cout << Current_Location.heading << " ";
	std::cout << Current_Location.pitch_rate << " ";
	std::cout << Current_Location.roll_rate << " ";
	std::cout << Current_Location.heading_rate << " " << std::endl;

	


	return;
//*/


	//int starting = micros();
	//timeSinceReset = millis() - lastReset;

	/*

	//Do I2C bus recovery every time 0 only takes a few microseconds and prevents bus from dropping
	//try i2c bus recovery at 100kHz = 5uS high, 5uS low
	pinMode(SDAPIN, OUTPUT);//keeping SDA high during recovery
	digitalWrite(SDAPIN, HIGH);
	pinMode(CLKPIN, OUTPUT);
	for (int i = 0; i < 2; i++) { //9nth cycle acts as NACK
		digitalWrite(CLKPIN, HIGH);
		delayMicroseconds(5);
		digitalWrite(CLKPIN, LOW);
		delayMicroseconds(5);
	  }

	//a STOP signal (SDA from low to high while CLK is high)
	digitalWrite(SDAPIN, LOW);
	delayMicroseconds(5);
	digitalWrite(CLKPIN, HIGH);
	delayMicroseconds(2);
	digitalWrite(SDAPIN, HIGH);
	delayMicroseconds(2);
	//bus status is now : FREE

  
	//return to power up mode
	pinMode(SDAPIN, INPUT);
	pinMode(CLKPIN, INPUT);

	Wire.begin();
	Wire.setClock(100000); //Seems to have no effect on Location.estimate() speed from start to finish
	

  	//Haven't figured out why, but getEvent works (doesn't return zeros after a while) and getGv
    bno.getEvent(&orientationEvent, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&ratesEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);
*/


/*
Biggest problem here is BNO055 returning zeros.  Sensor itself needs to be reset to work again (not an I2C problem) either with the reset pin
connected to GPIO or through an I2C command (bno.begin() will do this).
Problem is, if reset in air, new zero will be in whatever position it's reset in.  So if call bno.begin(), need to adjust values by whatever they
were beforehand

*/
	
	/*
	if (orientationEvent.orientation.z == 0 && orientationEvent.orientation.y == 0 && orientationEvent.orientation.x == 0  && \
		ratesEvent.gyro.z == 0 && ratesEvent.gyro.y == 0 && ratesEvent.gyro.x == 0 && timeSinceReset > 2000){
		
			digitalWrite(39, LOW);
			delayMicroseconds(1);
			digitalWrite(39, HIGH);
			delay(20);
	
			//Put in NDOF mode (sensor fusion) - this is what bno.begin() does by default
			Wire.beginTransmission(0x28);         // Begin transmission to the slave address
			Wire.write(0x3D);        //Put register address in transmit buffer
			Wire.write(0x0C);
			Wire.endTransmission();      // Send and keep connection alive.




			bno.begin();
			lastReset = millis();
			Serial.print("RESETTING");
			reset_count = reset_count + 1;

			yaw_correction = last_yaw2;
	}




  	//In radians.
  	Location.pitch = orientationEvent.orientation.z * M_PI / 180 - 0.045; //Pitch - wide side placed left-right, 4 pins forward.  Pitch up is +.
  	Location.roll = orientationEvent.orientation.y * M_PI / 180 + 0.035; //Roll - wide side placed left-right, 4 pins forward.  Right roll is +.
  	
	//Set yaw location from 0 to 2*PI to -PI to PI
	//Yaw - wide side of BNO055 placed from left to right.  Going CW from top is +
	if (orientationEvent.orientation.x * M_PI/180 > M_PI){
		Location.yaw = orientationEvent.orientation.x * M_PI/180 - 2*M_PI;
	}
	else{
		Location.yaw = orientationEvent.orientation.x * M_PI/180;
	}
*/

/*
  	if (abs(last_pitch-Location.pitch) > 5){ //If sensor messed up, use last measurement instead

		Location.pitch = last_pitch;
		Location.roll = last_roll;
		Location.yaw = last_yaw;
		Location.pitch_rate = last_pitch_rate;
		Location.roll_rate = last_roll_rate;
		Location.yaw_rate = last_yaw_rate;

	}

	if (abs(last_roll-Location.roll) > 5){ //If sensor messed up, use last measurement instead
		Location.pitch = last_pitch;
		Location.roll = last_roll;
		Location.yaw = last_yaw;
		Location.pitch_rate = last_pitch_rate;
		Location.roll_rate = last_roll_rate;
		Location.yaw_rate = last_yaw_rate;

	}

	if (abs(last_pitch_rate-Location.pitch_rate) > 10){ //If sensor messed up, use last measurement instead
		Location.pitch = last_pitch;
		Location.roll = last_roll;
		Location.yaw = last_yaw;
		Location.pitch_rate = last_pitch_rate;
		Location.roll_rate = last_roll_rate;
		Location.yaw_rate = last_yaw_rate;

	}

	if (abs(last_roll_rate-Location.roll_rate) > 10){ //If sensor messed up, use last measurement instead
		Location.pitch = last_pitch;
		Location.roll = last_roll;
		Location.yaw = last_yaw;
		Location.pitch_rate = last_pitch_rate;
		Location.roll_rate = last_roll_rate;
		Location.yaw_rate = last_yaw_rate;

	}

	*/

  	//Different signs from euler orientations - values in degrees per second (even though Adafruit library documentation says rad/sec)
 
 /* 	Location.pitch_rate = -ratesEvent.gyro.x * M_PI / 180;
  	Location.roll_rate = -ratesEvent.gyro.y * M_PI / 180;
  	Location.yaw_rate = -ratesEvent.gyro.z * M_PI / 180;
  	*/

/*

  	  	//Location checks
	Serial.println("RPY: ");

  	Serial.print(Location.roll,3); Serial.print(" "); //Roll
  	Serial.print(Location.pitch,3);  Serial.print(" "); //Pitch
  	Serial.print(Location.yaw,3); Serial.println(" "); //Yaw

  	Serial.print(Location.roll_rate,3); Serial.print(" "); //Roll
  	Serial.print(Location.pitch_rate,3);  Serial.print(" "); //Pitch
  	Serial.print(Location.yaw_rate,3); Serial.println(" "); //Yaw





  	last_roll2 = last_roll;
	last_pitch2 = last_pitch;
	last_yaw2 = last_yaw;
	last_roll_rate2 = last_roll_rate;
	last_pitch_rate2 = last_pitch_rate;
	last_yaw_rate2 = last_yaw_rate;
	//last_time2 = last_time;

	last_roll = Location.roll;
	last_pitch = Location.pitch;
	last_yaw = Location.yaw;
	last_roll_rate = Location.roll_rate;
	last_pitch_rate = Location.pitch_rate;
	last_yaw_rate = Location.yaw_rate;
	//last_time = meas_time;

	*/
 /* 	
  	imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  	imu::Vector<3> euler_rates = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  	Serial.print(euler.y()); Serial.print("  ");
  	Serial.print(euler_rates.y()); Serial.print("  ");

	Serial.println(micros());



  	Serial.print(Location.pitch); Serial.print("  ");
  	Serial.print(Location.pitch_rate); Serial.print("  ");

	Serial.println(micros());


  	Serial.print(Location.yaw); Serial.print("  ");
  	Serial.print(Location.yaw_rate); Serial.print("  ");

  	Serial.print(Location.pitch); Serial.print("  ");
  	Serial.print(Location.pitch_rate); Serial.print("  ");

  	Serial.print(Location.roll); Serial.print("  ");
  	Serial.print(Location.roll_rate); Serial.print("  ");

  	Serial.print(reset_count); Serial.print("  ");
  	Serial.print(last_roll2); Serial.print("  ");
	Serial.print(last_pitch2); Serial.print("  ");
	Serial.print(last_yaw2); Serial.print("  ");
  
  	Serial.println(micros());

*/
  	/*
  	Serial.println(" ");
  	Serial.print("Loc dt: "); Serial.println(ending - starting);
  	*/

	}



void Location::Log()
	{


  	//Location checks
	std::cout << "IMU Log" << std::endl;

/*

	Serial.println("RPY: ");

  	Serial.print(Location.roll,3); Serial.print(" "); //Roll
  	Serial.print(Location.pitch,3);  Serial.print(" "); //Pitch
  	Serial.print(Location.yaw,3); Serial.println(" "); //Yaw

  	Serial.print(Location.roll_rate,3); Serial.print(" "); //Roll
  	Serial.print(Location.pitch_rate,3);  Serial.print(" "); //Pitch
  	Serial.print(Location.yaw_rate,3); Serial.println(" "); //Yaw

  	//Serial.println(micros());
  	*/

	}


