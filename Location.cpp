#include "Location.h"

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


void Location::init()
	{


		std::cout << "Starting IMU" << std::endl;
		setup_port();

		//XPlane_for_Location.UDP_Setup_Recv();


	}

// NOTE: can speed up Serial read by reading twice the message length and parsing for the start marker
// rather than reading for the start marker one byte at a time
void Location::readData() {
	// using a boolean 
	dataAvailable = true;
	int count=0;

	while(dataAvailable) {
		memset(&incomingByte, '\0', sizeof(incomingByte)); // clear the buffer holding the incoming byte
		int n = read(serial_port_read, &incomingByte, sizeof(incomingByte)); // read a single byte
		count=count+1;
		
		// error checks, we should log these to the SD card eventually
		if(n < 0) {
			printf("Error reading from serial port: %s\n", strerror(errno));
		}
		else if(n == 0) {
			std::cout << "Nothing received on serial port." << std::endl;
		}
		else {			
			if (incomingByte[0] == loc_startMarker) {
				int n2 = read(serial_port_read, &location_msg, sizeof(location_msg));
				//std::cout << count << " calls to read()\t";
				
				// Convert the character array to a float array
				convertMessage();

				location_msg[0] = '\0'; // clear the buffer for the next read() call
				dataAvailable = false; // stop reading from the serial port

				// Need to flush the buffer to prevent build up of delayed data (by 1-2 sec)
	  			//tcflush(serial_port_read,TCIOFLUSH);
			}
		}
	}
}

// Converts char array location message to a float array
void Location::convertMessage() {
	location_msg_ptr = strtok(location_msg, ",");

	int count = 0;
	while(location_msg_ptr) {
		std::string tmp1 = location_msg_ptr;

		location_vals[count] = (float)atof(tmp1.c_str());
		count = count + 1;

		location_msg_ptr = strtok(NULL, ",");
	}
}

void Location::setup_port() {
	serial_port_read = open(readPath.c_str(), O_RDWR | O_NOCTTY);

	if(serial_port_read < 0) {
		printf("Error %i from open: %s\n", errno, strerror(errno));			
	}

	// Read in existing settings, and handle any error
	if(tcgetattr(serial_port_read, &options) != 0) {
		printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
	}

	/*options.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
	options.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
	options.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
	options.c_cflag |= CS8; // 8 bits per byte (most common)
	options.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
	options.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

	options.c_lflag &= ~ICANON; // Disable canonical mode
	options.c_lflag &= ~IEXTEN; // Disable extended functions
	options.c_lflag &= ~ECHO; // Disable echo
	options.c_lflag &= ~ECHOE; // Disable erasure
	options.c_lflag &= ~ECHONL; // Disable new-line echo
	options.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
	options.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	options.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|INPCK); // Disable any special handling of received bytes

	options.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	options.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

	//.c_lflag |= ICANON; // run in canonical mode (receive data line by line)
	//options.c_cflag &= ~CRTSCTS; // disable hardware flow control

	struct serial_struct serial;
	ioctl(serial_port_read, TIOCGSERIAL, &serial);
	serial.flags |= ASYNC_LOW_LATENCY;
	ioctl(serial_port_read, TIOCGSERIAL, &serial);
	
	//options.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG | IEXTEN);

	options.c_cc[VTIME] = 0; // no timeout
	options.c_cc[VMIN] = numBytes; // always wait for expected number of bytes*/

	options.c_lflag |= ICANON;
	options.c_cflag &= ~CRTSCTS;

	/*options.c_cflag |= (CS8 | CREAD | CLOCAL); // 8 bits per byte
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;

	options.c_oflag &= ~(OPOST | ONLCR);
	options.c_iflag &= ~(IXON | IXOFF | IXANY);
	options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);*/
	cfsetispeed(&options, B460800);
	cfsetospeed(&options, B460800);

	// Save options settings, also checking for error
  	if (tcsetattr(serial_port_read, TCSANOW, &options) != 0) {
  		printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  	}

	//tcflush(serial_port_read, TCIOFLUSH);
}



void Location::estimate()
	{
	readData(); // Get location data over Serial from Arduino
	Current_Location.roll = location_vals[0];
	Current_Location.pitch = location_vals[1];
	Current_Location.heading = location_vals[2];
	Current_Location.roll_rate = location_vals[3];
	Current_Location.pitch_rate = location_vals[4];
	Current_Location.heading_rate = location_vals[5];

	std::ofstream LOG;
	LOG.open("testlog.txt", std::ios::out | std::ios::app);	
	LOG << "Location RPY: ";
	LOG << Current_Location.roll << ", ";
	LOG << Current_Location.pitch << ", ";
	LOG << Current_Location.heading << ", ";
	LOG << Current_Location.roll_rate << ", ";
	LOG << Current_Location.pitch_rate << ", ";
	LOG << Current_Location.heading_rate << "\n\n";
	LOG.close();

	Current_Location.air_density = 1.2;
	Current_Location.air_density_fraction = 1;
	Current_Location.longitudinal_Q = 0.0001;
	Current_Location.longitudinal_true_airspeed = 0;
	Current_Location.vertical_speed = 0;
	Current_Location.AOA = 0;

	/*std::cout << "RPY: ";
	printf("%8.3f", Current_Location.roll);
	printf("%8.3f", Current_Location.pitch);
	printf("%8.3f", Current_Location.heading);
	printf("%8.3f", Current_Location.roll_rate);
	printf("%8.3f", Current_Location.pitch_rate);
	printf("%8.3f\t", Current_Location.heading_rate);*/

	/*auto start = std::chrono::steady_clock::now();

///*

	//std::cout << "Starting estimate()" << std::endl;

	XPlane_for_Location.get_data_from_XPlane();	
///*
	Current_Location.pitch = XPlane_for_Location.Current_XPlane_data.pitch_rad;
	Current_Location.roll = XPlane_for_Location.Current_XPlane_data.roll_rad;
	Current_Location.heading = XPlane_for_Location.Current_XPlane_data.heading_rad;
	Current_Location.pitch_rate = XPlane_for_Location.Current_XPlane_data.pitch_rot_vel_calcd;
	Current_Location.roll_rate = XPlane_for_Location.Current_XPlane_data.roll_rot_vel_calcd;
	Current_Location.heading_rate = XPlane_for_Location.Current_XPlane_data.heading_rot_vel_calcd;



	Current_Location.time = XPlane_for_Location.Current_XPlane_data.flight_time;

	//Static pressure in pascals
	float static_pressure = XPlane_for_Location.Current_XPlane_data.static_pressure_inHg * 3386.39;

	//Need sensor to measure - diurnal variation 37-90% avg for Oakley, KS in July
	float relative_humidity = 0.5;

	float ambient_temp_C = XPlane_for_Location.Current_XPlane_data.ambient_temp; //Ambient temp in degrees C
	float ambient_temp_K = ambient_temp_C + 273.15;

	//Teten's equation for water vapor pressure at saturation as function of temperature, in Pascals
	float saturation_vapor_pressure_exponent = (17.27 * ambient_temp_C) / (ambient_temp_C + 237.3);
	float saturation_vapor_pressure = 610.78 * std::exp(saturation_vapor_pressure_exponent);

	float water_vapor_partial_pressure = relative_humidity * saturation_vapor_pressure; //Water vapor pressure in Pascals
	float dry_air_partial_pressure = static_pressure - water_vapor_partial_pressure; //Dry air pressure in Pascals

	float R = 8.31446; //Universal gas constant, J/(K * mol)
	float dry_air_molar_mass = 0.0289652; //kg/mol
	float water_vapor_molar_mass = 0.018016; //kg/mol

	//Air density fundamentally a function of absolute pressure, relative humidity, and temperature
	Current_Location.air_density = ((dry_air_partial_pressure * dry_air_molar_mass) + (water_vapor_partial_pressure * water_vapor_molar_mass)) / (R * ambient_temp_K);
	Current_Location.air_density_fraction = Current_Location.air_density / 1.2247;

	Current_Location.longitudinal_Q = XPlane_for_Location.Current_XPlane_data.longitudinal_Q;

	float longitudinal_true_airspeed_abs = std::sqrt(std::abs(Current_Location.longitudinal_Q) / (0.5 * Current_Location.air_density));

	//Allow for dynamic pressure in backwards direction
	Current_Location.longitudinal_true_airspeed = std::copysignf(longitudinal_true_airspeed_abs, Current_Location.longitudinal_Q);



	float elev_delta = XPlane_for_Location.Current_XPlane_data.elevation - XPlane_for_Location.Last_XPlane_data.elevation;
	float elapsed_time = XPlane_for_Location.Current_XPlane_data.elapsed_time;
	if (elapsed_time == 0){
		elapsed_time = 0.01;
	}
	Current_Location.vertical_speed = elev_delta / elapsed_time;

	float wing_incidence = 0;
	Current_Location.AOA = std::atan(-Current_Location.vertical_speed / Current_Location.longitudinal_true_airspeed) + Current_Location.pitch + wing_incidence;

	/*
	printf("Temp: %f\n", ambient_temp_C);

	printf("Static_pressure: %f\n", static_pressure);

	printf("Air density: %f\n", Current_Location.air_density);

	printf("Long true airspeed: %f\n", Current_Location.longitudinal_true_airspeed);

	printf("Vertical speed: %f\n", Current_Location.vertical_speed);

	printf("AOA: %f\n", AOA * 180 / M_PI);
	*/

//*/
 

	//For testing without IMU connected
	/*
	Current_Location.pitch = 0.1;
	Current_Location.roll = -0.1;
	Current_Location.heading = -0.1;
	Current_Location.pitch_rate = 0;
	Current_Location.roll_rate = 0;
	Current_Location.heading_rate = 0;
	//*/
/*
	printf("time: %f  ", Current_Location.time);

	printf("roll: %f ", Current_Location.roll);

	printf("roll rate: %f ", Current_Location.roll_rate);

	printf("pitch: %f  ", Current_Location.pitch);

	printf("pitch rate: %f ", Current_Location.pitch_rate);

	printf("heading: %f  ", Current_Location.heading);

	printf("heading rate: %f\n", Current_Location.heading_rate);
*/

	/*auto end = std::chrono::steady_clock::now();

	auto looptime = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();

	std::cout << "Estimate() time: " << looptime << std::endl;*/

	return;
//*/




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


