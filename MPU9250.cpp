#include "Wire.h" //Is this the best place to include this?
#include <Arduino.h>
#include "MPU9250.h"
#include <math.h> //For mag euler angles trig, accel LSB offset rounding

#define PI 3.14159265

/*
This function should only:
	-Make sure there's an up to date FIFO containing sensor values and sample times for accel, gyro, and mag


Arduino I2C Library Reference: https://www.arduino.cc/en/Reference/Wire

MPU9250 Register Map: https://www.invensense.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf

MPU9250 Datasheet: https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf

AK8963 Datasheet: https://www.akm.com/akm/en/file/datasheet/AK8963C.pdf

Magnetic field intensity at earth's surface map: https://www.ngdc.noaa.gov/geomag/WMM/data/WMM2015/WMM2015_F_MERC.pdf

Magnetic field intensity calculator: https://www.ngdc.noaa.gov/geomag-web/#igrfwmm

	In Mojave:
		Field strength: About 47,336 nanoTeslas = 47.336 microTeslas = 473.66 milligauss
		Declination: 12.108 degrees east (if field vector was projected onto ground, 12 deg east of line to actual, not mag, north pole)
		Inclination: 59.768 degrees down into earth (straight down into earth at magnetic north pole)
			North/south is arbitrary, but the south pole of a magnet will want to point straight into the ground at mag north pole

			- Field strength decreasing by 98.1 nanoTeslas per year (so 47.238 uT in Sept. 2019)
			- Declination becoming less East at 0.09 degrees/year
			- Inclination moving up at 0.002 deg/year

			1,000 feet above ground (3,700 MSL):
				-47.329 uT
				-Virtually no change in direction

			Land at Hi Vista:
				-47.193 uT
				-11.94 degrees declination
				-59.527 degrees inclination

Adapted from Sparkfun's MPU9250 library

*/

static void MPU_Interrupt(); //Is this the best place for this?  Interrupt functions put in cpp for receiver


MPU9250::MPU9250()
{

}




/*
Start I2C for MPU9250

Also set MPU9250 interrupt pin

Should maybe check that WHO_AM_I returns 0x71/113 and that connection is for sure good

*/
void MPU9250::initI2C()
{

	int i2c_hz = 400000;

	Wire.begin();

	Wire.setClock(i2c_hz); //Default is 100kHz.  Error above 10 MHz.  Stops connecting (no error, but reads 00000000's) > 4.67 MHz
	
	Serial.print(micros()); Serial.print(" ");
	Serial.print("I2C started at: ");
	Serial.println(i2c_hz);

	/*Careful: at 4.5 MHz, works at first and then always drops after about 30 seconds.

	Reading mags and gyro seems to work fine (non-interrupt style) at 400 KHz.  
	Mag hangs after a short time at 800KHz

	Gyro sometimes gives bad reading every 4-8 times or so even at 100 KHz.
	Always same bad reading for each time on: gotten 9, 257, 4105, but always different with new time on.
	Can't replicate - working at 400 KHz, no interrupt, AK8963 on
		

	*/


}







void MPU9250::init()
{  

	Wire1.begin();
	Serial.print(micros()); Serial.print(" ");
	Serial.print("MPU9250: ");  Serial.println(readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250));

 // wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear default sleep mode bit (6), select 20 MHz internal oscillator
  delay(100); // Wait for all registers to reset 

 // get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready, else 20 Mhz
  delay(200); 
  /*
  	- Reset registers and restore default
  	- Wake from sleep
  	- 3 clock options (20 Mhz, PLL, or stop clock)

  */


 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);  //Select gyro, temp low-pass filter
 /*
If not doing anything fancy with FIFO or EXT_SYNC_SET, 8 options here:
	-8 gyro (and temp) DLPF configuration options

[Bit 7: -]
[Bit 6: FIFO mode - when 1, additional writes not written to FIFO when FIFO full.  When 0, new writes
replace oldest value in FIFO]
[5:3 - aka EXT_SYNC_SET.  Enables FSYNC Pin data to be sampled - FSYNC pin not being used now on MPU9250 board.
FSYNC is for Frame synchronization digital input - resends with each transmission to avoid bit slips
0 (so 000) to disable]
[2:0 - aka DPLF_CFG.  For bandwidth, delay, sampling frequency settings for DLPF options.  This DLPF is only used by the
gyroscope and temp sensor, not accelerometer.  THIS ONLY WORKS IF GYRO_CONFIG F_CHOICE_B = 00!!
000 (DLPF_CFG 0) would have, for gyro: bandwidth 250 Hz, 0.97 ms delay, 8 KHz sampling	For temp sensor: 4000 Hz bandwidth, 0.04 ms delay
001 (DLPF_CFG 1) would have, for gyro: 184 Hz bandwidth, 2.9 ms delay, 1 Khz sampling	For temp sensor: 188 Hz bandwidth, 1.9 ms delay
010 (DLPF_CFG 2) would have, for gyro: 92 Hz bandwidth, 3.9 ms delay, 1 Khz sampling		For temp sensor: 98 Hz bandwidth, 2.8 ms delay
011 (DLPF_CFG 3) would have, for gyro: 41 Hz bandwidth, 5.9 ms delay, 1 Khz sampling		For temp sensor: 42 Hz bandwidth, 4.8 ms delay
100 (DLPF_CFG 4) would have, for gyro: 20 Hz bandwidth, 9.9 ms delay, 1 Khz sampling		For temp sensor: 20 Hz bandwidth, 8.3 ms delay
101 (DLPF_CFG 5) would have, for gyro: 10 Hz bandwidth, 17.85 ms delay, 1 Khz sampling	For temp sensor: 10 Hz bandwidth, 13.4 ms delay
110 (DLPF_CFG 6) would have, for gyro: 5 Hz bandwidth, 33.48 ms delay, 1 Khz sampling	For temp sensor: 5 Hz bandwidth, 18.6 ms delay
111 (DLPF_CFG 7) would have, for gyro: 3600 Hz bandwidth, 0.17 ms delay, 8 Khz sampling		For temp sensor: 4000 Hz bandwidth, 0.04 ms delay
(this last one not a typo)

*/



  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);
  /*
  	-256 overall sample rate division options

	Output Data Rate (ODR) can be reduced (fewer samples per second):
	Data rate with SMPLRT_DIV = Normal Data Rate / (1 + SMPLRT_DIV)

	NOTE: only effective when last two bits of GYRO_CONFIG (FCHOICE_B) are 00, meaning DLPF on.

	Tried setting this to 0xFF 1/ (255 + 1) and read accel (filter 0x06, then 0x03), micros(), and data_ready INT pin every 3,200 us
		-INT pin almost always set to zero (data not), went to 1 exactly every 256 milliseconds (since 1 ms for 1 kHz, then / 256)
		-Accel data read out was the same every time until INT pin went back to 1

		Same with 8 kHz sample rate (still 256 ms between new data)
 */

  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00); //250 dps, no self-test, DLPF enabled
/*
	- 4 gyro range options
	- 3 gyro filter/sample options (DLPF on, off at 32 kHz with 3600 Hz low-pass, 32 Khz with 8800 Hz low-pass)

	[X self-test(st)][Y st][Z st]
	[4:3 select from 250, 500, 1000, 2000 dps range.  00 for 250 
	to 11 for 2000]
	[-]
	[1:0 - aka FCHOICE_B.  To turn on/off digital low pass filter (DLPF) If off, can have two dif bandwiths, sensor delays].
	DLPF selection here very confusing - use the table on p. 15 of register map which shows filter/sampling settings for dif
	combinations of these two bits.  Confusing because bits used here must be inverted, and that inverted value is the one
	to look up in the table.  So, if 11 input to this register, use 00 on table to see what will happen.
	- 00 for [1:0] spots: 11 on table -> DLPF on, now possible to configure with CONFIG register bits [2:0]
	- 10 for [1:0] spots: 01 on table -> DLPF off, gyro oscillation higher than 3600 Hz won't be detected, 0.11 ms delay to get
	sensor value, 32 KHz sampling frequency (analog low pass filter preventing >3600Hz signals?)

	- 01 or 11 for [1:0] spots: 10 or 00 on table -> DLPF off, 8800 Hz max gyro oscillation detection, 0.064 ms delay
	32 KHz sampling frequency.		Measured P2P: ~800 LSB at 250 dps range
*/
  

  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); //0x00 is 2g, no self-test
  /*
  	- 4 accel range options

  [X self-test(st)][Y st][Z st][4:3 select from 2g, 4g, 8g, 16g range, 
  with 00 for 2g to 11 for 16g][2:0 -]
  */


  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x06); //Might want to go with 0x07 - low noise, low delay

  /* 
	8 options:
		-No accel DLPF (4 kHz sample rate, 0.503 ms delay)
		-7 (two of the 8 are the same) accel DLPF configuratons

	[7:4 -][Bit 3: 1 to bypass accelerometer DLPF, 0 to enable DLPF 
	- Bit3 = 1 (DLPF below bypassed - [2:0] has no effect): 1046 Hz 3dB bandwidth (signal still has half its original strength),
	4 KHz sample rate, Dec1 filter block (?), 0.503ms delay, 300 micro(g)/rtHz Noise Density (square root of power spectral density
	of the noise output, where power spectral density is the power of the noise (so measured signal - actual signal)
	vs. frequency.  Noise is Gaussian.  Noise constant across all filter values.)
	- Using 00001XXX: Dec1 filter 		1046 Hz 3dB bandwidth, 4 KHz sample rate, 0.503 ms delay.  Measured P2P: ~300 LSB at 2g range
	- [2:0] is 000,001(A_DLPF_CFG 0,1): 218.1 Hz 3dB BW, 1 KHz sampling, 1.88 ms delay  Measured P2P: ~150 LSB at 2g range
	- [2:0] is 010 (A_DLPF_CFG 2): 		99 Hz 3dB bandwidth, 1 KHz sampling, 2.88 ms delay  Measured P2P: ~90 LSB at 2g
	- [2:0] is 011 (A_DLPF_CFG 3): 		44.8 Hz 3dB bandwidth, 1 KHz sampling, 4.88 ms delay  Measured P2P: ~60 LSB at 2g
	- [2:0] is 100 (A_DLPF_CFG 4): 		21.2 Hz 3dB bandwidth, 1 KHz sampling, 8.87 ms delay   Measured P2P: ~50 LSB at 2g range
	- [2:0] is 101 (A_DLPF_CFG 5): 		10.2 Hz 3dB bandwidth, 1 KHz sampling, 16.83 ms delay   Measured P2P: ~40 LSB at 2g range
	- [2:0] is 110 (A_DLPF_CFG 6): 		5.05 Hz 3dB bandwidth, 1 KHz sampling, 32.48 ms delay.  Measured P2P: ~30 LSB at 2g range
	- [2:0] is 111 (A_DLPF_CFG 7): 		420 Hz 3dB BW, 1 KHz sampling, 1.38 ms delay.  Measured P2P: ~80 LSB at 2g range

	This filtering/sampling is only for the accelerometer.

	Note: Config 7 is super noisy at high vibration even though relatively ok noise at rest.  Config 6 is amazing.



*/

  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01); //0x01 to Enable Raw Sensor Data Ready interrupt to propagate to interrupt pin. 
  //The timing of the interrupt can vary depending on the setting in register 36 I2C_MST_CTRL, bit [6] WAIT_FOR_ES ->
  //If set at 1, waits for external sensor data before triggering interrupt, but not set at 1, so not a problem.

  writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x32); //Bit1 = 1 or else can't use magnetometer!
  /*
  Ones that matter here (others reserved, FSYNC, I2C master):
	[Bit0: RESERVED]

	[Bit1: aka BYPASS_EN.  When asserted, the i2c_master interface pins(ES_CL and ES_DA) will go into ‘bypass mode’ 
	when the i2c master interface is disabled. The pins will float high due to the internal pull-up if not enabled 
	and the i2c master interface is disabled.]

	[Bit2: aka FSYNC_INT_MODE_EN.  1 – This enables the FSYNC pin to be used as an interrupt.
	A transition to the active level described by the ACTL_FSYNC bit will cause an interrupt.
	The status of the interrupt is read in the I2C Master Status register PASS_THROUGH bit.]

	[Bit3: aka ACTL_FSYNC.  1 – The logic level for the FSYNC pin as an interrupt is active low.]

  	[Bit4: aka INT_ANYRD_2CLEAR.  1 – Interrupt status is cleared if any read operation is performed.
  	0 – Interrupt status is cleared only by reading INT_STATUS register]

  	[Bit5: aka LATCH_INT_EN.  1 – INT pin level held until interrupt status is cleared.
  	0 – INT pin indicates interrupt pulse’s is width 50us. (probably don't want this - might trigger multiple reads)]

  	[Bit6: aka OPEN.  1 – INT pin is configured as open drain.  0 – INT pin is configured as push-pull.]

  	[Bit7: aka ACTL.  1 – The logic level for INT pin is active low.  (Default is 3.3V, drops to 0 to signal interrupt)
  	0 – The logic level for INT pin is active high.]


  	10110000 = 0xB0
  	00110000 = 0x30
  	10100000 = 0xA0
  	00100000 = 0x20

  	00110010 = 0x32 Must use this (or other where Bit1 = 1) or else BYPASS_EN not on and can't read magnetometer!

  	With 0x20, for example:
  		Interrupt pin in normally 0V
  		When sensor data is ready, the pin goes to 3.3V (every 1000us like clockwork)
  		When you do readBytes() on the MPU9250, for example, it will go to 0V the instant that happens
  			and stay there until the next 3.3V interrupt


  */


  /*
	For AK8963: (this all used to be its own function)

	Old library (from Adafruit I think):

	Get adjustment scaling values (calculated from ASA = sensitivity adjustment values), power for 16 bit output, 100 Hz sample rate

	To initialize the magnetometer:
		-Power it down (CNTL1)
		-Get sensitivity adjustment values (ASA) for each axis by going into Fuse ROM access mode

			Hadj = H(measured) * {[(ASA value - 128) * 0.5]/128 + 1}, so get values for the {[(ASA value - 128) * 0.5]/128 + 1} part

			AKA: mag_adjustment_scaling[3] values (one for each axis)
			Do this for each axis and return something to tell what they are

		-Power it down again (CNTL1)
		-Power up with 16 bit resolution and 100Hz continuous sample rate setting: 00010110

	Now:
		-Power up with 16 bit resolution and 100Hz continuous sample rate setting: 00010110


	[Bit4: aka BIT. Output bit setting - 0 for 14 bit output (0.6 uT steps), 1 for 16 bit output setting (0.15 uT steps).  ]
	[3:0 aka MODE.  Operation mode setting.  
	"0000": Power-down mode - power to almost everything off, but registers still accessible and data there not changed
		If not powered down, continuously taking sensor measurements and writing to their output registers
	"0001": Single measurement mode - measures one time and powered down
	"0010": Continuous measurement mode 1 - 8 Hz sampling
	"0110": Continuous measurement mode 2 - 100 Hz sampling
		For both continuous measurement modes, actually powers down in between 8 Hz/100Hz measurements
		(but measurement takes 7.2 ms nominally)
	"0100": External trigger measurement mode
	"1000": Self-test mode
	"1111": Fuse ROM access mode

	00010110 = 0x16 hexadecimal = 16 bit output, 100 Hz continuous measurement mode

*/

	Serial.print("AK8963: ");  Serial.println(readByte(AK8963_ADDRESS, WHO_AM_I_AK8963));

	writeByte(AK8963_ADDRESS, AK8963_CNTL1, 0x16); 
  


}



void MPU9250::readAccel1()
{  

	//Check if the MPU9250 is on the bus and exit if not
	Wire1.beginTransmission(0x68);
    Wire1.write(0x75);
    //int status = Wire1.endTransmissionFast(false); //Custom function.  Edited Arduino library to add endTransmission with low timeout
	int status = Wire1.endTransmission(false);

/*
    if (status != 0){
    	Wire1.recover(71, 70);
    	return;
    }
    
    */

	uint8_t accelData[6];  // x/y/z accel register data stored here

	readBytes(MPU9250_ADDRESS1, ACCEL_XOUT_H, 6, &accelData[0]);  // Read the six raw data registers into data array

	int16_t accel1_x = ((int16_t)accelData[0] << 8) | accelData[1];
	int16_t accel1_y = ((int16_t)accelData[2] << 8) | accelData[3];
	int16_t accel1_z = ((int16_t)accelData[4] << 8) | accelData[5];

/*

	float X_accel_scaling = -0.002405; // Negative so that X arrow pointing down to ground is positive (follows gravity vector)
	float Y_accel_scaling = -0.002399; //
	float Z_accel_scaling = -0.00237; //
*/


    Serial.print("acc: ");
    Serial.print(accel1_x);
    Serial.print(" ");
    Serial.print(accel1_y);
    Serial.print(" ");
    Serial.print(accel1_z);
    Serial.print(" ");


}



void MPU9250::readAccel2()
{  


	//Check if the MPU9250 is on the bus and exit if not
	Wire1.beginTransmission(0x69);
    Wire1.write(0x75);
    //int status = Wire1.endTransmissionFast(false); //Custom function.  Edited Arduino library to add endTransmission with low timeout
    int status = Wire1.endTransmission(false);

/*
    if (status != 0){
    	Wire1.recover(71, 70);
    	return;
    }
    */

	uint8_t accelData[6];  // x/y/z accel register data stored here
	  
	readBytes(MPU9250_ADDRESS2, ACCEL_XOUT_H, 6, &accelData[0]);  // Read the six raw data registers into data array

	int16_t accel2_x = ((int16_t)accelData[0] << 8) | accelData[1];
	int16_t accel2_y = ((int16_t)accelData[2] << 8) | accelData[3];
	int16_t accel2_z = ((int16_t)accelData[4] << 8) | accelData[5];


/*

	float X_accel_scaling = -0.002405; // Negative so that X arrow pointing down to ground is positive (follows gravity vector)
	float Y_accel_scaling = -0.002399; //
	float Z_accel_scaling = -0.00237; //
*/


    Serial.print("acc2: ");
    Serial.print(accel2_x);
    Serial.print(" ");
    Serial.print(accel2_y);
    Serial.print(" ");
    Serial.print(accel2_z);
    Serial.print(" ");


}



void MPU9250::readGyro1()
{  


	//Check if the MPU9250 is on the bus and exit if not
	Wire1.beginTransmission(0x69);
    Wire1.write(0x75);
    //int status = Wire1.endTransmissionFast(false); //Custom function.  Edited Arduino library to add endTransmission with low timeout
    int status = Wire1.endTransmission(false);

/*
    //Do recover function and exit
    if (status != 0){
    	Wire1.recover(71, 70);
    	return;
    }
    */

	  uint8_t gyroData[6];  // x/y/z gyro register data stored here

	  readBytes(MPU9250_ADDRESS1, GYRO_XOUT_H, 6, &gyroData[0]);  // Read the six raw data registers into data array

	  int16_t gyro1_x = ((int16_t)gyroData[0] << 8) | gyroData[1];
	  int16_t gyro1_y = ((int16_t)gyroData[2] << 8) | gyroData[3];
	  int16_t gyro1_z = ((int16_t)gyroData[4] << 8) | gyroData[5];


/*
    float X_gyro_scaling = 0.007629; // 250/32768
	float Y_gyro_scaling = 0.007629; // 250/32768
	float Z_gyro_scaling = 0.007629; // 250/32768

*/


    Serial.print("gyro1: ");
    Serial.print(gyro1_x);
    Serial.print(" ");
    Serial.print(gyro1_y);
    Serial.print(" ");
    Serial.print(gyro1_z);
    Serial.print(" ");

}



void MPU9250::readGyro2()
{  

	//Check if the MPU9250 is on the bus and exit if not
	Wire1.beginTransmission(0x69);
    Wire1.write(0x75);
    //int status = Wire1.endTransmissionFast(false); //Custom function.  Edited Arduino library to add endTransmission with low timeout
    int status = Wire1.endTransmission(false);

/*
    if (status != 0){
    	Wire1.recover(71, 70);
    	return;
    }
*/

	  uint8_t gyroData[6];  // x/y/z gyro register data stored here


	  readBytes(MPU9250_ADDRESS2, GYRO_XOUT_H, 6, &gyroData[0]);  // Read the six raw data registers into data array

	  int16_t gyro2_x = ((int16_t)gyroData[0] << 8) | gyroData[1];
	  int16_t gyro2_y = ((int16_t)gyroData[2] << 8) | gyroData[3];
	  int16_t gyro2_z = ((int16_t)gyroData[4] << 8) | gyroData[5];


/*
    float X_gyro_scaling = 0.007629; // 250/32768
	float Y_gyro_scaling = 0.007629; // 250/32768
	float Z_gyro_scaling = 0.007629; // 250/32768

*/


    Serial.print("gyro2: ");
    Serial.print(gyro2_x);
    Serial.print(" ");
    Serial.print(gyro2_y);
    Serial.print(" ");
    Serial.print(gyro2_z);
    Serial.print(" ");

}



void MPU9250::readMag()
{  

	//Check if the MPU9250 is on the bus and exit if not
	Wire1.beginTransmission(0x68);
    Wire1.write(0x75);
    //int status = Wire1.endTransmissionFast(false); //Custom function.  Edited Arduino library to add endTransmission with low timeout
 	int status = Wire1.endTransmission(false);

/*
    if (status != 0){
    	Wire1.recover(71, 70);
    	return;
    }
    */

	  uint8_t magData[7]; //For 6 register axes plus required ST2 status register
	/* 
	if() here makes sure new data has been supplied to the sensor registers
		& is the bitwise & operator.  If both Bit1 1, then new Bit1 = 1, otherwise 0, etc.
		Thing here is that AK8963_ST1 register can have 1 in Bit1 place, like 00000010 or 00000011, 
		so bitwise AND with 00000001 creates a new byte that is just like AK8963_ST1 but without a 1 second from right
		if(byte) executes if byte not 0x00, so won't execute if last ST1 bit, data ready, is 0
	*/
    if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) 
    {

    	//Read all 6 registers and required ST2 
    	readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &magData[0]);

    	uint8_t ST2 = magData[6]; //Save value of overflow register in the 7th spot (so 6), ST2

    	// ST2 & 0x08 would return a byte with just ST2's bit value in Bit3's place.  So executes if no overflow, aka zero.
    	if(!(ST2 & 0x08))
    	{
    		mag_x = ((int16_t)magData[3] << 8) | magData[2]; //X-axis for breadboard arrow (even though register says Y)
    			//Goes with direction of arrow on breadboard - positive when arrow pointing in direction of magnetic field
    			//negative when point against magnetic field (positive when X arrow pointing down into ground)
    		mag_y = ((int16_t)magData[1] << 8) | magData[0]; //Y-axis for breadboard arrow (even though register says X)
    			//Goes with direction of arrow on breadboard - positive when arrow pointing in direction of magnetic field
    			//negative when point against magnetic field (positive when Y arrow pointing down into ground)
    		mag_z = -((int16_t)magData[5] << 8) | magData[4];
    			//Same thing - goes with direction of arrow on breadboard (feathers (the x) mean looking at the tail)
    			//So most positive when arrow pointing in to ground

    	}

    }


    Serial.print("mag: ");
    Serial.print(mag_x);
    Serial.print(" ");
    Serial.print(mag_y);
    Serial.print(" ");
    Serial.print(mag_z);
    Serial.print(" ");

}



/*

	For finding initial LSB offsets for Accel and Gyro - note that this will change some during flight

	Note: finding correct values with this function assumes:
		-Accelerometer is flat (assumes that accel is at zero in x and y and finds those offsets, so can't be tilted 
		or will think that position is flat which is only ok for pretty small angle)
		-Nothing is moving (or gyro won't know what zero dps is)

	Writing to the FIFO 250 times, with a 100 sample FIFO gives values almost always within 1% of the last time 
	(e.g. right around -520 for x_accel plus or minus 3 or 4.)  Same with gyro - super close from one time to time

*/
void MPU9250::findInitialAccelGyroOffsets(float * accel_gyro_LSB_offsets)
{

	/*
		Read the accel and gyro data to the FIFO a bunch of times until clean (no startup transients still in FIFO)
	*/

	writeAccelRegistersFIFO();
	writeGyroRegistersFIFO();

	Serial.print(micros()); Serial.print(" ");
	Serial.print("First accel x axis read from FIFO (verify sensor returning data): ");
	Serial.println(accelFIFO_current.x_axis[0]);

	Serial.print(micros()); Serial.print(" ");
	Serial.print("First gyro x axis read from FIFO (verify sensor returning data): ");
	Serial.println(gyroFIFO_current.x_axis[0]);


	for (int i = 0; i < 2000; i++){

		writeAccelRegistersFIFO();
		writeGyroRegistersFIFO();

	}



	/*
		Find the average of the contents of the FIFO for accel and gyro for each axis 
	*/
	int x_accel_LSB_total = 0;
	int y_accel_LSB_total = 0;

	int x_gyro_LSB_total = 0;
	int y_gyro_LSB_total = 0;
	int z_gyro_LSB_total = 0;


	for (int i = 0; i < IMU_FIFO_SIZE; i++){

		x_accel_LSB_total += accelFIFO_current.x_axis[i];
		y_accel_LSB_total += accelFIFO_current.y_axis[i];

		x_gyro_LSB_total += gyroFIFO_current.x_axis[i];
		y_gyro_LSB_total += gyroFIFO_current.y_axis[i];
		z_gyro_LSB_total += gyroFIFO_current.z_axis[i];

	}


	accel_gyro_LSB_offsets[0] = - (float) x_accel_LSB_total / (float) IMU_FIFO_SIZE;
	accel_gyro_LSB_offsets[1] = - (float) y_accel_LSB_total / (float) IMU_FIFO_SIZE;
	accel_gyro_LSB_offsets[2] = 0; //Should assume at first that it's correct (no offset).  Maybe more sophisticated update later.

	accel_gyro_LSB_offsets[3] = - (float) x_gyro_LSB_total / (float) IMU_FIFO_SIZE;
	accel_gyro_LSB_offsets[4] = - (float) y_gyro_LSB_total / (float) IMU_FIFO_SIZE;
	accel_gyro_LSB_offsets[5] = - (float) z_gyro_LSB_total / (float) IMU_FIFO_SIZE;

	Serial.print(micros()); Serial.print(" ");
	Serial.println("Gyro and accel offset results:");

	for(int i = 0;  i < 6; i++){

		Serial.print(accel_gyro_LSB_offsets[i]); Serial.print("  ");
	}

	Serial.println("  ");
	

}



void MPU9250::initMPU_Interrupt()
{


	pinMode(MPU_INT_PIN, INPUT); //Set Pin 41 as an input so can be used for interrupt

  	attachInterrupt(MPU_INT_PIN, &MPU_Interrupt, RISING); //Run MPU_Interrupt whenever Pin 41 goes low to high

}





/*
	Confirmed with microsecond() timer inside interrupt that it is being called at exactly 1000Hz by the rising edges of the
	MPU9250 INT pin.  Interrupt inited with:

	pinMode(MPU_INT_PIN [here, using 41], INPUT);
	attachInterrupt(MPU_INT_PIN, &MPU_Interrupt, RISING);

	And, in initMPU9250, using:
		writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x30);
		writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);



	At 4 MHz I2C, this takes 60-62 us to complete (reading 14 registers)
	At 1 MHz I2C, takes 279 us


*/
int sample_microseconds;
uint8_t accel_temp_gyro[14];
int sample_microseconds_end;

void MPU_Interrupt() //ISR - Must take no parameters and return nothing
	{ 

  		sample_microseconds = micros();


	  	Wire.beginTransmission(MPU9250_ADDRESS1);         // Begin transmission to the slave address
	    //If MPU9250's AD0 pin is pulled to GND, it will have the address 0x68.  If pulled high, it will be 0x69
	  
	  	Wire.write(ACCEL_XOUT_H);        //If write() called between beginTransmission and endTransmission, it will queue 
	    //that byte(s) for transmission to a slave (so looks like it's basically a request for the slave to get that data ready)
	  
	  	Wire.endTransmission(false);      // Sends the byte(s).  False makes it send a restart message
	    //on the bus to prevent the bus from being released, so ARM stays the master

		
	  	Wire.requestFrom(MPU9250_ADDRESS1, (uint8_t)14);  //Master uses this to request a certain number of bytes
	  	// !! This is what actually does a read from the MPU and causes the interrupt pin to go back to normal
	  	//This throws a compiler error, but uploads anyway
  		
  		uint8_t i = 0;
  		while (Wire.available()) { 		 //Wire.available() returns the number of bytes available for retreival with read()
   			 accel_temp_gyro[i++] = Wire.read(); }         // Put read results in the Rx buffer


   		sample_microseconds_end = micros();

	}



/*

	At 4 MHz I2C clock, time between start and finish of this function is 35-36 *micro*seconds.

	Data is ready (= this function gets a new good value) as long as this function is always called more than about 1,005 us
	apart.  At 1,000 us apart, data isn't ready once every couple thousand, but at 1,005 us, data is always ready.
	This is true at:
		- 0 in SMPLRT_DIV and 4kHz accel sampling (0x08) (used delayMicroseconds() to tune how often called)
		- 0 in SMPLRT_DIV and 1kHz accel sampling (0x07)
		- 0 in SMPLRT_DIV and 1kHz accel sampling, 32 ms delay (!), 5 Hz low-pass (0x06) (so each value good, just 32 ms old?)
		- 0 in SMPLRT_DIV and 1kHz accel sampling, 1.88 ms delay, 218 Hz low-pass (0x00) (very occasionally misses)

	!!INT_STATUS sometimes goes haywire (data not ready and lots of other 1's when normally all 0's) until move around
	(probably Wake-On-Motion?)

	At some point, this should also return the exact real timestamp associated with that data (probably have to use the
	INT Pin for Data Ready interrupt).  And then maybe subtract the delay time to get a base reference on when that data
	exists in time

	If call this at 950 Hz or so consistently, then even though getting close, it will phase lag more and more until the point
	where it samples, a new measurement is ready right after, the next measurement is ready, then it samples again.

		-One solution without interrupt is to sample at 2000 Hz or 4000 Hz and check if data is ready so will never be older
		than 500 us or 250 us + 1.38 ms delay (or whatever)


*/

void MPU9250::writeAccelRegistersFIFO()
{

  uint8_t rawData[6];  // x/y/z accel register data stored here

  int write_loc = accelFIFO_current.most_recent_write + 1;

  //If statement for when get to the end of the FIFO and have to write to the beginning again
  if (write_loc == IMU_FIFO_SIZE){
  	write_loc = 0;
  }


  accelFIFO_current.sample_us[write_loc] = micros(); //Might want to subtract a little bit from this to get real read time

  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array

  accelFIFO_current.x_axis[write_loc] = ((int16_t)rawData[0] << 8) | rawData[1];
  accelFIFO_current.y_axis[write_loc] = ((int16_t)rawData[2] << 8) | rawData[3];
  accelFIFO_current.z_axis[write_loc] = ((int16_t)rawData[4] << 8) | rawData[5];

  accelFIFO_current.most_recent_write = write_loc;


  
}


/*
	Breaks if:
		Called at more than 1 KHz - need a delay in most cases (for some reason putting in "if raw data ready" doesn't help)

	Currently:
		If called fast enough (would have to be 1 KHz for max performance), will make sure _MPU9250.gyroRegisters is up to date

	Outputs (to most recent slot in IMU_FIFO datatype, gyroFIFO):
		int16_t gyroFIFO_current.x_axis[0] = GYRO_XOUT concatenation
		int16_t gyroFIFO_current.y_axis[1] = GYRO_YOUT concatenation
		int16_t gyroFIFO_current.z_axis[2] = GYRO_ZOUT concatenation
		unsigned int gyroFIFO_current.sample_us = Microseconds since start (maxes out at 4.2 billion - more than an hour)

	At 4 MHz I2C clock, time between start and finish of this function is 37-38 *micro*seconds.

*/

void MPU9250::writeGyroRegistersFIFO()
{


  uint8_t rawData[6];  // x/y/z accel register data stored here

  int write_loc = gyroFIFO_current.most_recent_write + 1;

  //If statement for when get to the end of the FIFO and have to write to the beginning again
  if (write_loc == IMU_FIFO_SIZE){
  	write_loc = 0;
  }


  gyroFIFO_current.sample_us[write_loc] = micros(); //Might want to subtract a little bit from this to get real read time

  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array

  gyroFIFO_current.x_axis[write_loc] = ((int16_t)rawData[0] << 8) | rawData[1];
  gyroFIFO_current.y_axis[write_loc] = ((int16_t)rawData[2] << 8) | rawData[3];
  gyroFIFO_current.z_axis[write_loc] = ((int16_t)rawData[4] << 8) | rawData[5];

  gyroFIFO_current.most_recent_write = write_loc;



}



/*
For this to work:
	-BYPASS_EN must be set in initMPU9250

Currently:
	If called fast enough, will always make sure _MPU9250.magRegisters is up to date



This function should give the most recent int16_t values in three axes (so -32k to 32k, not 0 to 65k)

	-Check with ST1 to make sure new sensor value is ready (not reading the old one)
	-Read the sensor output registers
	-Check with ST2 to make sure overflow bit (Bit3) is 0
	-If both ready and no overflow, scale the output value and assign to destination variable

	H (in ) = {[16 bit register data]*[LSB scaling value]*[Adjustment scaling value]  +  [Magnetic field bias due to where on earth]}
				* [Mag]

	If new sensor value isn't ready or there's an overflow, just don't update the value at *destination
	That way, it will just stay at the last good value

	In destination[3], the byte is set to 0 if everything good, 1 for overflow, 2 for no data

	Things known to cause this to fail:
		-Reading sensor data too soon (11 ms delay between readMags is enough for 0% error in 1000's of samples)
		-Bad initial connection = WHO_AM_I wrong (unplug and plug back in)

	! What is the latency?  Seems like it would read back data up to 10 ms old if called right before next measurement taken.
		Datasheet says single measurement takes 7.2 ms - maybe accumulates over that time, so avg. 3-4 ms old (just a guess)

		Might want to call this at 500 or 1000 Hz to make sure phase shifting doesn't have as much of an effect (because
		clocks not synchronized with when measurement ready)


	Called:
		Should be called automatically at maybe 500 to 1000 Hz to avoid sensor delay phase shifting

	Outputs:
		int16_t destination[0] = MAG_XOUT concatenation
		int16_t destination[1] = MAG_YOUT concatenation
		int16_t destination[2] = MAG_ZOUT concatenation
		int16_t destination[3] = Sensor data ready/overflow flag
		int16_t destination[4] = Microseconds after most recent 10,000 microsecond mark sample taken


*/
void MPU9250::writeMagRegistersFIFO()
{
	/*
	From AK8963 datasheet:
		Must read the ST2 register to tell AK8963 that sensor reading has finished
			Does this because sensor registers are write protected while being read off, so need to read ST2
			to turn off the write protection and allow new data to be written to them
	*/

	int write_loc = magFIFO_current.most_recent_write + 1;

	//When get to the end of the FIFO and have to write to beginning again
	if (write_loc == IMU_FIFO_SIZE){
	  	write_loc = 0;
	 }

	uint8_t rawData[7]; //For 6 register axes plus required ST2
	//readBits(AK8963_ADDRESS, AK8963_ST1);
	/* 
	if() here makes sure new data has been supplied to the sensor registers
		& is the bitwise & operator.  If both Bit1 1, then new Bit1 = 1, otherwise 0, etc.
		Thing here is that AK8963_ST1 register can have 1 in Bit1 place, like 00000010 or 00000011, 
		so bitwise AND with 00000001 creates a new byte that is just like AK8963_ST1 but without a 1 second from right
		if(byte) executes if byte not 0x00, so won't execute if last ST1 bit, data ready, is 0
	*/
    if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) 
    {

    	//Time as close as possible to when most recent sample is confirmed ready 
	  	magFIFO_current.sample_us[write_loc] = micros(); //Might want to subtract a little bit from this to get real read time

    	//Read all 6 registers and required ST2 
    	readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);

    	uint8_t ST2 = rawData[6]; //Save value of overflow register in the 7th spot (so 6), ST2

    	// ST2 & 0x08 would return a byte with just ST2's bit value in Bit3's place.  So executes if no overflow, aka zero.
    	if(!(ST2 & 0x08))
    	{
    		//Serial.println("data ready, no overflow");
    		magFIFO_current.x_axis[write_loc] = ((int16_t)rawData[3] << 8) | rawData[2]; //X-axis for breadboard arrow (even though register says Y)
    			//Goes with direction of arrow on breadboard - positive when arrow pointing in direction of magnetic field
    			//negative when point against magnetic field (positive when X arrow pointing down into ground)
    		magFIFO_current.y_axis[write_loc] = ((int16_t)rawData[1] << 8) | rawData[0]; //Y-axis for breadboard arrow (even though register says X)
    			//Goes with direction of arrow on breadboard - positive when arrow pointing in direction of magnetic field
    			//negative when point against magnetic field (positive when Y arrow pointing down into ground)
    		magFIFO_current.z_axis[write_loc] = -((int16_t)rawData[5] << 8) | rawData[4];
    			//Same thing - goes with direction of arrow on breadboard (feathers (the x) mean looking at the tail)
    			//So most positive when arrow pointing in to ground

    		magFIFO_current.most_recent_write = write_loc;
    	}

    }

}




/*

	Analyze the accel FIFO data to get the values for the ACCEL_DATA struct
		-Most recent timestamp - microseconds
		-Most recent angle for past timestep, each axis - microdegrees
		-Put results into FIFO

*/
void MPU9250::calcMagData()
{

	int read_loc = magFIFO_current.most_recent_write;

	float X_scaling = 1.5; //For 0.15 uT/LSB spec in datasheet
	float Y_scaling = 1.5; //
	float Z_scaling = 1.5; //

	float X_mag = (magFIFO_current.x_axis[read_loc] + 71) * X_scaling;
	float Y_mag = (magFIFO_current.y_axis[read_loc] - 145) * Y_scaling;
	float Z_mag = (magFIFO_current.z_axis[read_loc] - 151) * Z_scaling;

/*
	Serial.print(X_mag); Serial.print(" ");
	Serial.print(Y_mag); Serial.print(" ");
	Serial.print(Z_mag); Serial.print(" ");

	Serial.println(" ");
	*/

	int x_mag_total = 0;
	int y_mag_total = 0;
	int z_mag_total = 0;

	int mag_filter_samples = 10;  //How many samples the moving average looks at

	//Find the total of the last [filter samples] most recent values in the FIFO
	for (int ii = 0; ii < mag_filter_samples; ii++){

		int read_loc = magFIFO_current.most_recent_write - ii; //Keep reading farther back in history

		//If get down to reading below zero, convert the negative values into their equivalent positive values at max end
		if(read_loc < 0){
			read_loc = IMU_FIFO_SIZE + read_loc;
		}

		x_mag_total += X_mag;
		y_mag_total += Y_mag;
		z_mag_total += Z_mag;

	}

	float X_mag_avg = (float)x_mag_total / mag_filter_samples;
	float Y_mag_avg = (float)y_mag_total / mag_filter_samples;
	float Z_mag_avg = (float)z_mag_total / mag_filter_samples;

	float X_Y_mag_avg = sqrt(X_mag_avg * X_mag_avg + Y_mag_avg * Y_mag_avg);

	//Serial.print(X_mag_avg); Serial.print(" ");
	//Serial.print(Y_mag_avg); Serial.print(" ");
	//Serial.print(Z_mag_avg); Serial.print(" ");
	//Serial.print(X_Y_mag_avg); Serial.print(" ");

	//Serial.println(" ");

	//float pitch_udeg = - atan2(Y_accel, -Z_accel) * 180 / PI * 1000000; //Positive pitch is pitching up
	//float roll_udeg = atan2(X_accel, -Z_accel) * 180 / PI * 1000000; //Positive roll is to the right (when looking at vehicle from behind)


	//Serial.print(pitch_udeg / 1000); Serial.print(" ");
	

	//int write_loc = most_recent_write_loc



}

/*

	Analyze the gyro FIFO data to get the values for the GYRO_DATA struct
		-Most recent timestamp - microseconds
		-Most recent angle delta over past timestep, each axis - microdegrees
		-Most recent angular rate over past timestep, each axis - microdegrees per second

*/
void MPU9250::calcGyroData()
{
	int current_sample_loc = gyroFIFO_current.most_recent_write;

	gyro_pitch_data_current.current_timestamp = gyroFIFO_current.sample_us[current_sample_loc];
	gyro_roll_data_current.current_timestamp = gyroFIFO_current.sample_us[current_sample_loc];
	gyro_yaw_data_current.current_timestamp = gyroFIFO_current.sample_us[current_sample_loc];


	int last_sample_loc = current_sample_loc - 1;
	if(last_sample_loc < 0){
		last_sample_loc = IMU_FIFO_SIZE - 1;
	}
	unsigned int last_gyro_timestamp = gyroFIFO_current.sample_us[last_sample_loc];

	gyro_pitch_data_current.last_delta_t = gyro_pitch_data_current.current_timestamp - last_gyro_timestamp;
	gyro_roll_data_current.last_delta_t = gyro_roll_data_current.current_timestamp - last_gyro_timestamp;
	gyro_yaw_data_current.last_delta_t = gyro_yaw_data_current.current_timestamp - last_gyro_timestamp;


	float X_scaling = 0.007629; // 250/32768
	float Y_scaling = 0.007629; // 250/32768
	float Z_scaling = 0.007629; // 250/32768

	//Current gyro DPS = (MPU9250 raw 16 bit output + offset found during no-movement startup) * Scaling factor (from datasheet)
	float X_DPS_gyro = (gyroFIFO_current.x_axis[current_sample_loc] + accel_gyro_LSB_offsets[3]) * X_scaling;
	float Y_DPS_gyro = (gyroFIFO_current.y_axis[current_sample_loc] + accel_gyro_LSB_offsets[4]) * Y_scaling;
	float Z_DPS_gyro = (gyroFIFO_current.z_axis[current_sample_loc] + accel_gyro_LSB_offsets[5]) * Z_scaling;

	//Serial.print(Y_DPS_gyro * 1000); Serial.print(" ");

	gyro_pitch_data_current.last_delta_udeg = X_DPS_gyro * gyro_pitch_data_current.last_delta_t;
	gyro_roll_data_current.last_delta_udeg = Y_DPS_gyro * gyro_roll_data_current.last_delta_t;
	gyro_yaw_data_current.last_delta_udeg = Z_DPS_gyro * gyro_yaw_data_current.last_delta_t;


	//Serial.print(Y_DPS_gyro * 100000);  Serial.print(" ");

	gyro_roll_data_current.gyro_integrator_udeg_test += gyro_roll_data_current.last_delta_udeg;
	//Serial.print(gyro_roll_data_current.gyro_integrator_udeg_test);  Serial.print(" ");

	gyro_pitch_data_current.last_udeg_sec = X_DPS_gyro * 1000000;
	gyro_roll_data_current.last_udeg_sec = Y_DPS_gyro * 1000000;
	gyro_yaw_data_current.last_udeg_sec = Z_DPS_gyro * 1000000;


	//Moving average for gyro data

	int gyro_filter_samples = 5;  //How many samples the moving average looks at

	int x_gyro_LSB_total = 0;
	int y_gyro_LSB_total = 0;
	int z_gyro_LSB_total = 0;

	//Find the total of the last [filter samples] most recent values in the FIFO
	for (int ii = 0; ii < gyro_filter_samples; ii++){

		int read_loc = gyroFIFO_current.most_recent_write - ii; //Keep reading farther back in history

		//If get down to reading below zero, convert the negative values into their equivalent positive values at max end
		if(read_loc < 0){
			read_loc = IMU_FIFO_SIZE + read_loc;
		}

		x_gyro_LSB_total += gyroFIFO_current.x_axis[read_loc];
		y_gyro_LSB_total += gyroFIFO_current.y_axis[read_loc];
		z_gyro_LSB_total += gyroFIFO_current.z_axis[read_loc];

	}

	double X_DPS_gyro_avg = (x_gyro_LSB_total / gyro_filter_samples + accel_gyro_LSB_offsets[3]) * X_scaling;
	double Y_DPS_gyro_avg = (y_gyro_LSB_total / gyro_filter_samples + accel_gyro_LSB_offsets[4]) * Y_scaling;
	double Z_DPS_gyro_avg = (z_gyro_LSB_total / gyro_filter_samples + accel_gyro_LSB_offsets[5]) * Z_scaling;

	gyro_pitch_data_current.filtered_dps = X_DPS_gyro_avg;
	gyro_roll_data_current.filtered_dps = Y_DPS_gyro_avg;
	gyro_yaw_data_current.filtered_dps = Z_DPS_gyro_avg;



}


/*

	Analyze the accel FIFO data to get the values for the ACCEL_DATA struct
		-Most recent timestamp - microseconds
		-Most recent angle for past timestep, each axis - microdegrees
		-Put results into FIFO

*/
void MPU9250::calcAccelData()
{

	int read_loc = accelFIFO_current.most_recent_write;

	float X_scaling = -0.002405; // Negative so that X arrow pointing down to ground is positive (follows gravity vector)
	float Y_scaling = -0.002399; //
	float Z_scaling = -0.00237; //

	float X_accel = (accelFIFO_current.x_axis[read_loc] + accel_gyro_LSB_offsets[0]) * X_scaling;
	float Y_accel = (accelFIFO_current.y_axis[read_loc] + accel_gyro_LSB_offsets[1]) * Y_scaling;
	float Z_accel = (accelFIFO_current.z_axis[read_loc] + accel_gyro_LSB_offsets[2]) * Z_scaling;

	/*
	Serial.print("accels: ");
	Serial.print(micros()); Serial.print(" ");
	Serial.print(X_accel); Serial.print(" ");
	Serial.print(Y_accel); Serial.print(" ");
	Serial.print(Z_accel); Serial.print(" ");
*/


	float accel_pitch = (float)accel_pitch_data_current.filtered_udeg / 1000000;
	double accel_roll = (double)accel_roll_data_current.filtered_udeg / 1000000;

	double filter_error = gyro_accel_error_roll_current.follow_accel_value_test - accel_roll;

	double freakout_correction = 0;

	float freakout_correction_ok = gyro_accel_error_roll_current.follow_accel_slope_test * filter_error;

	if (freakout_correction_ok < 0){
		freakout_correction = -gyro_accel_error_roll_current.follow_accel_slope_test * 0.01/(abs(filter_error) + 0.1);

	}

	double filter_acc = -filter_error / 100000 + freakout_correction;
	gyro_accel_error_roll_current.follow_accel_slope_test += filter_acc;
	gyro_accel_error_roll_current.follow_accel_value_test += gyro_accel_error_roll_current.follow_accel_slope_test;



	//Serial.print(accel_pitch * 1000); Serial.print(" ");
	//Serial.print(accel_roll * 1000); Serial.print(" ");
	//Serial.print(gyro_accel_error_roll_current.follow_accel_value_test * 1000); Serial.print(" ");

	//Serial.println(" ");
	

	float pitch_udeg = - atan2(Y_accel, -Z_accel) * 180 / PI * 1000000; //Positive pitch is pitching up
	float roll_udeg = atan2(X_accel, -Z_accel) * 180 / PI * 1000000; //Positive roll is to the right (when looking at vehicle from behind)

	accel_pitch_data_current.last_udeg = pitch_udeg;
	accel_roll_data_current.last_udeg = roll_udeg;

	//Serial.print(pitch_udeg / 1000); Serial.print(" ");
	

	//int write_loc = most_recent_write_loc



}

/*
	Use most recent accel FIFO data to get a guess of current value

	For this implementation now, just finds average of every value of certain axis in FIFO, so effectively a moving average


*/
void MPU9250::accelFilter()
{


	float x_accel_LSB_total = 0;
	float y_accel_LSB_total = 0;
	float z_accel_LSB_total = 0;

	int accel_filter_samples = 10;  //How many samples the moving average looks at

	//Find the total of the last [filter samples] most recent values in the FIFO
	for (int ii = 0; ii < accel_filter_samples; ii++){

		int read_loc = accelFIFO_current.most_recent_write - ii; //Keep reading farther back in history

		//If get down to reading below zero, convert the negative values into their equivalent positive values at max end
		if(read_loc < 0){
			read_loc = IMU_FIFO_SIZE + read_loc;
		}

		x_accel_LSB_total += accelFIFO_current.x_axis[read_loc];
		y_accel_LSB_total += accelFIFO_current.y_axis[read_loc];
		z_accel_LSB_total += accelFIFO_current.z_axis[read_loc];

	}

	float X_accel_avg = x_accel_LSB_total / accel_filter_samples;
	float Y_accel_avg = y_accel_LSB_total / accel_filter_samples;
	float Z_accel_avg = z_accel_LSB_total / accel_filter_samples;



	float X_scaling = -0.002405; // Negative so that X arrow pointing down to ground is positive (follows gravity vector)
	float Y_scaling = -0.002399; //
	float Z_scaling = -0.00237; //

	
	float X_accel = (X_accel_avg + accel_gyro_LSB_offsets[0]) * X_scaling;
	float Y_accel = (Y_accel_avg + accel_gyro_LSB_offsets[1]) * Y_scaling;
	float Z_accel = (Z_accel_avg + accel_gyro_LSB_offsets[2]) * Z_scaling;

	float pitch_udeg = - atan2(Y_accel, -Z_accel) * 180 / PI * 1000000; //Positive pitch is pitching up
	float roll_udeg = atan2(X_accel, -Z_accel) * 180 / PI * 1000000; //Positive roll is to the right (when looking at vehicle from behind)

	accel_pitch_data_current.filtered_udeg = pitch_udeg;
	accel_roll_data_current.filtered_udeg = roll_udeg;

	//Serial.print(micros()); Serial.print(" ");
	//Serial.print(accel_roll_data_current.last_udeg / 1000); Serial.print(" ");
	//Serial.print(accel_roll_data_current.filtered_udeg / 1000); Serial.print(" ");
	//Serial.println(" ");


/*
	Serial.print(pitch_udeg / 1000); Serial.print(" ");
	Serial.print(roll_udeg / 1000); Serial.print(" ");

	Serial.println(" ");
*/

	//Second-order best fit and confidence (opposite of variance/squared error from best fit line)


	//Serial.println(pitch); //Serial.print(" "); Serial.println(roll);

	/*
	for(int i=0; i < 2; i++){
		Serial.print(accelFIFO_current.sample_us[i] % 100000);  Serial.print(" ");

	}
	Serial.println(" ");
	*/

}





/*

Will output result of integration of gyro rates (applying offset found during no movement and datasheet scaling factor).

Adding the the most recent found delta angles will be wrong unless:
	-Called with every single new delta angle (can't skip any or will be wrong) - assured by calling every time after new data
	-Can't add same one twice (function protects against this)


So that this value doesn't blow up, a separate function, GyroDriftCorrection, will be looking at how this data compares
with the accelerometer and periodlically applying a large jump to the integrator one way or another so that it always stays
somewhat close to the actual value.

A separate function adds the fine drift adjustment to this integrator value for the final angle value

*/
void MPU9250::GyroIntegrator(GYRO_DATA * gyro_data)
{

	if (gyro_data->current_timestamp != gyro_data->last_timestamp_integrated){

		gyro_data->gyro_integrator_udeg += gyro_data->last_delta_udeg;

		//Show that this was the most recent timestamp put into the integrator
		gyro_data->last_timestamp_integrated = gyro_data->current_timestamp;


	}


	//Serial.println(gyro_data->gyro_integrator_udeg);
	
}


void MPU9250::CompFilter()
{

	if (gyro_pitch_data_current.current_timestamp != gyro_pitch_data_current.last_timestamp_integrated){


		gyro_pitch_data_current.complementary_udeg = 0.98 * (gyro_pitch_data_current.complementary_udeg + (float)gyro_pitch_data_current.last_delta_udeg) + 0.02 * (double)accel_pitch_data_current.filtered_udeg;

		gyro_roll_data_current.complementary_udeg = 0.98 * (gyro_roll_data_current.complementary_udeg + (float)gyro_roll_data_current.last_delta_udeg) + 0.02 * (double)accel_roll_data_current.filtered_udeg;
		//Serial.print(gyro_pitch_data_current.complementary_pitch_udeg / 1000);  Serial.print("  ");

		int write_loc = location_current.most_recent_write + 1;

		//When get to the end of the FIFO and have to write to beginning again
		if (write_loc == LOCATION_FIFO_SIZE){
		  	write_loc = 0;
		 }


		location_current.pitch[write_loc] = (double)gyro_pitch_data_current.complementary_udeg / 1000000;
		location_current.roll[write_loc] = (double)gyro_roll_data_current.complementary_udeg / 1000000;

		location_current.timestamp[write_loc] = gyro_pitch_data_current.current_timestamp;
		int last_delta_t_us = location_current.timestamp[write_loc] - location_current.timestamp[location_current.most_recent_write];


		double pitch_delta_udeg = (location_current.pitch[write_loc] - location_current.pitch[location_current.most_recent_write]) * 1000000;
		double roll_delta_udeg = (location_current.roll[write_loc] - location_current.roll[location_current.most_recent_write]) * 1000000;

		location_current.pitch_rate[write_loc] = pitch_delta_udeg / (double)last_delta_t_us;
		location_current.roll_rate[write_loc] = roll_delta_udeg / (double)last_delta_t_us;


/*
		Serial.print("COMP"); Serial.print(" ");
		Serial.print(location_current.pitch_rate[write_loc]); Serial.print(" ");
		Serial.print(pitch_delta_deg); Serial.print(" ");
		Serial.print(location_current.last_delta_t[write_loc]); Serial.println(" ");
*/

		location_current.most_recent_write = write_loc;

	}
	
}




#include <cmath>


/*
(Must be called every time or else get step changes in angle - or need to output slope
to follow until next time)

*
Gyro Integrator and Drift Correction Function:

	Find next slot of the accel gyro error FIFO to write to

	Get the last FIFO error avg


	If writing to first slot of FIFO and FIFO error avg > 100,000 or < -100,000:
		[Coarse Drift Adjustment] is 100,000 or -100,000
		
		//Find the new moving average error between the [Gyro Integrator] angle and [Accel Moving Avg] angle
		Make new error FIFO sum -= 100,000 * FIFO size
		Subtract value in slot to be written to from FIFO sum because about to be overwritten
		Find new error between the new [Gyro Integrator Angle] and [Accel Moving Avg] angle, by calling Gyro Integrator
			to find [Gyro Integrator Angle], which would do [Gyro Integrator Angle] = 
			[Last Gyro Integrator Angle] + [Most recent Gyro DPS * delta_t]] + [Drift Adjustment]
			This error should be about 100,000 smaller than last time ("about" because slight change in error from last
			to this time)
		Write the new gyro accel error to first slot of the error FIFO
		Add the new gyro accel error to the FIFO sum
		Record which slot the new data was written to
		Record the time of that data
		Trivially find average of FIFO by dividing sum by number of slots

		Find [Fine Drift Adjustment] using method below (in everything normal)

		Record that a new epoch transition has started (indicates whether to subtract whole or part of values later)

	/////
	Rationale for LOCATION_CALC_DATA.epoch_transition:
	If writing to any other slot, how do you know whether the value you're taking out is from an old epoch or not?
		Only affects how much to subtract from the FIFO sum -
			If old, would read 103,000, but subtract 3,000
			If new (same epoch), would read 103,000 and subtract 103,000
		Not reliable to check if it's a high/low number - unlikely, but could be noise.
		Instead, boolean records that an epoch transition has started when write to 1st slot, ends after write to last slot
	//////


	If during epoch transition

		Subtract value in slot to be overwritten + [Drift Adjustment] (for example, subtract 103,000 - 100,000)

		//Rest is same as below - for everything normal

		If writing to last slot in FIFO, mark epoch transition as false because ended
		

	If not, everything normal:

		[Coarse Drift Adjustment] = 0

		//Find the new moving average error between the [Gyro Integrator] angle and [Accel Moving Avg] angle
		Subtract value in slot to be written to from FIFO sum because about to be overwritten
		Find new [Gyro Integrator Angle] with function, [Gyro Integrator Angle] = [Last Gyro Integrator Angle] + 
			[Most recent Gyro DPS * delta_t] + [Drift Adjustment = 0]
			[Last Gyro Integrator Angle] = [Gyro Integrator Angle]
		Find new error between [Gyro Integrator Angle] and [Accel Moving Avg] angle
		Write new gyro accel angle error to that slot
		Add the new gyro accel error to the FIFO sum
		Record which slot the new data was written to
		Record the time of that data
		Trivially find average of FIFO by dividing sum by number of slots

		Find difference between last drift value guessed by fancy following algorithm and current average drift in the FIFO
		Find the freakout acceleration correction (if plunging up or down to line, try to stop before getting there)
			Freakout magnitude is normally 0 (does nothing)
			If sloping down and towards or sloping up and towards the moving drift average, apply freakout
			Freakout magnitude = - Slope magnitude * [function of how far absolute from the moving drift average
				weighted much more as get close, but not to infinity]

		Find how fast last fancy algorithm following the moving average should accelerate towards the current average
			Acceleration towards average = How far from average + Freakout magnitude
		Apply new acceleration over the last timestep to find new velocity towards the moving average
		Apply new velocity towards the moving average over last timestep to find new position = [Fine Drift Adjustment]
			
	
	[Angle] = [Gyro Integrator Angle] + [Accel Gyro Drift Magnitude]
		
		Note:
			Important that [Gyro Integrator Angle] and [Angle] are separate.  [Gyro Integrator Angle] should only integrate
			the gyro measurements over time, with occasional 100,000 or -100,000 changes.
			[Angle] takes this and applies the most recent fine tuning correction to make sure the final angle is smooth
			and can be differentiated with accurate results.

		(incremented or decremented error value.  If normally would output 103,000, output 3,000 - so gyro integrator
		does angle = [last gyro angle] + [100000] + [3,000])

	
	In summary, this function adds a jump correction to the gyro integrator if needed [Coarse Drift Adjustment]
	and finds the fine drift correction to apply to the gyro integrator

	To do this, applies all necessary corrections to reflect the change in the gyro accel error FIFO (FIFO avg sawtooth vs time)
	Finds the fine correction to apply to the gyro integrator [Fine Drift Adjustment] by using a very fancy filter that
	tries to match the error FIFO avg, but smoothly

*/

double MPU9250::GyroIntegrationDriftCorrection(GYRO_ACCEL_ERROR_DATA * gyro_accel_error_data, GYRO_DATA * gyro_data, ACCEL_DATA * accel_data)
{

	//Error in degrees between the most recent gyro integration angle value and accel angle value - no filtering anywhere
	//Note: using last angle data for accelerometer data is extremely noisy - doing trig on two super noisy sources
	double instantaneous_gyro_drift = double(accel_data->filtered_udeg - gyro_data->gyro_integrator_udeg)/1000000;

	//Find next slot of the accel gyro error FIFO to write to
	int write_loc = gyro_accel_error_data->most_recent_write_loc + 1;
	if (write_loc == GYRO_ACCEL_ERROR_FIFO_SIZE){
		write_loc = 0; 
	}

	//Take the value about to be overwritten out of the value to add to FIFO summation
	double add_to_FIFO_summation = -gyro_accel_error_data->gyro_accel_error_FIFO[write_loc];

	//Write the new instantaneous error to the FIFO
	gyro_accel_error_data->gyro_accel_error_FIFO[write_loc] = instantaneous_gyro_drift;

	//Mark that that spot in the FIFO was just written to
	gyro_accel_error_data->most_recent_write_loc = write_loc;

	//Add the value just put into the FIFO to the summation
	add_to_FIFO_summation += instantaneous_gyro_drift;

	//Now actually add the value
	gyro_accel_error_data->gyro_accel_error_FIFO_summation += add_to_FIFO_summation;

	//Find the FIFO average from the summation
	gyro_accel_error_data->gyro_accel_error_FIFO_avg = gyro_accel_error_data->gyro_accel_error_FIFO_summation / GYRO_ACCEL_ERROR_FIFO_SIZE;



	//Do very fancy filtration to make a value follow the average error smoothly
	//This value is how much the fancy filter value is off from the current moving average error
	double filter_error = gyro_accel_error_data->filtered_gyro_accel_error_value - (double)gyro_accel_error_data->gyro_accel_error_FIFO_avg;

	double freakout_correction = 0;


	float freakout_correction_ok = gyro_accel_error_data->filtered_gyro_accel_error_slope * filter_error;

	if (freakout_correction_ok < 0){
		freakout_correction = -gyro_accel_error_data->filtered_gyro_accel_error_slope * 50000000/(abs(filter_error) + 1000000);
		
	}

	double filter_acc = - 80 * filter_error + freakout_correction;
	gyro_accel_error_data->filtered_gyro_accel_error_slope += filter_acc * (double)gyro_data->last_delta_t / 1000000;
	gyro_accel_error_data->filtered_gyro_accel_error_value += gyro_accel_error_data->filtered_gyro_accel_error_slope * (double)gyro_data->last_delta_t / 1000000;

	/*
	Serial.print(gyro_data->gyro_integrator_udeg / 1000);  Serial.print(" ");
	Serial.print(accel_data->last_udeg / 1000);  Serial.print(" ");
	Serial.print(instantaneous_gyro_drift * 1000);  Serial.print(" ");
	Serial.print(gyro_accel_error_data->gyro_accel_error_FIFO_avg * 1000);  Serial.print(" ");
	Serial.print(gyro_accel_error_data->filtered_gyro_accel_error_value * 1000);  Serial.print(" ");

	Serial.print(freakout_correction);  Serial.print(" ");


	Serial.println(" ");
	
	*/


	return gyro_accel_error_data->filtered_gyro_accel_error_value;


}






/*



*/
/*
void MPU9250::calcCommands(LOCATION_FIFO location, float * motor_commands, float thrust)
{

	double pitch = location_current.pitch[location_current.most_recent_write];
	double roll = location_current.roll[location_current.most_recent_write];

	double pitch_rate = location_current.pitch_rate[location_current.most_recent_write]; //Pitching up is positive
	double roll_rate = location_current.roll_rate[location_current.most_recent_write]; //Rolling right is positive



	double pitch_error_int = 0;
	double roll_error_int = 0;

	for (int i = 0; i < 199; i++)
	{
		pitch_error_int += attitude_FIFO[4 * i];
		roll_error_int += attitude_FIFO[4 * i + 1];
	}

	double pitch_error_avg = pitch_error_int / 199;
	double roll_error_avg = roll_error_int / 199;

	//Serial.print(pitch_rate);  Serial.print("  ");  Serial.println(roll_rate);  //Serial.print("  ");  Serial.print(micros());
 
 	//Serial.print(thrust);  Serial.print("  ");
	//Serial.print(roll_rate);  Serial.print("  ");
	//Serial.print(pitch_rate);  Serial.println("  ");

	float pitch_p_gain = 0.01;
	float roll_p_gain = 0.01;

	float pitch_i_gain = 0.08;
	float roll_i_gain = 0.08;

	float pitch_d_gain = 0.001;
	float roll_d_gain = 0.001;

	float pitch_pid_output = pitch_p_gain * pitch + pitch_d_gain * pitch_rate;
	float roll_pid_output = roll_p_gain * roll + roll_d_gain * roll_rate;

	float freakout_correction = 0;

	float freakout_correction_ok = roll_rate * roll;

	if(freakout_correction_ok < 0){
		freakout_correction = -roll_rate * 1/(abs(pitch * 100) + 10);
	}

	float front_right = thrust - pitch_pid_output + roll_pid_output; //Front right
	float front_left = thrust - pitch_pid_output - roll_pid_output; //Front left
	float back_right = thrust + pitch_pid_output + roll_pid_output; //Back right
	float back_left = thrust + pitch_pid_output - roll_pid_output; //Back left

	front_right = constrain(front_right, 0 , 1);
	front_left = constrain(front_left, 0 , 1);
	back_right = constrain(back_right, 0 , 1);
	back_left = constrain(back_left, 0 , 1);

	motor_commands[0] = front_right;
	motor_commands[1] = front_left;
	motor_commands[2] = back_right;
	motor_commands[3] = back_left;

	if (thrust < 0.1){
		motor_commands[0] = 0;
		motor_commands[1] = 0;
		motor_commands[2] = 0;
		motor_commands[3] = 0;
	}


	if (readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250) != 113){
		motor_commands[0] = 0;
		motor_commands[1] = 0;
		motor_commands[2] = 0;
		motor_commands[3] = 0;
	}



	Serial.print(micros()); Serial.print("  ");
	//Serial.print(pitch * 1000); Serial.print("  ");
	//Serial.print(roll * 1000); Serial.print("  ");
	Serial.print(pitch); Serial.print("  ");
	Serial.print(roll); Serial.print("  ");

	//Serial.print(roll_p_gain * roll); Serial.print("  ");
	//Serial.print(roll_d_gain * roll_rate); Serial.print("  ");
	//Serial.print(pitch_p_gain * pitch); Serial.print("  ");
	//Serial.print(pitch_d_gain * pitch_rate); Serial.print("  ");
	Serial.print(pitch_rate); Serial.print("  ");
	Serial.print(roll_rate); Serial.print("  ");
	//Serial.print(gyroFIFO_current.x_axis[gyroFIFO_current.most_recent_write]); Serial.print("  ");
	//Serial.print(accelFIFO_current.x_axis[accelFIFO_current.most_recent_write]); Serial.print("  ");
	double roll_accel = (double)accel_roll_data_current.filtered_udeg / 1000000;
	Serial.print(roll_accel); Serial.print("  ");

	//double roll_integ = (double)gyro_roll_data_current.gyro_integrator_udeg / 1000000;
	//Serial.print(roll_integ); Serial.print("  ");

	//Serial.print("MPU9250: ");  Serial.print(readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250)); Serial.print("  ");
	//int SDA = digitalRead(20);
	//int SCL = digitalRead(21);

	//Serial.print(SDA); Serial.print("  "); //SDA pin
	//Serial.print(SCL); Serial.print("  "); //SCL pin

	

	for (int i = 0; i < 4; i++){
		Serial.print(motor_commands[i]); Serial.print("  ");
	}


	Serial.println("  ");




}
*/


/*
Problem with magnetometer for attitude is hysteresis and different with each startup.  Need good way to calibrate and adjust
for errors.  This may only happen in Kalman or other filter.
*/
void MPU9250::getMagEulers(int16_t * mag_registers, double * destination)
{

	double X_LSB_offset = -48; //Goes from -200 to 295 -> -48 and 0.191
	double X_scaling = 0.191; 

	double Y_LSB_offset = -65; //Goes from -190 to 320 -> -65 and 0.185
	double Y_scaling = 0.185; 

	double Z_LSB_offset = -140; //Goes from -140 to 420 -> -140 and 0.169
	double Z_scaling = 0.169;  

	double X_microteslas = ((double)mag_registers[0] + X_LSB_offset) * X_scaling;
	double Y_microteslas = ((double)mag_registers[1] + Y_LSB_offset) * Y_scaling;
	double Z_microteslas = ((double)mag_registers[2] + Z_LSB_offset) * Z_scaling;


	double yaw_rad = atan2(-X_microteslas, Y_microteslas); //Points to magnetic north now, positive is going clockwise
	
	/*How much pitched from magnetic plane inclined at 60deg - would read 0 everywhere when no pitch off of plane
	When pitched up 30 deg (90 - 60) and facing mag north, will read 0.  Will read 0 when pitched down 30 deg facing mag south
	When level and facing north, pitched 30 deg down from plane, so will read -30
	!!Without a good calibration technique, tends to be off.  -17 (13 deg error) at due north to 35 (5 deg error) at due south
	Next time on: -18, 35.  Next time -19, 33.  Then -10, 20.  Need quick way to adjust to 0 everywhere.
	*/
	double pitch_from_mag_plane = - atan2(Y_microteslas, -Z_microteslas) * 180 / PI;

	//This is a hack to avoid real calibration - basically want to multiply by 30/17 = 1.76 at due north, 0.85 at due south
	//Find multipliers by whatever actual values at north and south are (max_north, max_south)
	double max_north = -9; //Must be added at north
	double max_south = 31; //Must be added at south
	double north_scaler = -(90 - 59.768) / max_north;
	double south_scaler = (90 - 59.768) / max_south;
	double hack_scaler = (north_scaler - south_scaler)/2 * cos(yaw_rad) + north_scaler - (north_scaler - south_scaler)/2;

	//Correcting pitch offset above - adds (90 - 59.7) when facing due north, subtracts (90 - 59.7) when facing due south
	//So when due north pitched up 30 deg, you get 0 from mag plane + 30 = 30 deg, which is correct
	double yaw_corrected_pitch_from_mag_plane = pitch_from_mag_plane + (90 - 59.768) * cos(yaw_rad);

	/*This makes a slight correction to pitch scaling.  Above already accurate if pointing due mag north or south,
	but if partly west/east, then pitching down won't be all down - will be slightly horizontal because pitching "down"
	from magnetic plane.  At due west/east, an e.g. 45 deg pitch "down" from mag plane will only be about half that 
	because mag plane tilted at almost 60 deg.  So should multiply pitch by about half at due east/west and just * 1
	at due north/south.  This is confusing because mag plane happens to be at 60 deg, where should multiply by half (sin or cos?)
	If mag plane happened to be tilted even more at 80 deg, should go from 1 to cos(80) [= 0.17]
	To be clear, mag plane in Mojave is tilted up 30 deg (bc 60 down), so would go from 1 to cos(30) = 0.86, so small correction
	So, amplitude = [1 - cos(80)] / 2.  Should be 1 at due north = 0, so cos.  Shrinking amplitude means have to add to get
	to 1 at 0 = north.  Add {1 - [1 - cos(80)] / 2} = {0.5 + cos(80)/2}. (but in radians)
	So, scaler =  {[1 - cos(80)] / 2} * cos(yaw) + {1 - [1 - cos(80)] / 2}
	Double checked in Desmos - goes from 1 at 0, down to 0.17 (if using hypothetical, not real in Mojave, 80 deg mag plane)
	at due east/west
	Confirmed in Serial Monitor that this goes from 1 at north/south to 0.87 at east/west
	*/
	double pitch_scaler = ((1 - cos((90 - 59.768) * PI / 180)) / 2) * cos(2 * yaw_rad) + (0.5 + (cos((90 - 59.768) * PI / 180)) / 2);
	double pitch = yaw_corrected_pitch_from_mag_plane * pitch_scaler;

	destination[0] = pitch;


	destination[3] = (double)mag_registers[3];

/*
	Serial.print(X_microteslas);  Serial.print("  ");
	Serial.print(Y_microteslas);  Serial.print("  ");
	Serial.print(Z_microteslas);  Serial.print("  ");
	Serial.print(micros());  Serial.println("  ");
*/

}





/*
FIFO should store the last X number of samples from the magnetometer.  When full (all the time except very start),
should get rid of oldest value and put in newest value.

Should be called every time after readMags to put latest value from readMags into the FIFO 
(or at least called before FIFO used by something - maybe this and readMags should be sequenced in a separate function, maybe 
with other sensors or something)

Needed for filtering and optimization of calibration.

Inputs:
	-Most recent magnetometer sample where data was ready (as a magDataStruct)
	-Destination to put updated magnetometer FIFO

Called:
	Should only be called when there's new data ready from the magnetometer

Outputs:
	-Array of magDataStructs.  Location of most recent one will keep changing every time
	-Location of most recently updated measurement (26 would mean it's at magFIFO[26])

The way this should work:
	-Define array, MagFIFO[100], that can hold a bunch (100?) structs (containing 3 mag axes, sensor ready/overflow, sample time)
	-The first time it's called it should store the mag_sample in magFIFO[0] and return count as new location
	-count++
	-Next time, it should store in count++ % 100 (or however large)
	-Continue storing in next location (will wrap around to 0 after filled up)

*/
void MPU9250::writeMagFIFO(magDataStruct mag_sample, int16_t * FIFO_destination, int16_t * new_location)
{




}











/*
	DANGER: Probably shouldn't use this for flight.  Has a tendency to spike up, and peaks of spikes are recorded.
	Probably better to use known good data

!!!
	Basically, if assume that linearity of magnetometers is good, all you need for good magnetometer data (minus noise)
		is some way to get good, current values for the offset and scaling factor for 3 axes.

	Known that they must norm to 47.366 uT, so can get a ton of data and do gradient descent to see what offsets and scaling
		make the best fit

	Possible that good fits for one regime aren't for another (e.g. when upside down), so should narrow to smallest possible
		regime that expect to see (e.g. 45 degree bank all around, pointing upwards)

	May be possible to do gradient descent with some initial data moving it around, and then may be possible to update
		points in flight for best fit



	The whole point of this is that when you move a certain axis all around, the max it should ever experience is 47.336 uT
		and the min it should ever experience is -47.336 uT

		It might read -30 to 40 though, so (just to make math easy, assume 47.5 uT) it should be scaled by (95 / 70) 
		to get -40.7 to 54.3, and then it should be shifted down by 6.8 to get -47.5 to 47.5

		Each axis should do this, being passed through the magnetic field vector in opposite directions

	NB: See reference to magnetic field intensity at earth's surface above
		In Mojave:	47.336 uT, 12.108 deg east of line to north pole, 59.768 degrees into ground (south pole attracted down)

	The way this should work:
		- Pass all 3 axes through the magnetic field vector in both orientations
		- For each axis:
			- Find maximum positive and negative value (those equal +/- 47.336 uT)
			- Get the delta between most positive and most negative (= 47.336 * 2 = 94.672 uT)
			- Output the scaling factor needed to shift delta value in LSB to 94.672 uT
			- Find mid-point in LSB between max pos and neg, which would be where magnetic field = 0 uT
			- Output the shift in LSB needed

	NB: Technically everything should still work fine if value isn't scaled, or scaled wrong (e.g. if reads -93 to 93 uT)
		but probably better to use uT in case want to estimate mag field due to electric current, etc.

		Also, this means the ASA values and formula found in old library for initAK8943() aren't needed at all
			The 0.15 uT/LSB also isn't needed - might be useful to sanity check though (should read 315,316 LSB max for 47.336 uT)


	For X axis, got -310 to 168, -306 to 160, -304 to 173 -> mean: -71
	For Y axis, got -88 to 395, -109 to 393, -86 to 381 -> mean: 153
	For Z axis, got -161 to 444, -159 to 464,  --has a tendency to really spike up -> mean: 145

	The shift_dest values should be in LSB -> so when values outputted here are used:
		-Shift the LSB the right amount, then scale

*/

void MPU9250::getMagCalibrationVals(double * shift_dest, double * scale_dest)
{

	uint16_t ii = 0;
	uint16_t sample_count = 145;

	int16_t max_pos_mag[3] = {-32768, -32768, -32768}; //Start out max positive values as low as possible
	int16_t max_neg_mag[3] = {32767, 32767, 32767}; ////Start out max negative values as high as possible
	int16_t temporary_mag_data[4] = {0, 0, 0, 0x03}; //Start the error flag out as something other than one of the options


	for (ii = 0; ii < sample_count; ii++)
	{
		//Take a sensor reading
		//readMagRegisters(temporary_mag_data);
		delay(11); //Probably could go less than this, but to make it a minimum of 11 ms


		//If the new sensor reading is good (no error flag), check if there were any max positive or negative values
		if(! temporary_mag_data[3])
		{
			//For each of the three axes 
			for (int jj = 0; jj < 3; jj++)
			{
				//If the new value is most positive yet, record it to max_pos_mag for that axis.  Same for negative.
				if (temporary_mag_data[jj] > max_pos_mag[jj]){ 
	      			max_pos_mag[jj] = temporary_mag_data[jj];
	      		}

	      		if (temporary_mag_data[jj] < max_neg_mag[jj]){
	        		max_neg_mag[jj] = temporary_mag_data[jj];
	      		}
			}
		}

		//Trying to read mag registers, data wasn't ready or there was an overflow
		else{
			Serial.println("error flag:");
			Serial.println(temporary_mag_data[3]);
		}
/*
		Serial.print(temporary_mag_data[0]); Serial.print(" ");
		Serial.print(max_pos_mag[0]);  Serial.print(" ");
		Serial.print(max_neg_mag[0]); Serial.print("         ");

		Serial.print(temporary_mag_data[1]); Serial.print(" ");
		Serial.print(max_pos_mag[1]);  Serial.print(" ");
		Serial.print(max_neg_mag[1]); Serial.print("      "); 

		Serial.print(temporary_mag_data[2]); Serial.print(" ");
		Serial.print(max_pos_mag[2]);  Serial.print(" ");
		Serial.println(max_neg_mag[2]);
		*/

		
	}


	//Find LSB offsets and scaling factors
	for (int jj = 0; jj < 3; jj++){

		double range = max_pos_mag[jj] - max_neg_mag[jj]; //Magnitude of range of possible values in LSB

		scale_dest[jj] = ((double)MAG_FIELD_STRENGTH * 2) / range;

		double midpoint = max_neg_mag[jj] + range/2; //midpoint should be float in case range is odd

		shift_dest[jj] = midpoint * -1; //If the midpoint is 10, you should add -10 to each value

	}

}








// Function which accumulates gyro and accelerometer data after device
// initialization. It calculates the average of the at-rest readings and then
// loads the resulting offsets into accelerometer and gyro bias registers.
void MPU9250::calibrateMPU9250(float * gyroBias, float * accelBias)
{  
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
  // reset device
  // Write a one to bit 7 reset bit; toggle reset device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);
  delay(100);
   
 // get stable time source; Auto select clock source to be PLL gyroscope
 // reference if ready else use the internal oscillator, bits 2:0 = 001
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  
  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
  delay(200);                                    

  // Configure device for bias calculation
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);
  
// Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO  
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
  
  for (ii = 0; ii < packet_count; ii++)
  {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  );  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  );
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  );
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  );
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  );
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
  }
  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
   
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
// Push gyro biases to hardware registers
  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
  
// Output scaled gyro biases for display in the main program
  gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
  gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }
  
  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
  
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
 
// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

// Output scaled accelerometer biases for display in the main program
   accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
   accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;
}





// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250::MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
  float factoryTrim[6];
  uint8_t FS = 0;
   
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 1<<FS);  // Set full scale range for the gyro to 250 dps
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
  
    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }
  
// Configure the accelerometer for self-test
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  delay(25);  // Delay a while to let the device stabilize

  for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
  
    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }   
  
  // Configure the gyro and accelerometer for normal operation
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);  
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);  
  delay(25);  // Delay a while to let the device stabilize
   
  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
  selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
  selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
  selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
  selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
  selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
  factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
  factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
  factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
  factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
  factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
  factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation
 
 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get percent, must multiply by 100
  for (int i = 0; i < 3; i++) {
    destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i];   // Report percent differences
    destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // Report percent differences
  }
}



//Sometimes there's some problem with the I2C bus and the Arduino has to be unplugged and plugged in to work
uint8_t MPU9250::readByte(uint8_t slave_address, uint8_t register_address)
{

  uint8_t byte; //Initialize variable to write to
	
  Wire1.beginTransmission(slave_address);         // Begin transmission to the slave address
  //If MPU9250's AD0 pin is pulled to GND, it will have the address 0x68.  If pulled high, it will be 0x69
  
  Wire1.write(register_address);        //If write() called between beginTransmission and endTransmission, it will queue 
  //that byte(s) for transmission to a slave (so looks like it's basically a request for the slave to get that data ready)
  
  Wire1.endTransmission(false);      // Sends the byte(s).  False makes it send a restart message
  //on the bus to prevent the bus from being released, so ARM stays the master
  
  Wire1.requestFrom(slave_address, (uint8_t)1);  //Master uses this to request a certain number of bytes
  //(2nd argument) from a slave (1st argument)
  
  byte = Wire1.read();          // Reads a byte transmitted from a slave after a requestFrom()

  return byte;

}


//Same thing for reading multiple bytes to a certain destination
void MPU9250::readBytes(uint8_t slave_address, uint8_t register_address, uint8_t num_bytes_to_read,
                        uint8_t * destination)
{  
  Wire1.beginTransmission(slave_address);   // Initialize the Tx buffer
  Wire1.write(register_address);            // Put slave register address in Tx buffer
  Wire1.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire1.requestFrom(slave_address, num_bytes_to_read);  // Read bytes from slave register address 
  while (Wire1.available()) { 		 //Wire.available() returns the number of bytes available for retreival with read()
    destination[i++] = Wire1.read(); }         // Put read results in the Rx buffer
}


//Write a byte
void MPU9250::writeByte(uint8_t slave_address, uint8_t register_address, uint8_t data)
{
  Wire1.beginTransmission(slave_address);  // Initialize the Tx buffer
  Wire1.write(register_address);           // Put slave register address in the Tx buffer
  Wire1.write(data);                 // Put data in the Tx buffer
  Wire1.endTransmission();           // Send the Tx buffer
}


/*
Print the bits in a certain register to the serial port.  Just for seeing what's going on - not for flight.
This will print them out most significant to least significant, so they go in the same direction 
as on the register datasheet:
https://www.invensense.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf

Just how it says Bit7, etc. on the datasheet, it will print:
[Bit7][Bit6][Bit5][Bit4][Bit3][Bit2][Bit1][Bit0]

*/
void MPU9250::readBits(uint8_t slave_address, uint8_t register_address)
{
  uint8_t byte; //Initialize variable to write to
  Wire1.beginTransmission(slave_address);         // Begin transmission to the slave address
  Wire1.write(register_address);        //Put register address in transmit buffer
  Wire1.endTransmission(false);      // Send and keep connection alive.
  Wire1.requestFrom(slave_address, (uint8_t)1);  //Read byte gotten vack
  byte = Wire1.read();          // Reads a byte transmitted from a slave after a requestFrom()


  for (int i = 7; i >= 0; i--) {
	    Serial.print(bitRead(byte,i)); Serial.print("");
  }

  Serial.println("");

}


void MPU9250::readRegisters(int num_registers) //Just for testing - looking at bits in a bunch of registers at a time
{

	//uint8_t unsigned_raw_data[6];

    uint8_t rawData[num_registers];  // x/y/z accel register data stored here
    readBytes(MPU9250_ADDRESS, 0x7A, num_registers, &rawData[0]);  // Read the six raw data registers into data array
    //unsigned_raw_data[0] = (uint8_t)rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
    //unsigned_raw_data[1] = (uint8_t)rawData[1];


    for (int ii = 0; ii < num_registers; ii++){


    	for (int jj = 7; jj >= 0; jj--) {
		    Serial.print(bitRead(rawData[ii],jj)); Serial.print("");
	  	}
	  
	  Serial.print("   ");

    }

  	Serial.println("");

}


