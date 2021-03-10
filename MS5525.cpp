#include "MS5525.h"

/*
The way the MS5525 works is a command is sent on I2C to tell the ADC to sample a piezoelectric sensor/thermocouple
at some oversampling ratio (256x - 4096x oversampling).  More oversampling takes longer (9ms vs. 2 ms or so), uses more power,
but has lower noise.

Then, when the ADC has finished sampling, you have to retrieve the data by sending an ADC read command over I2C.  ADC gives a 24 bit value.

The ADC 24 bit value has to be scaled and temp compensated.  These are from values found during calibration at the factory - calibration values
are 16 bits and stored in the PROM.  On startup, do a reset command (not sure why needed), then read the PROM values.

The other values for scaling, etc. are specific to the MS5525 chip type.  mRo sensor is 1 psi, differential pressure.  These values are given
in the data sheet as Q1-Q6.

Specs:
	1.8-3.6V supply
	9.04 ms max delay for 4096x oversampling, 0.6 ms delay for 256x

Some of this code copied from/ideas from PX4's MS5525.cpp on Github:
https://github.com/PX4/PX4-Autopilot/blob/master/src/drivers/differential_pressure/ms5525/MS5525.cpp

Datasheet:
https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5525DSO%7FD2%7Fpdf%7FEnglish%7FENG_DS_MS5525DSO_D2.pdf%7FCAT-BLPS0003
(mRo sensor is the 1 psi, differential pressure version)

Experiments:
-Naturally reading ~30 (1/10,000 of psi) at ~70 F
-Taking outside to 27 F -> reads ~60
-Eliminating temp compensation by setting to values if at 20 C -> taking outside reads ~200
-Breathing on sensor (not pitot tube) raises to ~700
-Putting in front of heater with pitot tube out of flow compensated to stay within 20-40 (if breath on it after in front of heater, no effect)


*/


uint16_t C1;
uint16_t C2;
uint16_t C3;
uint16_t C4;
uint16_t C5;
uint16_t C6;

int press_start = 0; //Pressure ADC start time
int temp_start = 0;  //Temperature ADC start time

uint32_t press_32;
uint32_t temp_32;

int64_t pressure;
float velocity_ms;


/*
Reset and read the PROM.
The PROM stores the coefficients of temp, etc. found at the factory for that specific chip.  If using the same chip, don't need to
do this every time theoretically, but good to do every time in case using different one and to not have to store coefficients elsewhere.

For non-broken mRo:
C1 - 14980
C2 - 8468
C3 - 3624
C4 - 1743
C5 - 34934
C6 - 8076

*/
void MS5525::init(uint8_t slave_address)
{

//If testing MS5525 alone, make sure to uncomment lines below to start the I2C bus
 /*
	int i2c_hz = 400000;
	Wire.begin();
	Wire.setClock(i2c_hz); //Default is 100kHz.  Error above 10 MHz.  Stops connecting (no error, but reads 00000000's) > 4.67 MHz
	
	Serial.print("I2C bus #2 started at Hz: ");
	Serial.println(i2c_hz);
	*/


	//Check if the MPU9250 is on the bus and exit if not
	Wire.beginTransmission(0x76);
    Wire.write(0xA2);
    //int status = Wire1.endTransmissionFast(false); //Custom function.  Edited Arduino library to add endTransmission with low timeout
    //Custom function got overwritten when updated library.
    int status = Wire.endTransmission(false);

/*
    while (status != 0){
    	//Wire1.recover(71, 70);

    	Wire1.beginTransmission(0x76);
	    Wire1.write(0xA2);
	    //status = Wire1.endTransmissionFast(false); //Custom function.  Edited Arduino library to add endTransmission with low timeout
	    status = 0;

    }
    */


  Wire.beginTransmission(0x76); 
  Wire.write(0x1E); //0x1E is the Reset command.  Datasheet says to always send before reading PROM.
  Wire.endTransmission(false); 

  delay(100);

  C1 = read16bits(0x76, 0xA2);
  C2 = read16bits(0x76, 0xA4);
  C3 = read16bits(0x76, 0xA6);
  C4 = read16bits(0x76, 0xA8);
  C5 = read16bits(0x76, 0xAA);
  C6 = read16bits(0x76, 0xAC);

  Serial.print("C1:  "); Serial.print(C1);
  Serial.print("C2:  "); Serial.print(C2);
  

}


/*
Function should do one of these, cyclically each time called (or nothing if called too soon):
	-Read ADC (temp - if late enough after start) and start pressure ADC (and log time)
	-Read ADC (pressure - if late enough after start) and start temp ADC (and log time)


dT = D2 - Tref = D2 - C5 * 2^Q5 = D2 - 4,471,552
Temp = 2000 + dT * C6 / 2^Q6 = 2000 + dT * 0.0038509

Offset = Tref1 Offset + Offset temp coeff. * dT = C2*2^Q2 + C4*dT/2^Q4 = 1,109,917,696 + 54.46875 * dT
Sensitivity = Tref1 Sensitivity + Sensitivity temp coeff. * dT = C1*2^Q1 + C3*dT/2^Q3 = 490,864,640 + 28.3125 * dT
Pressure = ((D1 * Sensitivity / 2^21) - Offset) / 2^15 = 

*/
void MS5525::readPressure()
{
	//Exit function if still reading the ADC
	if ((micros()-press_start) < 9000 || (micros()-temp_start) < 2500){
		return;
	}

	//Check if the MPU9250 is on the bus and exit if not
	Wire.beginTransmission(0x68); //NOTE: This is not the MS5525 address.  Reading address messes up the ADC read, so for now use this to test I2C
    Wire.write(0x75);
    //int status = Wire1.endTransmissionFast(false); //Custom function.  Edited Arduino library to add endTransmission with low timeout
    int status = Wire.endTransmission(false);

/*
    if (status != 0){
    	//Wire1.recover(71, 70);
    	return;
    }
    */
    

	//If last ADC accumulation was for pressure (last pressure start time > last temp start time), read the pressure from the ADC and convert to psi
	//Then start temperature accumulation.
	if (press_start > temp_start){

		press_32 = readADC(0x76); //Read 24 bit pressure value in ADC (in 32 bit uint) 

		//Difference between ref temp and current temp, but not scaled properly and not 
	    //adjusted for temp sensitivity
	    //For non-broken mRo, C5 * (1UL << Q5) = 4,471,552
	    int64_t dTemp = temp_32 - int64_t(C5) * (1UL << Q5); //(1UL << Q5) is a clever way of writing a power of 2
	    //because C++ doesn't have built in power function (so can't do 2**5)
	  
  	    int64_t temp = 2000 + (dTemp * int64_t(C6)) / (1UL << Q6); //Temp in celsius times 100

	    int64_t offset = int64_t(C2) * (1UL << Q2) + (int64_t(C4) * dTemp) / (1UL << Q4); //Pressure offset at current temp

	    int64_t sens = int64_t(C1) * (1UL << Q1) + (int64_t(C3) * dTemp) / (1UL << Q3); //Pressure sensitivity at current temp

	    pressure = (press_32 * sens / (1UL << 21) - offset) / (1UL << 15); //Pressure in psi times 10,000

  
  		startADC(0x76, 0x54); //Start accumulating temperature reading into ADC
  		temp_start = micros();


		float psi_to_pa = 6894.76; //PSI to Pascal conversion
		float experimental_offset = 27; //At zero input velocity, one particular sensor reads ~30 (1/10k psi) when at zero
		float air_density = 1.1; //Air density in kg/m^3
		float pressure_pa = ((float)pressure - experimental_offset)/10000* psi_to_pa; //Static/Total pressure differential
		
		velocity_ms = sqrt(abs(pressure_pa) / (0.5 * air_density));

/*

		Serial.print("Temp: ");
		Serial.print((float)temp);

		Serial.print("Press: ");
		Serial.print((float)pressure);

		Serial.print(" Pressure: ");
		Serial.print(pressure_pa);
		Serial.print("Vel: ");
		Serial.println(velocity_ms);
*/

  	}

  	//If last ADC accumulation for temperature, read the temperature and start a pressure accumulation
  	else{

  		temp_32 = readADC(0x76); //Read 24 bit temperature value in ADC (in 32 bit uint)

  		startADC(0x76, 0x48); //Start accumulating pressure reading into ADC
  		press_start = micros();


  	}


}



/*
Starts the conversion from the piezoelectric pressure or temp sensor to bits in the ADC
Send the command for the MS5525 to sample either temp or pressure at a certain oversampling ratio (OSR).
Have to wait a few milliseconds (depends on the OSR used) after this before reading ADC.

Sending register address:
	0x40 - 256 OSR Pressure (0.54 ms typical conversion time)
	0x42 - 512 OSR Pressure
	0x44 - 1024 OSR Pressure (2.08 ms typical conversion time)
	0x46 - 2048 OSR Pressure
	0x48 - 4096 OSR Pressure (8.22 ms typical conversion time)
	0x50 - 256 OSR Temperature (all same conversion times for temp)
	0x52 - 512 OSR Temperature
	0x54 - 1024 OSR Temperature
	0x56 - 2048 OSR Temperature
	0x58 - 4096 OSR Temperature


 */
void MS5525::startADC(uint8_t slave_address, uint8_t register_address)
{

  Wire.beginTransmission(slave_address); 
  Wire.write(register_address);
  Wire.endTransmission(false);  //Don't really need Fast, but good because can't seem to get MS5525 I2C health, so good if breaks

  
}


/*
Read 24 bit value (just stored in a uint32_t) gotten by ADC.  The chip always puts the ADC value at register 0x00.

Will read 0 if don't wait long enough for ADC to finish.

*/
uint32_t MS5525::readADC(uint8_t slave_address)
{

  Wire.beginTransmission(slave_address); 
  Wire.write(0x00);
  Wire.endTransmission(false);  //Don't really need Fast, but good because can't seem to get MS5525 I2C health, so good if breaks

  Wire.requestFrom(slave_address, (uint8_t)3);
  
  uint8_t byte1 = Wire.read();          // Reads a byte transmitted from a slave after a requestFrom()
  uint8_t byte2 = Wire.read();
  uint8_t byte3 = Wire.read();

  uint32_t bits = byte1 << 16 | byte2 << 8 | byte3;

  return bits;
  
}



/*
 Used to read the 16 bit data in the PROM of the MS5525 - these are set in the factory from
 calibration data from that device.

 The mRo sensor is a 001D model of the MS5525, so 1 psi, differential pressure
 (1 psi dynamic pressure at 1.2 kg/m^3 would be 240 mph)

 According to data sheet, differential pressure means:
 "Output is proportional to the difference between Port 1 and Port 2. Output
swings positive when Port 2> Port 1. Output is 50% of total counts when Port 1=Port 2."

 */
uint16_t MS5525::read16bits(uint8_t slave_address, uint8_t register_address)
{

  Wire.beginTransmission(slave_address); 
  Wire.write(register_address);
  Wire.endTransmission(false);  

  Wire.requestFrom(slave_address, (uint8_t)2);
  uint8_t byte1 = Wire.read();
  uint8_t byte2 = Wire.read();
  uint16_t bits = byte1 << 8 | byte2;

  return bits;
  
}




void MS5525::Log()
	{
		

		Serial.print(" Pres: ");
	  	Serial.print((int)pressure); Serial.print(" "); //In 1/10k of psi

	  	Serial.print("  Vel: ");
	  	Serial.print(velocity_ms); Serial.print(" ");



	}




