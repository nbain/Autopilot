#include "LidarLite.h"
#include "Wire.h"

/*

Key thing to remember with Lidar Lite is good to power with 5V UBEC (because 1.3W max) and 470uF cap,
but gnd must also be connected to I2C master ground (Arduino gnd).

Red - 5V with 470 uF cap
Black - 5V gnd + Arduino/I2C master gnd
Green - I2C SCL (opposite from usual color)
Blue - I2C SDA

Other are optional power enable pin and pwm trigger control

Lidar Lite V3:
https://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf

*/





/*
 Send laser pulse train.  Not super clear how receiver bias correction works, but adjusts for
 changes in ambient light.  Takes longer, so doing once every 100 times or so recommended.

 Experimentally, distance to target affects time before measurement ready (not sure why 
 because speed of light difference should be tiny).
 
 Note: even if the measurement isn't ready, the last one's still stored in the register
 (not erased at pulse start), so can still read with minimal phase lag if don't wait
 for measurement to be ready.

 */
void LidarLite::sendLaserPulses(bool bias_correction){

  Wire1.beginTransmission(0x62);
  Wire1.write(0x00); //0x00 is the ACQ_COMMAND register
  if(bias_correction){
      Wire1.write(0x04); //Writing 0x04 sends pulses with receiver bias correction
  }
  else{
      Wire1.write(0x03); //Writing 0x03 sends pulses without receiver bias correction
  }
  Wire1.endTransmission();
  
}


/*
  Read the two measurement bytes, concatenate, and return as an int.  The int value is the
  distance in cm.

 */
int LidarLite::readMeasurement(){

  Wire1.beginTransmission(0x62);         // Begin transmission to the slave address
  Wire1.write(0x8f);        //Put register address in transmit buffer
  Wire1.endTransmission();      // Send and keep connection alive.
  Wire1.requestFrom(0x62, (uint8_t)2);  //Read byte gotten vack
  
  uint8_t byte1 = Wire1.read();          // Reads a byte transmitted from a slave after a requestFrom()
  uint8_t byte2 = Wire1.read();          // Reads a byte transmitted from a slave after a requestFrom()

  int dist = (byte1 << 8) + byte2;

  return dist;
  
}

/*
Function to make things easy.  Every time called, will log new distance measurement to serial (if ready) and start a new pulse train.

*/
void LidarLite::readDistance(){

	//Check if the Lidar is on the bus and exit if not
	Wire1.beginTransmission(0x62);
    Wire1.write(0x01);
    int status = 0;// CHANGE BACK!!!  USed to be: int status = Wire1.endTransmissionFast(false);
    //Custom function.  Edited Arduino library to add endTransmission with low timeout.
    //When updated Arduino library, it probably overwrote it so it doesn't exist.

    if (status != 0){
    	//Wire1.recover(71, 70);
    	return;
    }

	int dist = readMeasurement();

	Serial.print("Lidar: ");
    Serial.print(dist);
    Serial.print(" ");

	sendLaserPulses(true);
  
}


/*
  Check whether a measurement is ready.  Return true if so.
  Measurement ready represented by the rightmost bit of 0x01 being 0.

  Note: In future, may be good to read other bits of 0x01 (overflow, sensor health stuff)
  in same I2C read and return relevant info.


 */
bool LidarLite::checkMeasurement(){

  Wire1.beginTransmission(0x62);         // Begin transmission to the slave address
  Wire1.write(0x01);        //Put register address in transmit buffer
  Wire1.endTransmission();      // Send and keep connection alive.
  Wire1.requestFrom(0x62, (uint8_t)1);  //Read byte gotten back

  //The 0x01
  if(bitRead(Wire1.read(),0)){
    return false;
  }
  else{
    return true;
  }

}