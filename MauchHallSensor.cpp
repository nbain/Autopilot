#include "MauchHallSensor.h"

/*

The Mauch Hall Sensor converts battery voltage and current to voltages.  Looks like battery voltage is scaled down by exactly 10x,
so if battery at 12.48V, it will read 1.248V.  Same for current.  40A becomes 40mV or 0.4V (not sure which bc oscilloscope scales
by 10x in funny way, but should be clear when testing.)  Tested this by running two front motors to full power (about 20A each and got 40mV with 10x
probe setting on oscilloscope.)

Meant to be used with Pixhawk - pinout for Power1 would be connected to Mauch Hall Sensor:
https://discuss.ardupilot.org/t/pixhawk-2-1-pinout-doc-need-to-be-on-the-pixhawk-2-1-page/23604

Allegro ACS758 Hall Sensor:
https://www.digikey.com/catalog/en/partgroup/acs758/10579


*/


void MauchHallSensor::readCurrent()
{

	//Read an ADC pin on Due

}


void MauchHallSensor::readVoltage()
{

	//Read another ADC pin on Due

}