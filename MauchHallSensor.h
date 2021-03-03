#ifndef AUTOPILOT2_MAUCH_H
#define AUTOPILOT2_MAUCH_H

#include <Arduino.h> //Used here so compiler accepts uint16_t, etc. data types - not sure why int, etc. would work but not uint16_t, etc.



class MauchHallSensor
{


	public:

		void readCurrent();
		void readVoltage();


};


#endif