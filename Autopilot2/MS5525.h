#ifndef AUTOPILOT2_MS5525_H
#define AUTOPILOT2_MS5525_H

#include <Arduino.h> //Used here so compiler accepts uint16_t, etc. data types - not sure why int, etc. would work but not uint16_t, etc.
#include "Wire.h"
#include "math.h" //For sqrt() for pressure to velocity conversion - might not want this conversion here though, maybe just for checking/debugging

//For 001D, the 1 psi differential model (mRo sensor):
#define Q1 15
#define Q2 17
#define Q3 7
#define Q4 5
#define Q5 7
#define Q6 21


class MS5525
{


	public:

		void readPressure();
		void init(uint8_t slave_address);
		void startADC(uint8_t slave_address, uint8_t register_address);
		uint32_t readADC(uint8_t slave_address);
		uint16_t read16bits(uint8_t slave_address, uint8_t register_address);
		void Log();


};


#endif