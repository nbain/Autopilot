#ifndef AUTOPILOT2_MS4525_H
#define AUTOPILOT2_MS4525_H

#include <Arduino.h> //Used here so compiler accepts uint16_t, etc. data types - not sure why int, etc. would work but not uint16_t, etc.
#include "Wire.h"
#include "math.h" //For sqrt() for pressure to velocity conversion - might not want this conversion here though, maybe just for checking/debugging

//PX4AIRSPEEDV1.2 module with white sensor is 001D, the 1 psi differential model:
#define Q1 15
#define Q2 17
#define Q3 7
#define Q4 5
#define Q5 7
#define Q6 21


class MS4525
{


	

	public:

		void readPressure();


};


#endif