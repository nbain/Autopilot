#ifndef AUTOPILOT2_LIDAR_H
#define AUTOPILOT2_LIDAR_H


#include <Arduino.h> //Used here so compiler accepts uint16_t, etc. data types - not sure why int, etc. would work but not uint16_t, etc.



class LidarLite
{


	public:

		void sendLaserPulses(bool bias_correction);
		int readMeasurement();
		void readDistance(); 
		bool checkMeasurement();

};



#endif