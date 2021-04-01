#ifndef AUTOPILOT2_DIRECTOR
#define AUTOPILOT2_DIRECTOR

#include "Motors.h"
#include "Receiver.h"
#include "Location.h"
#include "Control.h"

#include "MS5525.h"
#include "MPU9250.h"
#include "LidarLite.h"

//Tasks
#define RECV 0		//Get receiver intent
#define LOC 1		//Run location code
#define CONT 2		//Run control code
#define MOT 3		//Send new motor signals

class Director
{

	public:

		int us;
		int Task;
		int timeBefore;
		int mostTimeElapsed;
		bool critical_task; //Whether or not a critical task (receiver, location, etc) is being done this loop
 
		int log_num; //There are a number of different things to log, should cycle through each using log_num

		/*
		Each function the director calls should have this struct.
		*/
		struct DIRECTOR_CALL
		{

			int task_id; //Task identifier

			int max_us; //Max microseconds between calls
			int min_us; //Min microseconds between calls
			int last_call_time; //Microsecond value of last call time

			int est_time; //Estimated time to complete in microseconds - probably not wanted at first

		};


		struct DIRECTOR_CALL Tasks[5];




	public:

		void initTasks();
		void callTask();


	private:

		Motors _Motors;
		Receiver _Receiver;
		Location _Location;
		Control _Control;

		MS5525 _MS5525;
		MPU9250 _MPU9250;
		LidarLite _LidarLite;


};

#endif