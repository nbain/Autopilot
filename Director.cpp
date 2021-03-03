/*
Flight Controller Director

This should be the only function in the control loop.  It should just call others as needed.

To work, must have:
	-List of tasks (functions)
		Each have:
		-Desired max time between calls (this could be dynamic in the future)
		-Estimated time to complete
		-Timestamp of last call

For example:
	sendCommands() (sends microsecond pulse to motors/servos), 2500us between, 50us to complete, 2,346,432 us at last call
	getReceiver() (get receiver PWM intent - actual pwms on interrupts), 20,000us between, 50us to complete, 2,342,424 us at last call


At first:
1. Should run through and find how overdue each is (negative if still ok)
2. Run most positive item


Note:
This function could get extremely sophisticated, acting like a real flight director and making logical/learned decisions

*/


#include "Arduino.h"
#include "Director.h"

#include <Wire.h>


void Director::initTasks()
{

	Serial.begin(115200);


	_Motors.init();

	_Receiver.init();

	_Location.init();

	_MS5525.init(0x76);

	//_MPU9250.init();


	//Fill in DIRECTOR_CALL structs
	Tasks[RECV].min_us = 40000;
	Tasks[RECV].last_call_time = 0;

	Tasks[LOC].min_us = 10000; //Testing the BNO055, still eventually lost the bus at ~4,000 us between calls (might still lose it above this).
	Tasks[LOC].last_call_time = 0;

	Tasks[CONT].min_us = 8000;
	Tasks[CONT].last_call_time = 0;

	Tasks[MOT].min_us = 8000;
	Tasks[MOT].last_call_time = 0;

}


void Director::callTask()
{

/*
	_Receiver.read_intent();

	_Location.estimate();

	_Control.run(&_Location.Location, &_Receiver.Receiver);

	_Motors.sendMotorSignals(&_Control.Commands);
*/

///*

	us = micros();
	critical_task = false; //Default to no critical task needing to be done - will change to true if any tasks ready
	mostTimeElapsed = 0; //Should start at zero and then, if tasks need to be done, get assigned increasingly negative values


	/*
	The way this should work is that each critical task should have a *minimum* (not maximum) time between calls - then any amount of time
	that passes after that minimum it's eligible to be called.  So each task will have a time before it's ready to be called (positive when still
	time left before being ready to be called again) and as soon as any number goes negative (or if there are multiple negatives), it should choose
	the most negative one (most time has passed since ready to be called again).

	If all the tasks have time left, do non-critical tasks.
	*/
	//Important thing to remember is that MOT, CONT, etc. are defined as numbers.
	for (int i=0; i<4; i++){

		timeBefore = Tasks[i].last_call_time + Tasks[i].min_us - us; //Get time before ready to call the task again.  Positive when still time left.

		//Serial.print(i); Serial.print(" "); Serial.print(timeBefore); Serial.print(" ");

		//Only choose as candidate for the next task to be done if enough time has elapsed before ready to call it again - only negative times
		if (timeBefore < 0){

			//If a new record most negative time, set Task to that task number
			if (timeBefore < mostTimeElapsed){

				mostTimeElapsed = timeBefore; //Make this the new record to beat
				Task = i; //Set the task to this
				critical_task = true;

			}

		}


	}


	//If no critical tasks need to be done, use this time to write to the log/record non-critical sensor data
	if (not critical_task){

			int log_start = micros();
			//int log_stop = log_start + leastTimeLeft; //Time by which logging must be finished -
			//from old way of setting max us between calls - probably good to do something similar so do multiple non-critical tasks in one loop


			if (log_num > 9){
				log_num = 0;
			}

			//Keep cycling through these - should all be pretty short so doesn't eat into time when critical function needs to run
			//Key thing is that the length of the longest running one in here will be the maximum amount a critical function can be late
			switch (log_num) {
				case 0:
					//_Motors.Log(); //Probably not usually needed
					log_num = log_num + 1;
			    	break;

		  		case 1:
					_Control.Log();
		  			//_Control.MotorTest(&_Receiver.Receiver);
					log_num = log_num + 1;
			    	break;

			   	case 2:
					_Receiver.Log();
					log_num = log_num + 1;
			    	break;

			    case 3:
					_Location.Log();
					log_num = log_num + 1;
			    	break;

			    case 4:
					//_MPU9250.readAccel1();
					log_num = log_num + 1;
			    	break;

			    case 5:
			    	//_MPU9250.readAccel2();
					log_num = log_num + 1;
			    	break;

			    case 6:
			    	//_MPU9250.readGyro1();
					log_num = log_num + 1;
			    	break;

			    case 7:
			    	_MS5525.readPressure();
			    	_MS5525.Log();
					log_num = log_num + 1;
			    	break;

			    case 8:
			    	//_LidarLite.readDistance();
					log_num = log_num + 1;
			    	break;

			    case 9:
					//Serial.print("Time: "); Serial.println(micros());
					log_num = log_num + 1;
			    	break;
				}


			
			
		}

	//If 
	else{


		switch (Task) {
	  		case RECV:
	  			Tasks[Task].last_call_time = micros();
				_Receiver.read_intent();
				//Serial.print("R");
				//Serial.println(Tasks[Task].last_call_time % 100000);
		    	break;

		   	case LOC:
				Tasks[Task].last_call_time = micros();
				_Location.estimate();
				//Serial.print("L");
				//Serial.println(Tasks[Task].last_call_time % 100000);
		    	break;

		    case CONT:
		    	Tasks[Task].last_call_time = micros();
				_Control.run(&_Location.Location, &_Receiver.Receiver);
				//Serial.print("C");
				//Serial.println(Tasks[Task].last_call_time % 100000);
		    	break;

		    case MOT:
		    	Tasks[Task].last_call_time = micros();
				_Motors.sendMotorSignals(&_Control.Commands);
				//Serial.print("M");
				//Serial.println(Tasks[Task].last_call_time % 100000);
		    	break;

		}
	}


}



