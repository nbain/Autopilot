//#include "main.h"


#include <iostream>
#include <chrono>
#include <cstdint>

#include <unistd.h>
#include <string.h> //For memset()



/*
Following guide here:
https://eigen.tuxfamily.org/dox/GettingStarted.html

Eigen folder in eigen-3.3.9 folder copied to usr/local/include using sudo cp -r

*/
#include <Eigen/LU>    // Calls inverse, determinant, LU decomp., etc.
//#include <Eigen/QR>

//#include "Director.h"
#include "Location.h"
#include "Receiver.h"
#include "XPlane.h"
#include "Control.h"


#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <chrono>
#include <iostream>
#include <fstream>




//Director _Director;

Control _Control;

Location _Location;

Receiver _Receiver;

//XPlane _XPlane;


/*

Compiled from /Autopilot-master with:
g++ -o main main.cpp Director.cpp Control.cpp Location.cpp Receiver.cpp

Measuring time:
https://stackoverflow.com/questions/3523442/difference-between-clock-realtime-and-clock-monotonic
	-> CLOCK_MONOTONIC should be used over CLOCK_REALTIME (can jump back and forth)

https://stackoverflow.com/questions/13263277/difference-between-stdsystem-clock-and-stdsteady-clock
	-> Use steady_clock rather than high_resolution_clock to use CLOCK_MONOTONIC instead of CLOCK_REALTIME 


*/





using namespace std::chrono;

int main()
{



	//_Director.initTasks();


	_Control.set_motor_and_servo_parameters();
	//std::ofstream LOG;
	//LOG.open("testlog.txt", std::ios::out);

	_Location.init();
	_Receiver.init();
	//_Control.setup_port();

	std::cout << "Starting loop" << std::endl;


	while(true)
	{
		usleep(4000);
		auto start = steady_clock::now();

        _Location.estimate(); 

		_Receiver.read_intent(); 

		_Control.run(&_Location.Current_Location, &_Receiver.Current_Receiver_Values);

		
		auto end = steady_clock::now();
		auto duration = duration_cast<microseconds>(end-start).count();
		std::cout << "\tLoop time (us): " << duration << std::endl;
		//_Control.run();

		//_XPlane.send_output_to_XPlane();




	}



}

