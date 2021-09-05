#include "XPlane.h"
/*
Notes on XPlane:

-Props appear to have an effect even when overriding engine forces (plane spinning with props on, relatively stable without)


*/


//Control Control_for_XPlane;

int XPlane::UDP_Setup_Send(){

	int option = 1;
	//struct sockaddr_in myaddr;
	memset(&myaddr_send, 0, sizeof(myaddr_send));
	myaddr_send.sin_family=AF_INET;
	myaddr_send.sin_addr.s_addr=inet_addr("192.168.1.25"); //IP Address of computer running XPlane
	myaddr_send.sin_port=htons(49009); //Port that plugin is listening on (49009 for modified XPlaneConnect)
	
	
	//int sock_send;
	sock_send =socket(AF_INET, SOCK_DGRAM, 0);
	setsockopt(sock_send, SOL_SOCKET, SO_REUSEADDR, (char*)&option, sizeof(option));

	int option2 = 1000;
	setsockopt(sock_send, SOL_SOCKET, SO_SNDBUF, (char*)&option2, sizeof(option2));

	int option3 = 1000;
	setsockopt(sock_send, SOL_SOCKET, SO_RCVBUF, (char*)&option3, sizeof(option3));

	bind(sock_send,( struct sockaddr *) &myaddr_send, sizeof(myaddr_send));


	
	return 0;

}





//Wait indefinitely until getting a UDP message, then fill UDP_Received with contents of message
int XPlane::UDP_Recv(char * UDP_Received){

	auto start = std::chrono::steady_clock::now();

    //UDP socket setup
    int sock = socket(AF_INET, SOCK_DGRAM, 0);

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(8888);


	int option2 = 1000;
	setsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char*)&option2, sizeof(option2));

	int option3 = 1000;
	setsockopt(sock, SOL_SOCKET, SO_RCVBUF, (char*)&option3, sizeof(option3));

    bind(sock, (struct sockaddr*) &addr, sizeof(addr));

    //Set timeout.  Not obvious if helps overall (haven't timed)
    struct timeval timeout;
  	timeout.tv_sec = 0;
	timeout.tv_usec = 1000;
	setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));


    while (1) {

    char msgbuf[256];
    socklen_t addrlen = sizeof(addr);
    int nbytes = recvfrom(sock,msgbuf,256,0,(struct sockaddr *) &addr, (socklen_t*)&addrlen);

    if (nbytes > 0) {
      memcpy(UDP_Received, &msgbuf, 256);
      close(sock);
      break;
  	  }

 	}

	auto end = std::chrono::steady_clock::now();
	auto looptime = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
	std::cout << "UDP_Recv() time: " << looptime << std::endl;

    return 0;

}






void XPlane::get_data_from_XPlane(){

        char XPlane_data_char[256];
        float XPlane_data_float[32];

        UDP_Recv(XPlane_data_char);



        //Parse character array received over UDP. and identify each value and convert into floats
        char * XPlane_data_ptr = strtok(XPlane_data_char, " ");

        //Start a count to use for the index in array
        int count = 0;

        //Canonical way to use strtok() is with a while loop, parsing off each substring until reaching end of the string
        while (XPlane_data_ptr != NULL)
		  {

		  	//Convert broken off section of the char array to a float and insert into float array
		    std::string tmp1 = XPlane_data_ptr;
		    float tmp2 = (float)atof(tmp1.c_str());
		    XPlane_data_float[count] = tmp2;

		    //Break off next section of char array
		    XPlane_data_ptr = strtok(NULL, " ");
		    count = count + 1;
		  }

		Last_XPlane_data = Current_XPlane_data;


		Current_XPlane_data.flight_time = XPlane_data_float[0]; //Flight time in seconds

		Current_XPlane_data.roll = XPlane_data_float[1]; //Roll angle in degrees
		Current_XPlane_data.pitch = XPlane_data_float[2]; //Pitch angle in degrees
		Current_XPlane_data.heading = XPlane_data_float[3]; //Heading angle in degrees relative to true north

		Current_XPlane_data.latitude = XPlane_data_float[4]; //Latitude in degrees
		Current_XPlane_data.longitude = XPlane_data_float[5]; //Longitude in degrees
		Current_XPlane_data.elevation = XPlane_data_float[6]; //Elevation in meters

		//Indicated airspeed dynamic pressure defined for an air density at sea-level, assuming 15C, 1013 hPa, 0% humidity -> 1.2247 kg/m^3
		Current_XPlane_data.airspeed_kias = XPlane_data_float[7]; //Airspeed in kias (takes into account density, wind)
		Current_XPlane_data.dist_agl = XPlane_data_float[8]; //Height AGL in meters
		Current_XPlane_data.static_pressure_inHg = XPlane_data_float[9]; //Static pressure at current flight altitude, weather condition in InHg

		Current_XPlane_data.OpenGL_acc_x = XPlane_data_float[10];	
		Current_XPlane_data.OpenGL_acc_y = XPlane_data_float[11];
		Current_XPlane_data.OpenGL_acc_z = XPlane_data_float[12];

		Current_XPlane_data.OpenGL_roll = XPlane_data_float[13];	
		Current_XPlane_data.OpenGL_pitch = XPlane_data_float[14];	
		Current_XPlane_data.OpenGL_hdg = XPlane_data_float[15];			

		//Not clear if these are useful values - appear to be wrong in unpredictable ways
		Current_XPlane_data.roll_rot_vel = XPlane_data_float[16];	
		Current_XPlane_data.pitch_rot_vel = XPlane_data_float[17];
		Current_XPlane_data.heading_rot_vel = XPlane_data_float[18]; //Heading is 1.6-2.2x higher than the real value

		Current_XPlane_data.mag_heading = XPlane_data_float[19]; //Heading angle in degrees relative to magnetic north

		Current_XPlane_data.ambient_temp = XPlane_data_float[20]; //Ambient temperature in degrees C




		//Calculated elapsed time
		Current_XPlane_data.elapsed_time = Current_XPlane_data.flight_time - Last_XPlane_data.flight_time;

		Current_XPlane_data.roll_rad = Current_XPlane_data.roll * (M_PI / 180);
		Current_XPlane_data.pitch_rad = Current_XPlane_data.pitch * (M_PI / 180);
		Current_XPlane_data.heading_rad = Current_XPlane_data.heading * (M_PI / 180);

		Current_XPlane_data.roll_rot_vel_calcd = (Current_XPlane_data.roll_rad - Last_XPlane_data.roll_rad) / Current_XPlane_data.elapsed_time;
		Current_XPlane_data.pitch_rot_vel_calcd = (Current_XPlane_data.pitch_rad - Last_XPlane_data.pitch_rad) / Current_XPlane_data.elapsed_time;

		float heading_delta = Current_XPlane_data.heading_rad - Last_XPlane_data.heading_rad;
		//Means went from, e.g., 0.01 (last) to 6.27 (current), so delta is negative
		if (heading_delta > M_PI){
			heading_delta = -Last_XPlane_data.heading_rad + (Current_XPlane_data.heading_rad - 2*M_PI);
		}
		//Means went from, e.g., 6.27 (last) to 0.01 (current), so delta is positive
		if (heading_delta < -M_PI){
			heading_delta = Current_XPlane_data.heading_rad + (2*M_PI - Last_XPlane_data.heading_rad);
		}

		Current_XPlane_data.heading_rot_vel_calcd = heading_delta / Current_XPlane_data.elapsed_time;


		float indicated_ms = Current_XPlane_data.airspeed_kias * 0.51444;

		//Allow for dynamic pressure in negative direction
		Current_XPlane_data.longitudinal_Q = std::copysignf(0.5 * 1.2247 * indicated_ms * indicated_ms, indicated_ms);

	//	/*
		//float roll_guess = Current_XPlane_data.roll_rot_vel * 0.02 * 180/M_PI + Current_XPlane_data.roll;
		//float pitch_guess = Current_XPlane_data.pitch_rot_vel * 0.02 * 180/M_PI + Current_XPlane_data.pitch;



///*
		printf(" time: %f ", Current_XPlane_data.flight_time);
		
		printf(" roll: %f ", Current_XPlane_data.roll);
		//printf(" roll guess: %f ", roll_guess);


		printf(" pitch: %f ", Current_XPlane_data.pitch);
		//printf(" pitch guess: %f ", pitch_guess);

		printf(" hdg: %f ", Current_XPlane_data.heading);

		printf(" roll rv: %f ", Current_XPlane_data.roll_rot_vel  * 180 / M_PI);
		printf(" roll rv c: %f ", Current_XPlane_data.roll_rot_vel_calcd * 180 / M_PI);

		printf(" pitch rv: %f ", Current_XPlane_data.pitch_rot_vel  * 180 / M_PI);
		printf(" pitch rv c: %f ", Current_XPlane_data.pitch_rot_vel_calcd * 180 / M_PI);

		printf(" hdg rv: %f ", Current_XPlane_data.heading_rot_vel  * 180 / M_PI);
		printf(" hdg rv c: %f ", Current_XPlane_data.heading_rot_vel_calcd * 180 / M_PI);


		//printf(" lat: %f ", Current_XPlane_data.latitude);
		//printf(" long: %f", Current_XPlane_data.longitude);
		printf(" elev: %f ", Current_XPlane_data.elevation);


		printf(" as: %f \n", Current_XPlane_data.airspeed_kias);
	//	*/			

}






int XPlane::UDP_Send(int array[11]){
	

    char tmp[52]; //Needs to be big enough to hold all values in array.  Appears to be 4 bytes per int, so must be 48 or greater for array of 12.
    memset(tmp, 0, 52); 

    sprintf(tmp, "%i %i %i %i %i %i %i %i %i %i %i", array[0],array[1],array[2],array[3],array[4],array[5],array[6],array[7],array[8],array[9], array[10]);
    
    sendto(sock_send, tmp, sizeof(tmp), 0,(struct sockaddr *)&myaddr_send, sizeof(myaddr_send));
    
	//When socket only created once at the beginning, close(sock_send);  will prevent further communication
	return 0;

}



/*
	Input: IP address of flight computer, for XPlane to send a response back to
	Output: None

	Send message to XPlane at the beginning to set IP address to send back to

*/
int XPlane::UDP_Send_IP(char IP_addr[20]){
	

    printf("Setting XPlane IP to send to: %s \n", IP_addr);

    sendto(sock_send, IP_addr, 20, 0,(struct sockaddr *)&myaddr_send, sizeof(myaddr_send));

	return 0;
}




/*

	Input:
		1. Force and moment array with the form [roll_moment pitch_moment yaw_moment x_force (forwards) z_force (upwards) y_force] in Newtons and Nm
		2. Servo angles in radians
		3. Loop time in seconds (not microseconds)
	Output: 

	Note: using arrays here rather than a struct to avoid circular include dependency between Control and XPlane


*/
void XPlane::send_output_to_XPlane(float force_and_moment_array[6], float servo_angle_array[4], float loop_time){


		int XPlane_input_array[11];


		/*
		XPlane doesn't model thrust correctly when applying a motor torque during transients.  Propwash speed doesn't change fast enough, leading to too-slow inflow velocity
		while spinning up, resulting in an overprediction of thrust while propwash speed is catching up.  Can adjust prop airfoil and incidence to match torque, rpm, and thrust
		to real values, so will be correct after transient, but not during transient.  Even though RPM is correct, thrust during transient is not.

		Best to just override forces and moments from motors and calculate directly from control code output.  Get rotational velocities calculated for end of current loop and 
		calculate moments and forces
		
		*/

		XPlane_input_array[0] = force_and_moment_array[0] * 100; //Roll moment (Nm), adjusted by 100 to make precise int
		XPlane_input_array[1] = force_and_moment_array[1] * 100; //Pitch moment (Nm), adjusted by 100 to make precise int
		XPlane_input_array[2] = force_and_moment_array[2] * 100; //Yaw moment (Nm), adjusted by 100 to make precise int

		XPlane_input_array[3] = force_and_moment_array[3] * 100; //Forwards force (N), adjusted by 100 to make precise int
		XPlane_input_array[4] = force_and_moment_array[4] * 100; //Upwards force (N), adjusted by 100 to make precise int
		XPlane_input_array[5] = force_and_moment_array[5] * 100; //Side force (N), adjusted by 100 to make precise int

		XPlane_input_array[6] = loop_time * 1000000; //Loop time, adjusted to microseconds



/*
		XPlane_input_array[0] = -Control_for_XPlane.Theoretical_End_of_Loop_Fan_Forces_and_Moments.x_force * 100; // 
		XPlane_input_array[1] = Control_for_XPlane.Theoretical_End_of_Loop_Fan_Forces_and_Moments.z_force * 100;
		XPlane_input_array[2] = 0; //Side force should always be zero from fans

		XPlane_input_array[3] = Control_for_XPlane.Theoretical_End_of_Loop_Fan_Forces_and_Moments.roll_moment * 100;
		XPlane_input_array[4] = Control_for_XPlane.Theoretical_End_of_Loop_Fan_Forces_and_Moments.pitch_moment * 100;
		XPlane_input_array[5] = Control_for_XPlane.Theoretical_End_of_Loop_Fan_Forces_and_Moments.yaw_moment * 100;
*/



		//Commanding servo angle happens instantaneously, so just give angle output of control code servo model to XPlane
		for (int i=0; i<4; i++)
    	{

    		//0 represents 45 degrees backwards tilt, 999 represents 45 degrees downwards tilt
    		//-PI/4 -> 0, 3*PI/4 -> 999, so each 1's place represents a 0.003145 radian movement -> each radians represents 317.99 1's places (= 999 / PI)
    		float angle_over_udp = (servo_angle_array[i] + M_PI /4) / (M_PI / 999);

			XPlane_input_array[i + 7] = angle_over_udp;

		}

	
		UDP_Send(XPlane_input_array);

}
