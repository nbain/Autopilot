// Copyright (c) 2013-2018 United States Government as represented by the Administrator of the
// National Aeronautics and Space Administration. All Rights Reserved.
//
// DISCLAIMERS
//     No Warranty: THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY KIND,
//     EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, ANY WARRANTY THAT
//     THE SUBJECT SOFTWARE WILL CONFORM TO SPECIFICATIONS, ANY IMPLIED WARRANTIES OF
//     MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY
//     THAT THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT DOCUMENTATION, IF PROVIDED,
//     WILL CONFORM TO THE SUBJECT SOFTWARE. THIS AGREEMENT DOES NOT, IN ANY MANNER, CONSTITUTE AN
//     ENDORSEMENT BY GOVERNMENT AGENCY OR ANY PRIOR RECIPIENT OF ANY RESULTS, RESULTING DESIGNS,
//     HARDWARE, SOFTWARE PRODUCTS OR ANY OTHER APPLICATIONS RESULTING FROM USE OF THE SUBJECT
//     SOFTWARE.  FURTHER, GOVERNMENT AGENCY DISCLAIMS ALL WARRANTIES AND LIABILITIES REGARDING
//     THIRD-PARTY SOFTWARE, IF PRESENT IN THE ORIGINAL SOFTWARE, AND DISTRIBUTES IT "AS IS."
//
//     Waiver and Indemnity: RECIPIENT AGREES TO WAIVE ANY AND ALL CLAIMS AGAINST THE UNITED STATES
//     GOVERNMENT, ITS CONTRACTORS AND SUBCONTRACTORS, AS WELL AS ANY PRIOR RECIPIENT.  IF
//     RECIPIENT'S USE OF THE SUBJECT SOFTWARE RESULTS IN ANY LIABILITIES, DEMANDS, DAMAGES, EXPENSES
//     OR LOSSES ARISING FROM SUCH USE, INCLUDING ANY DAMAGES FROM PRODUCTS BASED ON, OR RESULTING
//     FROM, RECIPIENT'S USE OF THE SUBJECT SOFTWARE, RECIPIENT SHALL INDEMNIFY AND HOLD HARMLESS THE
//     UNITED STATES GOVERNMENT, ITS CONTRACTORS AND SUBCONTRACTORS, AS WELL AS ANY PRIOR RECIPIENT,
//     TO THE EXTENT PERMITTED BY LAW.  RECIPIENT'S SOLE REMEDY FOR ANY SUCH MATTER SHALL BE THE
//     IMMEDIATE, UNILATERAL TERMINATION OF THIS AGREEMENT.
//
// X-Plane API
// Copyright(c) 2008, Sandy Barbour and Ben Supnik All rights reserved.
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
// associated documentation files(the "Software"), to deal in the Software without restriction,
// including without limitation the rights to use, copy, modify, merge, publish, distribute,
// sublicense, and / or sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Neither the names of the authors nor that of X - Plane or Laminar Research
//     may be used to endorse or promote products derived from this software
//     without specific prior written permission from the authors or
//     Laminar Research, respectively.


// X-Plane Connect Plugin
//
// DESCRIPTION
//     XPCPlugin Facilitates Communication to and from the XPlane
//
// INSTRUCTIONS
//     See Readme.md in the root of this repository or the wiki hosted on GitHub at
//     https://github.com/nasa/XPlaneConnect/wiki for requirements, installation instructions,
//     and detailed documentation.
//
// CONTACT
//     For questions email Christopher Teubert (christopher.a.teubert@nasa.gov)
//
// CONTRIBUTORS
//     CT: Christopher Teubert (christopher.a.teubert@nasa.gov)
//     JW: Jason Watkins (jason.w.watkins@nasa.gov)

/*
 Nick Bain comment:
 
 Should be possible to replace the code in XPCPlugin.cpp with the code below and everything should just work.
 
 The way to set up:
 
 1. Download XPlane Connect: https://github.com/nasa/XPlaneConnect
 2. Open xpcPlugin.xcodeproj in Xcode
 3. Delete xplaneConnect.c file (not needed and won't compile)
 4. Replace code in XPCPlugin.cpp with code below
 5. Compile by pressing arrow/play button in top left corner of Xcode
 6. Compiling will create a folder called XPlaneConnect in the xpcPlugin folder.  Drag this folder into Resources/plugins in XPlane 11
 7. Start XPlane and plugin should be installed
 
 
 
 The modifications:
 
 1. Added two lines in XPluginEnable that run when plugin starts that set frame rate
		XPLMSetDatai(Override_framerate_Dref, 1); //Make framerate overridable.
		XPLMSetDataf(framerate_dref,0.02); //Set frame rate in seconds.
 
 2.  Added a number of lines that make it possible to access and set XPlane datarefs
		e.g. XPLMDataRef paused_bool_dref = XPLMFindDataRef("sim/time/paused");
 
 3. Added UDPSend function that sends an array of floats over UDP
		The function opens and also closes a socket, so when listening with, e.g "nc -u -l 8888", only one communication will be received even if many are sent, because the socket is closed with each array sent
 
 4. Rewrote the main function, XPCFlightLoopCallback().
		-Called every frame (by setting, in XPluginEnable(): float interval = -1)
		Then:

			1. Get data from XPlane (position, attitude and their derivatives, etc)
 
			2. Exit function if less than a certain amount of time elapsed (allow to settle for a couple seconds)
 
			3. Package XPlane data into an array and send over UDP
				-UDP address set in UDP_Send(). (not originally in XPlaneConnect - written through experiment)
					-Address should be the address of the computer running the control code.
					-Works well communicating with Udoo X86 over WiFi
					-Any port works as long as not in use.  Computer running control code listens for data on this port.
 
			4.  Waits for a UDP message, using XPlaneConnect's UDP handler.
					-XPlaneConnect receives on port 49009 by default
						-Control code should send a UDP message of throttle and servo values to the IP Address of computer running XPlane on port 49009
						-Message format is a string of three digit numbers, each separated by a space
							-How to convert this 3-digit number into a thrust or angle can be played around with
								-Way to think about this is that the 3 digit number is just a ~10 bit piece of info
						
 
			5.  Throttle information (fraction of max from 0 to 1) and servo angle (?) is set
					 XPLMSetDatavf(Thrott_Dref,throttle_array,0,8);
					 XPLMSetDatavf(Vect_Dref,servo_array,0,8);
 

		Note: Pausing/unpausing the simulation isn't necessary because the callback gets called every single frame.  Because it waits indefinitely for thrust and servo angle delta, it will just wait (with the physics/graphics stopped) until getting a good value before proceeding
 
		Note 2: Shouldn't be possible to accidentally go faster than real time, only slightly slower - if plugin code and control code executes in an arbitarily fast time, say 0.0001 seconds, for a frame period of 0.02 seconds, then will perceive 0.02 seconds in physics model as taking 0.0201 seconds
 

 
 
 */

// XPC Includes
#include "DataManager.h"
#include "Drawing.h"
#include "Log.h"
#include "MessageHandlers.h"
#include "UDPSocket.h"
#include "Timer.h"

// XPLM Includes
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"
#include "XPLMDataAccess.h"

// System Includes
#include <cstdlib>
#include <cstring>
#include <cmath>
#ifdef __APPLE__
#include <mach/mach_time.h>
#endif

#include <stdio.h>
#include <string.h>
#include <unistd.h>


#define RECVPORT 49009 // Port that the plugin receives commands on
#define OPS_PER_CYCLE 20 // Max Number of operations per cycle

#define XPC_PLUGIN_VERSION "1.3-rc.1"

using namespace std;

XPC::UDPSocket* sock = NULL;
XPC::Timer* timer = NULL;

double start;
double lap;
static double timeConvert = 0.0;
int benchmarkingSwitch = 0; // 1 = time for operations, 2 = time for op + cycle;

PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc);
PLUGIN_API void	XPluginStop(void);
PLUGIN_API void XPluginDisable(void);
PLUGIN_API int XPluginEnable(void);
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFromWho, int inMessage, void* inParam);
static float XPCFlightLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon);





//Get simulation time
XPLMDataRef flight_time_dref = XPLMFindDataRef("sim/time/total_flight_time_sec"); //float y seconds Total time since the flight got reset by something
XPLMDataRef total_time_dref = XPLMFindDataRef("sim/time/total_running_time_sec"); //float y seconds Total time the sim has been up
		//Maybe try writing

//For IMU with internal Kalman-filter (assuming always correct).  All in degrees.
//sim/flightmodel2/position appears to be identical to sim/flightmodel/position for phi, theta, psi
XPLMDataRef roll_dref = XPLMFindDataRef("sim/flightmodel/position/true_phi"); //float n degrees The roll of the aircraft relative to the earth precisely below the aircraft
XPLMDataRef pitch_dref = XPLMFindDataRef("sim/flightmodel/position/true_theta"); //float n degrees The pitch of the aircraft relative to the earth precisely below the aircraft
XPLMDataRef hdg_dref = XPLMFindDataRef("sim/flightmodel/position/true_psi"); //float n degrees The heading of the aircraft relative to the earth precisely below the aircraft - true degrees north, always




//For GPS simulation
XPLMDataRef lat_dref = XPLMFindDataRef("sim/flightmodel/position/latitude"); //double n	degrees The latitude of the aircraft
XPLMDataRef long_dref = XPLMFindDataRef("sim/flightmodel/position/longitude"); //double	n degrees The longitude of the aircraft
XPLMDataRef elev_dref = XPLMFindDataRef("sim/flightmodel/position/elevation"); //double	n meters The elevation above MSL of the aircraft


//For Pitot tube simulation
XPLMDataRef airspeed_dref = XPLMFindDataRef("sim/flightmodel/position/indicated_airspeed"); //float y kias Air speed indicated - this takes into account air density and wind direction


//For Lidar simulation.  Note: should also take into account roll/pitch angle changing ray length
XPLMDataRef y_agl_dref = XPLMFindDataRef("sim/flightmodel/position/y_agl"); //float n meters AGL


//For altimeter simulation
XPLMDataRef baro_alt_dref = XPLMFindDataRef("sim/flightmodel/misc/h_ind"); //float y feet Indicated barometric altitude



//For accelerometer simulation.  May need to transform from OpenGL coordinates to real world coordinates.
XPLMDataRef OpenGL_acc_x_dref = XPLMFindDataRef("sim/flightmodel/position/local_ax");//float y mtr/sec2 The acceleration in local OGL coordinates
XPLMDataRef OpenGL_acc_y_dref = XPLMFindDataRef("sim/flightmodel/position/local_ay");//float y mtr/sec2 The acceleration in local OGL coordinates
XPLMDataRef OpenGL_acc_z_dref = XPLMFindDataRef("sim/flightmodel/position/local_az");//float y mtr/sec2 The acceleration in local OGL coordinates

//For transformation between OpenGL and real world coordinates.  Compare OpenGL euler angles with real world euler angles.
XPLMDataRef OpenGL_roll_dref = XPLMFindDataRef("sim/flightmodel/position/phi"); //float y degrees The roll of the aircraft in degrees - OpenGL coordinates
XPLMDataRef OpenGL_pitch_dref = XPLMFindDataRef("sim/flightmodel/position/theta"); //float y degrees The pitch relative to the plane normal to the Y axis in degrees - OpenGL coordinates relative to the earth precisely below the aircraft
XPLMDataRef OpenGL_hdg_dref = XPLMFindDataRef("sim/flightmodel/position/psi"); //float y degrees The true heading of the aircraft in degrees from the Z axis - OpenGL coordinates



//For gyro simulation
XPLMDataRef Prad_dref = XPLMFindDataRef("sim/flightmodel/position/Prad"); //float y rad/sec The roll rotation rates (relative to the flight)
XPLMDataRef Qrad_dref = XPLMFindDataRef("sim/flightmodel/position/Qrad"); //float y rad/sec The pitch rotation rates (relative to the flight)
XPLMDataRef Rrad_dref = XPLMFindDataRef("sim/flightmodel/position/Rrad"); //float y rad/sec The yaw rotation rates (relative to the flight)



//For magnetometer simulation
XPLMDataRef mag_hdg_dref = XPLMFindDataRef("sim/flightmodel/position/mag_psi"); //float n degrees The real magnetic heading of the aircraft

//For temperature simulation (-> dynamic pressure)
XPLMDataRef ambient_temp_dref = XPLMFindDataRef("sim/weather/temperature_ambient_c"); //float n degreesC The air temperature outside the aircraft (at altitude).

//For barometer simulation
XPLMDataRef static_pressure_dref = XPLMFindDataRef("sim/weather/barometer_current_inhg"); //float n 29.92+-.... This is the barometric pressure at the point the current flight is at.



//For engine force and moment override
XPLMDataRef propulsion_backward_force_dref = XPLMFindDataRef("sim/flightmodel/forces/faxil_prop"); //float y Newtons force backward by all engines on the ACF (usually this is a negative number).  Override with override_engines

XPLMDataRef propulsion_upward_force_dref = XPLMFindDataRef("sim/flightmodel/forces/fnrml_prop"); //float y Newtons force upward by all engines on the ACF.  Override with override_engines  Writable in v10 only or v11 with override

XPLMDataRef propulsion_side_force_dref = XPLMFindDataRef("sim/flightmodel/forces/fside_prop"); //float y Newtons force sideways by all engines on the ACF.  Override with override_engines or override_engine_forces


//Shouldn't go above ~10 Nm in flight (all right rotors on max , all left off about 11 Nm)
XPLMDataRef propulsion_roll_moment_dref = XPLMFindDataRef("sim/flightmodel/forces/L_prop"); //float y NM The roll moment due to prop forces. Override with override_engines or override_engine_forces - positive = right roll.

//All back on full, all front off about 17 Nm
XPLMDataRef propulsion_pitch_moment_dref = XPLMFindDataRef("sim/flightmodel/forces/M_prop"); //float y NM The pitch moment due to prop forces. Override with override_engines - positive = pitch up.

//Should be similar to roll, about 10 Nm max
XPLMDataRef propulsion_yaw_moment_dref = XPLMFindDataRef("sim/flightmodel/forces/N_prop"); //float y NM The yaw moment due to prop forces. Override with override_engines - positive = yaw right/clockwise.







//For thrust and angle control
XPLMDataRef throttle_override_dref = XPLMFindDataRef("sim/operation/override/override_throttles"); //int y boolean Override the throttles. Use ENGN_thro_use to control them.
XPLMDataRef throttle_dref = XPLMFindDataRef("sim/flightmodel/engine/ENGN_thro_use"); //float[8] y ratio Throttle (per engine) when overridden by you, plus with thrust vectors - use override_throttles to change

XPLMDataRef motor_torque_dref = XPLMFindDataRef("sim/flightmodel/engine/ENGN_TRQ");	//float[8] y NewtonMeters Torque (per engine)

XPLMDataRef servo_angles_dref = XPLMFindDataRef("sim/aircraft/prop/acf_vertcant");

XPLMDataRef planepath_override_dref = XPLMFindDataRef("sim/operation/override/override_planepath");//int[20] y boolean Override position updates of this plane

XPLMDataRef engine_forces_and_moments_override_dref = XPLMFindDataRef("sim/operation/override/override_engines"); //int y boolean overrides all engine calculations - write to LMN and g_nrml/side/axil.

XPLMDataRef engine_forces_and_moments_override2_dref = XPLMFindDataRef("sim/operation/override/override_engine_forces"); //int y boolean overrides all engine calculations - write to prop LMN and g_nrml/side/axil.

XPLMDataRef torque_override_dref = XPLMFindDataRef("sim/operation/override/override_torque_motors"); //int y boolean overrides all engine calculations but not the prop - write the torque your motor generates while the prop will still be simulated by X-Plane

XPLMDataRef paused_bool_dref = XPLMFindDataRef("sim/time/paused");

XPLMDataRef G_mss_dref = XPLMFindDataRef("sim/physics/g_sealevel");


XPLMDataRef Override_framerate_Dref = XPLMFindDataRef("sim/operation/override/override_timestep");
XPLMDataRef framerate_dref = XPLMFindDataRef("sim/operation/misc/frame_rate_period");

//Accel_body : m/s^2 in body frame
XPLMDataRef G_norm_dref = XPLMFindDataRef("sim/flightmodel2/misc/gforce_normal");//IMU Z acceleration
XPLMDataRef G_axial_dref = XPLMFindDataRef("sim/flightmodel2/misc/gforce_axil");//IMU Y acceleration (could be X - check with ins sketch)
XPLMDataRef G_side_dref = XPLMFindDataRef("sim/flightmodel2/misc/gforce_side");//IMU X acceleration (could be Y - check with ins sketch)


//Quaternion - XPlane Dref doesn't give unit.  Prob radians.
XPLMDataRef Quat_dref = XPLMFindDataRef("sim/flightmodel/position/q");//OpenGL to aircraft coordinates quaternion (4 float array)

//Velocity_ef : Find what X,Y,Z are in North, East, Down
XPLMDataRef Velx_dref = XPLMFindDataRef("sim/flightmodel/position/local_vx");//m/s in x(?) in local OpenGL coordinates
XPLMDataRef Vely_dref = XPLMFindDataRef("sim/flightmodel/position/local_vy");//m/s in y(?) in local OpenGL coordinates
XPLMDataRef Velz_dref = XPLMFindDataRef("sim/flightmodel/position/local_vz");//m/s in z(?) in local OpenGL coordinates

//Position : Find what X,Y,Z are in North, East, Down
XPLMDataRef Posx_dref = XPLMFindDataRef("sim/flightmodel/position/local_x");//Position in meters in x(?) in local OpenGL coordinates
XPLMDataRef Posy_dref = XPLMFindDataRef("sim/flightmodel/position/local_y");//Position in meters in y(?) in local OpenGL coordinates
XPLMDataRef Posz_dref = XPLMFindDataRef("sim/flightmodel/position/local_z");//Position in meters in z(?) in local OpenGL coordinates


float loop_time; //Loop time = physics model step time in seconds

char IP_addr[20]; //Control system IP address for XPlane to send to
bool IP_addr_recvd = false; //Check whether an IP address to send to has been received

int sock_send;
struct sockaddr_in myaddr_send;


/*
 Used separate function to initialize UDP socket to try to reduce latency by not creating a new socket with every transmission.
 Likely not necessary - no easily measureable improvement (and for sure less than ~1 ms).
 
 UDP latency purely affected by connection - over WiFi, seems to be ~1.5 ms each way, so adds 3 ms to loop time.
 Sharing internet from MacBook to Udoo X86 with USB-to-Ethernet adapter, latency is almost instantaneous (probably ~100 us or less)
 
 See comment in main function between UDP sending and receive on timing
	(over Ethernet, total loop time ~= XPlane frame simulation time + Control system loop time)
 
	-> Can speed up frame rate on XPlane:
		-Reducing flight models per frame (real metric to be concerned about is flight models/sec, not flight models/frame
			-Setting 0.01 frame period at 10 models frame -> 1000 Hz simulation
				-Simulation, on MacBook, is CPU limited rather than GPU limited.  Reducing to 2 flight models/frame runs entire simulation (when over Ethernet) at 80-90 frames/sec vs 50 frames/sec at 10 flight models/frame
						(tested with 0.02 sec frame period)
 
 To share internet on MacBook:
	-System Preferences -> Sharing -> Share Connection from: USB 10/100 LAN, to computers using: USB 10/100 LAN
	-Check Internet Sharing box
 
 */
int UDP_Send_Setup(char IP_addr[20]){
	
	int option = 1;
	int port = 8888;
	//struct sockaddr_in myaddr;
	memset(&myaddr_send, 0, sizeof(myaddr_send));
	myaddr_send.sin_family=AF_INET;
	myaddr_send.sin_addr.s_addr=inet_addr(IP_addr); //This should be the IP address of the computer running control code
	myaddr_send.sin_port=htons(port); //
	
	
	//int sock;
	sock_send =socket(AF_INET, SOCK_DGRAM, 0);
	setsockopt(sock_send, SOL_SOCKET, SO_REUSEADDR, (char*)&option, sizeof(option));
	
	//Set send and receive buffer sizes (in bytes).  Not clear if this reduces latency.
	int option2 = 300;
	setsockopt(sock_send, SOL_SOCKET, SO_SNDBUF, (char*)&option2, sizeof(option2));
	
	int option3 = 300;
	setsockopt(sock_send, SOL_SOCKET, SO_RCVBUF, (char*)&option3, sizeof(option3));
	
	bind(sock_send,( struct sockaddr *) &myaddr_send, sizeof(myaddr_send));
	
	XPC::Log::FormatLine(LOG_INFO, "EXEC", "Socket send IP address set to: %s", IP_addr);
	XPC::Log::FormatLine(LOG_INFO, "EXEC", "Socket send port: %i", port);
	
	return 0;
	
}



int UDP_Send(float array[21]){
	

	char tmp2[250]; //WAS 512 HERE
	memset(tmp2, 0, 250); //LAST ARG WAS 512
	
	sprintf(tmp2, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", array[0],array[1],array[2],array[3],array[4],array[5],array[6],array[7], \
			array[8],array[9],array[10],array[11],array[12],array[13],array[14],array[15],array[16],array[17],array[18],array[19],array[20]);
	
	sendto(sock_send, tmp2, sizeof(tmp2), 0,(struct sockaddr *)&myaddr_send, sizeof(myaddr_send));
	
	return 0;
}







PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc)
{
	strcpy(outName, "X-Plane Connect [Version 1.3-rc.1]");
	strcpy(outSig, "NASA.XPlaneConnect");
	strcpy(outDesc, "X Plane Communications Toolbox 2, modified by Nick Bain 2");

#if (__APPLE__)
	if ( timeConvert <= 1e-9 ) // is about 0
	{
		mach_timebase_info_data_t timeBase;
		(void)mach_timebase_info(&timeBase);
		timeConvert = (double)timeBase.numer /
		(double)timeBase.denom /
		1000000000.0;
	}
#endif
	XPC::Log::Initialize(XPC_PLUGIN_VERSION);
	XPC::Log::WriteLine(LOG_INFO, "EXEC", "Plugin Start");
	XPC::DataManager::Initialize();

	return 1;
}

PLUGIN_API void	XPluginStop(void)
{
	XPC::Log::WriteLine(LOG_INFO, "EXEC", "Plugin Shutdown");
	XPC::Log::Close();
}

PLUGIN_API void XPluginDisable(void)
{
	XPLMUnregisterFlightLoopCallback(XPCFlightLoopCallback, NULL);

	// Close sockets
	delete sock;
	sock = NULL;

	// Stop rendering messages to screen.
	XPC::Drawing::ClearMessage();

	// Stop rendering waypoints to screen.
	XPC::Drawing::ClearWaypoints();

	XPC::Log::WriteLine(LOG_INFO, "EXEC", "Plugin Disabled, sockets closed");
	
	timer->stop();
	delete timer;
	timer = NULL;
}

PLUGIN_API int XPluginEnable(void)
{
	// Open sockets
	sock = new XPC::UDPSocket(RECVPORT);
	timer = new XPC::Timer();
	
	XPC::MessageHandlers::SetSocket(sock);
	
	
	XPC::Log::WriteLine(LOG_INFO, "EXEC", "Plugin Enabled, sockets opened");
	if (benchmarkingSwitch > 0)
	{
		XPC::Log::FormatLine(LOG_INFO, "EXEC", "Benchmarking Enabled (Verbosity: %i)", benchmarkingSwitch);
	}
	XPC::Log::FormatLine(LOG_INFO, "EXEC", "Debug Logging Enabled (Verbosity: %i)", LOG_LEVEL);


	
	float interval = -1; // Call every frame
	void* refcon = NULL; // Don't pass anything to the callback directly
	XPLMRegisterFlightLoopCallback(XPCFlightLoopCallback, interval, refcon);

	
	int xpVer;
	XPLMGetVersions(&xpVer, NULL, NULL);
	
	timer->start(chrono::milliseconds(1000), [=]{
		XPC::MessageHandlers::SendBeacon(XPC_PLUGIN_VERSION, RECVPORT, xpVer);
	});
	
	
	
	
	

	return 1;
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFromWho, int inMessage, void* inParam)
{
	// XPC doesn't have anything useful to say to other plugins, so simply ignore
	// any messages received.
}



/*
 
 After a couple seconds of initialization, the plugin should, every frame (specified with interval = -1 above):
 
	-Get sensor and time data
	-Send data over UDP to control system
 
	-Wait for throttle and servo angle sequence response
 
	-Set fan forces and moments

 
This function runs right before updating the graphics and displaying new outputs on the XPlane display
	-Known because display stops at 2.99 when function returns if flight_time < 3
		-Actual flight time seen to be 3.00, and display and graphics not updated
 
		-> Result of this is, loop sequence looks like:
			1. Update time (= the time at the end of finishing the current loop, step #4)
			2. Run plugin (in this case, update forces, which are actually applied at time = 2.99)
				Note: control should output motor torques and servo accelerations that are correct for the time they are actually being applied in reality.  Even though there is delay between getting a sensor reading and outputting a control, the control system should take into account the time at which it is actually sending commands.  This means it should predict the location it will be at at the time the commands are sent and the commands sent should be based on that location.
						-> The XPlane model output essentially gives a correct prediction for the location at the time the commands are sent
 
				Way to think about this in general is that the control system is actually working with and giving a control output for 2.99 even though the current time already 3.00 - the 3.00 is a target after a time of applying new forces
						-> This means that the control code should actually send the forces and moments generated by the commands sent at time = 2.98, which have spun up by this time, and save the commands found for time = 2.99 to be sent as forces and moments at time = 3.00
 
			3. Run physics model (to get location, etc at 3.00)
			4. Output new graphics (at time = 3.00)
 
			-So plugin will send new time over UDP, but combined with model outputs for the last timestep (verified)
 
 */

float XPCFlightLoopCallback(float inElapsedSinceLastCall,
	float inElapsedTimeSinceLastFlightLoop,
	int inCounter,
	void* inRefcon)
{
	
	//XPC::Log::FormatLine(LOG_INFO, "EXEC", "Starting loop");
	
	//Get all data
	
	//Simulation time
	float flight_time = XPLMGetDataf(flight_time_dref); //Time since beginning of current flight, after most recent reset
	float total_time = XPLMGetDataf(total_time_dref);
	
	//For IMU with internal Kalman-filter (assuming always correct).  All in degrees.
	float roll = XPLMGetDataf(roll_dref);
	float pitch = XPLMGetDataf(pitch_dref);
	float heading = XPLMGetDataf(hdg_dref);
	
	//GPS
	float latitude = XPLMGetDataf(lat_dref);
	float longitude = XPLMGetDataf(long_dref);
	float elevation = XPLMGetDataf(elev_dref);
	
	//Pitot tube
	float airspeed = XPLMGetDataf(airspeed_dref);
	
	//Downward-pointing Lidar.  Note: should also take into account roll/pitch angle changing ray length
	float dist_agl = XPLMGetDataf(y_agl_dref);
	
	//Altimeter
	//float baro_alt = XPLMGetDataf(baro_alt_dref);
	float static_pressure = XPLMGetDataf(static_pressure_dref);
	
	//Accelerometer and OpenGL correction data
	float OpenGL_acc_x = XPLMGetDataf(OpenGL_acc_x_dref);
	float OpenGL_acc_y = XPLMGetDataf(OpenGL_acc_y_dref);
	float OpenGL_acc_z = XPLMGetDataf(OpenGL_acc_z_dref);
	
	float OpenGL_roll = XPLMGetDataf(OpenGL_roll_dref);
	float OpenGL_pitch = XPLMGetDataf(OpenGL_pitch_dref);
	float OpenGL_hdg = XPLMGetDataf(OpenGL_hdg_dref);
	
	
	//Gyro
	float roll_rot_vel = XPLMGetDataf(Prad_dref);//Roll rotation rate in rad/s
	float pitch_rot_vel = XPLMGetDataf(Qrad_dref);//Pitch rotation rate in rad/s
	float yaw_rot_vel = XPLMGetDataf(Rrad_dref);//Yaw rotation rate in rad/s
	
	//Magnetometer
	float mag_heading = XPLMGetDataf(mag_hdg_dref);

	//Temperature
	float ambient_temp = XPLMGetDataf(ambient_temp_dref);

	
	//Let XPlane load and settle for number of seconds (otherwise never loads)
	if (flight_time < 3){
		
		XPLMSetDataf(propulsion_backward_force_dref,0);
		XPLMSetDataf(propulsion_upward_force_dref,0);
		XPLMSetDataf(propulsion_side_force_dref,0);
		XPLMSetDataf(propulsion_roll_moment_dref,0);
		XPLMSetDataf(propulsion_pitch_moment_dref,0);
		XPLMSetDataf(propulsion_yaw_moment_dref,0);
		
		return -1;
	}
	
	

	//Block here until received an IP address to send back to.  After receiving, always skip this.
	if (!IP_addr_recvd){
	
		XPC::Message IPaddr_msg = XPC::Message::ReadFrom(*sock);
		
		XPC::Log::FormatLine(LOG_INFO, "EXEC", "Waiting for IP address", IP_addr);
		
		for(;;){
			IPaddr_msg = XPC::Message::ReadFrom(*sock);
			
			if (IPaddr_msg.GetSize() > 0){
				break;
			}
			usleep(1000);
		}
		
		const unsigned char* IPaddr_msg_const_char = IPaddr_msg.GetBuffer();
		
		char * IPaddr_msg_char_star = (char *)(IPaddr_msg_const_char);

		strcpy(IP_addr, IPaddr_msg_char_star);
		
		UDP_Send_Setup(IP_addr);
		
		XPC::Log::FormatLine(LOG_INFO, "EXEC", "Received IP.  Setting IP address to: %s", IP_addr);
		
		
		
		IP_addr_recvd = true;
		
	}




	
	
	//Send relevant data over UDP
	
	flight_time = flight_time - loop_time; //See comment at top of function for reasoning here

	//Send flight/sensor data into autopilot over UDP
	//Makes a new connection every time (might be better for reliability, no errors in buffer) - but may be slower
	float flightstateArray[21] ={flight_time, roll, pitch, heading, latitude, longitude, elevation,
		airspeed, dist_agl, static_pressure, OpenGL_acc_x, OpenGL_acc_y, OpenGL_acc_z, OpenGL_roll, OpenGL_pitch, OpenGL_hdg,
		roll_rot_vel, pitch_rot_vel, yaw_rot_vel, mag_heading, ambient_temp
	};
	
	

	UDP_Send(flightstateArray);
	//XPC::Log::FormatLine(LOG_INFO, "EXEC", "UDP sent to control system");
	
	XPC::Log::FormatLine(LOG_INFO, "EXEC", "UDP sent to controller.  Simulation flight time: %f", flight_time);
	
	
	
	//Listen for throttle and servo angle response
	XPC::Message msg = XPC::Message::ReadFrom(*sock);
	
	//Listen indefinitely for a throttle and servo angle response
	//If no response after a certain number of tries, send the XPlane data over UDP again
	

	//Time waited here, when connected to Ethernet, is essentially the time taken to complete the control loop (or within ~1 ms, verified by experiment).  Add ~3ms for UDP packets sent over local WiFi.
	int count = 0;
	
	for(;;){

		XPC::Log::FormatLine(LOG_INFO, "EXEC", "Count: %i", count);
		
		//Check if > 20 because don't want to confuse IP message (20 bytes) with control message
		if (msg.GetSize() > 20){
			break;
		}
		
		usleep(1000);
		
		msg = XPC::Message::ReadFrom(*sock);
		
		
		//If tried 1,000 times to read back data and still got nothing, try sending the data over again and reset the count
		//1,000,000 attempts seems to occur every 1-2 seconds
		if (count > 2000){
			UDP_Send(flightstateArray);
			
			XPC::Log::FormatLine(LOG_INFO, "EXEC", "UDP sent again to: %s", IP_addr);
			count = 0;
		}
		
		count = count + 1;
		
	}
	

	const unsigned char* input_array = msg.GetBuffer();
	
	char * input_array_char_star = (char *)(input_array);
	
	char input_array_char[50];
	std::strcpy(input_array_char, input_array_char_star);
	
	XPC::Log::FormatLine(LOG_INFO, "EXEC", "UDP received from control system: %s", input_array_char);
	

	//Parse UDP input char array into control_input_float array, with values delimited by spaces and made into floats
	float control_input_float[11];
	char * control_input_ptr = strtok(input_array_char, " ");
	
	int array_index = 0;
	
	while(control_input_ptr != NULL)
	{
		std::string tmp1 = control_input_ptr;
		float tmp2 = (float)atof(tmp1.c_str());
		control_input_float[array_index] = tmp2;
		
		control_input_ptr = strtok(NULL, " ");
		array_index = array_index + 1;
		
	}
	



	
	//Set motor forces and servo angles (angles may only be for visualization)

	//Allow propulsion forces and moments override - may need to overwrite both.  Only #1 causes XPlane to fight override every step.
	XPLMSetDatai(engine_forces_and_moments_override_dref,1);
	XPLMSetDatai(engine_forces_and_moments_override2_dref,1);
	
	XPLMSetDataf(total_time_dref,0); //Override 15 min limit
	
	
	//Max moments should be ~10 Nm.  0.01 Nm accuracy dividing by 100.
	float propulsion_roll_moment = control_input_float[0] / 100;
	float propulsion_pitch_moment = control_input_float[1] / 100;
	float propulsion_yaw_moment = control_input_float[2] / 100;
	
	//Max forces should be under 100N.  0.01N accuracy dividing by 100.
	float propulsion_backward_force = - control_input_float[3] / 100; //Forward force is negative, so switch positive forces to neg
	float propulsion_upward_force = control_input_float[4] / 100;
	float propulsion_side_force = control_input_float[5] / 100;

	//Input should be in microseconds
	float loop_time = control_input_float[6] / 1000000;
	
	XPLMSetDataf(propulsion_backward_force_dref,propulsion_backward_force);
	XPLMSetDataf(propulsion_upward_force_dref,propulsion_upward_force);
	XPLMSetDataf(propulsion_side_force_dref,propulsion_side_force);
	XPLMSetDataf(propulsion_roll_moment_dref,propulsion_roll_moment);
	XPLMSetDataf(propulsion_pitch_moment_dref,propulsion_pitch_moment);
	XPLMSetDataf(propulsion_yaw_moment_dref,propulsion_yaw_moment);
	XPLMSetDataf(framerate_dref,loop_time); //Set frame rate in seconds.
	
	
	XPC::Log::FormatLine(LOG_INFO, "EXEC", "Backward force set: %f", propulsion_backward_force);
	XPC::Log::FormatLine(LOG_INFO, "EXEC", "Upward force set: %f", propulsion_upward_force);
	XPC::Log::FormatLine(LOG_INFO, "EXEC", "Side force set: %f", propulsion_side_force);
	
	XPC::Log::FormatLine(LOG_INFO, "EXEC", "Roll moment set: %f", propulsion_roll_moment);
	XPC::Log::FormatLine(LOG_INFO, "EXEC", "Pitch moment set: %f", propulsion_pitch_moment);
	XPC::Log::FormatLine(LOG_INFO, "EXEC", "Yaw moment set: %f", propulsion_yaw_moment);
	
	XPC::Log::FormatLine(LOG_INFO, "EXEC", "Frame rate set: %f", loop_time);

	
	

	
	//Because control packaged as three characters separated by a space, need a way to map from a 3-digit number to an angle
	//0 should be pointed 45 degrees backwards, 999 should be pointed 45 degrees downwards.  90 degree nominal window with 45 degrees on either side.  Gives 0.18 degrees of accuracy (180/1000).  With the XPlane dataref, -45 deg -> 60 for dataref and 135 deg -> -120 for dataref.  0 over UDP -> 60 for dataref, 999 over UDP -> -120 for dataref.
	//Dataref value = 60 - 0.180180180 * (UDP value)
	float Servo1 = 60 - 0.18018 * control_input_float[7];
	float Servo2 = 60 - 0.18018 * control_input_float[8];
	float Servo3 = 60 - 0.18018 * control_input_float[9];
	float Servo4 = 60 - 0.18018 * control_input_float[10];
	
	
	XPC::Log::FormatLine(LOG_INFO, "EXEC", "Servo 1 set: %f", Servo1);
	XPC::Log::FormatLine(LOG_INFO, "EXEC", "Servo 2 set: %f", Servo2);
	XPC::Log::FormatLine(LOG_INFO, "EXEC", "Servo 3 set: %f", Servo3);
	XPC::Log::FormatLine(LOG_INFO, "EXEC", "Servo 4 set: %f", Servo4);
	
	
	//For some reason, dataref is off by 15 degrees and flipped negative.  Output window on XPlane shows correct value however.
	//-75 is really 0 degrees in model and XPlane data display, propeller pointed straight forward
	//15 is really 90 degrees in model, pointed straight up
	//Verified exactly 15 degrees by looking at force components
	//Note: XPlane propwash modelling seems unrealistic.  Non-experimental flight model reduces thrust of forward prop when back prop on(whose thrust is also reduced).  Experimental flight model shows back prop going from 12k to 17k rpm when in streamline of front prop
	//float servo_array[8] = {-75,-75,-75,-75,-75,-75,-75,-75};
	float servo_array[8] = {Servo1,Servo2,Servo3,Servo4,Servo3,Servo4,Servo3,Servo4};
	XPLMSetDatavf(servo_angles_dref,servo_array,0,8);


	XPC::Log::FormatLine(LOG_INFO, "EXEC", "All datarefs set in XPlane");
	
	return -1;
}
