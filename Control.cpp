#include "Control.h"
#include <math.h> //For sqrt() control

float dt;
float l_time = 0;
float desired_yaw = 0;
float yaw_trim = 0;
float master_tilt = 0; //So master_tilt used to be -Receiver->pitch * 90 with no slew limiter 
float desired_tilt = 0;
float max_tilt_rate;

float roll_rate_error_3 = 0;
float roll_rate_error_2 = 0;
float roll_rate_error_1 = 0;

float pitch_rate_error_3 = 0;
float pitch_rate_error_2 = 0;
float pitch_rate_error_1 = 0;

float yaw_rate_error_3 = 0;
float yaw_rate_error_2 = 0;
float yaw_rate_error_1 = 0;

float roll_rate_3 = 0;
float roll_rate_2 = 0;
float roll_rate_1 = 0;

float pitch_rate_3 = 0;
float pitch_rate_2 = 0;
float pitch_rate_1 = 0;

float yaw_rate_3 = 0;
float yaw_rate_2 = 0;
float yaw_rate_1 = 0;

float pitch_rate_error_int = 0;
float roll_rate_error_int = 0;





/*

Command a jerk up to certain alpha limits (prob something like -150 to 150 alpha/sec)
	-Will be state-based
		-Should generate first jerk in series that will work from current a, v, p (will prob always be max)



Physically possible to command -50 to 50 deg/s^2 (alpha)
	-Depending on state, alpha limits will change
	-Can only ramp to alpha at certain rate (also state dependent)

Say can ramp instantly and alpha limits are constant:
	-If at 10 deg right roll and rolling 10 deg/s right, apply alpha = -50
		-0.1 sec: 5 deg/s right, 10.75 deg right
		-0.2 sec: 0 deg/s right, 11 deg right
		-0.3 sec: 5 deg/s left, 10.75 deg right
		-0.4 sec: 10 deg/s left, 10 deg right
		-0.5 sec: 15 deg/s left, 8.75 deg right
		-0.6 sec: 20 deg/s left, 7 deg right
		-0.7 sec: 25 deg/s left, 4.75 deg right
		-0.8 sec: 20 deg/s left, 2.5 deg right
		-0.9 sec: 15 deg/s left, 0.75 deg right
		-1.0 sec: 10 deg/s left, 0.5 deg left
		-1.1 sec: 


Maximum velocity at given position where can still hit zero velocity at zero angle when going max accel:
	Max roll velocity = -2 * 0.5*Max Roll Accel *sqrt(roll angle / (0.5 * Max Roll Accel))
		-Intuition is that this is a just a stretched/scaled sqrt function where max ok velocity increases with root of distance

1.  If below this velocity, command max left roll acceleration until: Actual roll velocity = Max_roll_velocity(max_accel, roll_angle)
		-I.e. any time rolled right and rolling right, or rolled right and rolling slowly enough left

2.  When condition met, max right roll acceleration until at zero angle



*/


/*

Right front moments:

Roll moment = - Right Front Thrust (N) * 0.23m (y dist to CG) * cos(servo angle) + 0.0133 * Right Front Thrust (N) * sin(servo angle)
		[Prop Torque term is positive if spinning CW as viewed from top - will roll right if increase thrust]

Pitch moment = Right Front Thrust (N) * 0.95m (x dist to CG) * cos(servo angle)

Yaw moment = - Right Front Thrust (N) * 0.23m (y dist to CG) * sin(servo angle) - 0.0133 * Right Front Thrust (N) * cos(servo angle)
		[Prop torque term is negative if spinning CW as viewed from top - will yaw CCW if increase thrust]

Z_thrust = Right Front Thrust (N) * cos(servo angle)

X_thrust = Right Front Thrust (N) * sin(servo angle)


Right front fan thrust derivatives (at given servo angle):

	-Roll moment derivative (Nm delta per N thrust): -0.23 Nm * cos(servo angle) + 0.0133 Nm * sin(servo angle)
	-Pitch moment derivative (Nm delta per N thrust): 0.95 Nm * cos(servo angle)
	-Yaw moment derivative (Nm delta per N thrust): - 0.23 Nm * sin(servo angle) - 0.0133 Nm * cos(servo angle)
		-1.003 In-lb /39.3 in. *4.5 N /(1.914 Lbf*4.5N ) = 0.0133 Nm torque per N thrust at 11k rpm for 8*45MR(very close to linear across rpm)


Right front servo angle derivatives (at current angle and given thrust level):
	-Roll moment derivative: (Nm delta per rad servo movement): Thrust (N) * 0.23m * sin(servo angle) + 0.0133 * Thrust(N) * cos(servo angle)
	-Pitch moment derivative: -Thrust (N) * 0.95m * sin(servo angle)
	-Yaw moment derivative: -Thrust (N) * 0.23m * cos(servo angle) + 0.0133 * Thrust (N) * sin(servo angle)

Actual_actuations = [Right_front_thrust   Right_front_angle  Left_front_thrust  etc.]

Actual_moments = [Actual_roll_moment   Actual_pitch_moment   Actual_yaw_moment]
Target_moments = [Target_roll_moment   Target_pitch_moment   Target_yaw_moment]

Deltas_from_target_moment = [ Target_roll_moment - Actual_roll_moment    etc. ]


Moment derivatives (function of all servo angles, thrust levels.  In Nm per N thrust or Nm per rad servo angle) = 

[Front right fan roll derivative    Front right fan pitch derivative   Front right fan yaw derivative
Front right servo roll derivative    Front right servo pitch derivative   Front right servo yaw derivative
Front left fan roll derivative       etc.
etc.
]

=
In hover:
[-0.23  0.95  -0.0133;  (front right fan)
 0.133  0     -2.3;     (front right servo)
 0.23   0.95  0.0133;   (front left fan)
 -0.133 0     2.3;      (front left servo)
 23
 23
 23
 23
 23
 23
 23 
 0.35   0     6.1;     (back left servo)
 ]

 Control_deltas = [Right_front_delta_thrust  Right_front_delta_angle  Left_front_delta_thrust  etc.]


Moment_derivatives * Control_deltas = Deltas_from_target_moment

->

Control_deltas = Deltas_from_target_moment / Moment derivatives

Control = Actual_actuations + Control_deltas



-Transmitter controls z_thrust/x_thrust diagonal
-Dial 1 controls direction of z_thrust/x_thrust
-Others control angle targets

Max accelerations hard-coded (to maybe ~2 rad/s^2 for all axes to start)

	Always apply max acceleration in direction need to go until going too fast back towards target
	Max roll velocity = -2 * 0.5*Max Roll Accel *sqrt(roll angle / (0.5 * Max Roll Accel))


*/


/*

Fit to APC Prop data (and scaled from In-Lbs to N-m):
Torque (Nm) = (RPM / 32130)^2
https://www.desmos.com/calculator/l8om6idpub

1 rad/s = 9.549 RPM -> RPM = rad/s * 9.549
Aero Torque (Nm) = ([Rad/s] / 3365)^2

Thrust (Lbs) = (RPM / 7950)^2 (on same Desmos graph)
Thrust (N) = (sqrt(4.448) * [Rad/s] * 9.549 / 7950)^2 = ([rad/s]/395)^2


8 * 4.5 MR Propeller weight (without shaft adapter/coupler ring): 8.78 grams
	-If weight uniformly distributed across 8 inches (0.203 m.): (may slightly less since seems like more weight towards center, but probably close)
		->0.00003015 kg*m^2 (1/12 * m * L^2) (30.15 kg*mm^2)

Collet and adapter ring: 3.78 grams, 10mm diameter
	-If weight uniformly distributed as a 10mm diameter cylinder (seems ok since cut out in middle, but also tapering in at top)
		-> 0.00000004725 kg*m^2 (1/2 * m * r^2) (0.04725 kg*mm^2)

D2826-1400kv: 57.40 grams (with wires and male bullet connectors and bolted-in cross mount on  bottom)
	Rotor on taken apart motor (same diameter - 27.8mm): 33.46 grams
		-Cylindrical section length: 22.8 mm
	-15.0mm cylindrical section length
		-> Likely 15.0/22.8 * larger rotor (or slightly more since some overhead from aluminum going to center)
				-> would be 22.01 grams
		-If 23 grams and modeled as a shell with radius = 12mm (about at inside surface of magnets):
			-> 0.000003312 kg*m^2 (m * r^2) (3.312 kg*mm^2)

		Prob ~34 kg*mm^2 total inertia



Allows calculation of motor velocity at every time step if motor torque known:

	Motor acceleration (Rad/s^2) = (Motor torque [Nm] - Aero torque [Nm]) / Motor and prop moment of inertia [kg*m^2]

	motor_acceleration [rad/s^2] = (motor_torque [Nm] - ((motor_velocity [rad/s] / 3365)^2 ))  /  Motor and prop moment of inertia [kg*m^2]


Steady_state_commanded_rad_per_sec  = 4.448 * ([PWM - 1050] / 590) ^ 1.61

Commanded_thrust_increase_per_sec [N/sec] = 

Excess_motor_torque [Nm] = 

14 magnets -> 7 rotor poles/12 stator teeth -> 6 stator poles



Steady-state motor torque as a function of pwm (may not just be steady state):
	Thrust (N) = 4.448 * [((PWM - 1020)/ 590) ^ 1.61]
	Aero Torque (Nm) = 0.01378 * 4.448 * [((PWM - 1020)/ 590) ^ 1.61]
		-PWM = 1950 us -> 0.1275 Nm
			-3,750 rad/s^2 at 34 kg*mm^2 inertia (35,800 RPM / sec)

Net Voltage (V) = Batt_voltage - Motor_kv * RPM
	-1928 us at 2.00 lbs at 12.6 V (1400 kv)
	-11,240 RPM from APC data (at 2.00 lbs)
	-8.028 V back EMF at 11,240 RPM (1400 kv)
	-4.572 V net (12.6 V batt)
		-> 59.37 Amps at 77 mOhm if let get to steady state
		-> 304,800 Amps/sec at 15 uH
	-0.1224 Nm (11,240 RPM or 2.00 lbs)
		-> 0.0267 Nm/net V is max PWM (property of motor/motor controller?)

	If true:
		-1500 us -> 0.717 lbs (experimental from load cell at 12.6V) (52.8% of full pulse width)
		-6,730 RPM (APC data at 0.717 lbs)
		-4.807 back EMF at 6,730 RPM
		-7.793 V net (12.6 V batt) 
		-0.0439 Nm (0.717 lbs)
			-> 0.00563 Nm/net V (0.2108x 1928 us)



Good documentation on hobby-grade motor controllers:
https://www.ti.com/lit/an/slyt692/slyt692.pdf?ts=1615218851514

Inductance meter shows:
	-14-15 uH on each coil
	-77-78 mOhm on each coil -> 151.2 amps if left in steady state at 11.8 V
	-> 15/1,000,000 = Batt_voltage / (Coil current / s)
		-> 786,666 Amps/sec -> 0.048 ms to go from -19A to 19A




*/

/*

1400kv, 35A: 992-> 1440

0.004 lbs.  PWM = 992
9945304
0.004 lbs.39580
0.004 lbs.133984
0.009 lbs.228404
0.044 lbs.322820
0.249 lbs.417220
0.507 lbs.511616
0.574 lbs.606024
0.580 lbs.700420
0.579 lbs.794828
0.580 lbs.889236
0.580 lbs.983648
0.579 lbs.1078056
0.579 lbs.1172436
0.580 lbs.1266820

Same thing:
9827624
0.004 lbs.  PWM = 992
9921964
0.004 lbs.16228
0.004 lbs.110584
0.008 lbs.204932
0.038 lbs.299304
0.225 lbs.393672
0.494 lbs.488016
0.574 lbs.582364
0.581 lbs.676716
0.581 lbs.771088
0.581 lbs.865472
0.579 lbs.959868
0.579 lbs.1054272
0.578 lbs.1148636
0.578 lbs.1243028
0.579 lbs.1337392
0.578 lbs.1431772
0.578 lbs.1526156
0.579 lbs.1620548
0.577 lbs.1714924


Same thing:

9840140
0.007 lbs.  PWM = 992
9934524
0.006 lbs.28804
0.006 lbs.123192
0.010 lbs.217596
0.038 lbs.311980
0.218 lbs.406364
0.490 lbs.500740
0.575 lbs.595128
0.584 lbs.689512
0.585 lbs.783928
0.586 lbs.878316
0.585 lbs.972688
0.584 lbs.1067096
0.584 lbs.1161476
0.584 lbs.1255868
0.584 lbs.1350248
0.583 lbs.1444636
0.583 lbs.1539016
0.583 lbs.1633420


Now 1760 us:

9907060
0.006 lbs.  PWM = 992
1428
0.006 lbs.95712
0.006 lbs.190080
0.010 lbs.284432
0.050 lbs.378796
0.443 lbs.473156
1.205 lbs.567524
1.491 lbs.661936
1.522 lbs.756296
1.522 lbs.850680
1.522 lbs.945052
1.519 lbs.1039456
1.519 lbs.1133836
1.519 lbs.1228232
1.515 lbs.1322600
1.511 lbs.1416980
1.507 lbs.1511348



Also 1760 us:

9848372
0.011 lbs.  PWM = 992
9942740
0.011 lbs.37028
0.011 lbs.131404
0.015 lbs.225780
0.049 lbs.320168
0.414 lbs.414552
1.183 lbs.508960
1.490 lbs.603324
1.523 lbs.697708
1.522 lbs.792120
1.520 lbs.886512
1.517 lbs.980888
1.514 lbs.1075272
1.511 lbs.1169636
1.508 lbs.1264016
1.505 lbs.1358400
1.503 lbs.1452772
1.501 lbs.1547132
1.499 lbs.1641496
1.497 lbs.1735872
1.496 lbs.1830240
1.497 lbs.1924624
1.496 lbs.2019012
1.493 lbs.2113400
1.492 lbs.2207780
1.490 lbs.2302152
1.488 lbs.2396528
1.490 lbs.2490920
1.491 lbs.2585296
1.491 lbs.2679688
1.488 lbs.2774076
1.485 lbs.2868452
1.483 lbs.2962848
1.483 lbs.3057224
1.484 lbs.3151600
1.488 lbs.3245980
1.489 lbs.3340344
1.488 lbs.3434696
1.484 lbs.3529052
1.481 lbs.3623424
1.485 lbs.3717788
1.484 lbs.3812176
1.480 lbs.3906548
1.479 lbs.4000924
1.481 lbs.4095316
1.475 lbs.4189700
1.473 lbs.4284092
1.476 lbs.4378468
1.474 lbs.4472848
1.473 lbs.4567240
1.474 lbs.4661604
1.473 lbs.4756012
1.476 lbs.4850380
1.478 lbs.4944744



Another 1760 us:

0.011 lbs.  PWM = 992
9820332
0.011 lbs.  PWM = 992
9914716
0.010 lbs.8968
0.011 lbs.103372
0.015 lbs.197752
0.049 lbs.292124
0.411 lbs.386524 //(0.401 lbs. with offset -> 5,034 RPM)
1.177 lbs.480920 //(1.166 lbs. -> 8,585 RPM.)  3,551 RPM increase in 94,396 us -> 37,620 RPM/sec -> 0.1339 Nm avg excess torque
1.486 lbs.575300 //(1.475 lbs. -> 9,655 RPM.)  1,070 RPM increase in -> 94,380 -> 11,340 RPM/sec -> 0.0404 Nm avg excess torque
1.521 lbs.669688 //0.0932 Nm steady state required
1.522 lbs.764072 
1.519 lbs.858432
1.517 lbs.952808   //1.50 lbs -> 9,736 RPM (sqrt(1.5) * 7950), 6.954 V back EMF, 5.646 V net -> 0.1235 Hp = 92.09 W aero/shaft power
1.516 lbs.1047180  //On 6*4E propeller: 0.591 lbs -> 11,000.0 RPM, 4.743 V net.  0.046 Hp -> 34.3 W aero/shaft power. 
1.514 lbs.1141556
1.511 lbs.1235940
1.510 lbs.1330300
1.511 lbs.1424668
1.510 lbs.1519048
1.505 lbs.1613420
1.496 lbs.1707796
1.493 lbs.1802184
1.495 lbs.1896560
1.495 lbs.1990936
1.494 lbs.2085320
1.493 lbs.2179708
1.493 lbs.2274112
1.489 lbs.2368500
1.486 lbs.2462876
1.485 lbs.2557252
1.481 lbs.2651640
1.481 lbs.2746004
1.484 lbs.2840380
1.485 lbs.2934768
1.478 lbs.3029144
1.474 lbs.3123500
1.475 lbs.3217872
1.475 lbs.3312248
1.476 lbs.3406624
1.479 lbs.3501004
1.482 lbs.3595380
1.483 lbs.3689760
1.483 lbs.3784140
1.477 lbs.3878508
1.470 lbs.3972880
1.461 lbs.4067260
1.456 lbs.4161624
1.452 lbs.4256004
1.450 lbs.4350380
1.455 lbs.4444760
1.461 lbs.4539140
1.466 lbs.4633520
1.473 lbs.4727908
1.483 lbs.4822280
1.486 lbs.4916652
1.477 lbs.  PWM = 992


2,016 us:

0.010 lbs.  PWM = 992
9902520
0.011 lbs.  PWM = 992
9996936
0.010 lbs.91196
0.010 lbs.185576
0.013 lbs.279988
0.045 lbs.374372
0.444 lbs.468752 (0.434 lbs. with offset -> 5,237 RPM)
1.543 lbs.563160 (1.533 lbs. -> 9,843 RPM) -> 94,408 us to increase 4,606 RPM (48,790 RPM/sec -> 5,109 rad/s^2.  -> 0.1737 Nm avg. excess torque)
2.107 lbs.657576 (2.097 lbs. -> 11,512 RPM) ->94,416 us to increase 1,669 RPM (17,680 RPM/sec -> 0.0629 Nm avg. excess torque)
2.168 lbs.751992 //At 11,512 -> 8.223 V back EMF -> 4.377 V net  13,530 RPM -> 9.664 V back EMF -> 2.936 V net 0.088. 148.4 W aero power.  230.6 W input power.  64.3% efficiency.
2.166 lbs.846396 //Smaller one at max power: 0.895 lbs, 13,530 RPM (9.664 V back EMF -> 2.936 V net), 64.85 W aero power, 108.36 W input power 59.9% efficiency
2.160 lbs.940812  //0.1324 Nm.  (0.4499x motor max power at 11,512 vs 13,530 RPM for 12.6V input)
2.155 lbs.1035220
2.152 lbs.1129624 //0.1318 Nm torque required to maintain steady state at 2.15 lbs.
2.144 lbs.1224012
2.136 lbs.1318432
2.131 lbs.1412812
2.129 lbs.1507192
2.125 lbs.1601584
2.122 lbs.1695964
2.118 lbs.1790364
2.117 lbs.1884768
2.113 lbs.1979172
2.113 lbs.2073580
2.112 lbs.2167964
2.108 lbs.2262328
2.107 lbs.2356720
2.107 lbs.2451104
2.104 lbs.2545492
2.100 lbs.2639884
2.100 lbs.2734276
2.100 lbs.2828644
2.101 lbs.2923052
2.101 lbs.3017452
2.087 lbs.3111824
2.078 lbs.3206208
2.089 lbs.3300592
2.087 lbs.3394992
2.072 lbs.3489368
2.065 lbs.3583748
2.069 lbs.3678124
2.067 lbs.3772508
2.061 lbs.3866872
2.063 lbs.3961252
2.061 lbs.4055616
2.054 lbs.4149988
2.051 lbs.4244376
2.060 lbs.4338756
2.068 lbs.4433140
2.076 lbs.4527548
2.076 lbs.4621944
2.072 lbs.4716352
2.065 lbs.4810724
2.061 lbs.4905124
2.064 lbs.4999484
2.068 lbs.  PWM = 992
5093992
2.044 lbs.  PWM = 992
5188368
1.510 lbs.  PWM = 992
5282756
0.350 lbs.  PWM = 992
5377148
0.021 lbs.  PWM = 992
5471552
0.010 lbs.  PWM = 992
5565912
0.011 lbs.  PWM = 992
5660316
0.011 lbs.  PWM = 992
5754676
0.011 lbs.  PWM = 992




Also 2016 us:

0.013 lbs.  PWM = 992
9828080
0.012 lbs.  PWM = 992
9922452
0.012 lbs.16712
0.013 lbs.111092
0.016 lbs.205464
0.046 lbs.299832
0.435 lbs.394204
1.525 lbs.488576
2.092 lbs.582920
2.153 lbs.677332
2.151 lbs.771708
2.146 lbs.866080
2.143 lbs.960452
2.137 lbs.1054812
2.130 lbs.1149184
2.124 lbs.1243596
2.120 lbs.1337968
2.116 lbs.1432352
2.113 lbs.1526756
2.112 lbs.1621124
2.111 lbs.1715496
2.106 lbs.1809852
2.099 lbs.1904224
2.093 lbs.1998584
2.089 lbs.2092956
2.089 lbs.2187324
2.095 lbs.2281700
2.103 lbs.2376092
2.111 lbs.2470508
2.106 lbs.2564896
2.099 lbs.2659300
2.098 lbs.2753672
2.101 lbs.2848044
2.104 lbs.2942424
2.107 lbs.3036800
2.097 lbs.3131192
2.092 lbs.3225568
2.097 lbs.3319952
2.094 lbs.3414336
2.091 lbs.3508740
2.083 lbs.3603124
2.085 lbs.3697520
2.088 lbs.3791904
2.081 lbs.3886292
2.072 lbs.3980652
2.068 lbs.4075032
2.073 lbs.4169408
2.084 lbs.4263780
2.097 lbs.4358172
2.102 lbs.4452572
2.101 lbs.4546952
2.097 lbs.4641324
2.098 lbs.4735708
2.100 lbs.4830088
2.099 lbs.4924464
2.100 lbs.  PWM = 992
5018952
2.082 lbs.  PWM = 992
5113352
1.555 lbs.  PWM = 992
5207740
0.374 lbs.  PWM = 992
5302120
0.024 lbs.  PWM = 992
5396504
0.012 lbs.  PWM = 992
5490896
0.012 lbs.  PWM = 992
5585276
0.012 lbs.  PWM = 992
5679672
0.012 lbs.  PWM = 992
5774068
0.013 lbs.  PWM = 992
5868452
0.014 lbs.  PWM = 992



One more 2016 us:

9896592
0.013 lbs.  PWM = 992
9990976
0.013 lbs.85236 //Command sent directly before printing "85236" was to go to 2016 us
0.013 lbs.179616
0.017 lbs.274000
0.051 lbs.368392 //Startup sequence has low acceleration
0.476 lbs.462788
1.571 lbs.557172
2.100 lbs.651536
2.157 lbs.745892
2.154 lbs.840244
2.147 lbs.934592
2.142 lbs.1028932
2.142 lbs.1123276
2.138 lbs.1217632
2.131 lbs.1311988
2.122 lbs.1406352
2.115 lbs.1500748
2.115 lbs.1595116
2.115 lbs.1689496
2.116 lbs.1783868
2.113 lbs.1878236
2.109 lbs.1972620
2.106 lbs.2067004
2.101 lbs.2161392
2.103 lbs.2255792
2.115 lbs.2350184
2.121 lbs.2444576
2.114 lbs.2538976
2.103 lbs.2633364
2.096 lbs.2727764
2.092 lbs.2822148
2.091 lbs.2916556
2.097 lbs.3010940
2.094 lbs.3105316
2.084 lbs.3199688
2.080 lbs.3294056
2.077 lbs.3388464
2.071 lbs.3482860
2.070 lbs.3577272
2.072 lbs.3671704
2.077 lbs.3766100
2.079 lbs.3860496
2.083 lbs.3954888
2.087 lbs.4049292
2.084 lbs.4143700
2.086 lbs.4238092
2.088 lbs.4332488
2.079 lbs.4426884
2.072 lbs.4521248
2.064 lbs.4615612
2.058 lbs.4709984
2.056 lbs.4804360
2.060 lbs.4898728
2.066 lbs.4993088
2.069 lbs.  PWM = 992
5087560
2.043 lbs.  PWM = 992
5181940
1.493 lbs.  PWM = 992
5276300
0.338 lbs.  PWM = 992
5370660
0.021 lbs.  PWM = 992
5465032
0.011 lbs.  PWM = 992
5559424
0.012 lbs.  PWM = 992
5653808
0.012 lbs.  PWM = 992
5748176
0.012 lbs.  PWM = 992
5842580
0.013 lbs.  PWM = 992
5936964
0.014 lbs.  PWM = 992
6031344
0.013 lbs.  PWM = 992





NEW CODE:

1400kv, 35A: 992-> 1440


PWM = 992   
PWM = 992   
PWM = 992   
PWM = 992   
 
-0.004 lbs.1.00 
-0.004 lbs.2.00 
-0.004 lbs.96.00 
0.000 lbs.190.00 
0.034 lbs.285.00 
0.240 lbs.379.00 
0.504 lbs.473.00 
0.574 lbs.568.00 
0.580 lbs.662.00 
0.580 lbs.757.00 
0.580 lbs.851.00 
0.579 lbs.945.00 
0.579 lbs.1039.00 
0.579 lbs.1133.00 
0.581 lbs.1228.00 
0.582 lbs.1323.00 
0.582 lbs.1417.00 
0.580 lbs.1511.00 
0.580 lbs.1606.00 
0.580 lbs.1700.00 
0.579 lbs.1794.00 
0.578 lbs.1888.00 
0.578 lbs.1982.00 
0.576 lbs.2078.00 
0.574 lbs.2172.00 
0.575 lbs.2266.00 
0.575 lbs.2360.00 
0.575 lbs.2454.00 
0.574 lbs.2549.00 
0.575 lbs.2643.00 
0.576 lbs.2737.00 
0.577 lbs.2832.00 
0.576 lbs.2927.00 
0.575 lbs.3021.00 
0.575 lbs.3115.00 
0.577 lbs.3209.00 
0.579 lbs.3303.00 
0.579 lbs.3398.00 
0.578 lbs.3493.00 
0.576 lbs.3587.00 
0.575 lbs.3681.00 
0.574 lbs.3775.00 
0.574 lbs.3870.00 
0.574 lbs.3964.00 
0.574 lbs.4058.00 
0.572 lbs.4153.00 
0.572 lbs.4247.00 
0.573 lbs.4342.00 
0.574 lbs.4436.00 
0.574 lbs.4530.00 
0.575 lbs.4624.00 
0.575 lbs.4719.00 
0.578 lbs.4813.00 
0.577 lbs.4908.00 
0.575 lbs.5002.00PWM = 992   
0.567 lbs.5096.00PWM = 992   
0.403 lbs.5191.00PWM = 992   
0.076 lbs.5285.00PWM = 992   
-0.003 lbs.5379.00PWM = 992   
-0.005 lbs.5473.00PWM = 992   
-0.005 lbs.5567.00PWM = 992   
-0.005 lbs.5663.00PWM = 992   
-0.005 lbs.5757.00PWM = 992   
-0.005 lbs.5851.00PWM = 992   
-0.005 lbs.5945.00PWM = 992   
-0.004 lbs.6039.00PWM = 992



Same thing (1440 us):

PWM = 992   
PWM = 992   
PWM = 992   
 
-0.004 lbs.0.00 
-0.004 lbs.43.00 
-0.004 lbs.137.00 
0.006 lbs.232.00 
0.086 lbs.327.00 
0.363 lbs.421.00 
0.551 lbs.515.00 
0.582 lbs.609.00 
0.583 lbs.703.00 
0.582 lbs.798.00 
0.582 lbs.892.00 
0.583 lbs.986.00 
0.583 lbs.1081.00 
0.582 lbs.1175.00 
0.581 lbs.1270.00 
0.581 lbs.1364.00 
0.582 lbs.1458.00 
0.582 lbs.1552.00 
0.581 lbs.1647.00 
0.579 lbs.1741.00 
0.577 lbs.1836.00 
0.578 lbs.1930.00 
0.579 lbs.2024.00 
0.579 lbs.2119.00 
0.578 lbs.2213.00 
0.577 lbs.2307.00 
0.577 lbs.2401.00 
0.576 lbs.2495.00 
0.576 lbs.2591.00 
0.576 lbs.2685.00 
0.577 lbs.2779.00 
0.577 lbs.2873.00 
0.576 lbs.2967.00 
0.576 lbs.3062.00 
0.577 lbs.3156.00 
0.576 lbs.3250.00 
0.576 lbs.3344.00 
0.576 lbs.3440.00 
0.574 lbs.3534.00 
0.571 lbs.3628.00 
0.572 lbs.3722.00 
0.577 lbs.3816.00 
0.577 lbs.3911.00 
0.574 lbs.4005.00 
0.575 lbs.4099.00 
0.575 lbs.4194.00 
0.576 lbs.4288.00 
0.574 lbs.4383.00 
0.573 lbs.4477.00 
0.573 lbs.4571.00 
0.572 lbs.4665.00 
0.571 lbs.4759.00 
0.570 lbs.4854.00 
0.571 lbs.4949.00 
0.570 lbs.5043.00PWM = 992   
0.563 lbs.5137.00PWM = 992   
0.397 lbs.5232.00PWM = 992   
0.073 lbs.5326.00PWM = 992   
-0.004 lbs.5420.00PWM = 992   
-0.005 lbs.5514.00PWM = 992   
-0.005 lbs.5608.00PWM = 992   
-0.004 lbs.5703.00PWM = 992   
-0.004 lbs.5798.00PWM = 992   
-0.004 lbs.5892.00PWM = 992   
-0.004 lbs.5986.00PWM = 992 



1440 us:

PWM = 992   
PWM = 992   
PWM = 992   
 
-0.003 lbs.0.00 
-0.003 lbs.33.00 
-0.003 lbs.127.00 
0.006 lbs.221.00 
0.076 lbs.315.00 
0.345 lbs.411.00 
0.545 lbs.505.00 
0.582 lbs.599.00 
0.585 lbs.693.00 
0.584 lbs.787.00 
0.584 lbs.882.00 
0.584 lbs.976.00 
0.583 lbs.1071.00 
0.583 lbs.1165.00 
0.582 lbs.1259.00 
0.581 lbs.1354.00 
0.581 lbs.1448.00 
0.580 lbs.1542.00 
0.579 lbs.1636.00 
0.578 lbs.1732.00 
0.578 lbs.1826.00 
0.579 lbs.1920.00 
0.580 lbs.2014.00 
0.580 lbs.2108.00 
0.580 lbs.2203.00 
0.580 lbs.2297.00 
0.579 lbs.2391.00 
0.578 lbs.2485.00 
0.577 lbs.2580.00 
0.577 lbs.2675.00 
0.577 lbs.2769.00 
0.576 lbs.2863.00 
0.576 lbs.2957.00 
0.575 lbs.3051.00 
0.575 lbs.3146.00 
0.577 lbs.3241.00 
0.578 lbs.3335.00 
0.578 lbs.3429.00 
0.578 lbs.3524.00 
0.578 lbs.3618.00 
0.578 lbs.3712.00 
0.577 lbs.3806.00 
0.576 lbs.3900.00 
0.577 lbs.3995.00 
0.575 lbs.4090.00 
0.571 lbs.4184.00 
0.571 lbs.4278.00 
0.571 lbs.4372.00 
0.574 lbs.4467.00 
0.576 lbs.4561.00 
0.576 lbs.4655.00 
0.575 lbs.4749.00 
0.574 lbs.4844.00 
0.574 lbs.4939.00 
0.574 lbs.5033.00PWM = 992   
0.569 lbs.5127.00PWM = 992   
0.407 lbs.5221.00PWM = 992   
0.079 lbs.5316.00PWM = 992   
-0.002 lbs.5410.00PWM = 992 




1760 us:

PWM = 992   
PWM = 992   
PWM = 992   
 
-0.004 lbs.0.00 
-0.004 lbs.82.00 
-0.002 lbs.176.00 
0.025 lbs.270.00 
0.333 lbs.364.00 
1.118 lbs.460.00 
1.487 lbs.554.00 
1.530 lbs.648.00 
1.531 lbs.742.00 
1.529 lbs.837.00 
1.528 lbs.931.00 
1.527 lbs.1025.00 
1.526 lbs.1120.00 
1.527 lbs.1214.00 
1.528 lbs.1309.00 
1.524 lbs.1403.00 
1.518 lbs.1497.00 
1.516 lbs.1591.00 
1.513 lbs.1685.00 
1.510 lbs.1780.00 
1.509 lbs.1875.00 
1.509 lbs.1969.00 
1.507 lbs.2063.00 
1.506 lbs.2157.00 
1.506 lbs.2252.00 
1.508 lbs.2346.00 
1.510 lbs.2440.00 
1.505 lbs.2534.00 
1.501 lbs.2630.00 
1.501 lbs.2724.00 
1.496 lbs.2818.00 
1.495 lbs.2912.00 
1.496 lbs.3006.00 
1.496 lbs.3101.00 
1.498 lbs.3195.00 
1.505 lbs.3290.00 
1.507 lbs.3384.00 
1.501 lbs.3478.00 
1.497 lbs.3573.00 
1.495 lbs.3667.00 
1.491 lbs.3761.00 
1.488 lbs.3855.00 
1.485 lbs.3951.00 
1.484 lbs.4045.00 
1.483 lbs.4139.00 
1.496 lbs.4233.00 
1.508 lbs.4327.00 
1.510 lbs.4422.00 
1.506 lbs.4516.00 
1.504 lbs.4610.00 
1.498 lbs.4705.00 
1.493 lbs.4799.00 
1.492 lbs.4894.00 
1.492 lbs.4988.00 
1.487 lbs.5082.00PWM = 992   
1.468 lbs.5176.00PWM = 992   
1.071 lbs.5270.00PWM = 992   
0.230 lbs.5366.00PWM = 992   
0.003 lbs.5460.00PWM = 992   
-0.003 lbs.5554.00PWM = 992   
-0.003 lbs.5648.00PWM = 992   
-0.003 lbs.5743.00PWM = 992   
-0.002 lbs.5837.00PWM = 992   
-0.001 lbs.5931.00PWM = 992   
-0.001 lbs.6026.00PWM = 992




PWM = 992   
PWM = 992   
PWM = 992   
 
0.001 lbs.1.00 
0.001 lbs.70.00 
0.003 lbs.164.00 
0.023 lbs.258.00 
0.260 lbs.353.00 
1.024 lbs.447.00 
1.473 lbs.542.00 
1.536 lbs.636.00 
1.538 lbs.730.00 
1.536 lbs.824.00 
1.534 lbs.918.00 
1.532 lbs.1014.00 
1.529 lbs.1108.00 
1.529 lbs.1202.00 
1.530 lbs.1296.00 
1.528 lbs.1391.00 
1.524 lbs.1485.00 
1.521 lbs.1580.00 
1.518 lbs.1674.00 
1.514 lbs.1768.00 
1.509 lbs.1863.00 
1.507 lbs.1957.00 
1.508 lbs.2051.00 
1.507 lbs.2145.00 
1.503 lbs.2240.00 
1.499 lbs.2335.00 
1.498 lbs.2429.00 
1.499 lbs.2523.00 
1.503 lbs.2617.00 
1.502 lbs.2711.00 
1.502 lbs.2806.00 
1.505 lbs.2901.00 
1.507 lbs.2995.00 
1.499 lbs.3089.00 
1.498 lbs.3184.00 
1.499 lbs.3278.00 
1.500 lbs.3372.00 
1.498 lbs.3466.00 
1.492 lbs.3561.00 
1.482 lbs.3656.00 
1.471 lbs.3750.00 
1.467 lbs.3844.00 
1.473 lbs.3938.00 
1.476 lbs.4032.00 
1.469 lbs.4127.00 
1.470 lbs.4221.00 
1.475 lbs.4316.00 
1.482 lbs.4410.00 
1.488 lbs.4504.00 
1.491 lbs.4599.00 
1.490 lbs.4693.00 
1.488 lbs.4787.00 
1.480 lbs.4881.00 
1.482 lbs.4977.00 
1.482 lbs.5071.00PWM = 992   
1.465 lbs.5165.00PWM = 992   
1.040 lbs.5259.00PWM = 992   
0.211 lbs.5353.00PWM = 992   
0.005 lbs.5448.00PWM = 992   
0.000 lbs.5542.00PWM = 992   
0.001 lbs.5636.00PWM = 992  


PWM = 992   
PWM = 992   
PWM = 992   
 
0.002 lbs.0.00 
0.002 lbs.80.00 
0.005 lbs.175.00 
0.030 lbs.269.00 
0.322 lbs.363.00 
1.098 lbs.458.00 
1.484 lbs.552.00 
1.534 lbs.646.00 
1.536 lbs.740.00 
1.532 lbs.834.00 
1.530 lbs.930.00 
1.527 lbs.1024.00 
1.523 lbs.1118.00 
1.522 lbs.1212.00 
1.518 lbs.1307.00 
1.516 lbs.1401.00 
1.516 lbs.1495.00 
1.515 lbs.1590.00 
1.513 lbs.1684.00 
1.510 lbs.1779.00 
1.507 lbs.1873.00 
1.506 lbs.1967.00 
1.503 lbs.2061.00 
1.500 lbs.2155.00 
1.501 lbs.2250.00 
1.501 lbs.2345.00 
1.499 lbs.2439.00 
1.497 lbs.2533.00 
1.497 lbs.2628.00 
1.501 lbs.2722.00 
1.505 lbs.2816.00 
1.510 lbs.2910.00 
1.516 lbs.3004.00 
1.510 lbs.3099.00 
1.497 lbs.3194.00 
1.490 lbs.3288.00 
1.487 lbs.3382.00 
1.482 lbs.3476.00 
1.475 lbs.3571.00 
1.471 lbs.3665.00 
1.467 lbs.3759.00 
1.472 lbs.3853.00 
1.488 lbs.3947.00 
1.503 lbs.4043.00 
1.507 lbs.4137.00 
1.504 lbs.4231.00 
1.503 lbs.4325.00 
1.500 lbs.4420.00 
1.497 lbs.4514.00 
1.503 lbs.4608.00 
1.504 lbs.4702.00 
1.501 lbs.4796.00 
1.502 lbs.4892.00 
1.503 lbs.4986.00 
1.502 lbs.5080.00PWM = 992   
1.489 lbs.5174.00PWM = 992   
1.105 lbs.5268.00PWM = 992   
0.250 lbs.5363.00PWM = 992 


//Adding 30 ms delay to each loop after start:

PWM = 992   
PWM = 992   
PWM = 992   
 
-0.001 lbs.31.00 
-0.001 lbs.61.00 
-0.000 lbs.128.00 
0.009 lbs.222.00 
0.103 lbs.316.00 
0.703 lbs.411.00 
1.373 lbs.505.00 
1.526 lbs.599.00 
1.536 lbs.693.00 
1.534 lbs.788.00 
1.531 lbs.883.00 
1.530 lbs.977.00 
1.529 lbs.1071.00 
1.527 lbs.1165.00 
1.525 lbs.1259.00 
1.523 lbs.1354.00 
1.521 lbs.1449.00 
1.519 lbs.1543.00 
1.514 lbs.1637.00 
1.510 lbs.1731.00 
1.510 lbs.1826.00 
1.511 lbs.1920.00 
1.514 lbs.2015.00 
1.512 lbs.2109.00 
1.507 lbs.2204.00 
1.506 lbs.2298.00 
1.509 lbs.2392.00 
1.510 lbs.2486.00 
1.512 lbs.2580.00 
1.521 lbs.2676.00 
1.527 lbs.2770.00 
1.525 lbs.2864.00 
1.518 lbs.2958.00 
1.511 lbs.3052.00 
1.502 lbs.3147.00 
1.492 lbs.3241.00 
1.488 lbs.3336.00 
1.495 lbs.3430.00 
1.499 lbs.3525.00 
1.491 lbs.3619.00 
1.487 lbs.3713.00 
1.485 lbs.3807.00 
1.482 lbs.3901.00 
1.479 lbs.3997.00 
1.483 lbs.4091.00 
1.481 lbs.4185.00 
1.485 lbs.4279.00 
1.486 lbs.4373.00 
1.497 lbs.4468.00 
1.501 lbs.4562.00 
1.492 lbs.4657.00 
1.484 lbs.4751.00 
1.482 lbs.4845.00 
1.480 lbs.4940.00 
1.482 lbs.5034.00PWM = 992   
1.467 lbs.5128.00PWM = 992   
1.076 lbs.5222.00PWM = 992   
0.235 lbs.5318.00PWM = 992   
0.005 lbs.5412.00PWM = 992   
-0.001 lbs.5506.00PWM = 992   
-0.001 lbs.5600.00PWM = 992 


Same 30 ms delay:

PWM = 992   
PWM = 992   
PWM = 992   
 
-0.000 lbs.31.00 
-0.000 lbs.73.00 
0.002 lbs.167.00 
0.022 lbs.261.00 
0.275 lbs.355.00 
1.047 lbs.449.00 
1.478 lbs.545.00 
1.534 lbs.639.00 
1.536 lbs.733.00 
1.535 lbs.827.00 
1.534 lbs.922.00 
1.531 lbs.1016.00 
1.529 lbs.1110.00 
1.530 lbs.1205.00 
1.528 lbs.1299.00 
1.525 lbs.1394.00 
1.524 lbs.1488.00 
1.518 lbs.1582.00 
1.516 lbs.1676.00 
1.514 lbs.1770.00 
1.513 lbs.1865.00 
1.513 lbs.1960.00 
1.508 lbs.2054.00 
1.504 lbs.2148.00 
1.503 lbs.2242.00 
1.502 lbs.2337.00 
1.500 lbs.2431.00 
1.499 lbs.2525.00 
1.499 lbs.2619.00 
1.506 lbs.2714.00 
1.510 lbs.2809.00 
1.507 lbs.2903.00 
1.507 lbs.2997.00 
1.506 lbs.3091.00 
1.504 lbs.3186.00 
1.506 lbs.3280.00 
1.504 lbs.3374.00 
1.497 lbs.3468.00 
1.494 lbs.3563.00 
1.497 lbs.3658.00 
1.497 lbs.3752.00 
1.495 lbs.3846.00 
1.492 lbs.3940.00 
1.488 lbs.4034.00 
1.484 lbs.4129.00 
1.487 lbs.4224.00 
1.492 lbs.4318.00 
1.493 lbs.4412.00 
1.491 lbs.4507.00 
1.484 lbs.4601.00 
1.476 lbs.4695.00 
1.481 lbs.4789.00 
1.482 lbs.4883.00 
1.481 lbs.4979.00 
1.483 lbs.5073.00PWM = 992   
1.464 lbs.5167.00PWM = 992   
1.026 lbs.5261.00PWM = 992   
0.201 lbs.5355.00PWM = 992   
0.005 lbs.5450.00PWM = 992   
-0.000 lbs.5544.00PWM = 992   
-0.000 lbs.5638.00PWM = 992   
0.000 lbs.5733.00PWM = 992   



PWM = 992   
PWM = 992   
PWM = 992   
 
0.001 lbs.31.00 
0.001 lbs.87.00 
0.004 lbs.181.00 
0.033 lbs.275.00 
0.361 lbs.370.00 
1.142 lbs.464.00 
1.492 lbs.558.00 
1.533 lbs.652.00 
1.535 lbs.747.00 
1.532 lbs.842.00 
1.531 lbs.936.00 
1.530 lbs.1030.00 
1.527 lbs.1124.00 
1.523 lbs.1218.00 
1.516 lbs.1313.00 
1.515 lbs.1408.00 
1.517 lbs.1502.00 
1.515 lbs.1596.00 
1.512 lbs.1691.00 
1.511 lbs.1785.00 
1.510 lbs.1879.00 
1.506 lbs.1974.00 
1.505 lbs.2068.00 
1.504 lbs.2163.00 
1.500 lbs.2257.00 
1.497 lbs.2351.00 
1.497 lbs.2445.00 
1.498 lbs.2539.00 
1.502 lbs.2635.00 
1.507 lbs.2729.00 
1.507 lbs.2823.00 
1.507 lbs.2917.00 
1.502 lbs.3011.00 
1.496 lbs.3106.00 
1.499 lbs.3200.00 
1.504 lbs.3294.00 
1.506 lbs.3389.00 
1.505 lbs.3484.00 
1.502 lbs.3578.00 
1.501 lbs.3672.00 
1.503 lbs.3766.00 
1.503 lbs.3860.00 
1.505 lbs.3955.00 
1.511 lbs.4049.00 
1.508 lbs.4144.00 
1.500 lbs.4238.00 
1.493 lbs.4332.00 
1.492 lbs.4427.00 
1.493 lbs.4521.00 
1.492 lbs.4615.00 
1.494 lbs.4709.00 
1.497 lbs.4803.00 
1.492 lbs.4899.00 
1.493 lbs.4993.00 
1.495 lbs.5087.00PWM = 992   
1.477 lbs.5181.00PWM = 992   
1.045 lbs.5276.00PWM = 992   
0.211 lbs.5370.00PWM = 992   
0.006 lbs.5464.00PWM = 992   
0.001 lbs.5558.00PWM = 992   
0.001 lbs.5653.00PWM = 992  



With 60 ms delay:

PWM = 992   
PWM = 992   
PWM = 992   
PWM = 992   
 
0.002 lbs.60.00 
0.002 lbs.122.00 
0.006 lbs.193.00 
0.047 lbs.288.00 
0.451 lbs.382.00 
1.221 lbs.476.00 
1.506 lbs.570.00 
1.536 lbs.666.00 
1.537 lbs.760.00 
1.536 lbs.854.00 
1.536 lbs.948.00 
1.534 lbs.1042.00 
1.528 lbs.1137.00 
1.522 lbs.1231.00 
1.519 lbs.1326.00 
1.518 lbs.1420.00 
1.521 lbs.1514.00 
1.525 lbs.1609.00 
1.527 lbs.1703.00 
1.521 lbs.1797.00 
1.514 lbs.1891.00 
1.514 lbs.1985.00 
1.515 lbs.2081.00 
1.512 lbs.2175.00 
1.509 lbs.2269.00 
1.507 lbs.2363.00 
1.504 lbs.2458.00 
1.504 lbs.2552.00 
1.503 lbs.2646.00 
1.501 lbs.2740.00 
1.503 lbs.2834.00 
1.506 lbs.2930.00 
1.506 lbs.3024.00 
1.506 lbs.3118.00 
1.505 lbs.3212.00 
1.503 lbs.3306.00 
1.505 lbs.3401.00 
1.515 lbs.3495.00 
1.523 lbs.3590.00 
1.522 lbs.3684.00 
1.515 lbs.3778.00 
1.510 lbs.3873.00 
1.502 lbs.3967.00 
1.497 lbs.4061.00 
1.493 lbs.4155.00 
1.496 lbs.4251.00 
1.495 lbs.4345.00 
1.496 lbs.4439.00 
1.495 lbs.4533.00 
1.496 lbs.4627.00 
1.500 lbs.4722.00 
1.507 lbs.4817.00 
1.512 lbs.4911.00 
1.511 lbs.5005.00PWM = 992   
1.490 lbs.5099.00PWM = 992   
1.060 lbs.5194.00PWM = 992   
0.219 lbs.5288.00PWM = 992   
0.008 lbs.5382.00PWM = 992   
0.002 lbs.5477.00PWM = 992   
0.002 lbs.5571.00PWM = 992   
0.002 lbs.5666.00PWM = 992   
0.002 lbs.5760.00PWM = 992   
0.002 lbs.5854.00PWM = 992   
0.003 lbs.5948.00PWM = 992   
0.003 lbs.6043.00PWM = 992   
0.003 lbs.6138.00PWM = 992 



With 110 ms delay:

PWM = 992   
PWM = 992   
 
0.003 lbs.111.00 
0.004 lbs.222.00 
0.019 lbs.333.00 
0.924 lbs.444.00 
1.447 lbs.556.00 
1.533 lbs.668.00 
1.538 lbs.779.00 
1.535 lbs.891.00 
1.533 lbs.1001.00 
1.527 lbs.1113.00 
1.524 lbs.1225.00 
1.521 lbs.1336.00 
1.521 lbs.1448.00 
1.522 lbs.1559.00 
1.509 lbs.1671.00 
1.506 lbs.1783.00 
1.508 lbs.1893.00 
1.510 lbs.2005.00 
1.509 lbs.2117.00 
1.509 lbs.2228.00 
1.509 lbs.2340.00 
1.505 lbs.2451.00 
1.504 lbs.2563.00 
1.505 lbs.2675.00 
1.503 lbs.2785.00 
1.501 lbs.2897.00 
1.496 lbs.3008.00 
1.493 lbs.3120.00 
1.496 lbs.3232.00 
1.499 lbs.3343.00 
1.500 lbs.3455.00 
1.495 lbs.3567.00 
1.489 lbs.3678.00 
1.491 lbs.3789.00 
1.495 lbs.3900.00 
1.499 lbs.4012.00 
1.502 lbs.4124.00 
1.500 lbs.4235.00 
1.498 lbs.4347.00 
1.499 lbs.4458.00 
1.499 lbs.4570.00 
1.503 lbs.4682.00 
1.500 lbs.4792.00 
1.496 lbs.4904.00 
1.496 lbs.5015.00PWM = 992   
1.493 lbs.5127.00PWM = 992   
1.385 lbs.5239.00PWM = 992   
0.060 lbs.5350.00PWM = 992   
0.002 lbs.5462.00PWM = 992   
0.002 lbs.5574.00PWM = 992   



For 2016 us, no delays:

PWM = 992   
PWM = 992   
 
0.001 lbs.0.00 
0.001 lbs.8.00 
0.001 lbs.102.00 
0.006 lbs.197.00 
0.053 lbs.291.00 
0.596 lbs.385.00 
1.719 lbs.479.00 
2.153 lbs.574.00 
2.189 lbs.669.00 
2.187 lbs.763.00 
2.184 lbs.857.00 
2.179 lbs.951.00 
2.173 lbs.1045.00 
2.167 lbs.1141.00 
2.163 lbs.1235.00 
2.158 lbs.1329.00 
2.154 lbs.1423.00 
2.149 lbs.1517.00 
2.145 lbs.1612.00 
2.140 lbs.1706.00 
2.136 lbs.1801.00 
2.134 lbs.1895.00 
2.134 lbs.1990.00 
2.136 lbs.2084.00 
2.141 lbs.2178.00 
2.143 lbs.2272.00 
2.144 lbs.2366.00 
2.141 lbs.2462.00 
2.136 lbs.2556.00 
2.132 lbs.2650.00 
2.123 lbs.2744.00 
2.114 lbs.2838.00 
2.104 lbs.2933.00 
2.090 lbs.3027.00 
2.094 lbs.3122.00 
2.109 lbs.3216.00 
2.115 lbs.3311.00 
2.112 lbs.3405.00 
2.117 lbs.3499.00 
2.119 lbs.3593.00 
2.117 lbs.3687.00 
2.112 lbs.3782.00 
2.105 lbs.3877.00 
2.090 lbs.3971.00 
2.079 lbs.4065.00 
2.082 lbs.4159.00 
2.086 lbs.4254.00 
2.088 lbs.4348.00 
2.092 lbs.4442.00 
2.093 lbs.4537.00 
2.096 lbs.4631.00 
2.102 lbs.4726.00 
2.094 lbs.4820.00 
2.090 lbs.4914.00 
2.092 lbs.5008.00PWM = 992   
2.077 lbs.5103.00PWM = 992   
1.553 lbs.5198.00PWM = 992   
0.361 lbs.5292.00PWM = 992   
0.012 lbs.5386.00PWM = 992   
-0.000 lbs.5480.00PWM = 992   
0.000 lbs.5575.00PWM = 992   
0.001 lbs.5669.00PWM = 992   
0.000 lbs.5763.00PWM = 992   
0.000 lbs.5857.00PWM = 992   
0.001 lbs.5952.00PWM = 992   
0.001 lbs.6047.00PWM = 992   
0.002 lbs.6141.00PWM = 992




PWM = 992   
PWM = 992   
PWM = 992   
 
0.004 lbs.1.00 
0.003 lbs.78.00 
0.006 lbs.172.00 
0.029 lbs.267.00 
0.366 lbs.361.00 
1.459 lbs.456.00 
2.106 lbs.550.00 
2.185 lbs.644.00 
2.184 lbs.738.00 
2.178 lbs.832.00 
2.173 lbs.928.00 
2.169 lbs.1022.00 
2.167 lbs.1116.00 
2.161 lbs.1210.00 
2.155 lbs.1304.00 
2.151 lbs.1399.00 
2.148 lbs.1493.00 
2.144 lbs.1588.00 
2.142 lbs.1682.00 
2.142 lbs.1777.00 
2.137 lbs.1871.00 
2.133 lbs.1965.00 
2.131 lbs.2059.00 
2.135 lbs.2153.00 
2.144 lbs.2248.00 
2.140 lbs.2343.00 
2.126 lbs.2437.00 
2.120 lbs.2531.00 
2.122 lbs.2625.00 
2.120 lbs.2720.00 
2.111 lbs.2814.00 
2.094 lbs.2908.00 
2.087 lbs.3003.00 
2.091 lbs.3098.00 
2.088 lbs.3192.00 
2.088 lbs.3286.00 
2.092 lbs.3380.00 
2.079 lbs.3474.00 
2.077 lbs.3569.00 
2.068 lbs.3664.00 
2.058 lbs.3758.00 
2.058 lbs.3852.00 
2.064 lbs.3946.00 
2.064 lbs.4041.00 
2.062 lbs.4135.00 
2.062 lbs.4229.00 
2.071 lbs.4323.00 
2.080 lbs.4418.00 
2.083 lbs.4513.00 
2.090 lbs.4607.00 
2.099 lbs.4701.00 
2.103 lbs.4795.00 
2.105 lbs.4890.00 
2.110 lbs.4984.00 
2.109 lbs.5079.00PWM = 992   
2.084 lbs.5173.00PWM = 992   
1.507 lbs.5267.00PWM = 992   
0.331 lbs.5362.00PWM = 992   
0.015 lbs.5456.00PWM = 992   
0.004 lbs.5550.00PWM = 992   
0.004 lbs.5644.00PWM = 992   
0.004 lbs.5738.00PWM = 992   
0.004 lbs.5834.00PWM = 992   
0.004 lbs.5928.00PWM = 992   
0.005 lbs.6022.00PWM = 992   




PWM = 992   
PWM = 992   
PWM = 992   
 
0.005 lbs.1.00 
0.005 lbs.3.00 
0.005 lbs.97.00 
0.008 lbs.191.00 
0.044 lbs.286.00 
0.490 lbs.380.00 
1.606 lbs.475.00 
2.130 lbs.569.00 
2.184 lbs.663.00 
2.182 lbs.758.00 
2.176 lbs.852.00 
2.169 lbs.946.00 
2.164 lbs.1041.00 
2.158 lbs.1136.00 
2.154 lbs.1230.00 
2.151 lbs.1324.00 
2.148 lbs.1418.00 
2.139 lbs.1512.00 
2.131 lbs.1607.00 
2.130 lbs.1702.00 
2.129 lbs.1796.00 
2.130 lbs.1890.00 
2.131 lbs.1984.00 
2.128 lbs.2079.00 
2.128 lbs.2173.00 
2.130 lbs.2267.00 
2.140 lbs.2361.00 
2.140 lbs.2456.00 
2.129 lbs.2551.00 
2.126 lbs.2645.00 
2.138 lbs.2739.00 
2.151 lbs.2833.00 
2.142 lbs.2928.00 
2.127 lbs.3022.00 
2.114 lbs.3117.00 
2.099 lbs.3211.00 
2.095 lbs.3305.00 
2.090 lbs.3400.00 
2.083 lbs.3494.00 
2.068 lbs.3588.00 
2.056 lbs.3682.00 
2.064 lbs.3776.00 
2.080 lbs.3872.00 
2.096 lbs.3966.00 
2.105 lbs.4060.00 
2.109 lbs.4154.00 
2.105 lbs.4248.00 
2.100 lbs.4343.00 
2.102 lbs.4437.00 
2.109 lbs.4532.00 
2.107 lbs.4626.00 
2.108 lbs.4721.00 
2.116 lbs.4815.00 
2.122 lbs.4909.00 
2.117 lbs.5003.00PWM = 992   
2.092 lbs.5097.00PWM = 992   
1.539 lbs.5192.00PWM = 992   
0.351 lbs.5287.00PWM = 992   
0.016 lbs.5381.00PWM = 992   
0.004 lbs.5475.00PWM = 992   
0.005 lbs.5569.00PWM = 992   
0.005 lbs.5664.00PWM = 992   
0.005 lbs.5758.00PWM = 992   
0.005 lbs.5852.00PWM = 992 



//Adding 30 ms delay:

PWM = 992   
PWM = 992   
PWM = 992   
 
0.003 lbs.31.00 
0.002 lbs.61.00 
0.003 lbs.154.00 
0.019 lbs.249.00 
0.226 lbs.343.00 
1.207 lbs.437.00 
2.036 lbs.531.00 
2.178 lbs.626.00 
2.182 lbs.720.00 
2.176 lbs.814.00 
2.171 lbs.908.00 
2.167 lbs.1002.00 
2.167 lbs.1098.00 
2.167 lbs.1192.00 
2.158 lbs.1286.00 
2.151 lbs.1380.00 
2.146 lbs.1474.00 
2.142 lbs.1569.00 
2.139 lbs.1663.00 
2.136 lbs.1757.00 
2.134 lbs.1851.00 
2.134 lbs.1946.00 
2.136 lbs.2041.00 
2.138 lbs.2135.00 
2.137 lbs.2229.00 
2.134 lbs.2323.00 
2.131 lbs.2418.00 
2.130 lbs.2512.00 
2.131 lbs.2606.00 
2.127 lbs.2700.00 
2.121 lbs.2795.00 
2.119 lbs.2890.00 
2.114 lbs.2984.00 
2.100 lbs.3078.00 
2.087 lbs.3172.00 
2.095 lbs.3266.00 
2.103 lbs.3361.00 
2.104 lbs.3455.00 
2.103 lbs.3550.00 
2.102 lbs.3644.00 
2.095 lbs.3739.00 
2.098 lbs.3833.00 
2.109 lbs.3927.00 
2.112 lbs.4021.00 
2.113 lbs.4115.00 
2.112 lbs.4211.00 
2.111 lbs.4305.00 
2.110 lbs.4399.00 
2.109 lbs.4493.00 
2.104 lbs.4587.00 
2.099 lbs.4682.00 
2.099 lbs.4776.00 
2.102 lbs.4871.00 
2.096 lbs.4965.00 
2.090 lbs.5059.00PWM = 992   
2.071 lbs.5154.00PWM = 992   
1.528 lbs.5248.00PWM = 992   
0.347 lbs.5342.00PWM = 992   
0.013 lbs.5436.00PWM = 992   
0.002 lbs.5531.00PWM = 992   
0.002 lbs.5626.00PWM = 992   
0.003 lbs.5720.00PWM = 992 


//Same 30 ms delay

PWM = 992   
PWM = 992   
PWM = 992   
 
0.006 lbs.31.00 
0.006 lbs.69.00 
0.007 lbs.164.00 
0.025 lbs.258.00 
0.265 lbs.352.00 
1.280 lbs.446.00 
2.055 lbs.541.00 
2.177 lbs.635.00 
2.181 lbs.729.00 
2.176 lbs.824.00 
2.171 lbs.918.00 
2.168 lbs.1013.00 
2.164 lbs.1107.00 
2.157 lbs.1201.00 
2.151 lbs.1295.00 
2.143 lbs.1389.00 
2.135 lbs.1484.00 
2.132 lbs.1579.00 
2.133 lbs.1673.00 
2.131 lbs.1767.00 
2.130 lbs.1862.00 
2.133 lbs.1956.00 
2.133 lbs.2050.00 
2.122 lbs.2144.00 
2.116 lbs.2239.00 
2.115 lbs.2334.00 
2.117 lbs.2428.00 
2.118 lbs.2522.00 
2.109 lbs.2616.00 
2.102 lbs.2710.00 
2.101 lbs.2805.00 
2.101 lbs.2899.00 
2.094 lbs.2994.00 
2.091 lbs.3088.00 
2.085 lbs.3183.00 
2.095 lbs.3277.00 
2.108 lbs.3371.00 
2.106 lbs.3465.00 
2.097 lbs.3559.00 
2.091 lbs.3655.00 
2.087 lbs.3749.00 
2.088 lbs.3843.00 
2.100 lbs.3937.00 
2.111 lbs.4031.00 
2.113 lbs.4126.00 
2.116 lbs.4220.00 
2.118 lbs.4314.00 
2.113 lbs.4409.00 
2.100 lbs.4503.00 
2.090 lbs.4598.00 
2.076 lbs.4692.00 
2.066 lbs.4786.00 
2.066 lbs.4880.00 
2.076 lbs.4975.00 
2.080 lbs.5070.00PWM = 992   
2.056 lbs.5164.00PWM = 992   
1.497 lbs.5258.00PWM = 992   
0.336 lbs.5352.00PWM = 992   
0.017 lbs.5447.00PWM = 992   
0.006 lbs.5541.00PWM = 992   
0.006 lbs.5635.00PWM = 992   
0.006 lbs.5730.00PWM = 992   
0.005 lbs.5824.00PWM = 992   


//20 ms delay

PWM = 992   
PWM = 992   
PWM = 992   
 
0.004 lbs.20.00 
0.004 lbs.110.00 
0.009 lbs.204.00 
0.060 lbs.298.00 
0.617 lbs.392.00 
1.727 lbs.486.00 
2.147 lbs.581.00 
2.182 lbs.675.00 
2.179 lbs.770.00 
2.173 lbs.864.00 
2.167 lbs.958.00 
2.162 lbs.1053.00 
2.153 lbs.1147.00 
2.145 lbs.1241.00 
2.146 lbs.1335.00 
2.147 lbs.1429.00 
2.144 lbs.1525.00 
2.139 lbs.1619.00 
2.134 lbs.1713.00 
2.131 lbs.1807.00 
2.128 lbs.1902.00 
2.127 lbs.1996.00 
2.121 lbs.2090.00 
2.117 lbs.2185.00 
2.119 lbs.2279.00 
2.118 lbs.2374.00 
2.118 lbs.2468.00 
2.107 lbs.2562.00 
2.104 lbs.2656.00 
2.105 lbs.2750.00 
2.099 lbs.2845.00 
2.094 lbs.2940.00 
2.099 lbs.3034.00 
2.111 lbs.3128.00 
2.122 lbs.3222.00 
2.122 lbs.3317.00 
2.121 lbs.3411.00 
2.123 lbs.3505.00 
2.127 lbs.3599.00 
2.122 lbs.3694.00 
2.106 lbs.3789.00 
2.095 lbs.3883.00 
2.095 lbs.3977.00 
2.101 lbs.4071.00 
2.094 lbs.4166.00 
2.083 lbs.4260.00 
2.071 lbs.4354.00 
2.064 lbs.4448.00 
2.065 lbs.4542.00 
2.062 lbs.4638.00 
2.065 lbs.4732.00 
2.062 lbs.4826.00 
2.055 lbs.4920.00 
2.056 lbs.5014.00PWM = 992   
2.054 lbs.5109.00PWM = 992   
1.553 lbs.5203.00PWM = 992   
0.372 lbs.5298.00PWM = 992   
0.015 lbs.5392.00PWM = 992   
0.003 lbs.5487.00PWM = 992   
0.004 lbs.5581.00PWM = 992   
0.005 lbs.5675.00PWM = 992   
0.004 lbs.5769.00PWM = 992   
0.004 lbs.5863.00PWM = 992   



//Same 20 ms delay:

PWM = 992   
PWM = 992   
PWM = 992   
 
0.007 lbs.21.00 
0.007 lbs.109.00 
0.012 lbs.204.00 
0.061 lbs.298.00 
0.602 lbs.392.00 
1.709 lbs.486.00 
2.140 lbs.582.00 
2.177 lbs.676.00 
2.175 lbs.770.00 
2.169 lbs.864.00 
2.163 lbs.958.00 
2.160 lbs.1053.00 
2.157 lbs.1147.00 
2.149 lbs.1241.00 
2.143 lbs.1336.00 
2.137 lbs.1430.00 
2.135 lbs.1525.00 
2.137 lbs.1619.00 
2.133 lbs.1713.00 
2.128 lbs.1807.00 
2.124 lbs.1901.00 
2.122 lbs.1997.00 
2.119 lbs.2091.00 
2.118 lbs.2185.00 
2.123 lbs.2279.00 
2.122 lbs.2374.00 
2.121 lbs.2468.00 
2.124 lbs.2562.00 
2.118 lbs.2656.00 
2.113 lbs.2751.00 
2.107 lbs.2846.00 
2.093 lbs.2940.00 
2.086 lbs.3034.00 
2.083 lbs.3128.00 
2.091 lbs.3222.00 
2.099 lbs.3317.00 
2.099 lbs.3412.00 
2.095 lbs.3506.00 
2.096 lbs.3600.00 
2.085 lbs.3695.00 
2.071 lbs.3789.00 
2.071 lbs.3883.00 
2.079 lbs.3977.00 
2.094 lbs.4072.00 
2.103 lbs.4167.00 
2.103 lbs.4261.00 
2.108 lbs.4355.00 
2.107 lbs.4449.00 
2.101 lbs.4543.00 
2.102 lbs.4638.00 
2.099 lbs.4733.00 
2.088 lbs.4827.00 
2.077 lbs.4921.00 
2.079 lbs.5015.00PWM = 992   
2.063 lbs.5110.00PWM = 992   
1.466 lbs.5204.00PWM = 992   
0.310 lbs.5298.00PWM = 992   
0.017 lbs.5392.00PWM = 992   
0.007 lbs.5488.00PWM = 992   
0.006 lbs.5582.00PWM = 992   
0.006 lbs.5676.00PWM = 992 



//10 ms delay

PWM = 992   
PWM = 992   
PWM = 992   
 
0.005 lbs.11.00 
0.005 lbs.22.00 
0.006 lbs.116.00 
0.012 lbs.210.00 
0.070 lbs.304.00 
0.658 lbs.398.00 
1.755 lbs.492.00 
2.149 lbs.587.00 
2.181 lbs.681.00 
2.178 lbs.776.00 
2.173 lbs.870.00 
2.168 lbs.965.00 
2.164 lbs.1059.00 
2.158 lbs.1153.00 
2.150 lbs.1247.00 
2.144 lbs.1341.00 
2.141 lbs.1436.00 
2.140 lbs.1531.00 
2.139 lbs.1625.00 
2.136 lbs.1719.00 
2.132 lbs.1813.00 
2.130 lbs.1908.00 
2.130 lbs.2002.00 
2.132 lbs.2096.00 
2.137 lbs.2190.00 
2.146 lbs.2285.00 
2.144 lbs.2380.00 
2.135 lbs.2474.00 
2.133 lbs.2568.00 
2.131 lbs.2662.00 
2.123 lbs.2757.00 
2.115 lbs.2851.00 
2.104 lbs.2945.00 
2.108 lbs.3040.00 
2.122 lbs.3134.00 
2.126 lbs.3229.00 
2.115 lbs.3323.00 
2.105 lbs.3417.00 
2.097 lbs.3511.00 
2.093 lbs.3605.00 
2.109 lbs.3700.00 
2.119 lbs.3795.00 
2.128 lbs.3889.00 
2.126 lbs.3983.00 
2.117 lbs.4077.00 
2.123 lbs.4172.00 
2.130 lbs.4266.00 
2.132 lbs.4360.00 
2.133 lbs.4454.00 
2.133 lbs.4550.00 
2.136 lbs.4644.00 
2.132 lbs.4738.00 
2.125 lbs.4832.00 
2.114 lbs.4926.00 
2.106 lbs.5021.00PWM = 992   
2.082 lbs.5115.00PWM = 992   
1.514 lbs.5209.00PWM = 992   
0.339 lbs.5304.00PWM = 992   
0.017 lbs.5398.00PWM = 992   
0.006 lbs.5493.00PWM = 992   
0.006 lbs.5587.00PWM = 992   
0.006 lbs.5681.00PWM = 992 



NEW CODE (Transient from start):

1440->1760 us

0.586 lbs. 9372.00 
0.587 lbs. 9466.00 
0.586 lbs. 9561.00 
0.585 lbs. 9656.00 
0.585 lbs. 9750.00 
0.585 lbs. 9844.00 
0.585 lbs. 9938.00 
0.585 lbs. 10033.00 
0.590 lbs. 93.00 
0.789 lbs. 187.00 
1.300 lbs. 281.00 
1.505 lbs. 375.00 
1.525 lbs. 471.00 
1.526 lbs. 565.00 
1.525 lbs. 659.00 
1.524 lbs. 753.00 
1.525 lbs. 848.00 
1.525 lbs. 942.00 
1.516 lbs. 1036.00 
1.509 lbs. 1130.00 
1.512 lbs. 1225.00 
1.514 lbs. 1320.00 
1.513 lbs. 1414.00 
1.512 lbs. 1508.00 
1.513 lbs. 1602.00 
1.512 lbs. 1696.00 
1.519 lbs. 1791.00 
1.526 lbs. 1885.00 
1.529 lbs. 1979.00 
1.524 lbs. 2074.00 
1.334 lbs. 2169.00 
0.840 lbs. 2263.00 
0.625 lbs. 2357.00 
0.598 lbs. 2451.00 
0.594 lbs. 2545.00 
0.591 lbs. 2640.00 
0.585 lbs. 2734.00 
0.584 lbs. 2829.00 
0.584 lbs. 2923.00 
0.584 lbs. 3017.00 
0.583 lbs. 3112.00 
0.583 lbs. 3206.00 
0.584 lbs. 3300.00 
0.588 lbs. 3394.00 
0.593 lbs. 3488.00 
0.593 lbs. 3584.00 
0.590 lbs. 3678.00 
0.589 lbs. 3772.00 
0.590 lbs. 3866.00 
0.588 lbs. 3961.00 
0.584 lbs. 4055.00 
0.578 lbs. 4149.00 
0.413 lbs. 4243.00 
0.083 lbs. 4338.00 
0.003 lbs. 4433.00 
0.001 lbs. 4527.00 
0.001 lbs. 4621.00 



Again:

0.587 lbs. 9013.00 
0.588 lbs. 9107.00 
0.590 lbs. 9201.00 
0.591 lbs. 9295.00 
0.590 lbs. 9390.00 
0.588 lbs. 9484.00 
0.586 lbs. 9579.00 
0.586 lbs. 9673.00 
0.586 lbs. 9767.00 
0.584 lbs. 9862.00 
0.584 lbs. 9956.00 
0.585 lbs. 10050.00 
0.591 lbs. 93.00 
0.798 lbs. 187.00 
1.310 lbs. 282.00 
1.511 lbs. 377.00 
1.529 lbs. 471.00 
1.527 lbs. 565.00 
1.523 lbs. 660.00 
1.522 lbs. 754.00 
1.520 lbs. 848.00 
1.517 lbs. 942.00 
1.516 lbs. 1036.00 
1.517 lbs. 1131.00 
1.515 lbs. 1225.00 
1.513 lbs. 1320.00 
1.514 lbs. 1414.00 
1.513 lbs. 1508.00 
1.509 lbs. 1603.00 
1.509 lbs. 1697.00 
1.512 lbs. 1791.00 
1.514 lbs. 1885.00 
1.520 lbs. 1981.00 
1.518 lbs. 2075.00 
1.322 lbs. 2169.00 
0.827 lbs. 2263.00 
0.616 lbs. 2357.00 
0.589 lbs. 2452.00 
0.586 lbs. 2546.00 
0.585 lbs. 2640.00 
0.590 lbs. 2735.00 
0.594 lbs. 2829.00 
0.592 lbs. 2924.00 
0.591 lbs. 3018.00 
0.590 lbs. 3112.00 
0.590 lbs. 3206.00 
0.587 lbs. 3300.00 
0.585 lbs. 3395.00 
0.586 lbs. 3489.00 
0.588 lbs. 3584.00 
0.586 lbs. 3678.00 
0.584 lbs. 3773.00 
0.587 lbs. 3867.00 
0.589 lbs. 3961.00 
0.582 lbs. 4055.00 
0.404 lbs. 4149.00 
0.077 lbs. 4244.00 
0.004 lbs. 4338.00 
0.002 lbs. 4433.00 
0.003 lbs. 4527.00 
0.003 lbs. 4621.00 
0.003 lbs. 4716.00 
0.003 lbs. 4810.00 


0.583 lbs. 9172.00 
0.583 lbs. 9267.00 
0.582 lbs. 9361.00 
0.580 lbs. 9455.00 
0.580 lbs. 9549.00 
0.578 lbs. 9644.00 
0.578 lbs. 9738.00 
0.579 lbs. 9832.00 
0.580 lbs. 9926.00 
0.580 lbs. 10021.00 
0.588 lbs. 94.00 
0.810 lbs. 188.00 
1.318 lbs. 282.00 
1.503 lbs. 376.00 
1.520 lbs. 470.00 
1.522 lbs. 565.00 
1.524 lbs. 659.00 
1.521 lbs. 754.00 
1.520 lbs. 848.00 
1.518 lbs. 942.00 
1.517 lbs. 1037.00 
1.518 lbs. 1131.00 
1.517 lbs. 1225.00 
1.516 lbs. 1319.00 
1.517 lbs. 1414.00 
1.519 lbs. 1509.00 
1.520 lbs. 1603.00 
1.521 lbs. 1697.00 
1.523 lbs. 1791.00 
1.524 lbs. 1886.00 
1.520 lbs. 1980.00 
1.511 lbs. 2074.00 
1.305 lbs. 2168.00 
0.814 lbs. 2262.00 
0.612 lbs. 2357.00 
0.586 lbs. 2452.00 
0.585 lbs. 2546.00 
0.586 lbs. 2640.00 
0.585 lbs. 2734.00 
0.583 lbs. 2829.00 
0.581 lbs. 2923.00 
0.582 lbs. 3017.00 
0.585 lbs. 3111.00 
0.589 lbs. 3207.00 
0.593 lbs. 3301.00 
0.592 lbs. 3395.00 
0.588 lbs. 3489.00 
0.590 lbs. 3583.00 
0.592 lbs. 3678.00 
0.589 lbs. 3772.00 
0.586 lbs. 3866.00 
0.584 lbs. 3960.00 
0.578 lbs. 4055.00 
0.569 lbs. 4150.00 
0.396 lbs. 4244.00 
0.075 lbs. 4338.00 
0.004 lbs. 4432.00 
0.003 lbs. 4526.00 
0.003 lbs. 4621.00 
0.003 lbs. 4715.00 



1760 us -> 2016 us:

1.530 lbs. 8688.00 
1.530 lbs. 8782.00 
1.527 lbs. 8878.00 
1.524 lbs. 8972.00 
1.522 lbs. 9066.00 
1.519 lbs. 9160.00 
1.519 lbs. 9254.00 
1.520 lbs. 9349.00 
1.519 lbs. 9443.00 
1.513 lbs. 9537.00 
1.509 lbs. 9632.00 
1.509 lbs. 9726.00 
1.509 lbs. 9821.00 
1.510 lbs. 9915.00 
1.506 lbs. 10009.00 
1.513 lbs. 93.00 
1.675 lbs. 188.00 
2.037 lbs. 282.00 
2.164 lbs. 376.00 
2.170 lbs. 471.00 
2.155 lbs. 565.00 
2.139 lbs. 660.00 
2.131 lbs. 754.00 
2.126 lbs. 848.00 
2.120 lbs. 942.00 
2.120 lbs. 1036.00 
2.127 lbs. 1131.00 
2.128 lbs. 1226.00 
2.122 lbs. 1320.00 
2.121 lbs. 1414.00 
2.117 lbs. 1508.00 
2.108 lbs. 1603.00 
2.091 lbs. 1697.00 
2.095 lbs. 1791.00 
2.117 lbs. 1886.00 
2.127 lbs. 1981.00 
2.124 lbs. 2075.00 
2.108 lbs. 2169.00 
1.943 lbs. 2263.00 
1.584 lbs. 2357.00 
1.463 lbs. 2452.00 
1.465 lbs. 2546.00 
1.469 lbs. 2641.00 
1.479 lbs. 2735.00 
1.495 lbs. 2829.00 
1.506 lbs. 2924.00 
1.505 lbs. 3018.00 
1.497 lbs. 3112.00 
1.504 lbs. 3206.00 
1.511 lbs. 3302.00 
1.507 lbs. 3396.00 
1.498 lbs. 3490.00 
1.492 lbs. 3584.00 
1.488 lbs. 3678.00 
1.479 lbs. 3773.00 
1.473 lbs. 3867.00 
1.462 lbs. 3961.00 
1.465 lbs. 4056.00 
1.455 lbs. 4150.00 
1.046 lbs. 4245.00 
0.221 lbs. 4339.00 
0.008 lbs. 4433.00 
0.003 lbs. 4527.00 
0.003 lbs. 4621.00 
0.003 lbs. 4717.00 


1.522 lbs. 8843.00 
1.520 lbs. 8938.00 
1.517 lbs. 9032.00 
1.514 lbs. 9126.00 
1.511 lbs. 9221.00 
1.510 lbs. 9315.00 
1.509 lbs. 9409.00 
1.508 lbs. 9503.00 
1.505 lbs. 9597.00 
1.502 lbs. 9693.00 
1.502 lbs. 9787.00 
1.500 lbs. 9881.00 
1.496 lbs. 9975.00 
1.490 lbs. 10070.00 
1.497 lbs. 93.00 
1.658 lbs. 187.00 
2.008 lbs. 281.00 
2.127 lbs. 375.00 
2.139 lbs. 470.00 
2.142 lbs. 565.00 
2.145 lbs. 659.00 
2.144 lbs. 753.00 
2.132 lbs. 847.00 
2.118 lbs. 942.00 
2.121 lbs. 1036.00 
2.125 lbs. 1130.00 
2.126 lbs. 1224.00 
2.125 lbs. 1318.00 
2.130 lbs. 1413.00 
2.137 lbs. 1508.00 
2.135 lbs. 1602.00 
2.132 lbs. 1696.00 
2.135 lbs. 1791.00 
2.132 lbs. 1885.00 
2.118 lbs. 1979.00 
2.109 lbs. 2073.00 
1.963 lbs. 2167.00 
1.616 lbs. 2262.00 
1.494 lbs. 2356.00 
1.484 lbs. 2450.00 
1.488 lbs. 2545.00 
1.485 lbs. 2639.00 
1.488 lbs. 2734.00 
1.489 lbs. 2828.00 
1.490 lbs. 2922.00 
1.496 lbs. 3016.00 
1.494 lbs. 3110.00 
1.490 lbs. 3205.00 
1.488 lbs. 3299.00 
1.493 lbs. 3393.00 
1.494 lbs. 3487.00 
1.494 lbs. 3581.00 
1.495 lbs. 3677.00 
1.492 lbs. 3771.00 
1.485 lbs. 3865.00 
1.480 lbs. 3959.00 
1.462 lbs. 4054.00 
1.086 lbs. 4148.00 
0.251 lbs. 4242.00 
0.013 lbs. 4336.00 
0.005 lbs. 4430.00 
0.005 lbs. 4525.00 
0.006 lbs. 4620.00 
0.006 lbs. 4714.00 
0.006 lbs. 4808.00 



30 ms delay:

1.533 lbs. 8710.00 
1.531 lbs. 8804.00 
1.528 lbs. 8898.00 
1.525 lbs. 8992.00 
1.518 lbs. 9086.00 
1.514 lbs. 9181.00 
1.514 lbs. 9275.00 
1.513 lbs. 9370.00 
1.512 lbs. 9464.00 
1.513 lbs. 9559.00 
1.516 lbs. 9653.00 
1.512 lbs. 9747.00 
1.514 lbs. 9841.00 
1.517 lbs. 9935.00 
1.521 lbs. 10030.00 
1.519 lbs. 64.00 
1.589 lbs. 158.00 
1.918 lbs. 252.00 
2.125 lbs. 346.00 
2.149 lbs. 441.00 
2.151 lbs. 535.00 
2.154 lbs. 629.00 
2.154 lbs. 723.00 
2.149 lbs. 819.00 
2.138 lbs. 913.00 
2.132 lbs. 1007.00 
2.131 lbs. 1101.00 
2.129 lbs. 1195.00 
2.136 lbs. 1290.00 
2.141 lbs. 1384.00 
2.135 lbs. 1478.00 
2.127 lbs. 1572.00 
2.124 lbs. 1667.00 
2.129 lbs. 1762.00 
2.138 lbs. 1856.00 
2.146 lbs. 1950.00 
2.141 lbs. 2044.00 
1.979 lbs. 2138.00 
1.624 lbs. 2233.00 
1.506 lbs. 2327.00 
1.492 lbs. 2422.00 
1.481 lbs. 2516.00 
1.474 lbs. 2611.00 
1.473 lbs. 2705.00 
1.480 lbs. 2799.00 
1.490 lbs. 2893.00 
1.497 lbs. 2987.00 
1.506 lbs. 3082.00 
1.512 lbs. 3177.00 
1.510 lbs. 3271.00 
1.502 lbs. 3365.00 
1.494 lbs. 3459.00 
1.489 lbs. 3554.00 
1.486 lbs. 3648.00 
1.485 lbs. 3742.00 
1.477 lbs. 3836.00 
1.467 lbs. 3931.00 
1.455 lbs. 4026.00 
1.437 lbs. 4120.00 
1.069 lbs. 4214.00 
0.242 lbs. 4308.00 
0.009 lbs. 4403.00 
0.004 lbs. 4497.00 
0.005 lbs. 4591.00 
0.005 lbs. 4685.00 
0.006 lbs. 4780.00 
0.006 lbs. 4875.00 


20 ms delay:

1.518 lbs. 9011.00 
1.517 lbs. 9105.00 
1.514 lbs. 9199.00 
1.512 lbs. 9293.00 
1.513 lbs. 9388.00 
1.511 lbs. 9482.00 
1.515 lbs. 9576.00 
1.516 lbs. 9671.00 
1.517 lbs. 9765.00 
1.515 lbs. 9860.00 
1.509 lbs. 9954.00 
1.504 lbs. 10048.00 
1.509 lbs. 72.00 
1.612 lbs. 166.00 
1.956 lbs. 261.00 
2.123 lbs. 356.00 
2.133 lbs. 450.00 
2.134 lbs. 544.00 
2.136 lbs. 638.00 
2.136 lbs. 733.00 
2.134 lbs. 827.00 
2.134 lbs. 921.00 
2.142 lbs. 1015.00 
2.141 lbs. 1110.00 
2.136 lbs. 1204.00 
2.132 lbs. 1299.00 
2.122 lbs. 1393.00 
2.111 lbs. 1487.00 
2.109 lbs. 1582.00 
2.112 lbs. 1676.00 
2.116 lbs. 1770.00 
2.127 lbs. 1864.00 
2.136 lbs. 1958.00 
2.126 lbs. 2054.00 
1.956 lbs. 2148.00 
1.611 lbs. 2242.00 
1.497 lbs. 2336.00 
1.489 lbs. 2430.00 
1.484 lbs. 2525.00 
1.475 lbs. 2619.00 
1.477 lbs. 2713.00 
1.483 lbs. 2807.00 
1.481 lbs. 2903.00 
1.479 lbs. 2997.00 
1.481 lbs. 3091.00 
1.480 lbs. 3185.00 
1.479 lbs. 3279.00 
1.476 lbs. 3374.00 
1.478 lbs. 3468.00 
1.478 lbs. 3562.00 
1.481 lbs. 3656.00 
1.480 lbs. 3751.00 
1.479 lbs. 3846.00 
1.476 lbs. 3940.00 
1.457 lbs. 4034.00 
1.046 lbs. 4128.00 
0.223 lbs. 4222.00 
0.012 lbs. 4317.00 
0.006 lbs. 4411.00 
0.007 lbs. 4505.00 
0.007 lbs. 4600.00 
0.007 lbs. 4695.00 


25 ms delay:

1.510 lbs. 9259.00 
1.508 lbs. 9354.00 
1.509 lbs. 9448.00 
1.505 lbs. 9542.00 
1.505 lbs. 9636.00 
1.506 lbs. 9731.00 
1.508 lbs. 9825.00 
1.508 lbs. 9919.00 
1.502 lbs. 10013.00 
1.497 lbs. 67.00 
1.591 lbs. 162.00 
1.942 lbs. 257.00 
2.128 lbs. 351.00 
2.146 lbs. 445.00 
2.140 lbs. 539.00 
2.129 lbs. 634.00 
2.127 lbs. 728.00 
2.124 lbs. 822.00 
2.119 lbs. 916.00 
2.120 lbs. 1011.00 
2.122 lbs. 1106.00 
2.130 lbs. 1200.00 
2.144 lbs. 1294.00 
2.146 lbs. 1388.00 
2.143 lbs. 1483.00 
2.146 lbs. 1577.00 
2.147 lbs. 1671.00 
2.140 lbs. 1765.00 
2.130 lbs. 1859.00 
2.131 lbs. 1955.00 
2.129 lbs. 2049.00 
2.115 lbs. 2143.00 
1.970 lbs. 2237.00 
1.620 lbs. 2331.00 
1.492 lbs. 2426.00 
1.481 lbs. 2520.00 
1.481 lbs. 2614.00 
1.485 lbs. 2708.00 
1.479 lbs. 2804.00 
1.466 lbs. 2898.00 
1.454 lbs. 2992.00 
1.442 lbs. 3086.00 
1.430 lbs. 3180.00 
1.421 lbs. 3275.00 
1.424 lbs. 3369.00 
1.420 lbs. 3463.00 
1.427 lbs. 3557.00 
1.446 lbs. 3652.00 
1.455 lbs. 3747.00 
1.454 lbs. 3841.00 
1.458 lbs. 3935.00 
1.460 lbs. 4029.00 
1.437 lbs. 4123.00 
1.011 lbs. 4218.00 
0.204 lbs. 4312.00 
0.010 lbs. 4406.00 
0.006 lbs. 4501.00 
0.006 lbs. 4596.00 
0.007 lbs. 4690.00 
0.007 lbs. 4784.00 


15 ms delay:

1.516 lbs. 8774.00 
1.511 lbs. 8868.00 
1.509 lbs. 8964.00 
1.511 lbs. 9058.00 
1.511 lbs. 9152.00 
1.510 lbs. 9246.00 
1.509 lbs. 9340.00 
1.507 lbs. 9435.00 
1.505 lbs. 9529.00 
1.505 lbs. 9623.00 
1.503 lbs. 9717.00 
1.502 lbs. 9811.00 
1.505 lbs. 9907.00 
1.511 lbs. 10001.00 
1.515 lbs. 79.00 
1.610 lbs. 173.00 
1.954 lbs. 268.00 
2.126 lbs. 362.00 
2.145 lbs. 456.00 
2.148 lbs. 550.00 
2.144 lbs. 644.00 
2.135 lbs. 740.00 
2.129 lbs. 834.00 
2.128 lbs. 928.00 
2.135 lbs. 1022.00 
2.147 lbs. 1116.00 
2.150 lbs. 1211.00 
2.139 lbs. 1305.00 
2.122 lbs. 1399.00 
2.115 lbs. 1493.00 
2.112 lbs. 1588.00 
2.108 lbs. 1683.00 
2.105 lbs. 1777.00 
2.114 lbs. 1871.00 
2.118 lbs. 1965.00 
2.122 lbs. 2060.00 
2.116 lbs. 2154.00 
1.955 lbs. 2248.00 
1.613 lbs. 2342.00 
1.489 lbs. 2437.00 
1.473 lbs. 2532.00 
1.473 lbs. 2626.00 
1.474 lbs. 2720.00 
1.479 lbs. 2814.00 
1.485 lbs. 2908.00 
1.488 lbs. 3003.00 
1.478 lbs. 3097.00 
1.470 lbs. 3192.00 
1.466 lbs. 3286.00 
1.453 lbs. 3380.00 
1.449 lbs. 3475.00 
1.450 lbs. 3569.00 
1.448 lbs. 3663.00 
1.461 lbs. 3757.00 
1.474 lbs. 3852.00 
1.461 lbs. 3946.00 
1.448 lbs. 4041.00 
1.433 lbs. 4135.00 
1.043 lbs. 4229.00 
0.230 lbs. 4324.00 
0.015 lbs. 4418.00 
0.010 lbs. 4512.00 
0.010 lbs. 4606.00 
0.010 lbs. 4700.00 


40 ms delay:

1.509 lbs. 8943.00 
1.507 lbs. 9037.00 
1.504 lbs. 9132.00 
1.504 lbs. 9227.00 
1.502 lbs. 9321.00 
1.498 lbs. 9415.00 
1.497 lbs. 9509.00 
1.497 lbs. 9604.00 
1.496 lbs. 9698.00 
1.498 lbs. 9792.00 
1.500 lbs. 9886.00 
1.503 lbs. 9981.00 
1.510 lbs. 10076.00 
1.518 lbs. 53.00 
1.574 lbs. 147.00 
1.888 lbs. 241.00 
2.125 lbs. 335.00 
2.152 lbs. 430.00 
2.148 lbs. 524.00 
2.142 lbs. 619.00 
2.135 lbs. 713.00 
2.133 lbs. 808.00 
2.137 lbs. 902.00 
2.139 lbs. 996.00 
2.137 lbs. 1090.00 
2.135 lbs. 1184.00 
2.138 lbs. 1279.00 
2.138 lbs. 1374.00 
2.128 lbs. 1468.00 
2.125 lbs. 1562.00 
2.116 lbs. 1656.00 
2.110 lbs. 1751.00 
2.106 lbs. 1845.00 
2.109 lbs. 1939.00 
2.109 lbs. 2033.00 
1.959 lbs. 2127.00 
1.620 lbs. 2223.00 
1.494 lbs. 2317.00 
1.480 lbs. 2411.00 
1.483 lbs. 2505.00 
1.483 lbs. 2600.00 
1.483 lbs. 2694.00 
1.492 lbs. 2788.00 
1.501 lbs. 2882.00 
1.499 lbs. 2976.00 
1.492 lbs. 3071.00 
1.486 lbs. 3166.00 
1.484 lbs. 3260.00 
1.484 lbs. 3354.00 
1.479 lbs. 3448.00 
1.481 lbs. 3543.00 
1.487 lbs. 3637.00 
1.490 lbs. 3731.00 
1.488 lbs. 3825.00 
1.483 lbs. 3919.00 
1.461 lbs. 4014.00 
1.032 lbs. 4109.00 
0.216 lbs. 4203.00 
0.015 lbs. 4297.00 
0.010 lbs. 4392.00 
0.011 lbs. 4486.00 
0.012 lbs. 4580.00 
0.012 lbs. 4674.00 
0.012 lbs. 4768.00 



NEW PROP (6*4E) (EXACT SAME MOTOR, ESC, SETUP, CODE, ETC):

1,120 us -> 1,280 us

0.025 lbs. 8596.00 
0.025 lbs. 8690.00 
0.024 lbs. 8784.00 
0.024 lbs. 8879.00 
0.024 lbs. 8973.00 
0.025 lbs. 9067.00 
0.025 lbs. 9162.00 
0.025 lbs. 9256.00 
0.025 lbs. 9351.00 
0.025 lbs. 9445.00 
0.024 lbs. 9539.00 
0.024 lbs. 9633.00 
0.025 lbs. 9728.00 
0.025 lbs. 9822.00 
0.024 lbs. 9916.00 
0.024 lbs. 10010.00 
0.025 lbs. 53.00 
0.032 lbs. 148.00 
0.067 lbs. 243.00 
0.088 lbs. 337.00 
0.089 lbs. 431.00 
0.090 lbs. 525.00 
0.090 lbs. 620.00 
0.089 lbs. 714.00 
0.089 lbs. 808.00 
0.090 lbs. 902.00 
0.090 lbs. 996.00 
0.090 lbs. 1091.00 
0.090 lbs. 1186.00 
0.089 lbs. 1280.00 
0.089 lbs. 1374.00 
0.089 lbs. 1469.00 
0.089 lbs. 1563.00 
0.090 lbs. 1657.00 
0.088 lbs. 1751.00 
0.088 lbs. 1845.00 
0.089 lbs. 1940.00 
0.090 lbs. 2034.00 
0.089 lbs. 2129.00 
0.066 lbs. 2223.00 
0.030 lbs. 2317.00 
0.024 lbs. 2412.00 
0.024 lbs. 2506.00 
0.024 lbs. 2600.00 
0.024 lbs. 2694.00 
0.024 lbs. 2788.00 
0.024 lbs. 2883.00 
0.024 lbs. 2977.00 
0.024 lbs. 3071.00 
0.024 lbs. 3166.00 
0.025 lbs. 3261.00 
0.025 lbs. 3355.00 
0.025 lbs. 3449.00 
0.025 lbs. 3543.00 
0.025 lbs. 3637.00 
0.025 lbs. 3732.00 
0.024 lbs. 3826.00 
0.024 lbs. 3920.00 
0.024 lbs. 4014.00 
0.024 lbs. 4108.00 
0.020 lbs. 4204.00 
0.015 lbs. 4298.00 
0.015 lbs. 4392.00 
0.015 lbs. 4486.00 
0.014 lbs. 4580.00 
0.014 lbs. 4675.00 
0.015 lbs. 4769.00 
0.015 lbs. 4863.00 
0.015 lbs. 4957.00 
0.015 lbs. 5051.00 
0.015 lbs. 5147.00 
0.014 lbs. 5241.00 



1,280 us -> 1,440 us

0.089 lbs. 9699.00 
0.089 lbs. 9793.00 
0.090 lbs. 9887.00 
0.089 lbs. 9981.00 
0.089 lbs. 10077.00 
0.089 lbs. 53.00 
0.103 lbs. 147.00 
0.170 lbs. 241.00 
0.209 lbs. 336.00 
0.211 lbs. 430.00 
0.211 lbs. 524.00 
0.211 lbs. 618.00 
0.211 lbs. 713.00 
0.211 lbs. 808.00 
0.210 lbs. 902.00 
0.210 lbs. 996.00 
0.210 lbs. 1090.00 
0.211 lbs. 1184.00 
0.211 lbs. 1279.00 
0.211 lbs. 1373.00 
0.211 lbs. 1467.00 
0.210 lbs. 1562.00 
0.210 lbs. 1656.00 
0.210 lbs. 1751.00 
0.211 lbs. 1845.00 
0.211 lbs. 1939.00 
0.210 lbs. 2033.00 
0.173 lbs. 2128.00 
0.106 lbs. 2222.00 
0.089 lbs. 2316.00 
0.089 lbs. 2411.00 
0.089 lbs. 2505.00 
0.089 lbs. 2600.00 
0.089 lbs. 2694.00 
0.089 lbs. 2788.00 
0.088 lbs. 2882.00 
0.088 lbs. 2976.00 
0.089 lbs. 3071.00 
0.089 lbs. 3165.00 
0.089 lbs. 3260.00 
0.090 lbs. 3354.00 
0.089 lbs. 3448.00 
0.090 lbs. 3543.00 
0.090 lbs. 3637.00 
0.089 lbs. 3731.00 
0.089 lbs. 3825.00 
0.089 lbs. 3920.00 
0.089 lbs. 4014.00 
0.062 lbs. 4109.00 
0.021 lbs. 4203.00 
0.014 lbs. 4297.00 
0.014 lbs. 4392.00 
0.015 lbs. 4486.00 
0.015 lbs. 4580.00 


1440 us -> 1600 us:

0.211 lbs. 9086.00 
0.211 lbs. 9181.00 
0.211 lbs. 9275.00 
0.211 lbs. 9369.00 
0.210 lbs. 9463.00 
0.210 lbs. 9558.00 
0.210 lbs. 9653.00 
0.210 lbs. 9747.00 
0.211 lbs. 9841.00 
0.210 lbs. 9935.00 
0.211 lbs. 10030.00 
0.210 lbs. 53.00 
0.231 lbs. 147.00 
0.326 lbs. 241.00 
0.379 lbs. 335.00 
0.383 lbs. 430.00 
0.383 lbs. 525.00 
0.383 lbs. 619.00 
0.383 lbs. 713.00 
0.382 lbs. 807.00 
0.382 lbs. 902.00 
0.382 lbs. 996.00 
0.383 lbs. 1090.00 
0.384 lbs. 1184.00 
0.384 lbs. 1279.00 
0.385 lbs. 1374.00 
0.385 lbs. 1468.00 
0.385 lbs. 1562.00 
0.385 lbs. 1656.00 
0.384 lbs. 1751.00 
0.384 lbs. 1845.00 
0.385 lbs. 1939.00 
0.384 lbs. 2033.00 
0.333 lbs. 2127.00 
0.236 lbs. 2223.00 
0.212 lbs. 2317.00 
0.212 lbs. 2411.00 
0.210 lbs. 2505.00 
0.209 lbs. 2599.00 
0.210 lbs. 2694.00 
0.210 lbs. 2788.00 
0.209 lbs. 2883.00 
0.207 lbs. 2977.00 
0.207 lbs. 3072.00 
0.208 lbs. 3166.00 
0.209 lbs. 3260.00 
0.209 lbs. 3354.00 
0.207 lbs. 3448.00 
0.207 lbs. 3543.00 
0.206 lbs. 3637.00 
0.205 lbs. 3732.00 
0.206 lbs. 3826.00 
0.207 lbs. 3920.00 
0.208 lbs. 4015.00 
0.205 lbs. 4109.00 
0.130 lbs. 4203.00 
0.028 lbs. 4297.00 
0.015 lbs. 4391.00 
0.015 lbs. 4486.00 
0.016 lbs. 4581.00 
0.016 lbs. 4675.00 
0.015 lbs. 4769.00 
0.016 lbs. 4864.00 
0.016 lbs. 4958.00 
0.016 lbs. 5052.00 
0.015 lbs. 5146.00 



1440 us -> 1760 us:

0.212 lbs. 9361.00 
0.212 lbs. 9455.00 
0.212 lbs. 9549.00 
0.212 lbs. 9644.00 
0.211 lbs. 9739.00 
0.211 lbs. 9833.00 
0.211 lbs. 9927.00 
0.212 lbs. 10021.00 
0.212 lbs. 54.00 
0.256 lbs. 148.00 
0.467 lbs. 242.00 
0.584 lbs. 337.00 
0.591 lbs. 431.00  //0.591 lbs -> 11,000.0 RPM.  0.046 Hp -> 34.3 W aero power.  4.9 A on power supply.
0.593 lbs. 526.00 
0.594 lbs. 620.00 
0.594 lbs. 714.00 
0.593 lbs. 808.00 
0.594 lbs. 902.00 
0.594 lbs. 998.00 
0.593 lbs. 1092.00 
0.592 lbs. 1186.00 
0.591 lbs. 1280.00 
0.591 lbs. 1375.00 
0.590 lbs. 1469.00 
0.591 lbs. 1563.00 
0.591 lbs. 1658.00 
0.591 lbs. 1752.00 
0.589 lbs. 1847.00 
0.588 lbs. 1941.00 
0.585 lbs. 2035.00 
0.480 lbs. 2129.00 
0.280 lbs. 2223.00 
0.217 lbs. 2318.00 
0.212 lbs. 2413.00 
0.212 lbs. 2507.00 
0.212 lbs. 2601.00 
0.214 lbs. 2696.00 
0.215 lbs. 2790.00 
0.214 lbs. 2884.00 
0.215 lbs. 2978.00 
0.215 lbs. 3072.00 
0.215 lbs. 3167.00 
0.215 lbs. 3262.00 
0.214 lbs. 3356.00 
0.214 lbs. 3450.00 
0.213 lbs. 3544.00 
0.213 lbs. 3639.00 
0.212 lbs. 3733.00 
0.212 lbs. 3827.00 
0.213 lbs. 3921.00 
0.213 lbs. 4015.00 
0.210 lbs. 4111.00 
0.134 lbs. 4205.00 
0.029 lbs. 4299.00 
0.015 lbs. 4393.00 
0.016 lbs. 4488.00 
0.016 lbs. 4582.00 
0.016 lbs. 4676.00 
0.016 lbs. 4770.00 


1,760 us -> 2,016 us:

0.591 lbs. 8826.00 
0.590 lbs. 8921.00 
0.590 lbs. 9015.00 
0.591 lbs. 9109.00 
0.591 lbs. 9203.00 
0.592 lbs. 9297.00 
0.592 lbs. 9392.00 
0.591 lbs. 9486.00 
0.590 lbs. 9581.00 
0.590 lbs. 9675.00 
0.590 lbs. 9769.00 
0.590 lbs. 9864.00 
0.590 lbs. 9958.00 
0.590 lbs. 10052.00 
0.591 lbs. 53.00 
0.624 lbs. 148.00 
0.789 lbs. 243.00 
0.887 lbs. 337.00 
0.895 lbs. 431.00 -> 13,530 RPM -> 9.664 V back EMF -> 2.936 V net.  8.6-8.7A draw on power supply.  108.36 W input power.  64.87 W aero power.
0.896 lbs. 525.00 
0.897 lbs. 620.00 
0.899 lbs. 714.00 
0.901 lbs. 808.00 
0.899 lbs. 902.00 
0.899 lbs. 997.00 
0.899 lbs. 1092.00 
0.897 lbs. 1186.00 
0.895 lbs. 1280.00 
0.896 lbs. 1374.00 
0.896 lbs. 1468.00 
0.893 lbs. 1563.00 
0.891 lbs. 1657.00 
0.892 lbs. 1751.00 
0.891 lbs. 1846.00 
0.893 lbs. 1941.00 
0.890 lbs. 2035.00 
0.806 lbs. 2129.00 
0.636 lbs. 2223.00 
0.592 lbs. 2317.00 
0.590 lbs. 2412.00 
0.589 lbs. 2506.00 
0.588 lbs. 2600.00 
0.589 lbs. 2695.00 
0.589 lbs. 2789.00 
0.588 lbs. 2884.00 
0.590 lbs. 2978.00 
0.593 lbs. 3072.00 
0.596 lbs. 3166.00 
0.596 lbs. 3260.00 
0.594 lbs. 3355.00 
0.593 lbs. 3450.00 
0.592 lbs. 3544.00 
0.592 lbs. 3638.00 
0.594 lbs. 3733.00 
0.596 lbs. 3827.00 
0.598 lbs. 3921.00 
0.590 lbs. 4015.00 
0.401 lbs. 4109.00 
0.074 lbs. 4205.00 
0.016 lbs. 4299.00 
0.016 lbs. 4393.00 
0.017 lbs. 4487.00 
0.017 lbs. 4581.00 
0.017 lbs. 4676.00 
0.017 lbs. 4770.00 
0.017 lbs. 4864.00 
0.017 lbs. 4958.00 
0.017 lbs. 5054.00 




1920 us -> 1952 us

0.832 lbs. 8779.00 
0.832 lbs. 8873.00 
0.832 lbs. 8968.00 
0.832 lbs. 9063.00 
0.833 lbs. 9157.00 
0.834 lbs. 9251.00 
0.836 lbs. 9346.00 
0.836 lbs. 9440.00 
0.837 lbs. 9534.00 
0.837 lbs. 9629.00 
0.836 lbs. 9723.00 
0.834 lbs. 9818.00 
0.831 lbs. 9912.00 
0.831 lbs. 10006.00 
0.832 lbs. 53.00 
0.838 lbs. 147.00 
0.870 lbs. 243.00 
0.888 lbs. 337.00 
0.891 lbs. 431.00 
0.892 lbs. 525.00 
0.893 lbs. 620.00 
0.889 lbs. 714.00 
0.883 lbs. 808.00 
0.880 lbs. 903.00 
0.879 lbs. 997.00 
0.879 lbs. 1092.00 
0.879 lbs. 1186.00 
0.880 lbs. 1280.00 
0.880 lbs. 1374.00 
0.880 lbs. 1468.00 
0.878 lbs. 1564.00 
0.876 lbs. 1658.00 
0.874 lbs. 1752.00 
0.874 lbs. 1846.00 
0.874 lbs. 1940.00 
0.875 lbs. 2035.00 
0.878 lbs. 2129.00 
0.882 lbs. 2224.00 
0.884 lbs. 2318.00 
0.890 lbs. 2413.00 
0.893 lbs. 2507.00 
0.889 lbs. 2601.00 
0.887 lbs. 2695.00 
0.888 lbs. 2789.00 
0.887 lbs. 2884.00 
0.887 lbs. 2979.00 
0.886 lbs. 3073.00 
0.885 lbs. 3167.00 
0.886 lbs. 3261.00 
0.888 lbs. 3356.00 
0.888 lbs. 3450.00 
0.893 lbs. 3544.00 
0.896 lbs. 3638.00 
0.897 lbs. 3733.00 
0.896 lbs. 3828.00 
0.895 lbs. 3922.00 
0.890 lbs. 4016.00 
0.890 lbs. 4110.00 
0.893 lbs. 4205.00 
0.894 lbs. 4299.00 
0.895 lbs. 4394.00 
0.894 lbs. 4488.00 
0.892 lbs. 4582.00 
0.892 lbs. 4677.00 
0.891 lbs. 4771.00 
0.888 lbs. 4865.00 
0.888 lbs. 4959.00 
0.886 lbs. 5053.00 
0.865 lbs. 5149.00 
0.833 lbs. 5243.00 
0.825 lbs. 5337.00 
0.821 lbs. 5431.00 
0.819 lbs. 5525.00 
0.821 lbs. 5620.00 
0.821 lbs. 5714.00 
0.821 lbs. 5808.00 
0.821 lbs. 5903.00 
0.825 lbs. 5998.00 
0.828 lbs. 6092.00 
0.828 lbs. 6186.00 
0.826 lbs. 6280.00 
0.822 lbs. 6374.00 
0.820 lbs. 6469.00 
0.821 lbs. 6563.00 
0.823 lbs. 6658.00 
0.825 lbs. 6752.00 
0.823 lbs. 6846.00 
0.819 lbs. 6941.00 
0.819 lbs. 7035.00 
0.808 lbs. 7129.00 
0.559 lbs. 7223.00 
0.102 lbs. 7317.00 
0.016 lbs. 7413.00 
0.018 lbs. 7507.00 
0.018 lbs. 7601.00 
0.017 lbs. 7695.00 
0.017 lbs. 7790.00 



1,952 us -> 1984 us:

0.884 lbs. 8646.00 
0.885 lbs. 8740.00 
0.885 lbs. 8835.00 
0.885 lbs. 8929.00 
0.886 lbs. 9023.00 
0.887 lbs. 9118.00 
0.886 lbs. 9212.00 
0.885 lbs. 9307.00 
0.882 lbs. 9401.00 
0.881 lbs. 9495.00 
0.881 lbs. 9589.00 
0.884 lbs. 9683.00 
0.887 lbs. 9779.00 
0.888 lbs. 9873.00 
0.890 lbs. 9967.00 
0.890 lbs. 10061.00 
0.891 lbs. 54.00 
0.896 lbs. 148.00 
0.899 lbs. 242.00 
0.896 lbs. 336.00 
0.896 lbs. 431.00 
0.897 lbs. 526.00 
0.895 lbs. 620.00 
0.892 lbs. 714.00 
0.888 lbs. 808.00 
0.886 lbs. 902.00 
0.886 lbs. 997.00 
0.884 lbs. 1092.00 
0.884 lbs. 1186.00 
0.884 lbs. 1280.00 
0.886 lbs. 1374.00 
0.887 lbs. 1469.00 
0.887 lbs. 1563.00 
0.883 lbs. 1657.00 
0.882 lbs. 1752.00 
0.882 lbs. 1847.00 
0.881 lbs. 1941.00 
0.884 lbs. 2035.00 
0.887 lbs. 2129.00 
0.886 lbs. 2223.00 
0.882 lbs. 2318.00 
0.883 lbs. 2413.00 
0.884 lbs. 2507.00 
0.886 lbs. 2601.00 
0.885 lbs. 2695.00 
0.884 lbs. 2790.00 
0.885 lbs. 2884.00 
0.886 lbs. 2978.00 
0.884 lbs. 3073.00 
0.882 lbs. 3168.00 
0.882 lbs. 3262.00 
0.885 lbs. 3356.00 
0.888 lbs. 3450.00 
0.892 lbs. 3544.00 
0.893 lbs. 3639.00 
0.894 lbs. 3733.00 
0.892 lbs. 3828.00 
0.890 lbs. 3922.00 
0.878 lbs. 4016.00 
0.599 lbs. 4111.00 
0.109 lbs. 4205.00 
0.020 lbs. 4299.00 
0.020 lbs. 4394.00 
0.020 lbs. 4488.00 
0.020 lbs. 4583.00 
0.020 lbs. 4677.00 
0.021 lbs. 4771.00 
0.020 lbs. 4865.00 
0.020 lbs. 4960.00 






NOW USING 40A MOTOR CONTROLLER: !!!!!!

1,280 us -> 1,600 us

0.002 lbs. 8331.00 
0.002 lbs. 8426.00 
0.001 lbs. 8520.00 
0.001 lbs. 8614.00 
0.001 lbs. 8709.00 
0.014 lbs. 8803.00 
0.057 lbs. 8897.00 
0.079 lbs. 8992.00 
0.080 lbs. 9086.00 
0.079 lbs. 9181.00 
0.079 lbs. 9275.00 
0.080 lbs. 9369.00 
0.080 lbs. 9463.00 
0.080 lbs. 9559.00 
0.079 lbs. 9653.00 
0.080 lbs. 9747.00 
0.080 lbs. 9841.00 
0.079 lbs. 9935.00 
0.079 lbs. 10030.00 
0.083 lbs. 93.00 
0.164 lbs. 188.00 
0.304 lbs. 282.00 
0.336 lbs. 376.00 
0.336 lbs. 471.00 
0.337 lbs. 565.00 
0.336 lbs. 659.00 
0.335 lbs. 753.00 
0.336 lbs. 849.00 
0.337 lbs. 943.00 
0.336 lbs. 1037.00 
0.335 lbs. 1131.00 
0.335 lbs. 1225.00 
0.335 lbs. 1320.00 
0.335 lbs. 1414.00 
0.333 lbs. 1509.00 
0.334 lbs. 1603.00 
0.334 lbs. 1697.00 
0.334 lbs. 1792.00 
0.336 lbs. 1886.00 
0.337 lbs. 1980.00 
0.334 lbs. 2074.00 
0.254 lbs. 2168.00 
0.117 lbs. 2264.00 
0.080 lbs. 2358.00 
0.076 lbs. 2452.00 
0.076 lbs. 2546.00 
0.076 lbs. 2641.00 
0.076 lbs. 2735.00 
0.076 lbs. 2829.00 
0.077 lbs. 2924.00 
0.077 lbs. 3018.00 
0.077 lbs. 3113.00 
0.077 lbs. 3207.00 
0.076 lbs. 3301.00 
0.076 lbs. 3395.00 
0.076 lbs. 3489.00 
0.077 lbs. 3585.00 
0.076 lbs. 3679.00 
0.076 lbs. 3773.00 
0.076 lbs. 3867.00 
0.076 lbs. 3961.00 
0.076 lbs. 4056.00 
0.075 lbs. 4150.00 
0.048 lbs. 4244.00 
0.005 lbs. 4339.00 
-0.002 lbs. 4434.00 
-0.002 lbs. 4528.00 
-0.002 lbs. 4622.00 
-0.002 lbs. 4716.00 
-0.001 lbs. 4810.00 
-0.001 lbs. 4905.00 
-0.001 lbs. 4999.00 



1,120 us -> 1,440 us

0.000 lbs. 8396.00 
0.000 lbs. 8491.00 
0.001 lbs. 8585.00 
0.001 lbs. 8680.00 
0.005 lbs. 8774.00 
0.013 lbs. 8868.00 
0.014 lbs. 8963.00 
0.014 lbs. 9057.00 
0.014 lbs. 9151.00 
0.015 lbs. 9245.00 
0.014 lbs. 9340.00 
0.014 lbs. 9435.00 
0.014 lbs. 9529.00 
0.014 lbs. 9623.00 
0.014 lbs. 9717.00 
0.014 lbs. 9811.00 
0.014 lbs. 9906.00 
0.014 lbs. 10001.00 
0.016 lbs. 93.00 
0.063 lbs. 187.00 
0.163 lbs. 282.00 
0.190 lbs. 376.00 
0.191 lbs. 470.00 
0.191 lbs. 564.00 
0.191 lbs. 659.00 
0.190 lbs. 754.00 
0.190 lbs. 848.00 
0.190 lbs. 942.00 
0.190 lbs. 1036.00 
0.190 lbs. 1130.00 
0.190 lbs. 1226.00 
0.190 lbs. 1320.00 
0.190 lbs. 1414.00 
0.190 lbs. 1508.00 
0.190 lbs. 1602.00 
0.190 lbs. 1697.00 
0.190 lbs. 1792.00 
0.190 lbs. 1886.00 
0.190 lbs. 1980.00 
0.190 lbs. 2075.00 
0.186 lbs. 2169.00 
0.126 lbs. 2263.00 
0.031 lbs. 2357.00 
0.014 lbs. 2452.00 
0.014 lbs. 2547.00 
0.014 lbs. 2641.00 
0.013 lbs. 2735.00 
0.013 lbs. 2829.00 
0.013 lbs. 2923.00 
0.014 lbs. 3018.00 
0.014 lbs. 3113.00 
0.014 lbs. 3207.00 
0.014 lbs. 3301.00 
0.014 lbs. 3396.00 
0.014 lbs. 3490.00 
0.014 lbs. 3584.00 
0.014 lbs. 3678.00 
0.014 lbs. 3773.00 
0.015 lbs. 3868.00 
0.015 lbs. 3962.00 
0.014 lbs. 4056.00 
0.014 lbs. 4150.00 
0.009 lbs. 4244.00 
0.001 lbs. 4340.00 
0.000 lbs. 4434.00 
0.000 lbs. 4528.00 
-0.000 lbs. 4622.00 
-0.000 lbs. 4716.00 
-0.000 lbs. 4811.00 
-0.001 lbs. 4905.00 
-0.000 lbs. 5000.00 



1,440 us -> 1,760 us:

0.001 lbs. 8466.00 
0.001 lbs. 8560.00 
0.001 lbs. 8655.00 
0.001 lbs. 8750.00 
0.018 lbs. 8844.00 
0.113 lbs. 8938.00 
0.184 lbs. 9032.00 
0.192 lbs. 9126.00 
0.192 lbs. 9222.00 
0.191 lbs. 9316.00 
0.191 lbs. 9410.00 
0.191 lbs. 9504.00 
0.190 lbs. 9598.00 
0.191 lbs. 9693.00 
0.191 lbs. 9788.00 
0.191 lbs. 9882.00 
0.190 lbs. 9976.00 
0.190 lbs. 10071.00 
0.194 lbs. 93.00 
0.297 lbs. 187.00 
0.485 lbs. 282.00 
0.529 lbs. 376.00 
0.530 lbs. 471.00 
0.530 lbs. 565.00 
0.529 lbs. 659.00 
0.529 lbs. 753.00 
0.529 lbs. 848.00 
0.528 lbs. 943.00 
0.527 lbs. 1037.00 
0.527 lbs. 1131.00 
0.528 lbs. 1225.00 
0.531 lbs. 1320.00 
0.533 lbs. 1415.00 
0.534 lbs. 1509.00 
0.536 lbs. 1603.00 
0.536 lbs. 1697.00 
0.536 lbs. 1792.00 
0.538 lbs. 1886.00 
0.537 lbs. 1981.00 
0.531 lbs. 2075.00 
0.433 lbs. 2169.00 
0.252 lbs. 2264.00 
0.197 lbs. 2358.00 
0.192 lbs. 2452.00 
0.193 lbs. 2547.00 
0.192 lbs. 2641.00 
0.192 lbs. 2736.00 
0.192 lbs. 2830.00 
0.191 lbs. 2924.00 
0.191 lbs. 3018.00 
0.191 lbs. 3114.00 
0.191 lbs. 3208.00 
0.191 lbs. 3302.00 
0.191 lbs. 3396.00 
0.192 lbs. 3490.00 
0.192 lbs. 3585.00 
0.192 lbs. 3679.00 
0.191 lbs. 3774.00 
0.190 lbs. 3868.00 
0.191 lbs. 3962.00 
0.188 lbs. 4057.00 
0.117 lbs. 4151.00 
0.014 lbs. 4245.00 
-0.000 lbs. 4339.00 
0.000 lbs. 4435.00 
-0.000 lbs. 4529.00 
-0.000 lbs. 4623.00 
-0.000 lbs. 4717.00 
-0.001 lbs. 4811.00 
-0.000 lbs. 4906.00 


1,760 us -> 2,016 us:

0.000 lbs. 8321.00 
0.001 lbs. 8415.00 
0.001 lbs. 8509.00 
0.001 lbs. 8604.00 
0.002 lbs. 8698.00 
0.048 lbs. 8793.00 
0.317 lbs. 8887.00 
0.515 lbs. 8981.00 
0.532 lbs. 9075.00 
0.531 lbs. 9169.00 
0.532 lbs. 9265.00 
0.532 lbs. 9359.00 
0.532 lbs. 9453.00 
0.531 lbs. 9547.00 
0.530 lbs. 9641.00 
0.530 lbs. 9736.00 
0.531 lbs. 9831.00 
0.530 lbs. 9925.00 
0.528 lbs. 10019.00 
0.533 lbs. 94.00 
0.651 lbs. 188.00 
0.848 lbs. 282.00 
0.891 lbs. 376.00 
0.911 lbs. 471.00 
0.914 lbs. 566.00 
0.916 lbs. 660.00 
0.917 lbs. 754.00 
0.915 lbs. 848.00 
0.908 lbs. 942.00 
0.906 lbs. 1037.00 
0.904 lbs. 1132.00 
0.902 lbs. 1226.00 
0.902 lbs. 1320.00 
0.902 lbs. 1415.00 
0.899 lbs. 1509.00 
0.901 lbs. 1603.00 
0.904 lbs. 1698.00 
0.905 lbs. 1792.00 
0.905 lbs. 1887.00 
0.906 lbs. 1981.00 
0.905 lbs. 2075.00 
0.805 lbs. 2169.00 
0.600 lbs. 2264.00 
0.546 lbs. 2359.00 
0.542 lbs. 2453.00 
0.543 lbs. 2547.00 
0.547 lbs. 2641.00 
0.548 lbs. 2735.00 
0.548 lbs. 2830.00 
0.548 lbs. 2925.00 
0.547 lbs. 3019.00 
0.547 lbs. 3113.00 
0.546 lbs. 3208.00 
0.545 lbs. 3302.00 
0.545 lbs. 3396.00 
0.544 lbs. 3491.00 
0.544 lbs. 3585.00 
0.546 lbs. 3680.00 
0.547 lbs. 3774.00 
0.547 lbs. 3868.00 
0.547 lbs. 3962.00 
0.546 lbs. 4057.00 
0.538 lbs. 4152.00 
0.345 lbs. 4246.00 
0.056 lbs. 4340.00 
0.013 lbs. 4434.00 
0.014 lbs. 4528.00 
0.014 lbs. 4623.00 
0.014 lbs. 4718.00 
0.014 lbs. 4812.00 
0.014 lbs. 4906.00 
0.014 lbs. 5001.00 
0.014 lbs. 5095.00 
0.015 lbs. 5189.00 
0.015 lbs. 5283.00 
0.014 lbs. 5378.00 
0.015 lbs. 5473.00 


1,440 us -> 2,016 us:

0.014 lbs. 8260.00 
0.014 lbs. 8354.00 
0.014 lbs. 8449.00 
0.014 lbs. 8543.00 
0.014 lbs. 8637.00 
0.014 lbs. 8731.00 
0.035 lbs. 8826.00 
0.135 lbs. 8921.00 
0.200 lbs. 9015.00 
0.206 lbs. 9109.00 
0.205 lbs. 9203.00 
0.206 lbs. 9297.00 
0.206 lbs. 9392.00 
0.206 lbs. 9487.00 
0.205 lbs. 9581.00 
0.205 lbs. 9675.00 
0.205 lbs. 9769.00 
0.204 lbs. 9864.00 
0.205 lbs. 9958.00 
0.205 lbs. 10052.00 
0.213 lbs. 93.00 
0.414 lbs. 188.00 
0.804 lbs. 282.00 
0.901 lbs. 376.00 
0.904 lbs. 470.00 
0.904 lbs. 564.00 
0.903 lbs. 660.00 
0.903 lbs. 754.00 
0.904 lbs. 848.00 
0.903 lbs. 942.00 
0.906 lbs. 1036.00 
0.908 lbs. 1131.00 
0.908 lbs. 1225.00 
0.907 lbs. 1320.00 
0.905 lbs. 1414.00 
0.909 lbs. 1509.00 
0.914 lbs. 1603.00 
0.911 lbs. 1697.00 
0.910 lbs. 1791.00 
0.911 lbs. 1885.00 
0.912 lbs. 1981.00 
0.903 lbs. 2075.00 
0.716 lbs. 2169.00 
0.356 lbs. 2263.00 
0.234 lbs. 2357.00 
0.214 lbs. 2452.00 
0.212 lbs. 2546.00 
0.210 lbs. 2641.00 
0.210 lbs. 2735.00 
0.212 lbs. 2829.00 
0.211 lbs. 2924.00 
0.209 lbs. 3018.00 
0.209 lbs. 3112.00 
0.209 lbs. 3206.00 
0.209 lbs. 3302.00 
0.209 lbs. 3396.00 
0.209 lbs. 3490.00 
0.207 lbs. 3584.00 
0.207 lbs. 3678.00 
0.207 lbs. 3773.00 
0.207 lbs. 3867.00 
0.207 lbs. 3962.00 
0.204 lbs. 4056.00 
0.135 lbs. 4150.00 
0.030 lbs. 4245.00 
0.015 lbs. 4339.00 
0.015 lbs. 4433.00 
0.015 lbs. 4527.00 



*/





/*
	Input: current location information, receiver commands
	Output: series of accelerations that will result in control around target points

	At first, hard code in rotational acceleration limits and apply the max acceleration in direction needed

*/
void Control::acceleration_controller(Location::LOCATION * Location, Receiver::RECEIVER * Receiver)
	{

		//Translational Accelerations
		float total_translational_acceleration = Receiver->thrust * 17; //Max total acceleration set to 17 m/s^2

		if (Receiver->thrust < 0.1){
			total_translational_acceleration = 0;
		}


		//0 for accelerating straight up in body frame, 1.57 (rads) for directly forwards
		 //Dial goes from -1 to 1.  Set to 1.745 (100 degrees) so can multiply by 100 to get actual angle
		float translational_acceleration_vector_angle = 0;//Receiver->dial1 * 1.745; //CHANGE!!!!!!!


		//Z and X acceleration from total desired and desired vector
		Accelerations.x_acceleration = total_translational_acceleration * sin(translational_acceleration_vector_angle);

		Accelerations.z_acceleration = total_translational_acceleration * cos(translational_acceleration_vector_angle);



		//Angular Accelerations
		float max_roll_angle = 0.3; //Radians
		float roll_target = Receiver->roll * max_roll_angle;

		float max_pitch_angle = 0.3; //Radians
		float pitch_target = Receiver->pitch * max_pitch_angle;

		float max_yaw_angle = 0.3; //Radians
		float yaw_target = Receiver->yaw * max_yaw_angle;


		//Implementatation of max(0, total_translational_acceleration - 2) bc max() not working (may be Eigen library)
		float clipped_translational_acceleration = total_translational_acceleration - 2;
		if (total_translational_acceleration < 2){
			clipped_translational_acceleration = 0;
		}

		float max_hover_roll_acceleration = 0.7 * sqrt(clipped_translational_acceleration);
 		float max_cruise_roll_acceleration = 0.75 * sqrt(total_translational_acceleration);
 		float max_roll_acceleration = max_hover_roll_acceleration + Receiver->dial1 * (max_cruise_roll_acceleration - max_hover_roll_acceleration);


		//Helps to graph in Desmos.  Zero until 2 m/s^2 translational, then shifted (by 2) square root from there and scaled down to be at 2 rad/s^2 when at 10 m/s^2
 		float max_hover_pitch_acceleration = 0.7 * sqrt(clipped_translational_acceleration);
 		float max_cruise_pitch_acceleration = 0.75 * sqrt(total_translational_acceleration);
 		float max_pitch_acceleration = max_hover_pitch_acceleration + Receiver->dial1 * (max_cruise_pitch_acceleration - max_hover_pitch_acceleration);


 		float max_hover_yaw_acceleration = 0.15 * sqrt(clipped_translational_acceleration);
 		float max_cruise_yaw_acceleration = 0.15 * sqrt(total_translational_acceleration);
 		float max_yaw_acceleration = max_hover_yaw_acceleration + Receiver->dial1 * (max_cruise_yaw_acceleration - max_hover_yaw_acceleration);



		float radians_to_roll_target = roll_target - Location->roll; //Positive if need to roll right more to hit target
		float radians_to_pitch_target = pitch_target - Location->pitch; //Positive if need to pitch right more to hit target
		//IMPORTANT NOTE: fix before yawing more than 180 degrees
		float radians_to_yaw_target = yaw_target - Location->yaw; //Positive if need to yaw right more to hit target


		float transient_time = 0.5 * Receiver->dial1;

		//Max rate possible to decelerate down from at current distance from target
		//NOTE: Always positive.  Treat as absolute value of maximum angular rate.
		float max_roll_rate_abs_value = max_roll_acceleration * sqrt(2 * copysignf(radians_to_roll_target,1) / max_roll_acceleration);
		float max_pitch_rate_abs_value = max_pitch_acceleration * sqrt(2 * copysignf(radians_to_roll_target,1) / max_pitch_acceleration);
		float max_yaw_rate_abs_value = max_yaw_acceleration * sqrt(2 * copysignf(radians_to_roll_target,1) / max_yaw_acceleration);

		float abs_value_radians_from_target_to_transient_start_roll = 0.5 * pow((Location->roll_rate / max_roll_acceleration),2) * max_roll_acceleration
			  - 0.5 * max_roll_acceleration * pow(transient_time,2) + max_roll_acceleration * pow(transient_time, 2) / 3
			  + copysignf(Location->roll_rate, 1) * transient_time;


		float abs_value_radians_from_target_to_transient_start_pitch = 0.5 * pow((Location->pitch_rate / max_pitch_acceleration),2) * max_pitch_acceleration
			  - 0.5 * max_pitch_acceleration * pow(transient_time,2) + max_pitch_acceleration * pow(transient_time, 2) / 3
			  + copysignf(Location->pitch_rate, 1) * transient_time;


		//Roll Control
		//If rolling left
		if (Location->roll_rate < 0){
			
			//If rolling left but far away from the point where need to slow down
			if (Location->roll > abs_value_radians_from_target_to_transient_start_roll){
				//Copysignf(x,y) takes two floats (hence the f) and outputs a value with the magnitude of x and sign of y
				Accelerations.roll_acceleration = -max_roll_acceleration;
			}
			else{
				//If within deceleration range, start decelerating
				Accelerations.roll_acceleration = max_roll_acceleration;
			}

		}
		//If rolling right
		else{
			//If rolling right but far away from the point where need to slow down
			if (Location->roll < -abs_value_radians_from_target_to_transient_start_roll){
				//Copysignf(x,y) takes two floats (hence the f) and outputs a value with the magnitude of x and sign of y
				Accelerations.roll_acceleration = max_roll_acceleration;
			}
			else{
				//If within deceleration range, start decelerating
				Accelerations.roll_acceleration = - max_roll_acceleration;
			}

		}


		//Pitch Control
		//If pitching down
		if (Location->pitch_rate < 0){
			
			//If pitching down but far away from the point where need to slow down
			if (Location->pitch > abs_value_radians_from_target_to_transient_start_pitch){
				//Copysignf(x,y) takes two floats (hence the f) and outputs a value with the magnitude of x and sign of y
				Accelerations.pitch_acceleration = -max_pitch_acceleration;
			}
			else{
				//If within deceleration range, start decelerating
				Accelerations.pitch_acceleration = max_pitch_acceleration;
			}

		}
		//If pitching up
		else{
			//If pitching up but far away from the point where need to slow down
			if (Location->pitch < -abs_value_radians_from_target_to_transient_start_pitch){
				//Copysignf(x,y) takes two floats (hence the f) and outputs a value with the magnitude of x and sign of y
				Accelerations.pitch_acceleration = max_pitch_acceleration;
			}
			else{
				//If within deceleration range, start decelerating
				Accelerations.pitch_acceleration = - max_pitch_acceleration;
			}

		}

		//Yaw Control - n
		//If yawing slower than the max roll rate allowed at this point, apply max acceleration in direction of target yaw angle
		if (abs(Location->yaw_rate) < max_yaw_rate_abs_value){
			//Copysignf(x,y) takes two floats (hence the f) and outputs a value with the magnitude of x and sign of y
			Accelerations.yaw_acceleration = copysignf(max_yaw_acceleration, radians_to_yaw_target);
		}
		//Otherwise apply max acceleration in direction away from target pitch angle
		else{
			//Copysignf(x,y) takes two floats (hence the f) and outputs a value with the magnitude of x and sign of y
			Accelerations.yaw_acceleration = copysignf(max_yaw_acceleration, -radians_to_yaw_target);
		}

		///*
		Serial.println("");
		Serial.print("Cmd Acc, rpyxz: ");
		Serial.print(Accelerations.roll_acceleration); Serial.print(" ");
		Serial.print(Accelerations.pitch_acceleration); Serial.print(" ");
		Serial.print(Accelerations.yaw_acceleration); Serial.print(" ");
		Serial.print(Accelerations.x_acceleration); Serial.print(" ");
		Serial.print(Accelerations.z_acceleration); Serial.print(" ");
		Serial.println("");
		//*/

	}





/*
	Input: roll, pitch, yaw angular acceleration (rad/s^2), z acceleration (m/s^2), x acceleration (m/s^2)
	Output: series of forces (N) and moments (Nm) that will result in input accelerations (from fans only - nothing from wing, body aero)

*/
void Control::accelerations_to_forces_and_moments(Control::ACCELERATIONS * Accelerations)
	{

		//Torque = Moment of Inertia * Angular Acceleration
		Target_Forces_and_Moments.roll_moment = roll_moment_of_inertia * Accelerations->roll_acceleration;
		Target_Forces_and_Moments.pitch_moment = pitch_moment_of_inertia * Accelerations->pitch_acceleration;
		Target_Forces_and_Moments.yaw_moment = yaw_moment_of_inertia * Accelerations->yaw_acceleration;

		//Force = Mass * Acceleration
		Target_Forces_and_Moments.x_force = vehicle_mass * Accelerations->x_acceleration;
		Target_Forces_and_Moments.z_force = vehicle_mass * Accelerations->z_acceleration;


		/*
		Serial.println("");
		Serial.print("Commanded Forces and Moments [roll moment, pitch, yaw, x force, z force]: ");
		Serial.print(Target_Forces_and_Moments.roll_moment); Serial.print(" ");
		Serial.print(Target_Forces_and_Moments.pitch_moment); Serial.print(" ");
		Serial.print(Target_Forces_and_Moments.yaw_moment); Serial.print(" ");
		Serial.print(Target_Forces_and_Moments.x_force); Serial.print(" ");
		Serial.print(Target_Forces_and_Moments.z_force); Serial.print(" ");
		Serial.println("");
		*/

	}


/*
	Input: Series of forces (N) from all fans, series of fan angles (rads), motor directions, motor x/y locations, motor torque per thrust
	Output: None.  Update Theoretical_Fan_Forces_and_Moments struct.

*/
MatrixXf Control::calculate_theoretical_fan_forces_and_moments(MatrixXf control_guess)
	{

		float motor_thrusts[8] = {control_guess(0,0), control_guess(1,0), control_guess(2,0), control_guess(3,0), control_guess(4,0), control_guess(5,0), control_guess(6,0), control_guess(7,0)};

		//Servo angles mapped to match with array of motors
		float servo_angles_mapped[8] = {control_guess(8,0), control_guess(9,0), control_guess(10,0), control_guess(11,0), control_guess(10,0), control_guess(11,0), control_guess(10,0), control_guess(11,0)};

		float fan_i_roll_moment;
		float fan_i_pitch_moment;
		float fan_i_yaw_moment;
		float fan_i_x_thrust;
		float fan_i_z_thrust;

		float total_roll_moment = 0;
		float total_pitch_moment = 0;
		float total_yaw_moment = 0;
		float total_x_force = 0;
		float total_z_force = 0;

		for (int i=0; i<8; i++)
    	{

			fan_i_roll_moment = -motor_thrusts[i] * motor_x_locs[i] * cos(servo_angles_mapped[i]) + motor_directions[i] * motor_torque_per_thrust * motor_thrusts[i] * sin(servo_angles_mapped[i]);
			total_roll_moment = total_roll_moment + fan_i_roll_moment;

			fan_i_pitch_moment = motor_thrusts[i] * motor_y_locs[i] * cos(servo_angles_mapped[i]);
			total_pitch_moment = total_pitch_moment + fan_i_pitch_moment;

			fan_i_yaw_moment = -motor_thrusts[i] * motor_x_locs[i] * sin(servo_angles_mapped[i]) - motor_directions[i] * motor_torque_per_thrust * motor_thrusts[i] * cos(servo_angles_mapped[i]);
			total_yaw_moment = total_yaw_moment + fan_i_yaw_moment;

			fan_i_x_thrust = motor_thrusts[i] * sin(servo_angles_mapped[i]);
			total_x_force = total_x_force + fan_i_x_thrust;

			fan_i_z_thrust = motor_thrusts[i] * cos(servo_angles_mapped[i]);
			total_z_force = total_z_force + fan_i_z_thrust;

		}

		//Set Theoretical_Fan_Forces_and_Moments to latest values.
		Theoretical_Fan_Forces_and_Moments.roll_moment = total_roll_moment;
		Theoretical_Fan_Forces_and_Moments.pitch_moment = total_pitch_moment;
		Theoretical_Fan_Forces_and_Moments.yaw_moment = total_yaw_moment;

		Theoretical_Fan_Forces_and_Moments.x_force = total_x_force;
		Theoretical_Fan_Forces_and_Moments.z_force = total_z_force;


		MatrixXf theoretical_forces_and_moments_from_control_guess(1,5);
		theoretical_forces_and_moments_from_control_guess(0,0) = Theoretical_Fan_Forces_and_Moments.roll_moment;
		theoretical_forces_and_moments_from_control_guess(0,1) = Theoretical_Fan_Forces_and_Moments.pitch_moment;
		theoretical_forces_and_moments_from_control_guess(0,2) = Theoretical_Fan_Forces_and_Moments.yaw_moment;
		theoretical_forces_and_moments_from_control_guess(0,3) = Theoretical_Fan_Forces_and_Moments.x_force;
		theoretical_forces_and_moments_from_control_guess(0,4) = Theoretical_Fan_Forces_and_Moments.z_force;


		//Serial.print("Theoretical Forces: ");
		//print_mtxf(theoretical_forces_and_moments_from_control_guess);

		return theoretical_forces_and_moments_from_control_guess;


	}



/*
	Input: Series of forces (N) from all fans, series of fan angles (rads), motor directions, motor x/y locations, motor torque per thrust
	Output: None.  Update kinematics_derivatives variable.

*/
	
MatrixXf Control::calculate_kinematics_derivatives(MatrixXf control_guess)
	{

		float motor_thrusts[8] = {control_guess(0,0), control_guess(1,0), control_guess(2,0), control_guess(3,0), control_guess(4,0), control_guess(5,0), control_guess(6,0), control_guess(7,0)};

		//Servo angles mapped to match with array of motors
		float servo_angles_mapped[8] = {control_guess(8,0), control_guess(9,0), control_guess(10,0), control_guess(11,0), control_guess(10,0), control_guess(11,0), control_guess(10,0), control_guess(11,0)};

		//Derivatives of 3 moments and 2 forces with respect to thrust and servo angle
		MatrixXf kinematics_derivatives(12,5);

		//Derivatives of moments and forces with respect to servo angle for each motor
		MatrixXf servo_kinematics_derivatives(8,5);

		//Get derivatives of roll, pitch, and yaw moments, x and z thrusts with respect to motor thrust at given thrust point and servo angle point for each motor
		for (int i=0; i<8; i++)
    	{

			kinematics_derivatives(i,0) = -motor_x_locs[i] * cos(servo_angles_mapped[i]) + motor_directions[i] * motor_torque_per_thrust * sin(servo_angles_mapped[i]);

			kinematics_derivatives(i,1) = motor_y_locs[i] * cos(servo_angles_mapped[i]);

			kinematics_derivatives(i,2) = -motor_x_locs[i] * sin(servo_angles_mapped[i]) - motor_directions[i] * motor_torque_per_thrust * cos(servo_angles_mapped[i]);

			kinematics_derivatives(i,3) = sin(servo_angles_mapped[i]);

			kinematics_derivatives(i,4) = cos(servo_angles_mapped[i]);

		}


		//Get derivatives of roll, pitch, and yaw moments, z and x thrusts with respect to servo angle at given thrust point and servo angle for each motor
		for (int i=0; i<8; i++)
    	{

			servo_kinematics_derivatives(i,0) = motor_thrusts[i] * motor_x_locs[i] * sin(servo_angles_mapped[i]) + motor_directions[i] * motor_torque_per_thrust * motor_thrusts[i] * cos(servo_angles_mapped[i]);

			servo_kinematics_derivatives(i,1) = -motor_thrusts[i] * motor_y_locs[i] * sin(servo_angles_mapped[i]);

			servo_kinematics_derivatives(i,2) = -motor_thrusts[i] * motor_x_locs[i] * cos(servo_angles_mapped[i]) + motor_directions[i] * motor_torque_per_thrust * motor_thrusts[i] * sin(servo_angles_mapped[i]);

			servo_kinematics_derivatives(i,3) = motor_thrusts[i] * cos(servo_angles_mapped[i]);

			servo_kinematics_derivatives(i,4) = -motor_thrusts[i] * sin(servo_angles_mapped[i]);

		}


		kinematics_derivatives.row(8) = servo_kinematics_derivatives.row(0);
		kinematics_derivatives.row(9) = servo_kinematics_derivatives.row(1);
		kinematics_derivatives.row(10) = servo_kinematics_derivatives.row(2) + servo_kinematics_derivatives.row(4) + servo_kinematics_derivatives.row(6);
		kinematics_derivatives.row(11) = servo_kinematics_derivatives.row(3) + servo_kinematics_derivatives.row(5) + servo_kinematics_derivatives.row(7);

		return kinematics_derivatives;

	}



/*
	Inputs:
		-Series of target moments and forces from fans
		-Estimated fan thrust and servo angle required to generate given target moments and forces
		-Motor directions, motor x/y locations, motor torque per thrust

	Output: None.  Update estimated_control_input variable.

*/
MatrixXf Control::calculate_gradient_descended_control_guess(MatrixXf control_guess, MatrixXf target_forces_and_moments)
	{


		//200 micros on first iteration, 520 micros on second iteration
		MatrixXf theoretical_forces_and_moments_from_control_guess = calculate_theoretical_fan_forces_and_moments(control_guess);
		//Serial.print("Theoretical Forces: ");
		//print_mtxf(theoretical_forces_and_moments_from_control_guess);




		MatrixXf deltas_to_target_forces_and_moments(1,5);
		deltas_to_target_forces_and_moments = target_forces_and_moments - theoretical_forces_and_moments_from_control_guess;
		//Serial.print("Deltas to Target Forces: ");
		//print_mtxf(deltas_to_target_forces_and_moments);


		//Serial.print("Control guess: ");
		//print_mtxf(control_guess);



		//360 micros on first iteration, 960 on second iteration (more as get closer?)
		MatrixXf kinematics_derivatives = calculate_kinematics_derivatives(control_guess);
		//Serial.print("Kinematics derivatives: ");
		//print_mtxf(kinematics_derivatives);



		//Takes around 800-1000 microseconds
		MatrixXf estimated_control_input_deltas_from_control_guess = kinematics_derivatives.transpose().fullPivLu().solve(deltas_to_target_forces_and_moments.transpose());
		//Serial.print("Estimated input deltas: ");
		//print_mtxf(estimated_control_input_deltas_from_control_guess);


		//completeOrthogonalDecomposition() - 19,091 micros entire control loop
		//fullPivLu() - 9,970 micros entire loop
		//partialPivLu() - 6,252 micros entire loop. !!Doesn't work. Matrix must be invertible, can't find solution for sample initial condition.
		//colPivHouseholderQr() - 14,513 micros entire loop
		//householderQr() - 9762 micros entire loop.  !!Doesn't work.  Some matrix values are NaN.
		//llt() - doesn't run.  Matrix must be positive definite.
		//ldlt() - doesn't run.  Matrix must be positive or negative semidefinite.
		//fullPivHouseholderQr() - 11783 micros entire loop


		MatrixXf estimated_control_input(12,1);
		estimated_control_input = control_guess + estimated_control_input_deltas_from_control_guess;
		//Serial.print("Estimated inputs: ");
		//print_mtxf(estimated_control_input);


		return estimated_control_input;


	}







/*
	Input: roll, pitch, yaw moments (Nm), z thrust (N), x thrust (N)
	Output: series of motor thrusts (N) and servo angles (radians) that will result in input forces and moments after transient

*/
void Control::forces_and_moments_to_thrusts_and_angles(Control::FORCES_AND_MOMENTS * Target_Forces_and_Moments)
	{

		//Initial guess for servo angles in radians from forces required
		float avg_servo_angle = atan2(Target_Forces_and_Moments->x_force, Target_Forces_and_Moments->z_force);
		float avg_thrust = sqrt(pow(Target_Forces_and_Moments->x_force, 2) + pow(Target_Forces_and_Moments->z_force,2)) / 8; 

		//Initial guess at motor thrusts and servo angles that will generate the target moments and forces
		MatrixXf control_guess(1,12);

		for (int i=0; i<8; i++)
    	{
    		control_guess(0,i) = avg_thrust;
		}
		for (int i=8; i<12; i++)
    	{
    		control_guess(0,i) = avg_servo_angle;
		}


		//Serial.println("START: ");


		//Serial.print("Control guess: ");
		//print_mtxf(control_guess);
		
		

		//Derivatives of moments and forces with respect to servo angle for each motor
		MatrixXf target_forces_and_moments(1,5);
		target_forces_and_moments(0,0) = Target_Forces_and_Moments->roll_moment;
		target_forces_and_moments(0,1) = Target_Forces_and_Moments->pitch_moment;
		target_forces_and_moments(0,2) = Target_Forces_and_Moments->yaw_moment;
		target_forces_and_moments(0,3) = Target_Forces_and_Moments->x_force;
		target_forces_and_moments(0,4) = Target_Forces_and_Moments->z_force;
		//Serial.print("Target forces: ");
		//print_mtxf(target_forces_and_moments);


		//1460 micros for first one, 2600 micros for second one (harder as gets closer to answer?)
		MatrixXf new_guess = calculate_gradient_descended_control_guess(control_guess, target_forces_and_moments);
		//Serial.print("Guess #1: ");
		//print_mtxf(new_guess);




		//MatrixXf estimated = calculate_theoretical_fan_forces_and_moments(new_guess);
		//Serial.print("Estimated forces #1: ");
		//print_mtxf(estimated);



		MatrixXf estimated_control_input = calculate_gradient_descended_control_guess(new_guess, target_forces_and_moments);
		//Serial.print("Guess #2: ");
		//print_mtxf(estimated_control_input);



		//MatrixXf estimated = calculate_theoretical_fan_forces_and_moments(estimated_control_input);
		//Serial.print("Estimated forces #2: ");
		//print_mtxf(estimated);



		//estimated_control_input = calculate_gradient_descended_control_guess(estimated_control_input, target_forces_and_moments);
		//Serial.print("Guess #3: ");
		//print_mtxf(estimated_control_input);


		//estimated = calculate_theoretical_fan_forces_and_moments(estimated_control_input);
		//Serial.print("Estimated forces #3: ");
		//print_mtxf(estimated);

/*int start = micros();
		int end = micros();

		Serial.print("Time: ");
		Serial.println(end-start);*/


		//Output thrusts from matrix in Newtons
		Thrusts_and_Angles.front_right_fan_thrust = estimated_control_input(0,0);
		Thrusts_and_Angles.front_left_fan_thrust = estimated_control_input(1,0);
		Thrusts_and_Angles.back_right_fan_thrust = estimated_control_input(2,0);
		Thrusts_and_Angles.back_left_fan_thrust = estimated_control_input(3,0);
		Thrusts_and_Angles.back_mid_right_fan_thrust = estimated_control_input(4,0);
		Thrusts_and_Angles.back_mid_left_fan_thrust = estimated_control_input(5,0);
		Thrusts_and_Angles.back_far_right_fan_thrust = estimated_control_input(6,0);
		Thrusts_and_Angles.back_far_left_fan_thrust = estimated_control_input(7,0);

		//Output servo angles from matrix in Radians
		Thrusts_and_Angles.front_right_servo_angle = estimated_control_input(8,0);
		Thrusts_and_Angles.front_left_servo_angle = estimated_control_input(9,0);
		Thrusts_and_Angles.back_right_servo_angle = estimated_control_input(10,0);
		Thrusts_and_Angles.back_left_servo_angle = estimated_control_input(11,0);




		Serial.println("");
		Serial.print("Newtons and Rads: ");
		Serial.print(Thrusts_and_Angles.front_right_fan_thrust); Serial.print(" ");
		Serial.print(Thrusts_and_Angles.front_left_fan_thrust); Serial.print(" ");
		Serial.print(Thrusts_and_Angles.back_right_fan_thrust); Serial.print(" ");
		Serial.print(Thrusts_and_Angles.back_left_fan_thrust); Serial.print(" ");
		Serial.print(Thrusts_and_Angles.back_mid_right_fan_thrust); Serial.print(" ");
		Serial.print(Thrusts_and_Angles.back_mid_left_fan_thrust); Serial.print(" ");
		Serial.print(Thrusts_and_Angles.back_far_right_fan_thrust); Serial.print(" ");
		Serial.print(Thrusts_and_Angles.back_far_left_fan_thrust); Serial.print(" ");

		Serial.print(Thrusts_and_Angles.front_right_servo_angle); Serial.print(" ");
		Serial.print(Thrusts_and_Angles.front_left_servo_angle); Serial.print(" ");
		Serial.print(Thrusts_and_Angles.back_right_servo_angle); Serial.print(" ");
		Serial.print(Thrusts_and_Angles.back_left_servo_angle); Serial.print(" ");
		Serial.println("");

		

	}




/*
	Input: series of thrusts to all motors in Newtons
	Output: series of pwms that will result in input thrusts after transient

	Mapping of pwm to thrust as a function of:
		-Fan unit type
		-Battery voltage
		-Inflow velocity
		-Inflow angle

*/
void Control::thrusts_and_angles_to_pwms(Control::THRUSTS_AND_ANGLES * Thrusts_and_Angles)
	{


	float batt_voltage = 11.8; //Set to actual value in future


	/*
	Fill motor parameter values in structs.  If for more than one variable that has some struct type, needs to be done within function.
		(Otherwise done as an initialization, but doesn't make sense to initialize to more than one set of values)
	May want to write another function that only gets called once at the beginning that adds this data to the structs.
	*/
	motor_1400kv.voltage1 = 11.1;
	motor_1400kv.voltage1_min_pwm = 1020;
	motor_1400kv.voltage1_hover_thrust_exponent = 1.65;
	motor_1400kv.voltage1_hover_thrust_curve_coefficient = 660;
	motor_1400kv.voltage2 = 12.6;
	motor_1400kv.voltage2_min_pwm = 1020;
	motor_1400kv.voltage2_hover_thrust_exponent = 1.61;
	motor_1400kv.voltage2_hover_thrust_curve_coefficient = 590;


	motor_1500kv.voltage1 = 11.1;
	motor_1500kv.voltage1_min_pwm = 1050;
	motor_1500kv.voltage1_hover_thrust_exponent = 1.55;
	motor_1500kv.voltage1_hover_thrust_curve_coefficient = 610;
	motor_1500kv.voltage2 = 12.6;
	motor_1500kv.voltage2_min_pwm = 1050;
	motor_1500kv.voltage2_hover_thrust_exponent = 1.5;
	motor_1500kv.voltage2_hover_thrust_curve_coefficient = 530;

	front_right_servo.zero_deg_pwm = 950;
	front_right_servo.ninety_deg_pwm = 2000;

	front_left_servo.zero_deg_pwm = 950;
	front_left_servo.ninety_deg_pwm = 2000;

	back_right_servo.zero_deg_pwm = 950;
	back_right_servo.ninety_deg_pwm = 2000;

	back_left_servo.zero_deg_pwm = 950;
	back_left_servo.ninety_deg_pwm = 2000;


		

	//1589 micros for all 8 to execute
	PWM_Output.front_right_fan_micros = motor_thrust_to_pwm(&motor_1500kv, Thrusts_and_Angles->front_right_fan_thrust, batt_voltage);
	PWM_Output.front_left_fan_micros = motor_thrust_to_pwm(&motor_1500kv, Thrusts_and_Angles->front_left_fan_thrust, batt_voltage);

	PWM_Output.back_right_fan_micros = motor_thrust_to_pwm(&motor_1400kv, Thrusts_and_Angles->back_right_fan_thrust, batt_voltage);
	PWM_Output.back_left_fan_micros = motor_thrust_to_pwm(&motor_1400kv, Thrusts_and_Angles->back_left_fan_thrust, batt_voltage);

	PWM_Output.back_mid_right_fan_micros = motor_thrust_to_pwm(&motor_1400kv, Thrusts_and_Angles->back_mid_right_fan_thrust, batt_voltage);
	PWM_Output.back_mid_left_fan_micros = motor_thrust_to_pwm(&motor_1400kv, Thrusts_and_Angles->back_mid_left_fan_thrust, batt_voltage);

	PWM_Output.back_far_right_fan_micros = motor_thrust_to_pwm(&motor_1500kv, Thrusts_and_Angles->back_far_right_fan_thrust, batt_voltage);
	PWM_Output.back_far_left_fan_micros = motor_thrust_to_pwm(&motor_1500kv, Thrusts_and_Angles->back_far_left_fan_thrust, batt_voltage);
	

	PWM_Output.front_right_servo_micros = servo_angle_to_pwm(&front_right_servo, Thrusts_and_Angles->front_right_servo_angle);
	PWM_Output.front_left_servo_micros = servo_angle_to_pwm(&front_left_servo, Thrusts_and_Angles->front_left_servo_angle);
	PWM_Output.back_right_servo_micros = servo_angle_to_pwm(&back_right_servo, Thrusts_and_Angles->back_right_servo_angle);
	PWM_Output.back_left_servo_micros = servo_angle_to_pwm(&back_left_servo, Thrusts_and_Angles->back_left_servo_angle);
/*

	PWM_Output.front_right_servo_micros = servo_angle_to_pwm(&front_right_servo, 0);
	PWM_Output.front_left_servo_micros = servo_angle_to_pwm(&front_left_servo, 0);
	PWM_Output.back_right_servo_micros = servo_angle_to_pwm(&back_right_servo, 0);
	PWM_Output.back_left_servo_micros = servo_angle_to_pwm(&back_left_servo, 0);
*/

	
	Serial.println("");
	Serial.print("PWM: ");
	Serial.print(PWM_Output.front_right_fan_micros); Serial.print(" ");
	Serial.print(PWM_Output.front_left_fan_micros); Serial.print(" ");
	Serial.print(PWM_Output.back_right_fan_micros); Serial.print(" ");
	Serial.print(PWM_Output.back_left_fan_micros); Serial.print(" ");

	Serial.print(PWM_Output.back_mid_right_fan_micros); Serial.print(" ");
	Serial.print(PWM_Output.back_mid_left_fan_micros); Serial.print(" ");
	Serial.print(PWM_Output.back_far_right_fan_micros); Serial.print(" ");
	Serial.print(PWM_Output.back_far_left_fan_micros); Serial.print(" ");

	Serial.print(PWM_Output.front_right_servo_micros); Serial.print(" ");
	Serial.print(PWM_Output.front_left_servo_micros); Serial.print(" ");
	Serial.print(PWM_Output.back_right_servo_micros); Serial.print(" ");
	Serial.print(PWM_Output.back_left_servo_micros); Serial.print(" ");
	Serial.println("");
	

	}



/*
	Input: Servo parameters, servo angle in radians
	Output: PWM signal length in microseconds that will result in input angle after transient

*/
int Control::servo_angle_to_pwm(Control::SERVO_PARAMETERS * Servo_Parameters, float servo_angle)
	{

	//Fraction from vertical angle (0) to horizontal (1).  Could be negative to indicate backwards tilt or >1 when past horizontal.
	float angle_fraction = servo_angle / (M_PI / 2);

	//Microsecond command interpolated/extrapolated from zero and ninety degree values
	int PWM_micros = Servo_Parameters->zero_deg_pwm + (Servo_Parameters->ninety_deg_pwm - Servo_Parameters->zero_deg_pwm) * angle_fraction;

	return PWM_micros;

	}



/*
	Input: Motor parameters, thrust in Newtons, battery voltage (probably airspeed, inflow angle, etc in future)
	Output: PWM signal length in microseconds that will result in input thrust after transient

	Time: 196 microseconds (timed from inside function, stopping right before "return").  Calculating pow() takes >90% of the time in this function.

*/
int Control::motor_thrust_to_pwm(Control::FAN_PARAMETERS * Fan_Parameters, float motor_thrust, float batt_voltage)
	{




	//Fraction of from min to max voltage
	float voltage_fraction = (batt_voltage - Fan_Parameters->voltage1) / (Fan_Parameters->voltage2 - Fan_Parameters->voltage1);

	//Interpolate between voltages 1 and 2 for all values
	float min_pwm_thrust = Fan_Parameters->voltage1_min_pwm + (Fan_Parameters->voltage2_min_pwm - Fan_Parameters->voltage1_min_pwm) * Fan_Parameters->voltage_fraction;

	float operating_voltage_hover_thrust_exponent = Fan_Parameters->voltage1_hover_thrust_exponent + (Fan_Parameters->voltage2_hover_thrust_exponent - Fan_Parameters->voltage1_hover_thrust_exponent) * Fan_Parameters->voltage_fraction;

	float operating_voltage_hover_thrust_curve_coefficient = Fan_Parameters->voltage1_hover_thrust_curve_coefficient + (Fan_Parameters->voltage2_hover_thrust_curve_coefficient - Fan_Parameters->voltage1_hover_thrust_curve_coefficient) * Fan_Parameters->voltage_fraction;

	float test = (1/operating_voltage_hover_thrust_exponent);
	float test2 = motor_thrust / 4.448;

	
	//Changing to a function that doesn't have pow() reduces total function time from 196 micros to 12 micros
	float PWM_micros_float = pow((motor_thrust / 4.448), (1/operating_voltage_hover_thrust_exponent)) * operating_voltage_hover_thrust_curve_coefficient + min_pwm_thrust;

	int PWM_micros = int(PWM_micros_float);


	if (motor_thrust == 0){
		PWM_micros = 1000;
	}

	return PWM_micros;

	}




/*
	Input: series of pwms to all motors and servos
	Output: None

	Sets PWMs for each motor on GPIO pins

*/
void Control::send_pwms(Control::PWM_OUTPUT * PWM_Output)
{

	//Send motor PWMs to GPIOs
	send_microseconds(FRONT_RIGHT_MOT_PIN, PWM_Output->front_right_fan_micros);
	send_microseconds(FRONT_LEFT_MOT_PIN, PWM_Output->front_left_fan_micros);

	send_microseconds(BACK_RIGHT_MOT_PIN, PWM_Output->back_right_fan_micros);
	send_microseconds(BACK_LEFT_MOT_PIN, PWM_Output->back_left_fan_micros);

	send_microseconds(BACK_MID_RIGHT_MOT_PIN, PWM_Output->back_mid_right_fan_micros);
	send_microseconds(BACK_MID_LEFT_MOT_PIN, PWM_Output->back_mid_left_fan_micros);

	send_microseconds(BACK_FAR_RIGHT_MOT_PIN, PWM_Output->back_far_right_fan_micros);
	send_microseconds(BACK_FAR_LEFT_MOT_PIN, PWM_Output->back_far_left_fan_micros);


	//Send servo PWMs to GPIOs
	send_microseconds(FRONT_RIGHT_SERVO_PIN, PWM_Output->front_right_servo_micros);
	send_microseconds(FRONT_LEFT_SERVO_PIN, PWM_Output->front_left_servo_micros);

	send_microseconds(BACK_RIGHT_SERVO_PIN, PWM_Output->back_right_servo_micros);
	send_microseconds(BACK_LEFT_SERVO_PIN, PWM_Output->back_left_servo_micros);

}



/*
Send pulse train of certain number of microseconds to a certain pin, at whatever frequency is being used
until rewritten by a new analogWrite

Nearest_bit_value must be between 0 and 255 inclusive, so microseconds must be from 0 to 2490
(technically a few microseconds beyond that because of rounding, but no reason to do that) 

At 400Hz, pulses are actually 2490us apart (2490us between starts, ends)
Each one goes up by 9.765 us (confirmed to the thosandths place by oscilloscope, and equals 2500 / 256)
Value of 254 is 2,480.0 us long, value of 1 is 9.765 long
0 is always off, 255 is always on
*/
void Control::send_microseconds(int motor_pin, int microseconds)
{

	if (microseconds > 2500){
		microseconds = 2500;
	}
	//float analog_write_bit_us = 1000000.0f / (float)MOTORS_PWM_FREQ / 255;

	float precise_bit_value = (float)microseconds / 10;

	int nearest_bit_value = int(precise_bit_value); //Changed from round() because Eigen library seems to redefine round() and messes this up

	analogWrite(motor_pin, nearest_bit_value);

}




/*
	Run the control system functions and send the output directly to the motors

*/
void Control::run(Location::LOCATION * Location, Receiver::RECEIVER * Receiver)
	{

		

		//Get desired accelerations based on location and receiver commands.
		acceleration_controller(Location, Receiver); //32 microseconds

		

		//forces and moments required to generate accelerations from controller
		accelerations_to_forces_and_moments(&Accelerations); //6 microseconds

		

		//Get thrusts and angles required from motors and servos to generate desired forces and moments
		forces_and_moments_to_thrusts_and_angles(&Target_Forces_and_Moments); //4113 microseconds

		

		//Get pwms resulting in thrusts and angles after transient
		thrusts_and_angles_to_pwms(&Thrusts_and_Angles); //1713 microseconds



		//Send PWMs directly to GPIO pins
		send_pwms(&PWM_Output); //87 microseconds

		Serial.print("Loop: ");
		Serial.println(micros());

		if (millis() > 5000){
			delay(1000000);
		}



	}











void Control::print_mtxf(const Eigen::MatrixXf& X)  
{
   int i, j, nrow, ncol;
   
   nrow = X.rows();
   ncol = X.cols();

   //Serial.print("nrow: "); Serial.println(nrow);
   //Serial.print("ncol: "); Serial.println(ncol);       
   //Serial.println();
   
   for (i=0; i<nrow; i++)
   {
       for (j=0; j<ncol; j++)
       {
           Serial.print(X(i,j));   // print 6 decimal places
           Serial.print(", ");
       }
       Serial.println();
   }
   Serial.println();
}










void Control::run2(Location::LOCATION * Location, Receiver::RECEIVER * Receiver)
	{




	float Pitch_P = 0.32;
	float Roll_P = 0.08;

	float Pitch_D = 0.015;
	float Roll_D = 0.007;

	float fr_x = 37.5; //X is front-back axis, front is +
	float fr_y = 9; //Y is left-right axis, right is +

	float fl_x = 37.5;
	float fl_y = -9;

	float brc_x = -12.5;
	float brc_y = 15;

	float blc_x = -12.5;
	float blc_y = -15;


	float max_roll_angle = 15; //Degrees
	float roll_target = Receiver->roll * max_roll_angle;
	//roll_target = 0; //CHANGE!!!

	float max_pitch_angle = 25; //Degrees
	float pitch_target = Receiver->pitch * max_pitch_angle;
	//pitch_target = 0; //CHANGE!!!



	//Something quick to stabilize yaw
	float yaw_slew = 0; //Max rate of change in desired yaw - degrees per second
	dt = micros() - l_time;
	//Handle problem where very first dt is huge because l_time starts at 0.
	if (l_time == 0){
		dt = 10000;
	}
	l_time = micros();
	float desired_yaw_delta = yaw_slew * Receiver->yaw * dt / 1000000;
	//desired_yaw_delta = 0; //CHANGE!!!

	//desired_yaw = desired_yaw + desired_yaw_delta; //Desired yaw in degrees from 0 to 360.  For slewing desired yaw angle.  Hard to control.

	float max_yaw_delta = 20; //Max desired deviation from starting yaw.  Yaw in center always desired yaw = starting yaw.
	desired_yaw = Receiver->yaw * max_yaw_delta;
	desired_yaw = 0; //CHANGE!!!

	if (desired_yaw > 360){
		desired_yaw = desired_yaw - 360;
	}
	if(desired_yaw < 0){
		desired_yaw = desired_yaw + 360;
	}

	//Yaw error from -360 to 360.  So if at 20 deg and desired yaw is 10 deg, should be 10.  At 0, should be -10.
	float yaw_error = Location->yaw - desired_yaw;
	if (yaw_error > 180){
		yaw_error = yaw_error - 360;
	}
	if(yaw_error < -180){
		yaw_error = yaw_error + 360;
	}



	//Rate control method - try to get to a desired roll, pitch, yaw rate.  Probably should be some function of error (Ardupilot does sqrt)
	//Linear around 0, then goes to square root (or else very fast rate change around 0)
	//Even without any D gain, this is super stable - disturbances have a little shake but not much
	float pitch_rate_target;
	if (abs(pitch_target - Location->pitch) > 2){
		//copysignf (f for float) makes value of first argument the sign (+ or -) of second argument
		pitch_rate_target = 6 * copysignf(sqrt(abs(pitch_target - Location->pitch)-1), (pitch_target - Location->pitch));//5 deg->12dps, 10dps>18deg/s
	}
	else{
		pitch_rate_target = (pitch_target - Location->pitch) * 3;
	}

	float roll_rate_target;
	if (abs(roll_target - Location->roll) > 2){
		//copysignf (f for float) makes value of first argument the sign (+ or -) of second argument
		roll_rate_target = 6 * copysignf(sqrt(abs(roll_target - Location->roll)), (roll_target - Location->roll)); //If rolled right, should be neg
	}
	else{
		roll_rate_target = (roll_target - Location->roll) * 3;
	}
	

	//roll_rate_target = roll_rate_target * 3 * Receiver->dial1;


	float yaw_rate_target;
	if (abs(yaw_error) > 2){
		//copysignf (f for float) makes value of first argument the sign (+ or -) of second argument
		yaw_rate_target = 6 * copysignf(sqrt(abs(yaw_error)), (-yaw_error)); //Negative if yawed right
	}
	else{
		yaw_rate_target = (-yaw_error) * 3;
	}
	yaw_rate_target = constrain(yaw_rate_target, -45, 45);


	Pitch_Rate_P = 0.03; //How much to accelerate towards target rate.  More than about 0.03 on Deathtrap and starts to shake.
	Roll_Rate_P = 0.03; //0.1, with sqrt p = 6, -> 1.8 for 10 deg with no rate (3 and 1 for 10 deg, no rate PID pitch, roll)
	Yaw_Rate_P = 0.09;



	//Serial.println(" ");
	//Serial.print("Roll_Rate_P = ");  Serial.print(Roll_Rate_P);

	float pitch_rate_error = (Location->pitch_rate - pitch_rate_target);
	float roll_rate_error = (Location->roll_rate - roll_rate_target); //If rolled right at no rate, should be positive
	float yaw_rate_error = (Location->yaw_rate - yaw_rate_target); //If yawed right and no rate, should be positive



	float pitch_rate_error_deriv = (pitch_rate_error - pitch_rate_error_3) / (3*dt / 1000000); //

	float roll_rate_error_deriv = (roll_rate_error - roll_rate_error_3) / (3*dt / 1000000);



	//Adding in D term for Location->pitch_rate, etc. always makes more shaky - tried adjusting with dial from 0-0.04.  0 best.
	//For rate_error_derivs, starts to shudder around 0.0015.  0.0005 no real shudder but makes control feel tighter (oscillations immediately damped)
	//pitch_rate_error_int * 0.0015, above 0.0015 it starts oscillations
	float Pitch_Thrust = pitch_rate_error * Pitch_Rate_P + pitch_rate_error_int * 0 + pitch_rate_error_deriv * 0.0005;
	//Pitch_Thrust = 0;

	float Roll_Thrust = roll_rate_error * Roll_Rate_P + roll_rate_error_deriv * 0.0005;
	//Roll_Thrust = 0;

	float Yaw_Thrust = yaw_rate_error * Yaw_Rate_P; //If yawed right and no rate, should be positive
	//Yaw_Thrust = 0;

	//Record most recent roll, pitch rates.  Super stable with just P rate gain
	roll_rate_error_3 = roll_rate_error_2;
	roll_rate_error_2 = roll_rate_error_1;
	roll_rate_error_1 = roll_rate_error;

	pitch_rate_error_3 = pitch_rate_error_2;
	pitch_rate_error_2 = pitch_rate_error_1;
	pitch_rate_error_1 = pitch_rate_error;

	yaw_rate_error_3 = yaw_rate_error_2;
	yaw_rate_error_2 = yaw_rate_error_1;
	yaw_rate_error_1 = yaw_rate_error;

	roll_rate_3 = roll_rate_2;
	roll_rate_2 = roll_rate_1;
	roll_rate_1 = Location->roll_rate;

	pitch_rate_3 = pitch_rate_2;
	pitch_rate_2 = pitch_rate_1;
	pitch_rate_1 = Location->pitch_rate;

	yaw_rate_3 = yaw_rate_2;
	yaw_rate_2 = yaw_rate_1;
	yaw_rate_1 = Location->yaw_rate;



	//PD method
	/*
	float Pitch_Thrust = (Location->pitch - pitch_target) * Pitch_P + Location->pitch_rate * Pitch_D;

	float Roll_Thrust = (Location->roll - roll_target) * Roll_P + Location->roll_rate * Roll_D; //If rolled right at no rate, should be positive

	float Yaw_Thrust = yaw_error * 0.08 + Location->yaw_rate * 0.015;
	*/


	//Pitch thrust should go from full value at 0 deg, half at 45 deg, none at 90 deg (try sinusoid later)
	float pitch_thrust_fr = Pitch_Thrust * (90 - Commands.front_right_servo) / 90;//0 degrees, use 0 roll tilt, at 45 deg, half, 90 deg, full tilt
	float pitch_thrust_fl = Pitch_Thrust * (90 - Commands.front_left_servo) / 90;
	float pitch_thrust_br = Pitch_Thrust * (90 - Commands.back_right_servo) / 90;
	float pitch_thrust_bl = Pitch_Thrust * (90 - Commands.back_left_servo) / 90;


	//Roll thrust should go from full value at 0 deg, half at 45 deg, none at 90 deg (try sinusoid later)
	float roll_thrust_fr = Roll_Thrust * (90 - Commands.front_right_servo) / 90;//0 degrees, use 0 roll tilt, at 45 deg, half, 90 deg, full tilt
	float roll_thrust_fl = Roll_Thrust * (90 - Commands.front_left_servo) / 90;
	float roll_thrust_br = Roll_Thrust * (90 - Commands.back_right_servo) / 90;
	float roll_thrust_bl = Roll_Thrust * (90 - Commands.back_left_servo) / 90;


	//Yaw thrust should go from nothing at 0 deg, half at 45 deg, full at 90 deg (try sinusoid later)
	float yaw_thrust_fr = Yaw_Thrust * (Commands.front_right_servo) / 90;
	float yaw_thrust_fl = Yaw_Thrust * (Commands.front_left_servo) / 90;
	float yaw_thrust_br = Yaw_Thrust * (Commands.back_right_servo) / 90;
	float yaw_thrust_bl = Yaw_Thrust * (Commands.back_left_servo) / 90;


	Commands.front_right = constrain(Receiver->thrust -pitch_thrust_fr / fr_x + roll_thrust_fr / fr_y + yaw_thrust_fr / fr_y, 0, 1);
	Commands.front_left = constrain(Receiver->thrust -pitch_thrust_fl / fl_x + roll_thrust_fl / fl_y + yaw_thrust_fl / fl_y, 0, 1);

	Commands.back_right = constrain(Receiver->thrust -pitch_thrust_br / brc_x + roll_thrust_br / brc_y + yaw_thrust_br / brc_y, 0, 1);
	Commands.back_left = constrain(Receiver->thrust -pitch_thrust_bl / blc_x + roll_thrust_bl / blc_y + yaw_thrust_bl / blc_y, 0, 1);

	Commands.back_mid_right = constrain(Receiver->thrust -pitch_thrust_br / brc_x + roll_thrust_br / brc_y + yaw_thrust_br / brc_y, 0, 1);
	Commands.back_mid_left = constrain(Receiver->thrust -pitch_thrust_bl / blc_x + roll_thrust_bl / blc_y + yaw_thrust_bl / blc_y, 0, 1);

	Commands.back_far_right = constrain(Receiver->thrust -pitch_thrust_br / brc_x + roll_thrust_br / brc_y + yaw_thrust_br / brc_y, 0, 1);
	Commands.back_far_left = constrain(Receiver->thrust -pitch_thrust_bl / blc_x + roll_thrust_bl / blc_y + yaw_thrust_bl / blc_y, 0, 1);

/*
	Serial.println(Receiver->thrust);
	Serial.println(Pitch_Thrust);
	Serial.println(Roll_Thrust);
	Serial.println(Yaw_Thrust);
*/


	//Arming: Aux1 (top far left) must go all the way down (can start anywhere) and all the way up to arm
	//If Aux1 starts down (1) or goes down (to 1), set Aux1_down true (always starts false)
	if (Receiver->aux1 < 0.3){
		Aux1_been_up = true;
	}
	
	if (Receiver->aux1 > 0.7){
		Aux1_been_down = true;
	}

	/*
	Arming off for now

	if (not Aux1_been_up || not Aux1_been_down){
		Commands.front_right = 0;
		Commands.front_left = 0;
		Commands.back_right = 0;
		Commands.back_left = 0;
		Commands.back_mid_right = 0;
		Commands.back_mid_left = 0;
		Commands.back_far_right = 0;
		Commands.back_far_left = 0;
	}
	*/


	if (Receiver->thrust < 0.1){

		Commands.front_right = 0;
		Commands.front_left = 0;
		Commands.back_right = 0;
		Commands.back_left = 0;
		Commands.back_mid_right = 0;
		Commands.back_mid_left = 0;
		Commands.back_far_right = 0;
		Commands.back_far_left = 0;

	}




	float PD_Yaw = yaw_rate_error * 0.2 + roll_rate_error_deriv * 0; //0.2
	float yaw_tilt_fr = PD_Yaw * (90 - Commands.front_right_servo) / 90;//0 degrees, use full yaw tilt, at 45 deg, half as much, 90 deg, none
	float yaw_tilt_fl = PD_Yaw * (90 - Commands.front_left_servo) / 90;
	float yaw_tilt_br = PD_Yaw * (90 - Commands.back_right_servo) / 90;
	float yaw_tilt_bl = PD_Yaw * (90 - Commands.back_left_servo) / 90;

	//Intuitively, if rolled 5 deg, might want 2-3 deg arm tilt (just guess based on yaw effectiveness), 45 deg/sec might need 4-5 deg tilt
	float PD_Roll_Tilt = roll_rate_error * 0.2 + roll_rate_error_deriv * 0; //0.2
	float roll_tilt_fr = PD_Roll_Tilt * (Commands.front_right_servo) / 90;//0 degrees, use 0 roll tilt, at 45 deg, half, 90 deg, full tilt
	float roll_tilt_fl = PD_Roll_Tilt * (Commands.front_left_servo) / 90;
	float roll_tilt_br = PD_Roll_Tilt * (Commands.back_right_servo) / 90;
	float roll_tilt_bl = PD_Roll_Tilt * (Commands.back_left_servo) / 90;

	//Just guess, if pitched up 5 deg, might want 2-3 deg arm tilt, 45 deg/sec might need 4-5 deg tilt
	float PD_Pitch_Tilt = pitch_rate_error * 0.6 + pitch_rate_error_deriv * 0; //0.3
	float pitch_tilt_fr = PD_Pitch_Tilt * (Commands.front_right_servo) / 90;//0 degrees, use 0 pitch tilt, at 45 deg, half, 90 deg, full tilt
	float pitch_tilt_fl = PD_Pitch_Tilt * (Commands.front_left_servo) / 90;
	float pitch_tilt_br = PD_Pitch_Tilt * (Commands.back_right_servo) / 90;
	float pitch_tilt_bl = PD_Pitch_Tilt * (Commands.back_left_servo) / 90;


	//Quick thing to limit slew rate on master tilt (controls still move at max rate)
	float max_tilt_rate_slow = 15; //Deg/sec max tilt rate of master_tilt
	float max_tilt_rate_fast = 30; //Deg/sec max tilt rate of master_tilt


	//If flipped up (0), move to forward position at slow speed (no matter what)
	//If flipped to the middle (0.5), move to hover position at slow speed (no matter what)
	//If flipped down (1), move to reverse thrust at fast speed (no matter what)
	if (Receiver->aux2 < 0.25){
		max_tilt_rate = max_tilt_rate_slow;
		desired_tilt = Receiver->dial1 * 100;
	}

	if (Receiver->aux2 >= 0.25 && Receiver->aux2 <= 0.75){
		max_tilt_rate = max_tilt_rate_slow;
		desired_tilt = 25;
	}

	if (Receiver->aux2 > 0.75){
		max_tilt_rate = max_tilt_rate_fast;
		desired_tilt = 0;
	}


	//Flipping aux1 up makes it go to hover position fast no matter what
	if (Receiver->aux1 < 0.7){
		max_tilt_rate = max_tilt_rate_fast;
		desired_tilt = 0;
	}


	if (dt > 20000){
		dt = 10000;
	}


	if (master_tilt < desired_tilt){
		master_tilt = master_tilt + max_tilt_rate * dt / 1000000; //(dt from above)
	}
	else{
		master_tilt = master_tilt - max_tilt_rate * dt / 1000000;
	}

	//Limit backwards tilt to avoid servo crushing into itself
	if (master_tilt < -32){
		master_tilt = -32;
	}

/*
	Serial.println(" ");
	Serial.print("master_tilt: "); Serial.print(master_tilt);
	Serial.println(" ");
	Serial.print("des tilt: "); Serial.print(desired_tilt);
	Serial.println(" ");
	Serial.print("yaw: "); Serial.print(yaw_tilt_fr);
*/

	Commands.front_right_servo = master_tilt + constrain(yaw_tilt_fr, -10, 10) - constrain(roll_tilt_fr, -18, 18) + constrain(pitch_tilt_fr, -18, 18);
	Commands.front_left_servo = master_tilt - constrain(yaw_tilt_fl, -10, 10) + constrain(roll_tilt_fl, -18, 18) + constrain(pitch_tilt_fl, -18, 18);

	Commands.back_right_servo = master_tilt + constrain(yaw_tilt_br, -10, 10) - constrain(roll_tilt_br, -18, 18) - constrain(pitch_tilt_br, -18, 18);
	Commands.back_left_servo = master_tilt  - constrain(yaw_tilt_bl, -10, 10) + constrain(roll_tilt_bl, -18, 18) - constrain(pitch_tilt_br, -18, 18);




/*
	Serial.println(Commands.front_right_servo);
	Serial.println(Commands.front_left_servo);
	Serial.println(Commands.back_right_servo);
	Serial.println(Commands.back_far_right_servo);
	Serial.println(Commands.back_left_servo);
	Serial.println(Commands.back_far_left_servo);

	Serial.println(" ");
	Serial.print("Loop: "); Serial.print(dt); Serial.println(" ");
*/

}


void Control::Log()
	{

		//Command checks
		Serial.print("Cmds: ");
	
		Serial.print(Commands.front_right); Serial.print(" ");
		Serial.print(Commands.front_left); Serial.print(" ");
		Serial.print(Commands.back_right); Serial.print(" ");
		Serial.print(Commands.back_left); Serial.print(" ");
		Serial.print(Commands.back_mid_right); Serial.print(" ");
		Serial.print(Commands.back_mid_left); Serial.print(" ");
		Serial.print(Commands.back_far_right); Serial.print(" ");
		Serial.print(Commands.back_far_left); Serial.print(" ");

		Serial.print(Commands.front_right_servo); Serial.print(" ");
		Serial.print(Commands.front_left_servo); Serial.print(" ");
		Serial.print(Commands.back_right_servo); Serial.print(" ");
		Serial.print(Commands.back_left_servo); Serial.print(" ");

		//Serial.print(Pitch_Rate_P); Serial.print(" ");
		//Serial.print(Roll_Rate_P); Serial.print(" ");

		//Serial.println(micros());

		//Serial.print(dt); Serial.print(" "); //Put in commands instead so reports every time

	

	}


void Control::MotorTest(Receiver::RECEIVER * Receiver)
	{

	//Individual motor testing.  Motor speed is throttle, set motor by combination of aux switches and pitch forward/backward for front/back motor

	if (Receiver->pitch < 0){

		if (Receiver->aux2 > 0.5){
			Commands.front_right = constrain(Receiver->thrust, 0, 1);
			Commands.front_left = constrain(0, 0, 1);

			Commands.back_right = constrain(0, 0, 1);
			Commands.back_left = constrain(0, 0, 1);
			Commands.back_far_right = constrain(0, 0, 1);
			Commands.back_far_left = constrain(0, 0, 1);
		}
		else{
			Commands.front_right = constrain(0, 0, 1);
			Commands.front_left = constrain(Receiver->thrust, 0, 1);

			Commands.back_right = constrain(0, 0, 1);
			Commands.back_left = constrain(0, 0, 1);
			Commands.back_far_right = constrain(0, 0, 1);
			Commands.back_far_left = constrain(0, 0, 1);
		}

	}

	else{

		if (Receiver->aux2 > 0.5){

			if (Receiver->aux1 > 0.5){

				Commands.front_right = constrain(0, 0, 1);
				Commands.front_left = constrain(0, 0, 1);

				Commands.back_right = constrain(Receiver->thrust, 0, 1);
				Commands.back_left = constrain(0, 0, 1);
				Commands.back_far_right = constrain(0, 0, 1);
				Commands.back_far_left = constrain(0, 0, 1);
			}
			else{

				Commands.front_right = constrain(0, 0, 1);
				Commands.front_left = constrain(0, 0, 1);

				Commands.back_right = constrain(0, 0, 1);
				Commands.back_left = constrain(0, 0, 1);
				Commands.back_far_right = constrain(Receiver->thrust, 0, 1);
				Commands.back_far_left = constrain(0, 0, 1);
			}

		}

		else{

			if (Receiver->aux1 > 0.5){

				Commands.front_right = constrain(0, 0, 1);
				Commands.front_left = constrain(0, 0, 1);

				Commands.back_right = constrain(0, 0, 1);
				Commands.back_left = constrain(Receiver->thrust, 0, 1);
				Commands.back_far_right = constrain(0, 0, 1);
				Commands.back_far_left = constrain(0, 0, 1);
			}
			else{
				Commands.front_right = constrain(0, 0, 1);
				Commands.front_left = constrain(0, 0, 1);

				Commands.back_right = constrain(0, 0, 1);
				Commands.back_left = constrain(0, 0, 1);
				Commands.back_far_right = constrain(0, 0, 1);
				Commands.back_far_left = constrain(Receiver->thrust, 0, 1);
			}

		}


	}


	Commands.front_right_servo = Receiver->dial1 * 90;
	Commands.front_left_servo = Receiver->dial1 * 90;
	Commands.back_right_servo = Receiver->dial1 * 90;
	Commands.back_left_servo = Receiver->dial1 * 90;


}
