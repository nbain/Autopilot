#ifndef AUTOPILOT2_MPU9250_H
#define AUTOPILOT2_MPU9250_H //Underscore in MPU_9250 because the Arduino library is MPU9250 - don't want to get confused

//FOR ALL REGISTERS, COMMENT GOES: [Bit7][Bit6][Bit5][Bit4][Bit3][Bit2][Bit1][Bit0]

//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00 // should return 0x48 (read only)
#define INFO             0x01 //Device info (nothing about how to interpret these info bits) (read only)
#define AK8963_ST1       0x02  // Status 1.  [7-2: -] 
/*
	[Bit1: aka DOR, Data Overrun.  Returns 1 if data has been skipped = not read in continuous measurement mode
		Goes to 0 when any of AK8963_XOUT_L to AK8963_ST2 are read]
	[Bit0 = 1 if data is ready, 0 otherwise] (read only)


	If you constantly read this bit without reading from the data registers, it's always 1
	If you constantly read from the data registers, this bit is always 0
*/
#define AK8963_XOUT_L    0x03  // Lower 8 bits of X axis data
#define AK8963_XOUT_H    0x04  // Stored in two's complement, with Little Endian (just like accel, gyro)
#define AK8963_YOUT_L    0x05  //Measurement range is -32,760 to 32,760 (not 32,768)
#define AK8963_YOUT_H    0x06  //32760 corrseponds to 4,912 microTeslas (so 0.15 uT steps), -32760 to -4,912 uT
#define AK8963_ZOUT_L    0x07 
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09 
/*
[7:5 -]
[Bit4: aka BITM, for Bit mirror, because this read-only version mirrors what BIT is set as below.  Read-only.
	Output bit setting - 0 for 14 bit output, 1 for 16 bit output setting.  Set with BIT in CNTL below
	Confirmed that this outputs 0/1 for different sample rates]
[Bit3: aka HOFL = Magnetic Sensor Overflow.  Means measurement data will not be correct due to abs(X) + abs(Y) + abs(Z)
	being higher than 4912 microTeslas.  Not a data corruption problem caused by accident - just too high of magnetic field.  
	When next measurement starts it will return to 0]
[2:0 -]
*/
#define AK8963_CNTL1      0x0A  //Not sure, but might be possible to run single measurement mode at higher than 100 Hz
/*
For faster than 100Hz: Not sure, but supports 400 KHz I2C.
	Nevermind - datasheet says time for a single measurement is 7.2 ms

[7:5 -]
[Bit4: aka BIT. Output bit setting - 0 for 14 bit output (0.6 uT steps), 1 for 16 bit output setting (0.15 uT steps).  ]
[3:0 aka MODE.  Operation mode setting.  
"0000": Power-down mode - power to almost everything off, but registers still accessible and data there not changed
	If not powered down, continuously taking sensor measurements and writing to their output registers
"0001": Single measurement mode - measures one time and powered down
"0010": Continuous measurement mode 1 - 8 Hz sampling
"0110": Continuous measurement mode 2 - 100 Hz sampling
	For both continuous measurement modes, actually powers down in between 8 Hz/100Hz measurements
"0100": External trigger measurement mode
"1000": Self-test mode
"1111": Fuse ROM access mode
Other code settings are prohibited]

*/
#define AK8963_CNTL2     0x0B  //Not defined in old library. [7:1 -] [Bit0: aka SRST. 1 for Soft-Reset, returns to 0 automatically]
//Soft-Reset resets values in registers.
#define AK8963_ASTC      0x0C  // Self-test Control.
/*
[7 -]
[Bit6: aka SELF.  1 to generate magnetic field for self-test, 0 to return to normal.]
[5:0 -]
DANGER: If 1 is written to any other bit in this register, normal measurement does not work!
*/
#define AK8963_I2CDIS    0x0F  // I2C Disable.  Only function is: to disable, write: "00011011"
//I2C bus enabled by default, but if disable it here, only way to get it back is to turn off/on or input start condition 8 times

//Fuse ROM is ROM with a fuse that can be burnt out at the factory so the bits can't be written to
//Set CNTL1 to Fuse ROM access mode to access
#define AK8963_ASAX      0x10  // Sensitivity adjustment stored to Fuse ROM x-axis on shipment (read only)
#define AK8963_ASAY      0x11  // Sensitivity adjustment stored to Fuse ROM y-axis on shipment (read only)
#define AK8963_ASAZ      0x12  // Sensitivity adjustment stored to Fuse ROM z-axis on shipment (read only)
/*
To apply the sensitivity adjustment to the measured data, do:

Hadj = H(measured) * {[(ASA value - 128) * 0.5]/128 + 1}

Haven't confirmed, but ASA almost definitely two's complement

*/


//MPU9250 Registers:

#define SELF_TEST_X_GYRO 0x00  //These are the values gotten for self-tests at the factory for each axis, for accel and gyro
	//11000011
#define SELF_TEST_Y_GYRO 0x01
	//11001101
#define SELF_TEST_Z_GYRO 0x02
	//11100011

/*
A bunch from an old version, presumably - not on register map and also commented out in old library
	Reading them, almost all have data in their registers though

#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D
	//01100011
#define SELF_TEST_Y_ACCEL 0x0E
	//01011010
#define SELF_TEST_Z_ACCEL 0x0F 
	//01110011
#define SELF_TEST_A       0x10  //!! Not on register map - not used anywhere in the library
	//Reads 10110100 though
#define XG_OFFSET_H       0x13  // Gyro offset registers.  Adds or subtracts a certain number of degrees per second
	//00000000
#define XG_OFFSET_L       0x14  //from whatever gyro sensor register indicates.  So give a 16 bit value to indicate
	//00000000
#define YG_OFFSET_H       0x15  //degrees per second to offset.
	//00000000
#define YG_OFFSET_L       0x16  
	//00000000
#define ZG_OFFSET_H       0x17
	//00000000
#define ZG_OFFSET_L       0x18
	//00000000
/*
Offset in degrees per second = int16_t([XG_OFFSET_H][XG_OFFSET_L]) * 4 / (2 ^ [FS_SEL = 0, 1, 2, 3] /  Gyro_sensitivity)

Gyro sensitivity = 2^16/500 = 2^15/250 for FS_SEL = 0 (250 dps range)

With FS_SEL = 0: 32,565 * 4 / (2 ^ 0) / (2^15 / 250) = 999.969 dps offset, exactly what's on the datasheet
-1,000 dps would be max on other side.

*/

#define SMPLRT_DIV        0x19 //Divides the normal update rate of sensor register (from whatever filter mode, etc. being used)
//So makes it sample at a slower rate
	//00000100 - this 

/*
Whole text from register map:

Divides the internal sample rate (see register CONFIG) to generate the sample rate that controls 
sensor data output rate, FIFO sample rate. 
NOTE: This register is only effective when
 Fchoice = 2’b11 (FCHOICE_B register bits in GYRO_CONFIG below are 2’b00), and 
 (0 < dlpf_cfg < 7), such that the average filter’s output is selected ("see chart below" - refers to 'chart' below in CONFIG).

This is the update rate of sensor register.
SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV)

Data should be sampled at or above sample rate; SMPLRT_DIV is only used for 1kHz internal sampling.





*/
#define CONFIG            0x1A //00000011
/*
[Bit 7: -]
[Bit 6: FIFO mode - when 1, additional writes not written to FIFO when FIFO full.  When 0, new writes
replace oldest value in FIFO]
[5:3 - aka EXT_SYNC_SET.  Enables FSYNC Pin data to be sampled - FSYNC pin not being used now on MPU9250 board.
FSYNC is for Frame synchronization digital input - resends with each transmission to avoid bit slips
0 (so 000) to disable]
[2:0 - aka DPLF_CFG.  For bandwidth, delay, sampling frequency settings for DLPF options.  This DLPF is only used by the
gyroscope and temp sensor, not accelerometer.  THIS ONLY WORKS IF GYRO_CONFIG F_CHOICE_B = 00!!
000 (DLPF_CFG 0) would have, for gyro: bandwidth 250 Hz, 0.97 ms delay, 8 KHz sampling	For temp sensor: 4000 Hz bandwidth, 0.04 ms delay
001 (DLPF_CFG 1) would have, for gyro: 184 Hz bandwidth, 2.9 ms delay, 1 Khz sampling	For temp sensor: 188 Hz bandwidth, 1.9 ms delay
010 (DLPF_CFG 2) would have, for gyro: 92 Hz bandwidth, 3.9 ms delay, 1 Khz sampling		For temp sensor: 98 Hz bandwidth, 2.8 ms delay
011 (DLPF_CFG 3) would have, for gyro: 41 Hz bandwidth, 5.9 ms delay, 1 Khz sampling		For temp sensor: 42 Hz bandwidth, 4.8 ms delay
100 (DLPF_CFG 4) would have, for gyro: 20 Hz bandwidth, 9.9 ms delay, 1 Khz sampling		For temp sensor: 20 Hz bandwidth, 8.3 ms delay
101 (DLPF_CFG 5) would have, for gyro: 10 Hz bandwidth, 17.85 ms delay, 1 Khz sampling	For temp sensor: 10 Hz bandwidth, 13.4 ms delay
110 (DLPF_CFG 6) would have, for gyro: 5 Hz bandwidth, 33.48 ms delay, 1 Khz sampling	For temp sensor: 5 Hz bandwidth, 18.6 ms delay
111 (DLPF_CFG 7) would have, for gyro: 3600 Hz bandwidth, 0.17 ms delay, 8 Khz sampling		For temp sensor: 4000 Hz bandwidth, 0.04 ms delay
(this last one not a typo)
]

MPU9250 has 512 byte FIFO, so up to that much at a time can be read by the ARM Cortex at a time


*/
#define GYRO_CONFIG       0x1B //00000000
/*[X self-test(st)][Y st][Z st]
[4:3 select from 250, 500, 1000, 2000 dps range.  00 for 250 
to 11 for 2000]
[-]
[1:0 - aka FCHOICE_B.  To turn on/off digital low pass filter (DLPF) If off, can have two dif bandwiths, sensor delays].
DLPF selection here very confusing - use the table on p. 15 of register map which shows filter/sampling settings for dif
combinations of these two bits.  Confusing because bits used here must be inverted, and that inverted value is the one
to look up in the table.  So, if 11 input to this register, use 00 on table to see what will happen.
- 00 for [1:0] spots: 11 on table -> DLPF on, now possible to configure with CONFIG register bits [2:0]
- 10 for [1:0] spots: 01 on table -> DLPF off, gyro oscillation higher than 3600 Hz won't be detected, 0.11 ms delay to get
sensor value, 32 KHz sampling frequency (analog low pass filter preventing >3600Hz signals?)
- 01 or 11 for [1:0] spots: 10 or 00 on table -> DLPF off, 8800 Hz max gyro oscillation detection, 0.064 ms delay
32 KHz sampling frequency.
*/

#define ACCEL_CONFIG      0x1C //00000000
//[X self-test(st)][Y st][Z st][4:3 select from 2g, 4g, 8g, 16g range, 
//with 00 for 2g to 11 for 16g][2:0 -]

#define ACCEL_CONFIG2     0x1D //00000011
/* 
[7:4 -][Bit 3: 1 to bypass accelerometer DLPF, 0 to enable DLPF 
- Bit3 = 1 (DLPF bypassed): 1046 Hz 3dB bandwidth (where signal still has half its original strength),
4 KHz sample rate, Dec1 filter block (?), 0.503ms delay, 300 micro(g)/rtHz Noise Density (square root of power spectral density
of the noise output, where power spectral density is the power of the noise (so measured signal - actual signal)
vs. frequency.  Noise is Gaussian.  Noise constant across all filter values.)
- [2:0] is 000 or 001: DLPF filter, 218.1 Hz 3dB bandwidth, 1 KHz sampling, 1.88 ms delay
- [2:0] is 010: DLPF filter, 99 Hz 3dB bandwidth, 1 KHz sampling, 2.88 ms delay
- [2:0] is 011: DLPF filter, 44.8 Hz 3dB bandwidth, 1 KHz sampling, 4.88 ms delay
- [2:0] is 100: DLPF filter, 21.2 Hz 3dB bandwidth, 1 KHz sampling, 8.87 ms delay
- [2:0] is 101: DLPF filter, 10.2 Hz 3dB bandwidth, 1 KHz sampling, 16.83 ms delay
- [2:0] is 110: DLPF filter, 5.05 Hz 3dB bandwidth, 1 KHz sampling, 32.48 ms delay
- [2:0] is 111: Dec2 filter (?), 420 Hz 3dB bandwidth, 1 KHz sampling, 1.38 ms delay

This filtering/sampling is only for the accelerometer.

Output Data Rate (ODR) can be reduced (fewer samples per second) with the SMPLRT_DIV register (0x19 - above)
Data rate with SMPLRT_DIV = Normal Data Rate / (1 + SMPLRT_DIV)
All this controlled by a different register though, to be clear

*/
#define LP_ACCEL_ODR      0x1E //Low-power accelerometer output data rate - controlled with 4 bits, lookup with a table
	//00000000
#define WOM_THR           0x1F //Wake-On-Motion interrupt threshold - accel only, can set from 4mg to 1020mg
	//00000000

// Duration counter threshold for motion interrupt generation, 1 kHz rate,
// LSB = 1 ms
#define MOT_DUR           0x20 //!! Not on register map - not used anywhere in library
// Zero-motion detection threshold bits [7:0]
	//00000000
#define ZMOT_THR          0x21 //Not on register map - not used anywhere in library
// Duration counter threshold for zero motion interrupt generation, 16 Hz rate,
// LSB = 64 ms
	//00000000
#define ZRMOT_DUR         0x22 //Not on register map - not used anywhere in library
	//00000000

#define FIFO_EN            0x23 //Each bit controls whether to send something to the FIFO.  0 for disabled, 1 to send
/*that data at the sample rate.  Buffering to FIFO happens even if data path is in standby (Set by power management, PWR_MGMT_1)
[Bit7 - TEMP_OUT_H and TEMP_OUT_L]
[Bit6 - GYRO_XOUT_H and GYRO_XOUT_L]
[Bit5 - GYRO_YOUT_H and GYRO_YOUT_L]
[Bit4 - GYRO_ZOUT_H and GYRO_ZOUT_L]
[Bit3 - ACCELX_OUT_H and ACCELX_OUT_L, and for accel Y axis and Z axis]
[Bit2 - I2C slave 2]
[Bit2 - I2C slave 1]
[Bit2 - I2C slave 0]
	//00000000
*/
#define I2C_MST_CTRL       0x24 //I2C Master Control. [Multimaster and external sensor data stuff] 
//[3:0 - sets I2C master clock speed between 258-500 KHz <- This would only be used to control slaves from MPU9250 because
//it's a master]
	//00000000, and all 00000000 down here
#define I2C_SLV0_ADDR      0x25 //MPU9250 has auxiliary master I2C bus.  Below all for control of these
#define I2C_SLV0_REG       0x26 //e.g. can be used for pressure sensor
#define I2C_SLV0_CTRL      0x27 
#define I2C_SLV1_ADDR      0x28
#define I2C_SLV1_REG       0x29
#define I2C_SLV1_CTRL      0x2A
#define I2C_SLV2_ADDR      0x2B
#define I2C_SLV2_REG       0x2C
#define I2C_SLV2_CTRL      0x2D
#define I2C_SLV3_ADDR      0x2E
#define I2C_SLV3_REG       0x2F
#define I2C_SLV3_CTRL      0x30
#define I2C_SLV4_ADDR      0x31
#define I2C_SLV4_REG       0x32
#define I2C_SLV4_DO        0x33
#define I2C_SLV4_CTRL      0x34
#define I2C_SLV4_DI        0x35
#define I2C_MST_STATUS     0x36 //I2C Master Status - for MPU9250 slaves, and FSYNC interrupt status (nothing relevant w/o slaves)
#define INT_PIN_CFG        0x37 //To control INT pin (not used) and FSYNC as an interrupt (nothing relevant w/o slaves)
	//00100010 
/*
	[Bit0: RESERVED]

	[Bit1: aka BYPASS_EN.  When asserted, the i2c_master interface pins(ES_CL and ES_DA) will go into ‘bypass mode’ 
	when the i2c master interface is disabled. The pins will float high due to the internal pull-up if not enabled 
	and the i2c master interface is disabled.]
		Must be 1 to use magnetometer!!

	[Bit2: aka FSYNC_INT_MODE_EN.  1 – This enables the FSYNC pin to be used as an interrupt.
	A transition to the active level described by the ACTL_FSYNC bit will cause an interrupt.
	The status of the interrupt is read in the I2C Master Status register PASS_THROUGH bit.]

	[Bit3: aka ACTL_FSYNC.  1 – The logic level for the FSYNC pin as an interrupt is active low.]

  	[Bit4: aka INT_ANYRD_2CLEAR.  1 – Interrupt status is cleared if any read operation is performed.
  	0 – Interrupt status is cleared only by reading INT_STATUS register]

  	[Bit5: aka LATCH_INT_EN.  1 – INT pin level held until interrupt status is cleared.
  	0 – INT pin indicates interrupt pulse’s is width 50us. (probably don't want this - might trigger multiple reads)]

  	[Bit6: aka OPEN.  1 – INT pin is configured as open drain.  0 – INT pin is configured as push-pull.]

  	[Bit7: aka ACTL.  1 – The logic level for INT pin is active low.  (Default is 3.3V, drops to 0 to signal interrupt)
  	0 – The logic level for INT pin is active high.]

	*/
#define INT_ENABLE         0x38 //Set things that can trigger INT pin (FIFO overflow, Wake On Motion) (not relevant w/o INT pin)
	//00000001
	/*
	Bit0:  1 – Enable Raw Sensor Data Ready interrupt to propagate to interrupt pin. 
	The timing of the interrupt can vary depending on the setting in register 36 I2C_MST_CTRL, bit [6] WAIT_FOR_ES.
	*/
#define DMP_INT_STATUS     0x39  // Check DMP interrupt
	//00000000
#define INT_STATUS         0x3A //Sets various bits to 1 if different interrupts occur (Wake On Motion, FIFO overflow)
//When MPU9250 on, Bit0, for Raw Data Ready interrupt, goes between 0 and 1 when on, all others zero 
	//Goes between 00000001 and 00000010
	/*
	Bit0:  1 – Sensor Register Raw Data sensors are updated and Ready to be read. 
	The timing of the interrupt can vary depending on the setting in register 36 I2C_MST_CTRL, bit [6] WAIT_FOR_ES.
	Bit1: -- (reserved)
	Bit2: -- (reserved) But this very occasionally goes to one
	*/

#define ACCEL_XOUT_H       0x3B //Get value from 0 to 65,535 with binary of [XOUT_H][XOUT_L].  0-32767 is for positive G's
#define ACCEL_XOUT_L       0x3C //(X arrow on board pointing up), with 0 for 0 positive G's and 32767 for maxing out positive
#define ACCEL_YOUT_H       0x3D //32,768 to 65,536 is for negative  G's.  65,636 is for 0, and down to 32,768 
#define ACCEL_YOUT_L       0x3E //for most negative G's.  To read useful info, must read the byte as int16_t, not uint16_t
#define ACCEL_ZOUT_H       0x3F //Confirmed on datasheet in ADC Word Length field- "accelerometer outputs 2's complement"
#define ACCEL_ZOUT_L       0x40 //Same for Y and Z.  For Z, you can see the "tail" of the arrow - so points into board
#define TEMP_OUT_H         0x41 // Temp in degrees C = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity) + 21degC
#define TEMP_OUT_L         0x42 //Datasheet says sensitivity is 333.87 LSB/deg C.  RoomTemp_Offset is 0 at 21deg C.
//In house, got binary value of 2734, which would be about 84-85F, so seems right.  Below 21C it might go to 65,536.
#define GYRO_XOUT_H        0x43 //Gyro also outputs two's complement
#define GYRO_XOUT_L        0x44
#define GYRO_YOUT_H        0x45
#define GYRO_YOUT_L        0x46
#define GYRO_ZOUT_H        0x47
#define GYRO_ZOUT_L        0x48
#define EXT_SENS_DATA_00   0x49 //This is for external sensors for Slave 0-4 on the auxiliary I2C interface
#define EXT_SENS_DATA_01   0x4A //All these are 00000000
#define EXT_SENS_DATA_02   0x4B
#define EXT_SENS_DATA_03   0x4C
#define EXT_SENS_DATA_04   0x4D
#define EXT_SENS_DATA_05   0x4E
#define EXT_SENS_DATA_06   0x4F
#define EXT_SENS_DATA_07   0x50
#define EXT_SENS_DATA_08   0x51
#define EXT_SENS_DATA_09   0x52
#define EXT_SENS_DATA_10   0x53
#define EXT_SENS_DATA_11   0x54
#define EXT_SENS_DATA_12   0x55
#define EXT_SENS_DATA_13   0x56
#define EXT_SENS_DATA_14   0x57
#define EXT_SENS_DATA_15   0x58
#define EXT_SENS_DATA_16   0x59
#define EXT_SENS_DATA_17   0x5A
#define EXT_SENS_DATA_18   0x5B
#define EXT_SENS_DATA_19   0x5C
#define EXT_SENS_DATA_20   0x5D
#define EXT_SENS_DATA_21   0x5E
#define EXT_SENS_DATA_22   0x5F
#define EXT_SENS_DATA_23   0x60
#define MOT_DETECT_STATUS  0x61 //Not on datasheet - prob just interrupt for "Significant Motion"
	//00000000
#define I2C_SLV0_DO        0x63 //Data out for I2C slaves
#define I2C_SLV1_DO        0x64 //All these 00000000
#define I2C_SLV2_DO        0x65
#define I2C_SLV3_DO        0x66
#define I2C_MST_DELAY_CTRL 0x67 //Controls when I2C slaves can be accessed
	//00000000
#define SIGNAL_PATH_RESET  0x68 //[7:3 -] Bits 0, 1, 2 reset temp, gyro, accel digital signal paths(?), respectively  - not used
	//00000000
#define MOT_DETECT_CTRL    0x69 //Wake-On Motion detection enable with Bit7, Bit6 compare current with last sample, [5:0 -]
	//00000000
#define USER_CTRL          0x6A //From old library: "Bit7 enable DMP, bit3 reset DMP. Bit0 resets 3 * 3 signal paths and sensor reg"
	//00000000
/*
[7 -]
[Bit6: aka FIFO_EN - 1 to enable FIFO operation, 0 to disable FIFO access from serial port]
[Bit5: aka I2C_MST_EN.  1 to enable I2C master interface module]
[Bit4: aka I2C_IF_DIS.  1 to disable I2C slave module and put serial interface in SPI only mode]
[3 -]
[Bit2: aka FIFO_RST.  Resets FIFO module.  Bit autoclears after one clock cycle]
[Bit1: aka I2C_MST_RST.  1 to reset I2C master module.  Autoclears after one clock cycle. Should only be reset if I2C master hangs]
[Bit0: aka SIG_COND_RST.  1 to reset all gyro, accel, and temp signal paths and clear sensor registers]
*/
#define PWR_MGMT_1         0x6B // Device defaults to SLEEP mode.  1 in Bit7 resets internal registers, restores default settings
	//00000001
/*
[Bit7: aka H_RESET.  1 to reset the internal registers and restore default settings.  Bit will autoclear.]
[Bit6: aka SLEEP.  1 to set chip to sleep mode.  Apparently defaults to 1, so default to sleep.]
[Bit5: aka CYCLE.  For Low-power.  When SLEEP above and GYRO_STANDBY below are 0, setting to 1 will cycle 
between sleep and taking a sample at rate given by LP_ACCEL_ODR register above.]
[Bit4: aka GYRO_STANDBY.  For Low-power. 1 to enable gyro drive and PLL, but not the sense paths - used for saving power, but quick wakeup]
[Bit3: aka PD_PTAT. For Low-power. 1 to power down PTAT(Proportional to Absolute Temperature) voltage generator, PTAT analog to digital converter]
[2:0 is aka CLKSEL.  Select either internal 20MHz oscillator or PLL (if ready)]
000 is: Internal 20MHz oscillator
001 through 101 are: auto-select best available clock source - PLL if ready, otherwise 20MHz osciallator
110 is, again: Internal 20MHz oscillator
111 is: stop clock and keep timing generator in reset

*/
#define PWR_MGMT_2         0x6C // 
	//00000000
/*
[7:6 -]
[Bit5: aka DISABLE_XA.  1 to disable X accelerometer, 0 to turn on.]
[Bit4: aka DISABLE_YA.  1 to disable Y accelerometer, 0 to turn on.]
[Bit3: aka DISABLE_ZA.  1 to disable Z accelerometer, 0 to turn on.]
[Bit2: aka DISABLE_XG.  1 to disable X gyro, 0 to turn on.]
[Bit1: aka DISABLE_YG.  1 to disable Y gyro, 0 to turn on.]
[Bit0: aka DISABLE_YG.  1 to disable Z gyro, 0 to turn on.]

*/
#define DMP_BANK           0x6D  // Not in register map, not used in library.  From old: Activates a specific bank in the DMP
	//00000000
#define DMP_RW_PNT         0x6E  // Not in register map, not used in library.  From old: Set read/write pointer to a specific start address in specified DMP bank
	//00000100, 00010000, 00011100 - updates with each frame
#define DMP_REG            0x6F  // Not in register map, not used in library.  From old: Register in DMP from which to read or to which to write
	//01011111, 00001010, 11000101 - updates with each frame
#define DMP_REG_1          0x70  // Not in register map, not used in library.
	//00000000
#define DMP_REG_2          0x71  // Not in register map, not used in library.
	//00000000
#define FIFO_COUNTH        0x72  // [7:5 -] [4:0 - High bits in count of number of written bytes in FIFO]
	//00000000
#define FIFO_COUNTL        0x73  // [7:0 - Low bits in FIFO byte count]	
	//00000000									
#define FIFO_R_W           0x74  //FIFO Read Write.  Used to read and write to the FIFO buffer.  
	//00000000
#define WHO_AM_I_MPU9250   0x75 // ID of device.  Should read 0x71
	//01110001 = 113 = 0x71
#define XA_OFFSET_H        0x77 //Upper bits of 15 bit accel offset. 0.98mg steps, +/-16g offset range
	//11100110
#define XA_OFFSET_L        0x78 //(so 2^15 0.98mg steps makes sense for +/- 16g)  Lower bits of 15 bit offset [7:1] [Bit0 -]
	//10010100
#define YA_OFFSET_H        0x7A //Would be in two's complement
	//11100101
#define YA_OFFSET_L        0x7B
	//11100101
#define ZA_OFFSET_H        0x7D
	//00100010
#define ZA_OFFSET_L        0x7E
	//11010100

// Using the MPU-9250 breakout board, ADO is set to 0
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define ADO 0

#define MPU9250_ADDRESS 0x68
#define MPU9250_ADDRESS1 0x68  // Device address when ADO = 0
#define MPU9250_ADDRESS2 0x69  // Device address when ADO = 1
#define AK8963_ADDRESS  0x0C   // Address of magnetometer



/*
	Registers that matter:



	XG_OFFSET_H to ZG_OFFSET_L

	SMPLRT_DIV - Controls (by dividing) update rate of the sample register ()

	CONFIG - Filter settings

	GYRO_CONFIG - Self-test, DPS range, filter and sample rate

	ACCEL_CONFIG - Set scale (2g probably - using 00000000), and self-test on [7:5]
	
	ACCEL_CONFIG2 - Accelerometer filter, sample rate

	INT_STATUS - Data ready if Bit0 goes to 1

	PWR_MGMT_1 - Reset, sleep, clock

	XA_OFFSET_H to ZA_OFFSET_L



*/










#define MAG_FIELD_STRENGTH 47.336 //Magnetic field strength at current location in uT (microTeslas)

/*
Hard coded scaling and shift for each axis
Calibrating each time not trustworthy - can get spikes or miss highest amount

Best way to ensure all three are good is to take norm of 3 axes, which should be theoretically constant at 47.336.
	-norm of Z arrow pointing straight into magnetic field should be same as pointing straight out
	-Adjust LSB_SHIFTs until close.  Only way to isolate only one axis.
*/

//For X axis, consistent high value (not highest noise value) at 155.  Lowest at -307.  So:
#define X_MAG_SCALING 0.2049 //Would be .2049, but scaled by 0.94 to get closer to (mx^2 + my^2 + mz^2) = (47.336^2)
#define X_MAG_LSB_SHIFT 76 //Would be 76 This should be added to the LSB output.

//-60 to 410
#define Y_MAG_SCALING 0.2014 //Would be 0.2014
#define Y_MAG_LSB_SHIFT -175 //Would be -175

//-90 to 460
#define Z_MAG_SCALING 0.1721 //Would be 0.1721
#define Z_MAG_LSB_SHIFT -185 //Would be -185
//Norm goes from about 42 to 65

#define MPU_INT_PIN 41

/*
	Struct for an individual magnetometer sample's data so that it can be put in FIFO array
*/
struct magDataStruct
{

	int16_t x_axis_uT;
	int16_t y_axis_uT;
	int16_t z_axis_uT;

	int8_t sensor_status; //For data ready/overflow

	int sample_microseconds; //Limit is 2.14 billion, so weird after 35 mins

};

#define IMU_FIFO_SIZE 1000 //Size of the FIFO used for the struct holding accel, gyro, and mag data

#define LOCATION_FIFO_SIZE 20 //Size of the FIFO used for the struct holding location data

#define ACCEL_FIFO_SIZE 20

#define GYRO_ACCEL_ERROR_FIFO_SIZE 1000 //Size of the FIFO holding the error between gyro integrator and filtered accel data
//Smaller (e.g. 100) has better steady state accuracy, but lower response time.
//Bigger has worse steady state accuracy, worse response time.

#define COARSE_DRIFT_ADJ_MAGNITUDE 100000 //Magnitude by which to shift gyro integrator when goes out of bounds
//As long as below about 20 million with a size 100 FIFO (= 2B total), shouldn't have any effect - only optimization between
//adding drift correction numbers each time vs. doing a bunch of epoch transitions


class MPU9250
{

	protected:

	    // Choose either 1 for 16-bit (0.15 μT/step) or 0 for 14-bit (0.6 μT/step) magnetometer resolution
	    uint8_t Mag_resolution_bit = 1; //Confirmed that this does change the BITM bit, which indicates sample rate

	    //0x02 for 8 Hz, 0x06 for 100 Hz sampling rate
	    uint8_t Mag_mode_4bits = 0x06; //This is really a byte, but you just set it to some byte value like
	    //00001111 or 00000000 or 00000001 or 00001000 so that only the last 4 bits are used
	    //This byte is then modified by the resolution bit right above, which is shifted into the 5th from right (Bit4) place


	public:
		
		int16_t mag_x;
		int16_t mag_y;
		int16_t mag_z;

	    float pitch, yaw, roll;
	    float temperature;   // Stores the real internal chip temperature in Celsius
	    int16_t tempCount;   // Temperature raw count output
	    uint32_t delt_t = 0; // Used to control display output rate

	    uint32_t count = 0, sumCount = 0; // used to control display output rate
	    float deltat = 0.0f, sum = 0.0f;  // integration interval for both filter schemes
	    uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
	    uint32_t Now = 0;        // used to calculate integration interval

	    int16_t accelRegisters[4];  // Stores the 16-bit signed accelerometer sensor output + micros() when sampled
	    int16_t gyroRegisters[4];   // Stores the 16-bit signed gyro sensor output
	    int16_t magRegisters[5] = {0,0,0,0,0};    // Stores 16-bit magnetometer sensor output + data ready, no overflow + micros()

	    int16_t accel_FIFO[121];  // Stores the most recent accel measurements and times
	    int16_t gyro_FIFO[41];  // Stores the most recent accel measurements and times

	    double attitude_FIFO[801];

	    double accel_filtered_eulers[4];

	    float accel_gyro_LSB_offsets[6]; //LSB offsets when flat, not moving for 3 axes of accel and gyro

	    float offset_angles[3]; //AttitudeEstimation will find something not quite on zero, so finds what angle needed to correct
	    //Not perfect, but prevents being always a little wrong easily - probably want to look at root cause later
	    //e.g. what are the actual offsets needed, how they change over time (possible to find how they are changing by knowing
	    //vehicle must be straight up on average)

	    double euler_rates[4]; //Angular velocity in DPS for [pitch, roll, not sure, nor sure]

	    double est_eulers[8]; //Last and current eulers and time outputted by filter (current pitch, roll, yaw, time, last pitch...)
	    double final_eulers[4]; //Final values after all filtering, estimation, etc. - what goes into the attitude controller
	    //[pitch, roll, pitch dps, roll_dps]

	    double accelEulers[4];
	    double gyroDPS[4];
	    double magEulers[4];

	    float motorCommands[4]; //This is a hack to get motor commands done quickly

	    // Scale resolutions per LSB for the sensors
	    float aRes, gRes;
	    // Variables to hold latest sensor data values
	    float ax, ay, az, gx, gy, gz, mx, my, mz;
	    // Factory mag calibration and mag bias
	    float mag_adjustment_scaling[3] = {0, 0, 0};
	    float magbias[3] = {0, 0, 0};


	    double shift_destination[3] = {0,0,0};
	    double scale_destination[3] = {0,0,0};
	    

	    // Bias corrections for gyro and accelerometer
	    float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};
	    float SelfTest[6];
	    

	    int16_t registerData[6]; //For reading out registers for testing

		/*
		Struct for IMU FIFO's.  Stores IMU data as a list of each of the three axes and associated sample times.

		Not exactly sure why, but has to be defined in this MPU9250 class and not up above - multiple definitions error
		*/
		struct IMU_FIFO
		{

			int16_t x_axis[IMU_FIFO_SIZE];
			int16_t y_axis[IMU_FIFO_SIZE];
			int16_t z_axis[IMU_FIFO_SIZE];

			unsigned int sample_us[IMU_FIFO_SIZE]; //Largest unsigned int is 4.2 billion, so weird after a more than hour

			int most_recent_write = IMU_FIFO_SIZE - 1; //Value from 0 to [FIFO size - 1] to indicate where most recent value is stored
			//Note: starts as [IMU_FIFO_SIZE - 1] for last write because that would mean this next write is to the first location

		} ;

		//Idea behind ..._current is that may want to save a certain FIFO during flight (startup? etc.) for reference
		IMU_FIFO accelFIFO_current; //Should always have the most recent accel data.
		IMU_FIFO gyroFIFO_current; //Most recent gyro data
		IMU_FIFO magFIFO_current; //Most recent mag data


		
		/*
		This FIFO contains the estimated attitude history (the actual output used by control law)

		Note: this is only the estimate given the data at that point in time.  Given future data, might estimate something
		different from what is in here.

		Struct for location in space - orientation and position - and their first and second derivatives wrt time

		*/
		struct LOCATION_FIFO
		{

			double pitch[LOCATION_FIFO_SIZE]; //In degrees (for now - might want degrees in float later)
			double roll[LOCATION_FIFO_SIZE];
			double yaw[LOCATION_FIFO_SIZE];

			double pitch_rate[LOCATION_FIFO_SIZE]; //In dps.Found from looking at pitch delta and delta t - no new information
			double roll_rate[LOCATION_FIFO_SIZE];
			double yaw_rate[LOCATION_FIFO_SIZE];

			float pitch_acc[LOCATION_FIFO_SIZE];
			float roll_acc[LOCATION_FIFO_SIZE];
			float yaw_acc[LOCATION_FIFO_SIZE];

			//Will fill in position - lat, long, height MSL - when ready


			
			unsigned int timestamp[LOCATION_FIFO_SIZE]; //In microseconds.  Limit for each of these is 4.3 billion, so weird after an hour
			int last_delta_t[LOCATION_FIFO_SIZE]; //In microseconds.  Should be found from last timesteps in FIFO - no new information

			int most_recent_write = LOCATION_FIFO_SIZE - 1; //Value from 0 to [FIFO size - 1] to show where most recent value is stored
			//Note: starts as [LOCATION_FIFO_SIZE - 1] for last write because that would mean this next write is to the first location

		};

		LOCATION_FIFO location_current; //Most recent location data

		/*
		Struct for accel FIFO analysis.  Different from IMU FIFO in that that is just registers, times, whereas this data has 
		been analyzed - e.g. trig for angles, filtering.

		*/
		struct ACCEL_DATA
		{

			unsigned int current_timestamp; //Timestamp of most recent measurement in microseconds
			int last_delta_t; //Most recent delta t between samples in microseconds

			int accel_FIFO[ACCEL_FIFO_SIZE];
			int most_recent_write_loc = ACCEL_FIFO_SIZE - 1;
			int most_recent_write_timestamp = 0;

			int accel_FIFO_summation = 0;
			int accel_FIFO_avg = 0;

			int filtered_udeg; //Moving average

			int last_udeg;


		} ;

		ACCEL_DATA accel_pitch_data_current;
		ACCEL_DATA accel_roll_data_current;
		ACCEL_DATA accel_yaw_data_current;

		/*
		Struct for gyro FIFO analysis.  For now, finds angle deltas, but any parameters calculated by analyzing
		the gyro FIFO can be put here (maybe moving averages, regressions)

		*/
		struct GYRO_DATA
		{

			unsigned int current_timestamp; //Timestamp of most recent measurement in microseconds
			int last_delta_t; //Most recent delta t between samples in microseconds

			int last_delta_udeg; //If going at current sample angular rate for last timestep - maybe in future do trapezoidal

			int last_udeg_sec; //Most recent angular rate - just a conversion to udeg/sec

			double filtered_dps;

			int gyro_integrator_udeg; //Used by Gyro Integrator - will drift, but never by more than 100,000 udeg
			//If no movement, would look like a sawtooth wave (drifts, corrects, drifts, corrects)

			int gyro_integrator_udeg_test;

			double complementary_udeg;

			unsigned int last_timestamp_integrated; //Used to tell whether a timestamp has already been put in the integrator

			//Might want more below - e.g. moving average, etc.

		} ;

		GYRO_DATA gyro_pitch_data_current;
		GYRO_DATA gyro_roll_data_current;
		GYRO_DATA gyro_yaw_data_current;


		/*
		Struct for intermediate layer location analysis.  For storing calculated values that are used for analyzing/filtering
		location data.

		*/
		struct GYRO_ACCEL_ERROR_DATA
		{

			//List of delta values in microdegrees between gyro integrated and filtered accel attitudes
			//The sign of the error is: Gyro value - Accel value
			double gyro_accel_error_FIFO[GYRO_ACCEL_ERROR_FIFO_SIZE];
			int most_recent_write_loc = GYRO_ACCEL_ERROR_FIFO_SIZE - 1;
			int most_recent_write_timestamp = 0;

			double gyro_accel_error_FIFO_summation = 0;
			double gyro_accel_error_FIFO_avg = 0;

			double filtered_gyro_accel_error_value = 0;
			double filtered_gyro_accel_error_slope = 0;

			double follow_accel_value_test = 0;
			double follow_accel_slope_test = 0;



		} ;

		GYRO_ACCEL_ERROR_DATA gyro_accel_error_pitch_current;
		GYRO_ACCEL_ERROR_DATA gyro_accel_error_roll_current;


	public:

		MPU9250();

		void init();
	    void readAccel1();
	    void readAccel2();
	    void readGyro1();
	    void readGyro2();
	    void readMag();

		void initI2C();
		void initMPU_Interrupt();

	    void getGres();
	    void getAres();
	    void readRegisters(int num_registers); //For testing

	    void writeAccelRegistersFIFO();
	    void writeGyroRegistersFIFO();
	    void writeMagRegistersFIFO();

	    void calcAccelData();
	    void calcGyroData();
	    void calcMagData();

	    void CompFilter();

	    double GyroIntegrationDriftCorrection(GYRO_ACCEL_ERROR_DATA *, GYRO_DATA *, ACCEL_DATA *);
	    void GyroIntegrator(GYRO_DATA *); //Returns latest value gotten by the gyro integrator (in the function)
	    //and can use input argument to coarse adjust the integrator after large drifts


	    void findInitialAccelGyroOffsets(float *);
	    void findOffsets(float *);

	    void accelFilter();

	    //void ComplementaryFilter(double *, int16_t *, double *, double *);
	    void findEulers(double *);

	    void writeAttitudeFIFO(double *, double *);

	    void getAccelEulers(int16_t * , double *);
	    void getGyroDPS(int16_t * , double *);
	    void getMagEulers(int16_t * , double *);

	    //void calcCommands(float *, double *, double *, float thrust); //This is a hack.  Find motor thrusts, no filtering, nothing.


	    void writeMagFIFO(magDataStruct mag_sample, int16_t * FIFO_destination, int16_t * new_location);

	    void initAK8963();
	    void getMagCalibrationVals(double *, double *);
	    void getMagsQuaternion(int16_t *);

	    void calibrateMPU9250(float * gyroBias, float * accelBias);
		void MPU9250SelfTest(float * destination);


		uint8_t readByte(uint8_t slave_address, uint8_t register_address);

		void readBytes(uint8_t slave_address, uint8_t register_address, uint8_t num_bytes_to_read,
                        uint8_t * destination);

		void writeByte(uint8_t slave_address, uint8_t register_address, uint8_t data);

		void readBits(uint8_t slave_address, uint8_t register_address);
		

};



#endif