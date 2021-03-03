# Autopilot

Arduino Due PWM has to be modified to be the right frequency.  Edit:

/Users/nickbain/Library/Arduino15/packages/arduino/hardware/sam/1.6.12/variants/arduino_due_x/variant.h

And lines ~220 to 240 should look like:

/*
 * PWM
 *  On Arduino Due, sets PWM (aka analogWrite) for only 6,7,8,9
 */
#define PWM_INTERFACE		PWM
#define PWM_INTERFACE_ID	ID_PWM
#define PWM_FREQUENCY		392  //Used 392 so that analogWrite() will write exactly (within 1-2 us) 10x the analogWrite argument
#define PWM_MAX_DUTY_CYCLE	255
#define PWM_MIN_DUTY_CYCLE	0
#define PWM_RESOLUTION		8

/*
 * TC
 */
#define TC_INTERFACE        TC0
#define TC_INTERFACE_ID     ID_TC0
#define TC_FREQUENCY        392 //Nick Bain - used to be 1000.  Same 392 as above.  analogWrite(9,25) does 250us to pin 9.  All pins good on Picoscope.
#define TC_MAX_DUTY_CYCLE   255
#define TC_MIN_DUTY_CYCLE   0
#define TC_RESOLUTION		8
