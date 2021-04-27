#include <avr/dtostrf.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define SDAPIN 20
#define CLKPIN 21

//#define GPIO 12
#define CHECK_PIN 31

#define FRONT_RIGHT_SERVO_PIN 2
#define FRONT_LEFT_SERVO_PIN 3
#define BACK_RIGHT_SERVO_PIN 4
#define BACK_LEFT_SERVO_PIN 5

#define FRONT_RIGHT_MOTOR_PIN 6
#define FRONT_LEFT_MOTOR_PIN 7
#define BACK_RIGHT_MOTOR_PIN 8
#define BACK_LEFT_MOTOR_PIN 9
#define BACK_MID_RIGHT_MOTOR_PIN 10
#define BACK_MID_LEFT_MOTOR_PIN 11
#define BACK_FAR_RIGHT_MOTOR_PIN 12
#define BACK_FAR_LEFT_MOTOR_PIN 13

#define RECV_CHAN0PIN 53
#define RECV_CHAN1PIN 51
#define RECV_CHAN2PIN 49
#define RECV_CHAN3PIN 47
#define RECV_CHAN4PIN 45
#define RECV_CHAN5PIN 43
#define RECV_CHAN6PIN 41

#define MIN_PWM_THRUST 988
#define MAX_PWM_THRUST 2008

#define MIN_PWM_PITCH 1000
#define MAX_PWM_PITCH 2012
#define PITCH_PWM_ZERO 1502

#define MIN_PWM_ROLL 988
#define MAX_PWM_ROLL 2008
#define ROLL_PWM_ZERO 1497

#define MIN_PWM_YAW 1006 //Reading 96.5 (not sure why, not super easy to see how to change to -100.0)
#define MAX_PWM_YAW 2012 //Reading 100.0
#define YAW_PWM_ZERO 1500 //Reading 0.0 on QX7 display

#define MIN_PWM_AUX1 988
#define MAX_PWM_AUX1 2012

#define MIN_PWM_AUX2 988
#define MAX_PWM_AUX2 2012

#define MIN_PWM_DIAL1 988
#define MAX_PWM_DIAL1 2012

//int state;
//int previousState = 0;
//char pulses[] = "0.1,0.2,0.3,0.4,0.5,0.6,0.7";
char floatBuffer[10];
char startMarker = '<';
char loc_startMarker = '&';
char pwm_startMarker = '#';
char endMarker = '>';
//char writeMarker = '&';
char incomingByte;
//char previousByte = '0';
const char numBytes = 60;
char receivedMsg[numBytes];
//char expectedMsg[] = "Hello! This message was sent from the control code.";

boolean dataAvailable;

int ch0_start;//micros at which a certain pin went to high
int ch0_end; //micros at which a certain pin went to low
int ch0_us; //delta between start and finish micros when pin goes low
//ch0_us will always, automatically (because of the ISR), be the most recent pulse from the receiver

int ch1_start, ch1_end, ch1_us;
int ch2_start, ch2_end, ch2_us;
int ch3_start, ch3_end, ch3_us;
int ch4_start, ch4_end, ch4_us;
int ch5_start, ch5_end, ch5_us;
int ch6_start, ch6_end, ch6_us;

struct RECEIVER {
  float thrust = 0;
  float pitch;
  float roll;
  float yaw;
  float aux1;
  float aux2;
  float dial1;
};

struct LOCATION {
  float roll;
  float pitch;
  float heading;

  float roll_rate;
  float pitch_rate;
  float heading_rate;
};

struct PROPULSION_UNIT {
  int pin;
  int PWM_micros;
};

PROPULSION_UNIT fr_motor{FRONT_RIGHT_MOTOR_PIN, 0};
PROPULSION_UNIT fl_motor{FRONT_LEFT_MOTOR_PIN, 0};
PROPULSION_UNIT br_motor{BACK_RIGHT_MOTOR_PIN, 0};
PROPULSION_UNIT bl_motor{BACK_LEFT_MOTOR_PIN, 0};
PROPULSION_UNIT bmr_motor{BACK_MID_LEFT_MOTOR_PIN, 0};
PROPULSION_UNIT bml_motor{BACK_MID_LEFT_MOTOR_PIN, 0};
PROPULSION_UNIT bfr_motor{BACK_FAR_RIGHT_MOTOR_PIN, 0};
PROPULSION_UNIT bfl_motor{BACK_FAR_LEFT_MOTOR_PIN, 0};
PROPULSION_UNIT fr_servo{FRONT_RIGHT_SERVO_PIN, 0};
PROPULSION_UNIT fl_servo{FRONT_LEFT_SERVO_PIN, 0};
PROPULSION_UNIT br_servo{BACK_RIGHT_SERVO_PIN, 0};
PROPULSION_UNIT bl_servo{BACK_LEFT_SERVO_PIN, 0};

PROPULSION_UNIT propulsion_units[12] = {fr_motor, fl_motor, br_motor, bl_motor, bmr_motor, bml_motor, bfr_motor, bfl_motor, 
                                        fr_servo, fl_servo, br_servo, bl_servo};
                                        
char* PWM_msg_ptr;

RECEIVER Current_Receiver_Values;
LOCATION Current_Location;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

sensors_event_t orientationEvent;
sensors_event_t ratesEvent;

float *dataToSend[] = {&Current_Receiver_Values.thrust,
                       &Current_Receiver_Values.pitch,
                       &Current_Receiver_Values.roll,
                       &Current_Receiver_Values.yaw,
                       &Current_Receiver_Values.aux1,
                       &Current_Receiver_Values.aux2,
                       &Current_Receiver_Values.dial1,
                       &Current_Location.roll,
                       &Current_Location.pitch,
                       &Current_Location.heading,
                       &Current_Location.roll_rate,
                       &Current_Location.pitch_rate,
                       &Current_Location.heading_rate};

int len = sizeof(dataToSend)/sizeof(dataToSend[0]);

void setup() {
  Serial.begin(460800);
  SerialUSB.begin(460800);

  //pinMode(LED_BUILTIN, OUTPUT);
  pinMode(CHECK_PIN, OUTPUT);

  // Motors & servos setup
  for(int i=0; i<12; i++) {
    pinMode(propulsion_units[i].pin, OUTPUT);
  }

  // Receiver setup
  pinMode(RECV_CHAN0PIN, INPUT);
  pinMode(RECV_CHAN1PIN, INPUT);
  pinMode(RECV_CHAN2PIN, INPUT);
  pinMode(RECV_CHAN3PIN, INPUT);
  pinMode(RECV_CHAN4PIN, INPUT);
  pinMode(RECV_CHAN5PIN, INPUT);
  pinMode(RECV_CHAN6PIN, INPUT);

  attachInterrupt(RECV_CHAN0PIN, &ch0_PWM, CHANGE);
  attachInterrupt(RECV_CHAN1PIN, &ch1_PWM, CHANGE);
  attachInterrupt(RECV_CHAN2PIN, &ch2_PWM, CHANGE);
  attachInterrupt(RECV_CHAN3PIN, &ch3_PWM, CHANGE);
  attachInterrupt(RECV_CHAN4PIN, &ch4_PWM, CHANGE);
  attachInterrupt(RECV_CHAN5PIN, &ch5_PWM, CHANGE);
  attachInterrupt(RECV_CHAN6PIN, &ch6_PWM, CHANGE);

  //Serial.println("CHANGE interrupts on receiver pins are set");

  // IMU setup
  //BNO055 reset - (Due pin 39 connected to BNO055 RST pin).  Need to do this after restarting after a bus loss or else have to try a couple times.
  pinMode(39, OUTPUT);
  digitalWrite(39, LOW);
  delay(1);
  digitalWrite(39, HIGH);
  delay(650);
   
  while(!bno.begin()) {
    recoverIMU();
  }

  //Serial.println("I2C bus up, BNO055 detected");

  bno.setExtCrystalUse(true);
  
  Wire.begin();
  Wire.setClock(100000); //Seems to have no effect on Current_Location.estimate() speed from start to finish
}

void loop() {
  /*state = digitalRead(GPIO);
  digitalWrite(LED_BUILTIN, state);

  // Send data only if the GPIO pin is logic high AND the pin was previously logic LOW
  // This prevents the Due from sending even more data before the Neo has had time to finish reading
  if (state == 1 && previousState == 0) {
    updateReceiver(&Current_Receiver_Values);
    writeData();
  }

  // Read data if data avilable on the serial port AND Neo confirms it has received the last message
  // This prevents the Due from reading it's own sent data on the Serial port
  if (Serial.available() > 0 && state == 0)
    readData();

  previousState = state;*/

  // Reading PWM data from native serial port isn't working because the serial port
  // keeps changing everytime we run the control loop
  // Reading it from the programming port works but then the functions that write
  // location and receiver data over the native port fail
  
  updateReceiver(&Current_Receiver_Values);
  updateLocation(&Current_Location);
  //Serial.println("testing...");
  writeLocationData();
  writeReceiverData();
  //Serial.flush(); // empty the buffer
  
  if(SerialUSB.available() > 0) {
    readPWMS();
    send_PWMS();
  }
}

void recoverIMU() {
  //Serial.println("Starting I2C bus recovery - init");

  //try i2c bus recovery at 100kHz = 5uS high, 5uS low
  pinMode(SDAPIN, OUTPUT);//keeping SDA high during recovery
  digitalWrite(SDAPIN, HIGH);
  pinMode(CLKPIN, OUTPUT);
  for (int i = 0; i < 2; i++) { //9nth cycle acts as NACK
    digitalWrite(CLKPIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(CLKPIN, LOW);
    delayMicroseconds(5);
  }

  //a STOP signal (SDA from low to high while CLK is high)
  digitalWrite(SDAPIN, LOW);
  delayMicroseconds(5);
  digitalWrite(CLKPIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(SDAPIN, HIGH);
  delayMicroseconds(2);
  //bus status is now : FREE

  //Serial.println("Bus recovery done - init");
  //return to power up mode
  pinMode(SDAPIN, INPUT);
  pinMode(CLKPIN, INPUT);
}

void updateLocation(LOCATION *loc) {
  //Haven't figured out why, but getEvent works (doesn't return zeros after a while) and getGv
  bno.getEvent(&orientationEvent, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&ratesEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);

  loc->roll = orientationEvent.orientation.y;
  loc->pitch = orientationEvent.orientation.z;
  loc->heading = orientationEvent.orientation.x;

  loc->roll_rate = ratesEvent.gyro.y;
  loc->pitch_rate = ratesEvent.gyro.x;
  loc->heading_rate = ratesEvent.gyro.z;
}

void updateReceiver(RECEIVER *rcvr) {
  rcvr->thrust = ((float)ch0_us - MIN_PWM_THRUST) / (float)(MAX_PWM_THRUST - MIN_PWM_THRUST);

  if (rcvr->thrust > 1.1){
      rcvr->thrust = 0; //Problem where erroneous startup throttle caused flip
    }
  if (millis() < 4000){
    rcvr->thrust = 0; //Never start up with motors on no matter what
  }
  

  //Pitch, roll, and yaw have a little more math to make -1 to 1.  Linear between zero point (resting) and edges.
  //If don't do between resting point, zero point will always be off by about 0.03 or so (since doesn't rest
  //exactly in the middle)
  if (ch1_us < PITCH_PWM_ZERO){ //For when below the zero point, (Delta from zero)/(Range from that edge to mid)
    rcvr->pitch = ((float)(ch1_us - PITCH_PWM_ZERO) / (float)(MIN_PWM_PITCH - PITCH_PWM_ZERO));}
  else{ //For when above the zero point
    rcvr->pitch = ((float)(ch1_us - PITCH_PWM_ZERO) / (float)(PITCH_PWM_ZERO - MAX_PWM_PITCH));}

  if (ch2_us < ROLL_PWM_ZERO){
    rcvr->roll = ((float)(ch2_us - ROLL_PWM_ZERO) / (float)(ROLL_PWM_ZERO - MIN_PWM_ROLL));}
  else{
    rcvr->roll = ((float)(ch2_us - ROLL_PWM_ZERO) / (float)(MAX_PWM_ROLL - ROLL_PWM_ZERO));}

  if (ch3_us < YAW_PWM_ZERO){
    rcvr->yaw = ((float)(ch3_us - YAW_PWM_ZERO) / (float)(YAW_PWM_ZERO - MIN_PWM_YAW));}
  else{
    rcvr->yaw = ((float)(ch3_us - YAW_PWM_ZERO) / (float)(MAX_PWM_YAW - YAW_PWM_ZERO));}
  if (abs(rcvr->yaw) < 0.03){ //If yaw close to zero, just make zero to avoid unintentional slew rate
    rcvr->yaw = 0;}


  rcvr->aux1 = ((float)ch4_us - MIN_PWM_AUX1) / (float)(MAX_PWM_AUX1- MIN_PWM_AUX1);
  rcvr->aux2 = ((float)ch5_us - MIN_PWM_AUX2) / (float)(MAX_PWM_AUX2 - MIN_PWM_AUX2);

  rcvr->dial1 = ((float)ch6_us - (MIN_PWM_DIAL1 + MAX_PWM_DIAL1)/2) / (float)(MAX_PWM_DIAL1 - (MIN_PWM_DIAL1 + MAX_PWM_DIAL1)/2);
}

void writeReceiverData() {
  Serial.print(startMarker);
  
  for(int i=0; i<7; i++) {
    // Convert floats to character arrays
    // Receiver data is 5 total characters with precision of 2
    dtostrf(*dataToSend[i], 5, 2, floatBuffer); // converts float to character array, 5 total characters with precision of 2
    Serial.print(floatBuffer);
    Serial.print(",");
  }

  //Serial.println(endMarker);
  Serial.println();
}

void writeLocationData() {
  Serial.print(loc_startMarker);
  
  for(int i=7; i<len; i++) {
    // Convert floats to character arrays
    // Location data is 8 total characters with precision of 3
    dtostrf(*dataToSend[i], 7, 3, floatBuffer); // converts float to character array, 5 total characters with precision of 2
    Serial.print(floatBuffer);
    Serial.print(",");
  }

  //Serial.println(endMarker);
  Serial.println();
}

void readPWMS() {
  dataAvailable = true;
  
   while (SerialUSB.available() > 0 && dataAvailable) {
    //digitalWrite(LED_BUILTIN, HIGH);
    incomingByte = SerialUSB.read();

    if(incomingByte == pwm_startMarker) {
      size_t n = SerialUSB.readBytes(receivedMsg, 59);
      //SerialUSB.flush();
      //Serial.println(receivedMsg);
      //checkMsg();
      convertMsg();
      receivedMsg[0] = '\0';
      dataAvailable = false;
    }
  }
}

// Takes char array of PWMS and converts to int array
void convertMsg() {
  PWM_msg_ptr = strtok(receivedMsg, ",");

  int count = 0;
  while(PWM_msg_ptr) {
    propulsion_units[count].PWM_micros = atoi(PWM_msg_ptr);

    count = count+1;
    PWM_msg_ptr = strtok(NULL, ",");
  }

  bool flag=false;

  // Print debugging, ignore
  for(int i=0; i<12; i++) {
    if(propulsion_units[i].PWM_micros != 1000 + 100*i) {
      flag = true;
      break;
    }
  }

  if(flag)
    digitalWrite(CHECK_PIN, LOW);
  else
    digitalWrite(CHECK_PIN, HIGH);
}

void send_PWMS() {
  for(int i=0; i<12; i++) {
    analogWriteResolution(12);
    analogWrite(propulsion_units[i].pin, propulsion_units[i].PWM_micros);
    //analogWrite(propulsion_units[i].pin, 690);
  }
}

// only use for debugging
// writes digital HIGH to CHECK_PIN if the message received from the flight computer matches the expected message
// this function was necessary because it wasn't possible to print to a Serial port as we currently read from
/*void checkMsg() {
  if (strcmp(receivedMsg, expectedMsg) == 0)
    digitalWrite(CHECK_PIN, HIGH);
  else
    digitalWrite(CHECK_PIN, LOW);
}*/

void ch0_PWM(){ 
  int ch0_pin = digitalRead(RECV_CHAN0PIN);

  if (ch0_pin == 1){ //Pin just turned on
    ch0_start = micros();
  }
  if (ch0_pin == 0){ //Pin just turned off
    ch0_end = micros();
    ch0_us = ch0_end - ch0_start; //Only calculate 'us' when it turns off.
    //Otherwise might get an end time from the last pulse if this one's not done, so would get a 
    //value near 18ms, or 18,000, and negative.
  }
}


void ch1_PWM() { 
  int ch1_pin = digitalRead(RECV_CHAN1PIN);

  if (ch1_pin == 1){ //Pin just turned on
    ch1_start = micros();
  }
  if (ch1_pin == 0){ //Pin just turned off
    ch1_end = micros();
    ch1_us = ch1_end - ch1_start; //Only calculate us when it turns off.
  }
}

void ch2_PWM() { 
  int ch2_pin = digitalRead(RECV_CHAN2PIN);

  if (ch2_pin == 1){ //Pin just turned on
    ch2_start = micros();
  }
  if (ch2_pin == 0){ //Pin just turned off
    ch2_end = micros();
    ch2_us = ch2_end - ch2_start; //Only calculate us when it turns off.
  }
}

void ch3_PWM() { 
  int ch3_pin = digitalRead(RECV_CHAN3PIN);

  if (ch3_pin == 1){ //Pin just turned on
    ch3_start = micros();
  }
  if (ch3_pin == 0){ //Pin just turned off
    ch3_end = micros();
    ch3_us = ch3_end - ch3_start; //Only calculate us when it turns off.
  }
}

void ch4_PWM(){ 
  int ch4_pin = digitalRead(RECV_CHAN4PIN);

  if (ch4_pin == 1){ //Pin just turned on
    ch4_start = micros();
  }
  if (ch4_pin == 0){ //Pin just turned off
    ch4_end = micros();
    ch4_us = ch4_end - ch4_start; //Only calculate us when it turns off.
  }
}

void ch5_PWM() { 
  int ch5_pin = digitalRead(RECV_CHAN5PIN);

  if (ch5_pin == 1){ //Pin just turned on
    ch5_start = micros();
  }
  if (ch5_pin == 0){ //Pin just turned off
    ch5_end = micros();
    ch5_us = ch5_end - ch5_start; //Only calculate us when it turns off.
  }
}

void ch6_PWM() { 
  int ch6_pin = digitalRead(RECV_CHAN6PIN);

  if (ch6_pin == 1){ //Pin just turned on
    ch6_start = micros();
  }
  if (ch6_pin == 0){ //Pin just turned off
    ch6_end = micros();
    ch6_us = ch6_end - ch6_start; //Only calculate us when it turns off.
  }
}