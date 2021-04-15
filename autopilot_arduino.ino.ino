  
#include <avr/dtostrf.h>

#define GPIO 12
#define CHECK_PIN 4
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

int state;
int previousState = 0;
//char pulses[] = "0.1,0.2,0.3,0.4,0.5,0.6,0.7";
char floatBuffer[10];
char startMarker = '<';
char endMarker = '>';
char writeMarker = '&';
char incomingByte;
char previousByte = '0';
const char numBytes = 60;
char receivedMsg[numBytes];
char expectedMsg[] = "100,200,300,400,500,600,700,800,100,200,300,400";

boolean dataAvailable;
int len = 7;

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
  float pitch;
  float roll;
  float heading;

  float pitch_rate;
  float roll_rate;
  float heading_rate;
};

RECEIVER Current_Receiver_Values;
LOCATION Current_Location;

float *dataToSend[] = {&Current_Receiver_Values.thrust,
                       &Current_Receiver_Values.pitch,
                       &Current_Receiver_Values.roll,
                       &Current_Receiver_Values.yaw,
                       &Current_Receiver_Values.aux1,
                       &Current_Receiver_Values.aux2,
                       &Current_Receiver_Values.dial1};

void setup() {
  Serial.begin(460800);

  pinMode(LED_BUILTIN, OUTPUT);
  //clearGPIO();
  pinMode(GPIO, INPUT);
  pinMode(CHECK_PIN, OUTPUT);

  digitalWrite(CHECK_PIN, HIGH);

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

  updateReceiver(&Current_Receiver_Values);
  writeData();
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

void writeData() {
  Serial.print(startMarker);
  for(int i=0; i<len; i++) {
    //ftoa(dataToSend[i], floatBuffer, 10);
    //sprintf(floatBuffer, "%f", dataToSend[i]);
    dtostrf(*dataToSend[i], 5, 2, floatBuffer); // converts float to character array, 5 total characters with precision of 2
    Serial.print(floatBuffer);

    if(i!=len-1)
      Serial.print(",");
  }
  //Serial.print(pulses);
  Serial.println(endMarker);

  //while(Serial.available() != 0) {digitalWrite(LED_BUILTIN, LOW);}

  //digitalWrite(LED_BUILTIN, HIGH);
}

void readData() {
  dataAvailable = true;
  static boolean recvInProgress = false;
  static byte indx = 0;

  //while (Serial.available() > 0 && dataAvailable) 
  while (Serial.available() > 0) {
    digitalWrite(LED_BUILTIN, HIGH);
    incomingByte = Serial.read();

    if (incomingByte == writeMarker && previousByte == endMarker) {
      //digitalWrite(LED_BUILTIN, HIGH);
      updateReceiver(&Current_Receiver_Values);
      writeData();
      recvInProgress = false;
      //digitalWrite(LED_BUILTIN, LOW);
    }

    if (recvInProgress) {
      if (incomingByte != endMarker) {
        receivedMsg[indx] = incomingByte;
        indx++;

        if (indx >= numBytes)
          indx = numBytes - 1;
      }
      else {
        //checkMsg();
        receivedMsg[0] = '\0';
        recvInProgress = false;
        indx = 0;
        dataAvailable = false;
      }
    }
    else if (incomingByte == startMarker) {
      recvInProgress = true;
    }

    previousByte = incomingByte;
  }

  digitalWrite(LED_BUILTIN, LOW);
}

// only use for debugging
// writes digital HIGH to CHECK_PIN if the message received from the Neo matches the expected message
void checkMsg() {
  if (strcmp(receivedMsg, expectedMsg) == 0)
    digitalWrite(LED_BUILTIN, HIGH);
  else
    digitalWrite(LED_BUILTIN, LOW);
}

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
