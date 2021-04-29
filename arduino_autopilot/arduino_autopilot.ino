#include <avr/dtostrf.h>
#include <Wire.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BNO055.h>

#define SDAPIN 20
#define CLKPIN 21

//#define GPIO 12
//#define CHECK_PIN 31

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

//RECEIVER Current_Receiver_Values;
//LOCATION Current_Location;
//Adafruit_BNO055 bno = Adafruit_BNO055(55);

//sensors_event_t orientationEvent;
//sensors_event_t ratesEvent;

/*float *dataToSend[] = {&Current_Receiver_Values.thrust,
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

int len = sizeof(dataToSend)/sizeof(dataToSend[0]);*/

void setup() {
  //Serial.begin(460800);
  //SerialUSB.begin(115200);

  //pinMode(LED_BUILTIN, OUTPUT);
  //pinMode(CHECK_PIN, OUTPUT);

  // Motors & servos setup
  for(int i=0; i<12; i++) {
    pinMode(propulsion_units[i].pin, OUTPUT);
  }

  // Receiver setup
  /*pinMode(RECV_CHAN0PIN, INPUT);
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
  attachInterrupt(RECV_CHAN6PIN, &ch6_PWM, CHANGE);*/

  //Serial.println("CHANGE interrupts on receiver pins are set");

  // IMU setup
  //BNO055 reset - (Due pin 39 connected to BNO055 RST pin).  Need to do this after restarting after a bus loss or else have to try a couple times.
  /*pinMode(39, OUTPUT);
  digitalWrite(39, LOW);
  delay(1);
  digitalWrite(39, HIGH);
  delay(650);*/
   
  //while(!bno.begin()) {
  //  recoverIMU();
  //}

  //Serial.println("I2C bus up, BNO055 detected");

  //bno.setExtCrystalUse(true);
  
  //Wire.begin();
  //Wire.setClock(100000); //Seems to have no effect on Current_Location.estimate() speed from start to finish
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
  
  //updateReceiver(&Current_Receiver_Values);
  //updateLocation(&Current_Location);
  //Serial.println("testing...");
  //writeLocationData();
  //writeReceiverData();
  //Serial.flush(); // empty the buffer

  analogWriteResolution(12);
  //analogWrite(6, 1000);
  
  if(Serial.available() > 0) {
    //digitalWrite(6, HIGH);
    //int n = Serial.read();
    //int num = millis() % 1000;
    //analogWrite(6, 500);
    readPWMS();
    send_PWMS();
  }
  //else
    //digitalWrite(6, LOW);
}

void readPWMS() {
  dataAvailable = true;
  
   while (Serial.available() > 0 && dataAvailable) {
    //digitalWrite(LED_BUILTIN, HIGH);
    incomingByte = Serial.read();

    if(incomingByte == pwm_startMarker) {
      size_t n = Serial.readBytes(receivedMsg, 59);
      //SerialUSB.println(receivedMsg);
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
}

void send_PWMS() {
  for(int i=0; i<12; i++) {
    analogWriteResolution(12);
    analogWrite(propulsion_units[i].pin, propulsion_units[i].PWM_micros);
    //SerialUSB.print(propulsion_units[i].PWM_micros);
    //SerialUSB.print(",");
    //analogWrite(propulsion_units[i].pin, 690);
  }
  //SerialUSB.println();
}
