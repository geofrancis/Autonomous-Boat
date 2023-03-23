#include <Wire.h>
#include <VL53L1X.h>
#define stepAng  18      // step angle
#define numStep 10        // = 180/stepAng 
#define TIMER0_INTERVAL_MS        50
#define MIN_MICROS      800  
#define MAX_MICROS      2450


VL53L1X sensor;


#include "TimerInterrupt_Generic.h"
ESP32Timer ITimer1(1);

#define TIMER_INTERRUPT_DEBUG       0
#define ISR_SERVO_DEBUG             0
#define USE_ESP32_TIMER_NO          0
#include "ESP32_S2_ISR_Servo.h"



//#include <IBusBM.h>
//IBusBM IBus;    // IBus object



#define SERVOLIDAR 34
#define MOTOR 40     
#define RUDDER 38
#define MODE 36
#define PUMP 39


//throttle limits
#define ReverseDistance 500    
#define MINDistance 150  

//steering limits
#define manouverDistance    3000







//RC inputs   



int ch1 = 1500;  //throttle
int ch2 = 1500;  //rudder
int ch3 = 1500;  //camera yaw
int ch4 = 1500;  //lights
int ch5 = 1500;  //anchor winch
int ch6 = 1500;  //tow winch
int ch7 = 1500;  //bilge pump / fire monitor
int ch8 = 1500;  //flight mode
int ch9 = 1500;  //avoid gain
int ch10 = 1500; //avoid mode



uint32_t scanner  = -1;
uint32_t MOTORout  = -1;
uint32_t RUDDERout  = -1;
uint32_t modeout  = -1;
uint32_t pumpout  = -1;


int MOT;
int RUD;
int MOD;
int PUM;




//GPIO

int bilge;

//servo values

int RCRud = 1500;
int RCThr = 1500;
int AVOIDMODE = 900;
int MULTI = 1500; 
int flightmode;

int esc;
int escs;
int yaw;
int out = 1500;
int rudout = 1500;
int yawsmooth = 1500;
int throttlesmooth = 1500;

int turnmulti;
int lmix;
int rmix;
int rudmix;
const int numReadings = 10;
int readings[numReadings];  // the readings from the analog input
int readIndex = 0;          // the index of the current reading
int total = 0;              // the running total
int average = 0;            // the average


int maxthrottle;
int minthrottle;
int averagescale;
int mindistance; 
int closest;
int avoiddirection;
int avoidturn;

int followturn;

int leftwallaverage;
int rightwallaverage;
int wallsteer;


int pos = 0;          // servo position
int dir = 1;          // servo moving direction: +1/-1
int val;              // LiDAR measured value

float distances[numStep+1]; // array for the distance at each angle, including at 0 and 180
float leftSum, rightSum, frontSum, leftsumscaled, rightsumscaled;
volatile unsigned long next;

const int ledPin =  LED_BUILTIN;
int ledState = LOW;   
unsigned long previousMillis = 0; 
const long interval = 1000;   



bool IRAM_ATTR TimerHandler0(void * timerNo)
{
  static bool toggle0 = false;
    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
        digitalWrite(ledPin, ledState);


  return true;
}





//=======================================================================

void setup() {

 Serial.begin(115200);
Wire.begin(33, 35);
 //Serial1.begin(115200, SERIAL_8N1, 11, 12);
 //IBus.begin(Serial1,1);    // iBUS object connected to serial2 RX2 pin and use timer 1
 pinMode(LED_BUILTIN, OUTPUT);

if (ITimer1.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, TimerHandler0))

ESP32_ISR_Servos.useTimer(USE_ESP32_TIMER_NO);
 scanner = ESP32_ISR_Servos.setupServo(SERVOLIDAR, 500, 2500);
  MOTORout = ESP32_ISR_Servos.setupServo(MOTOR, MIN_MICROS, MAX_MICROS);
  RUDDERout = ESP32_ISR_Servos.setupServo(RUDDER, MIN_MICROS, MAX_MICROS);
  modeout = ESP32_ISR_Servos.setupServo(MODE, MIN_MICROS, MAX_MICROS);
  pumpout = ESP32_ISR_Servos.setupServo(PUMP, MIN_MICROS, MAX_MICROS);

  
  sensor.setTimeout(500);
  if (!sensor.init()){val = 1;}
  if (sensor.init()){
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(35000); //35ms
  sensor.startContinuous(35);
 }
}



void loop() { 

//unsigned long currentMillis = millis();
 // if (currentMillis - previousMillis >= interval) {
//   save the last time you blinked the LED
//    previousMillis = currentMillis;

//Measure distance--------------------------------------------

  pos += dir;
ESP32_ISR_Servos.setPosition(scanner,(pos*stepAng));
val = sensor.read();
  if (sensor.timeoutOccurred()) { val = 1; }
  distances[pos] = val;
  total = total - readings[readIndex];
  readings[readIndex] = val;
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
  
  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  
 average = total / numReadings;

  
  if (pos == numStep)
  {
    dir = -1;
  }
  else if (pos == 0)
  {
    dir = 1;
  }

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
        digitalWrite(ledPin, ledState);
 
  
//READ RC CHANNELS------------------------------------
// while (Serial1.available()) {
//ch1 = IBus.readChannel(0);
//ch2 = IBus.readChannel(1);
//ch3 = IBus.readChannel(2);
//ch4 = IBus.readChannel(3);
//ch5 = IBus.readChannel(4);
//ch6 = IBus.readChannel(5);
//ch7 = IBus.readChannel(6);
//ch8 = IBus.readChannel(7);
//ch9 = IBus.readChannel(8);
//ch10 = IBus.readChannel(9);
// }
   
RCThr = ch1;
RCRud = ch2; 
AVOIDMODE = ch10;
MULTI = ch9;
flightmode = ch8;

//bilge = analogRead(A0);



// AVERAGE LEFT + RIGHT----------------------------------------------
    if (average < manouverDistance)
  { 
    yaw = (int)((rightsumscaled-leftsumscaled)+1500); 
     // yaw = (int)((rightSum-leftSum)+1500); 
  }
  else {(yaw = RCRud);
  }
 
 for (int i=0; i < 40; i++) {
 yawsmooth = yawsmooth + yaw;
 }
 yawsmooth = yawsmooth/5;

  
// AVERAGE THROTTLE AVOID-----------------------------------------------
    if (average < ReverseDistance)
  {  
    esc = map (average, ReverseDistance, MINDistance, 1500, 500);
  }
    else{
      esc = RCThr;
  }          

 for (int i=0; i < 20; i++) {
 throttlesmooth = throttlesmooth + esc;
 }
 throttlesmooth = throttlesmooth/5;


  

/////////////////////////////////////////////////////////////////////////////
   // find the left and right average sum
  if (pos > (numStep/2))
   rightSum = 0.3*rightSum + 1.4*distances[pos]/numStep;
  // leftSum = 0.3*leftSum + 1.4*distances[pos]/numStep; //tank
  else if (pos < (numStep/2))
  leftSum = 0.3*leftSum + 1.4*distances[pos]/numStep;
  //rightSum = 0.3*rightSum + 1.4*distances[pos]/numStep; //tank
   
   
 turnmulti = map (MULTI, 1000, 2000, 1, 6);
 leftsumscaled = (leftSum*turnmulti);
 rightsumscaled = (rightSum*turnmulti);

   
// find the front average sum-------------------------------------------------
  if ((pos > (numStep/4)) && (pos < (numStep*3/4)))
    frontSum = 0.3*frontSum + 1.4*distances[pos]/numStep; 
    
//find closest object--------------------------------------------------------
  if (distances[pos] < closest){
  closest = (distances[pos]);
  avoiddirection = pos;
}
  if (distances[avoiddirection] > closest){
    closest = (distances[pos]);

}

//track distance to wall-----------------------------------------------------

leftwallaverage = ((distances[0]+distances[1])/2);
rightwallaverage = ((distances[9]+distances[8])/2);
 
//MODE SELECTION/////////////////////////////////////////////////////////



//off
if (AVOIDMODE < 1000) {
out = RCThr;
rudout = RCRud;
}
//average steering only
if (AVOIDMODE > 1001 && AVOIDMODE < 1200) {
rudout = ((yawsmooth + RCRud)/2);
out = RCThr;
}
//throttle
if (AVOIDMODE > 1201 && AVOIDMODE < 1400) {
out = ((throttlesmooth + RCThr)/2);
rudout = RCRud;
}  
//steering and throttle
if (AVOIDMODE > 1401 && AVOIDMODE < 1600) {
out = ((throttlesmooth + RCThr)/2);
rudout = ((yawsmooth + RCRud)/2);
}  

MOT = map(MOTORout, 1000, 2000, 0, 180);
RUD = map(RUDDERout, 1000, 2000, 0, 180);
MOD = map(modeout, 1000, 2000, 0, 180);


ESP32_ISR_Servos.setPosition(scanner,(pos*stepAng));
ESP32_ISR_Servos.setPosition(MOTORout,MOT);
ESP32_ISR_Servos.setPosition(RUDDERout,RUD);
ESP32_ISR_Servos.setPosition(modeout,MOD);



//if (bilge <500){
//  ESP32_ISR_Servos.setPulseWidth(pumpout,(2000));
//}
//else{
// ESP32_ISR_Servos.setPulseWidth(pumpout,iBus.readChannel(6));
// }

//PUM = map(MOTORout, 1000, 2000, 50, 1);
    Serial.print(pos*stepAng);
    Serial.print("   ");
    Serial.println(val);
}
