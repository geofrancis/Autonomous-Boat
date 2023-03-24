#include <Wire.h>

#define stepAng           10      // step angle
#define numStep           18        // = 180/stepAng 
#define MIN_MICROS        1000  
#define MAX_MICROS        2000

#define minreverse         500    
#define fullreverse        150  

#define turnrange         3000

#define SERVOLIDAR           9
#define MOTOR               40     
#define RUDDER              38
#define MODE                36
#define PUMP                39



#include <VL53L1X.h>
VL53L1X sensor;

//#include "TimerInterrupt_Generic.h"
//ESP32Timer ITimer1(1);
//#define TIMER0_INTERVAL_MS        50
//#define TIMER_INTERRUPT_DEBUG       0


#define ISR_SERVO_DEBUG             0
#define USE_ESP32_TIMER_NO          2
#include "ESP32_S2_ISR_Servo.h"

#include <IBusBM.h>
IBusBM IBus;    // IBus object


int oldval;
int newval;
int intval;
int oltval;

int avoidmode = 0;
int scanhold = 0;

int yawsmoothen = 0;
int escsmoothen = 0;
int pointyawen = 0;
int pointescen = 0;
int wallsteeren = 0;
int followturnen = 0;

int yawfollow;
int yawsmooth = 1500;
int escsmooth = 1500;


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
int ch10 = 1350; //avoid mode
int ch11 = 1350; //avoid mode
int ch12 = 1350; //avoid mode
int ch13 = 1350; //avoid mode
int ch14 = 1350; //avoid mode


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
int AVOIDMODE = 1500;
int MULTI = 1500; 
int flightmode;

int esc;
int pointesc;
int escs;
int yaw;
int pointyaw;
int out = 1500;
int rudout = 1500;

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

int averagescale;
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
int range;
float distances[numStep+1]; // array for the distance at each angle, including at 0 and 180
float leftSum, rightSum, frontSum, leftsumscaled, rightsumscaled;
volatile unsigned long next;

const int ledPin =  LED_BUILTIN;
int ledState = LOW;   
unsigned long previousMillis = 0; 
const long interval = 1000;   






//bool IRAM_ATTR TimerHandler0(void * timerNo)
//{
 


//  return true;
//}



//=======================================================================

void setup() {

Serial.begin(115200);
Wire.begin(33, 35);
IBus.begin(Serial1, IBUSBM_NOTIMER);
pinMode(LED_BUILTIN, OUTPUT);

//if (ITimer1.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, TimerHandler0))

ESP32_ISR_Servos.useTimer(USE_ESP32_TIMER_NO);
 scanner = ESP32_ISR_Servos.setupServo(SERVOLIDAR, 500, 2500);
  MOTORout = ESP32_ISR_Servos.setupServo(MOTOR, MIN_MICROS, MAX_MICROS);
  RUDDERout = ESP32_ISR_Servos.setupServo(RUDDER, MIN_MICROS, MAX_MICROS);
  modeout = ESP32_ISR_Servos.setupServo(MODE, MIN_MICROS, MAX_MICROS);
  pumpout = ESP32_ISR_Servos.setupServo(PUMP, MIN_MICROS, MAX_MICROS);

  
  sensor.setTimeout(100);
  if (!sensor.init()){val = 1;}
  if (sensor.init()){


  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(35000); //35ms
  sensor.startContinuous(35);
 }



 
}



void loop() { 

getReading();
readrc();
modeselect();
avoidmodes();
controlmodes();
servooutput();
gpio();
serialprint();

}

  

/////////////////////////////////////////////////////////////////Measure distance+ move servo


void getReading(){
  if (sensor.dataReady())  {
  val = sensor.read();
  pos += dir;
  ESP32_ISR_Servos.setPosition(scanner,(pos*stepAng));
  distances[pos] = val;
  if (pos == numStep)
  {
    dir = -1;
  }
  
  else if (pos == 0)
  {
    dir = 1;
  }

////////////////////////////////////////LEFT RIGHT AVERAGE        
   // find the left and right average sum
  if (pos > (numStep/2))
  rightSum = ((rightSum + distances[pos])/2);
   //rightSum = 0.3*rightSum + 1.4*distances[pos]/numStep;
   else if (pos < (numStep/2))
   leftSum = ((leftSum + distances[pos])/2);
  // leftSum = 0.3*leftSum + 1.4*distances[pos]/numStep;

   
 turnmulti = map (MULTI, 1000, 2000, 0.1, 10);
 leftsumscaled = (leftSum*turnmulti);
 rightsumscaled = (rightSum*turnmulti);
  

//////////////////////////////////////////FIND TOTAL AVERAGE///////////
  total = total - readings[readIndex];
  readings[readIndex] = val;
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) {
    readIndex = 0;
  }  
 average = total / numReadings;

 //////////////////////////////////////////FIND CLOSEST OBJECT////



  /////////////////////////////////////////LED//
static bool toggle0 = false;
    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
        digitalWrite(ledPin, ledState);
}
  

}


  
///////////////////////////////////////////////READ RC CHANNELS//////////////
void readrc(){
IBus.loop();
ch1 = IBus.readChannel(0);
ch2 = IBus.readChannel(1);
ch3 = IBus.readChannel(2);
ch4 = IBus.readChannel(3);
ch5 = IBus.readChannel(4);
ch6 = IBus.readChannel(5);
ch7 = IBus.readChannel(6);
ch8 = IBus.readChannel(7);
ch9 = IBus.readChannel(8);
ch10 = IBus.readChannel(9);
ch11 = IBus.readChannel(10);
ch12 = IBus.readChannel(11);
ch13 = IBus.readChannel(12);
ch14 = IBus.readChannel(13);

   
RCThr = ch1;
RCRud = ch2; 
AVOIDMODE = ch10;
MULTI = ch9;
flightmode = ch8;
}



//////////////////////////////////////////////////AVOID MODE SELECTION/////////
void modeselect(){


if (AVOIDMODE <= 1000) {avoidmode = 0;}
if (AVOIDMODE > 1001 && AVOIDMODE < 1100) {avoidmode = 1;}
if (AVOIDMODE > 1101 && AVOIDMODE < 1200) {avoidmode = 2;}
if (AVOIDMODE > 1201 && AVOIDMODE < 1300) {avoidmode = 3;}
if (AVOIDMODE > 1301 && AVOIDMODE < 1400) {avoidmode = 4;}  
if (AVOIDMODE > 1401 && AVOIDMODE < 1500) {avoidmode = 5;}  
if (AVOIDMODE > 1501 && AVOIDMODE < 1600) {avoidmode = 6;}  
if (AVOIDMODE > 1601 && AVOIDMODE < 1700) {avoidmode = 7;}  
if (AVOIDMODE > 1701 && AVOIDMODE < 1800) {avoidmode = 8;}  
if (AVOIDMODE > 1801 && AVOIDMODE < 1900) {avoidmode = 9;}  
if(AVOIDMODE > 1901 && AVOIDMODE < 2000) {avoidmode = 10;}
if (AVOIDMODE >= 2000) {avoidmode = 11;}
      

}

///////////////////////////////////////////////////CONTROL MODE SELECTION////
void avoidmodes(){

 if (avoidmode == 0) {
  
 yawsmoothen = 0;
 escsmoothen = 0;
 pointyawen = 0; 
 pointescen = 0;
 wallsteeren = 0;
 followturnen = 0;

   out = RCThr;
   rudout = RCRud;
}

//average steering only
 if (avoidmode == 1) {
  
 yawsmoothen = 1;
 escsmoothen = 0;
 pointyawen = 0; 
 pointescen = 0;
 wallsteeren = 0;
 followturnen = 0;
  
   rudout = ((yawsmooth + RCRud)/2);
   out = RCThr;
}
//throttle
if (avoidmode == 2) {
  
 yawsmoothen = 0;
 escsmoothen = 1;
 pointyawen = 0; 
 pointescen = 0;
 wallsteeren = 0;
 followturnen = 0;
 
   out = ((escsmooth + RCThr)/2);
   rudout = RCRud;
}  
//steering and throttle
if (avoidmode == 3) {
 yawsmoothen = 1;
 escsmoothen = 1;
 pointyawen = 0; 
 pointescen = 0;
 wallsteeren = 0;
 followturnen = 0;

   out = ((escsmooth + RCThr)/2);
   rudout = ((yawsmooth + RCRud)/2);

}  
//point steering only
if (avoidmode == 4) {
 yawsmoothen = 0;
 escsmoothen = 0;
 pointyawen = 1; 
 pointescen = 0;
 wallsteeren = 0;
 followturnen = 0;

   rudout = ((pointyaw + RCRud)/2);
   out = RCThr;
}
//point throttle
if (avoidmode == 5) {
 yawsmoothen = 0;
 escsmoothen = 0;
 pointyawen = 0; 
 pointescen = 1;
 wallsteeren = 0;
 followturnen = 0;
 
   out = ((pointesc + RCThr)/2);
   rudout = RCRud;
}



//wall following
if (avoidmode == 6) {
 yawsmoothen = 0;
 escsmoothen = 1;
 pointyawen = 0; 
 pointescen = 0;
 wallsteeren = 1;
 followturnen = 0;
out = ((esc + RCThr)/2);
rudout = ((yaw + RCRud + wallsteer)/3);
}  

//object following
if (avoidmode == 7) {
 yawsmoothen = 1;
 escsmoothen = 1;
 pointyawen = 0; 
 pointescen = 0;
 wallsteeren = 0;
 followturnen = 1;
 out = ((esc + RCThr)/2);
 rudout = ((yaw + RCRud + followturn)/3);
}  


  
}

/////////////////////////////////////////////////CONTROL MODES///////
void controlmodes(){

  

// AVOID AVERAGE LEFT + RIGHT----------------------------------------------
   // find the left and right average sum
 
if (yawsmoothen == 1)
  { 
    if (average < turnrange)
  { 
    yaw = (int)((rightsumscaled-leftsumscaled)+1500); 

  }
  else {(yaw = RCRud);
  }
 
 for (int i=0; i < 1; i++) {
 yawsmooth = yawsmooth + yaw;
 }
 yawsmooth = yawsmooth/1;
  }
// AVERAGE THROTTLE AVOID-----------------------------------------------

if (escsmoothen == 1)
  {
    average = total / numReadings;
    if (average < minreverse)
  {  
    esc = map (average, minreverse, fullreverse, 1500, 900);
  }
    else{
      esc = RCThr;
        }          

 for (int i=0; i < 1; i++) {
 escsmooth = escsmooth + esc;
 }
 escsmooth = escsmooth/1;

  }
//AVOID POINT DIRECTION  -----------------------------------

if (pointyawen == 1)
  {
  if (average < turnrange)
  {  
   if (avoiddirection < numStep/2){
      avoidturn =0;
    }
    else avoidturn = 1;
      
   if (avoidturn = 0) {
       pointyaw = map (avoiddirection, 0, (numStep/2), 1500, 1000);}
  if (avoidturn = 1) 
      {pointyaw = map (avoiddirection, (numStep/2), (numStep), 1500, 2000);}
  }
    else{
      pointyaw = RCRud;
        }

  }   
// POINT THROTTLE AVOID-----------------------------------------------
 if (pointescen == 1)
  {
 if (closest < minreverse)
  {  
 pointesc = map (closest, minreverse, fullreverse, 1500, 900);
  }
    else{
    esc = RCThr;
        }          
  }

//POINT YAW TO FOLLOW CLOSEST OBJECT----------------------------------------------

if (pointyawen == 1)
  {
if (average < turnrange){
if (avoidturn = 0) 
   followturn = map (avoiddirection, 0, (numStep/2), 1500, 1700);}
  else {followturn = map (avoiddirection, (numStep/2), numStep, 1700, 1500);}       
  }


//AVERAGE YAW TO FOLLOW -----------------------------------------------------

if (followturnen == 1)
  {
 if (average < turnrange)
  { 
    yawfollow = (int)((leftsumscaled - rightsumscaled)+1500); 
  }
  else {(yawfollow = RCRud);
  }
 for (int i=0; i < 40; i++) {
 yawsmooth = yawsmooth + yawfollow;
 }
 yawsmooth = yawsmooth/40;
  }

  
//SCALE YAW TO FOLLOW WALL----------------------------------------------------------

if ( wallsteeren == 1);
{
if (leftwallaverage<=rightwallaverage){
wallsteer = map (leftsumscaled, 2000, 1500,  1400, 1600);
}
if (rightwallaverage<leftwallaverage){
  wallsteer = map (rightsumscaled, 2000, 1500,  1600, 1400);
}
if (wallsteer > 1600){wallsteer = 1600;}
if (wallsteer < 1400) {wallsteer = 1400;}
}

}


void servooutput(){
MOT = map(MOTORout, 1000, 2000, 0, 180);
RUD = map(RUDDERout, 1000, 2000, 0, 180);
MOD = map(modeout, 1000, 2000, 0, 180);

ESP32_ISR_Servos.setPosition(MOTORout,MOT);
ESP32_ISR_Servos.setPosition(RUDDERout,RUD);
ESP32_ISR_Servos.setPosition(modeout,MOD);

}

void gpio(){
  
//bilge = analogRead(A0);
//if (bilge <500){
//  ESP32_ISR_Servos.setPulseWidth(pumpout,(2000));
//}
//else{
// ESP32_ISR_Servos.setPulseWidth(pumpout,iBus.readChannel(6));
// }

//PUM = map(MOTORout, 1000, 2000, 50, 1);


}
void serialprint (){
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;


Serial.print("Direction: ");
        Serial.print(pos*stepAng);
        Serial.print("\t  range: ");
        Serial.print(val);
        Serial.print("\t  rightsum: ");
        Serial.print(rightSum);
        Serial.print("\t  leftsum: ");
        Serial.print(leftSum);
        Serial.print("\t  average: ");
        Serial.print(average);
        Serial.print("\t  ch10 ");
        Serial.print(ch10);
        Serial.print("\t  avoidmode: ");
        Serial.print(avoidmode);
        Serial.print("\t  avoiddirection: ");
        Serial.println(avoiddirection);
}
}
