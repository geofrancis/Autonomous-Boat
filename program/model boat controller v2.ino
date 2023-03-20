#include <Wire.h>
#include <VL53L1X.h>
#include <Servo.h>
#define stepAng  18      // step angle
#define numStep 10        // = 180/stepAng 
#define CtrlIntv  4000000    // this gives 0.05 sec or 50ms




//throttle limits
#define ReverseDistance 300    
#define MINDistance 120  


//steering limits

#define manouverDistance    3000



#define scannerPin 16
#define MOTOR 4     
#define RUDDER 0
#define MODE 5
#define PUMP 2

//RC inputs   
#include "FlySkyiBusESP8266.h"
FlySkyiBusESP8266 iBus(13,15); //rx and tx pins set to 115200  



int ch1;  //throttle
int ch2;  //rudder
int ch3;  //camera yaw
int ch4;  //fire monitor
int ch5;  //anchor winch
int ch6;  //tow winch
int ch7;  //
int ch8;  //flight mode
int ch9;  //avoid gain
int ch10; //avoid mode


Servo scanner;     
Servo MOTORout;
Servo RUDDERout;
Servo modeout; 
Servo pump; 

VL53L1X sensor;

//servo values

int RCRud;
int RCThr;
int AVOIDMODE;
int MULTI;
int flightmode;

int bilge;

int esc;
int pointesc;
int escs;
int yaw;
int pointyaw;
int out;
int rudout;



int yawfollow;
int yawsmooth;
int throttlesmooth;

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


int count = sizeof(distances) / sizeof(distances[0]);
int closestWayPoint = count;
int closestWayPointValue = 32767;



//=======================================================================
// software timer interrupt: counts clock cycles at 80MHz
void inline motorCtrl_ISR(void){


// AVOID AVERAGE LEFT + RIGHT----------------------------------------------
    if (average < manouverDistance)
  { 
    yaw = (int)((rightsumscaled-leftsumscaled)+1500); 

  }
  else {(yaw = RCRud);
  }
 
 for (int i=0; i < 40; i++) {
 yawsmooth = yawsmooth + yaw;
 }
 yawsmooth = yawsmooth/160;

  
// AVERAGE THROTTLE AVOID-----------------------------------------------
    if (average < ReverseDistance)
  {  
    esc = map (average, ReverseDistance, MINDistance, 1500, 900);
  }
    else{
      esc = RCThr;
  }          

 for (int i=0; i < 20; i++) {
 throttlesmooth = throttlesmooth + esc;
 }
 throttlesmooth = throttlesmooth/80;


//AVOID POINT DIRECTION  -----------------------------------
  if (average < manouverDistance)
  {  
   if (avoiddirection < 5){
      avoidturn =0;
    }
    else avoidturn = 1;
    
  if (avoidturn = 0) {
       pointyaw = map (avoiddirection, 0, 4, 1500, 1000);}
  if (avoidturn = 1) 
      {pointyaw = map (avoiddirection, 5, 10, 1500, 2000);}
  }
    else{
      pointyaw = RCRud;
    }

    
// POINT THROTTLE AVOID-----------------------------------------------
 
 if (closest < ReverseDistance)
  {  
 pointesc = map (closest, ReverseDistance, MINDistance, 1500, 900);
  }
    else{
      esc = RCThr;
  }          


//POINT YAW TO FOLLOW CLOSEST OBJECT----------------------------------------------
if (average < manouverDistance){
if (avoidturn = 0) 
   followturn = map (avoiddirection, 0, 9, 1500, 1700);}
  else {followturn = map (avoiddirection, 10, 19, 1700, 1500);}       



//AVERAGE YAW TO FOLLOW -----------------------------------------------------

 if (average < manouverDistance)
  { 
    yawfollow = (int)((leftsumscaled - rightsumscaled)+1500); 
  }
  else {(yawfollow = RCRud);
  }
 for (int i=0; i < 40; i++) {
 yawsmooth = yawsmooth + yawfollow;
 }
 yawsmooth = yawsmooth/40;


//SCALE YAW TO FOLLOW WALL----------------------------------------------------------
if (leftwallaverage<=rightwallaverage){
wallsteer = map (leftsumscaled, 2000, 1500,  1400, 1600);
}
if (rightwallaverage<leftwallaverage){
  wallsteer = map (rightsumscaled, 2000, 1500,  1600, 1400);
}
if (wallsteer > 1600){wallsteer = 1600;}
if (wallsteer < 1400) {wallsteer = 1400;}



  next=ESP.getCycleCount()+CtrlIntv;
  timer0_write(next);
}
//============================================================================================
void setup() {

  scanner.attach(scannerPin);  //attach scanner servo
  MOTORout.attach(MOTOR);    //attach RIGHTMOTORout servo
  RUDDERout.attach(RUDDER);  
  modeout.attach(MODE);  
  pump.attach(PUMP); 
  
 Wire.begin(12, 14); 
 Wire.setClock(400000); // use 400 kHz I2C

  //Initialize the timer
  noInterrupts();
  timer0_isr_init();
  timer0_attachInterrupt(motorCtrl_ISR);
  next=ESP.getCycleCount()+CtrlIntv;
  timer0_write(next);
  interrupts();

  sensor.setTimeout(500);
  if (!sensor.init())
 {

    while (1);
  }

  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(35000); //35ms
  sensor.startContinuous(35);

}





void loop() { 


//READ RC CHANNELS------------------------------------
ch1 = iBus.readChannel(0);
ch2 = iBus.readChannel(1);
ch3 = iBus.readChannel(2);
ch4 = iBus.readChannel(3);
ch5 = iBus.readChannel(4);
ch6 = iBus.readChannel(5);
ch7 = iBus.readChannel(6);
ch8 = iBus.readChannel(7);
ch9 = iBus.readChannel(8);
ch10 = iBus.readChannel(9);

RCThr = ch1;
RCRud = ch2; 
AVOIDMODE = ch10;
MULTI = ch9;
flightmode = ch8;

bilge = analogRead(A0);


//Measure distance//////////////////////////////////////////////////////////
  pos += dir;
  scanner.write(pos*stepAng);
  val = sensor.read();
  
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  
  distances[pos] = 0.3*distances[pos] + 0.7*val;
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

    
//FIND CLOSEST OBJECT////////////////////////////////////////////////////////////////////

  for (int i = 0; i < count; i++)
  {
    if (distances[i] < closestWayPointValue)
    {
      closestWayPointValue = distances[i];
      closestWayPoint = i;
    }
  }
avoiddirection = closestWayPoint;


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
if (AVOIDMODE > 1001 && AVOIDMODE < 1100) {
rudout = ((yawsmooth + RCRud)/2);
out = RCThr;
}
//throttle
if (AVOIDMODE > 1101 && AVOIDMODE < 1200) {
out = ((throttlesmooth + RCThr)/2);
rudout = RCRud;
}  
//steering and throttle
if (AVOIDMODE > 1201 && AVOIDMODE < 1300) {
out = ((throttlesmooth + RCThr)/2);
rudout = ((yawsmooth + RCRud)/2);
}  
//point steering only
if (AVOIDMODE > 1301 && AVOIDMODE < 1400) {
rudout = ((pointyaw + RCRud)/2);
out = RCThr;
}
//point throttle
if (AVOIDMODE > 1401 && AVOIDMODE < 1500) {
out = ((pointesc + RCThr)/2);
rudout = RCRud;
}

//wall following
//if (AVOIDMODE > 1601 && AVOIDMODE < 1800) {
//out = ((esc + RCThr)/2);
//rudout = ((yaw + RCRud + wallsteer)/3);
//}  

//object following
//if (AVOIDMODE > 1801 && AVOIDMODE < 2000) {
//out = ((esc + RCThr)/2);
//rudout = ((yaw + RCRud + followturn)/3);
//}  
//if (AVOIDMODE >= 2000) {
//}
//OUTPUT///////////////////////////////////////////////////////
 
//if (out >1500){rudmix = (rudout);}
// else{
//  rudmix = (1500 - rudout + 1500);
// }


if (out > 2000){out = 2000;}
if (out < 1000) {out = 1000;}

if (rudout > 2000){rudout = 2000;}
if (rudout < 1000) {rudout = 1000;}

 
MOTORout.writeMicroseconds(out);
RUDDERout.writeMicroseconds(rudout);
modeout.writeMicroseconds(flightmode);



if (bilge <500){
  pump.writeMicroseconds(2000);
}
else{
 pump.writeMicroseconds(ch7);
 }


}

 
