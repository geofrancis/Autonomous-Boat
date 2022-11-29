#include <Wire.h>
#include <VL53L1X.h>
#include <Servo.h>
#define stepAng  18      // step angle
#define numStep 10        // = 180/stepAng 
#define CtrlIntv  4000000    // this gives 0.05 sec or 50ms
#define TurnDelay 300       // turn for 300ms
#define ReverseDistance 1000    
#define AvoidDistance 3000     
#define MINDistance 300     

#define scannerPin 16
#define RIGHTMOTOR 4     
#define LEFTMOTOR 5
#define RUDDER 2      
  
//RC inputs   
#include "FlySkyiBusESP8266.h"
FlySkyiBusESP8266 iBus(D7,D8); //rx and tx pins set to 115200  


//RC inputs     
#define RCmotpin 0        
#define RCrudpin 3    




#include <Adafruit_MCP23X17.h>
Adafruit_MCP23X17 mcp;

int ch1;
int ch2;
int ch3;
int ch4;
int ch5;
int ch6;
int ch7;
int ch8;
int ch9;
int ch10;


Servo scanner;     
Servo RIGHTMOTORout;
Servo LEFTMOTORout;
Servo RUDDERout;

VL53L1X sensor;

//servo values

int RCRud;
int RCThr;
int AVOIDMODE;


int esc;
int escs;
int yaw;
int out;
int rudout;

int lmix;
int rmix;
int rudmix;
const int numReadings = 10;
int readings[numReadings];  // the readings from the analog input
int readIndex = 0;          // the index of the current reading
int total = 0;              // the running total
int average = 0;            // the average


#define manouverDistance    3000
#define avoiddistance       1000 
#define minreversedistance  1000
#define fullreversedistance  300 
#define followdistance       300 

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

//=======================================================================
// software timer interrupt: counts clock cycles at 80MHz
void inline motorCtrl_ISR(void){


// AVERAGE LEFT + RIGHT----------------------------------------------
    if (average < AvoidDistance)
  { 
    yaw = (int)((rightsumscaled-leftsumscaled)+1500); 
  }
  else {(yaw = RCRud);
  }
  averagescale = (manouverDistance-average);

  
// AVERAGE THROTTLE AVOID-----------------------------------------------
    if (average < ReverseDistance)
  {  
    esc = map (average, ReverseDistance, MINDistance, 1500, 500);
  }
    else{
      esc = RCThr;
  }          
//SELECT TURN DIRECTION AND SCALE YAW -----------------------------------
    if (avoiddirection < 5){
      avoidturn =0;
    }
    else avoidturn = 1;
  
  if (avoidturn = 0) {
       yaw = map (avoiddirection, 0, 4, 1500, 1000);}
  if (avoidturn = 1) 
      {yaw = map (avoiddirection, 5, 10, 1500, 2000);}

// AVERAGE DISTANCE THROTTLE LIMITER------------------------------
if (average < manouverDistance) {  
   minthrottle = map (average, fullreversedistance, manouverDistance, 1400, 1000);
   maxthrottle = map (average, fullreversedistance, manouverDistance, 1600, 2000);
}
else{
  minthrottle = 1000;
  maxthrottle = 2000;
}      


//SCALE THROTTLE TO CLOSEST DISTANCE--------------------------------
if (closest < minreversedistance) {  
   esc = map (average, minreversedistance, fullreversedistance, 1500, 500);
  }
    else{
   esc = RCThr;
} 

//SCALE YAW TO FOLLOW CLOSEST OBJECT----------------------------------------------
if (average < manouverDistance && average > avoiddistance){
  if (avoidturn = 0) 
        followturn = map (avoiddirection, 0, 9, 1500, 1700);}
  else {followturn = map (avoiddirection, 10, 19, 1700, 1500);}        


//SCALE YAW TO FOLLOW WALL----------------------------------------------------------
if (leftwallaverage<=rightwallaverage){
 wallsteer = map (leftwallaverage, 3, 2.5,  1400, 1600);
}
if (rightwallaverage<leftwallaverage){
  wallsteer = map (rightwallaverage, 3, 2.5,  1600, 1400);
}
if (wallsteer > 1600){wallsteer = 1600;}
if (wallsteer < 1400) {wallsteer = 1400;}



  next=ESP.getCycleCount()+CtrlIntv;
  timer0_write(next);
}
//============================================================================================
void setup() {

  pinMode(RCrudpin, INPUT);    //setup rudder PWM servo input
  pinMode(RCmotpin, INPUT);    //setup throttle PWM servo input
  

  scanner.attach(scannerPin);  //attach scanner servo
  RIGHTMOTORout.attach(RIGHTMOTOR);    //attach RIGHTMOTORout servo
  LEFTMOTORout.attach(LEFTMOTOR);        //attach centre motor
  RUDDERout.attach(RUDDER);        //attach centre motor
  

mcp.pinMode(1, OUTPUT);
mcp.pinMode(2, OUTPUT);
mcp.pinMode(3, OUTPUT);
mcp.pinMode(4, OUTPUT);
mcp.pinMode(5, OUTPUT);
mcp.pinMode(6, OUTPUT);
mcp.pinMode(7, OUTPUT);
mcp.pinMode(8, OUTPUT);

mcp.pinMode(9, INPUT_PULLUP);
mcp.pinMode(10, INPUT_PULLUP);  
mcp.pinMode(11, INPUT_PULLUP);
mcp.pinMode(12, INPUT_PULLUP); 
mcp.pinMode(13, INPUT_PULLUP);
mcp.pinMode(14, INPUT_PULLUP); 
mcp.pinMode(15, INPUT_PULLUP);
mcp.pinMode(16, INPUT_PULLUP); 




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
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(35000); //35ms
  sensor.startContinuous(35);



    
   Serial.println(" setup ");
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
//RCThr = pulseIn(RCmotpin, HIGH); use pwm input for flight controller
//RCRud = pulseIn(RCrudpin, HIGH); 
AVOIDMODE = ch3;

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

  //change servo direction-----------------    
  if (pos == numStep)
  {
    dir = -1;
  }
  else if (pos == 0)
  {
    dir = 1;
  }

////////////////////////////////////////////////////////////// 
   // find the left and right average sum
  if (pos > (numStep/2))
    leftSum = 0.3*leftSum + 1.4*distances[pos]/numStep; 
  else if (pos < (numStep/2))
    rightSum = 0.3*rightSum + 1.4*distances[pos]/numStep;


    leftsumscaled = (leftSum + averagescale);
   rightsumscaled = (rightSum + averagescale);

   
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
 
 
//MODE SELECTION/////////////////////////////////////////////////////////



//off
if (AVOIDMODE < 1000) {
out = RCThr;
rudout = RCRud;
}
//average steering only
if (AVOIDMODE > 1001 && AVOIDMODE < 1200) {
rudout = ((yaw + RCRud)/2);
out = RCThr;
}
//throttle
if (AVOIDMODE > 1201 && AVOIDMODE < 1400) {
out = ((esc + RCThr)/2);
rudout = RCRud;
}  
//steering and throttle
if (AVOIDMODE > 1401 && AVOIDMODE < 1600) {
out = ((esc + RCThr)/2);
rudout = ((yaw + RCRud)/2);
}  
//wall following
if (AVOIDMODE > 1601 && AVOIDMODE < 1800) {
out = ((esc + RCThr)/2);
rudout = ((yaw + RCRud + wallsteer)/3);
}  

//object following
if (AVOIDMODE > 1801 && AVOIDMODE < 2000) {
out = ((esc + RCThr)/2);
rudout = ((yaw + RCRud + followturn)/3);
}  
if (AVOIDMODE >= 2000) {

}
//OUTPUT///////////////////////////////////////////////////////

 rmix = ((out + rudout)/2);
 lmix = ((1500 + (out - rudout)));
 
 if (out >1500){
 rudmix = (rudout);}
 else{
  rudmix = (1500 - rudout + 1500);
 }
 
  RUDDERout.writeMicroseconds(rudmix);
  
if (RCThr < 700){
  LEFTMOTORout .writeMicroseconds(1500);
  RIGHTMOTORout.writeMicroseconds(1500);
}
else {
  LEFTMOTORout.writeMicroseconds(lmix);
  RIGHTMOTORout.writeMicroseconds(rmix);
}

//lights
if (ch10 > 1500){
mcp.digitalWrite(1,1);
}
else {mcp.digitalWrite(1,0);
}
if (ch10 > 1800){
mcp.digitalWrite(2,1);
} 
else {mcp.digitalWrite(2,0);
}

//pumps
if (ch9 < 1000){
mcp.digitalWrite(1,1);
}
else {mcp.digitalWrite(1,0);
}
if (ch9 > 2000){
mcp.digitalWrite(2,1);
} 
else {mcp.digitalWrite(2,0);
}

//vibration motor
if (ch8 < 1500){
mcp.digitalWrite(1,1);
}
else {mcp.digitalWrite(1,0);
}



//


}

 
