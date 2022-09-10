/* This robot automatically avoid the obstracles and find the direction by using
  a LiDAR sensor sweeping in front of the robot. The speed of the wheels is
  controled by the PWM signal using the analogWrite() function.
  
  Author: Udom
  Date: 9 Oct 2019
  *** you may need to disconnect the motor control pins before uploading the sketch ***
*/
//defines/////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include <VL53L1X.h>
#include <Servo.h>
#define stepAng  30       // step angle
#define numStep 6        // = 180/stepAng 
#define CtrlIntv  100000        // this gives 0.05 sec or 50ms  // sampling interval for the motor control @80MHz
#define MinDistance 100     // 100mm

#define rudderpin D1     
#define scannerPin D2       
#define motrpin D5        
#define motlpin D6       
#define motpin D0        
#define RCmotpin D7        
#define RCrudpin D8      


Servo scanner;      // create servo object to control a servo
Servo motorL;
Servo motorR;
Servo Rudder;
Servo motor;

int Lesc = 1500;
int Resc = 1500;
int Lescs = 500;
int Rescs = 500;
int escs = 500;
int rudders = 1500;
int esc = 1500;
int rudd = 1500;


VL53L1X sensor;
int LDir = 1;  
int LSpd = 1; 
int RSpd = 1;
int RDir = 1; 
int RCRud;
int RCThr;
int lmix;
int rmix;
int lout;
int rout;
int out;


int pos = 0;          // servo position
int dir = 1;          // servo moving direction: +1/-1
int val;              // LiDAR measured value

float distances[numStep+1]; // array for the distance at each angle, including at 0 and 180

float leftSum, rightSum, frontSum;

volatile unsigned long next;

//=======================================================================
// software timer interrupt: counts clock cycles at 80MHz
void inline motorCtrl_ISR(void){
  if (frontSum < MinDistance)
  {
    if (leftSum < rightSum)
      turnRight();     
    else
      turnLeft();
  }          


  int lsp = (int)(1000*rightSum/(leftSum+rightSum));
  int rsp = (int)(1000*leftSum/(leftSum+rightSum));

  
  
  LeftMove(lsp, true);
  RightMove(rsp, true); 

  next=ESP.getCycleCount()+CtrlIntv;
  timer0_write(next);
  
}

void setup() {////////////////////////////////////////////////////////////////////////
  Serial.begin(115200);
  scanner.attach(scannerPin);  
  motorL.attach(motlpin);
  motorR.attach(motrpin);
  Rudder.attach(rudderpin);
  motor.attach(motpin);
  
  pinMode(RCrudpin, INPUT);
  pinMode(RCmotpin, INPUT);
;
  delay(1000);  
  
  Wire.begin(D3, D4); //SDA, 
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
}

void LeftMove(int speed, bool dir)
{
LDir = dir; 
  if (dir = 1)
    Lescs =   (speed);
  else
    Lescs =   (-speed); 


}

void RightMove(int speed, bool dir)
{
LDir = dir; 
  if (dir = 1)
    Rescs =   (speed);
  else
    Rescs = (-speed); 
}


void turnRight()
{
  Rescs = 50;
  Lescs = 100;  
  delay(1000);  
}

void turnLeft()
{
  Rescs = 100;
  Lescs = 50;
  delay(1000);  
}

void loop() { 
  pos += dir;
  scanner.write(pos*stepAng);
    
  val = sensor.read();
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  distances[pos] = 0.3*distances[pos] + 0.7*val;
  
  if (pos == numStep)
  {
    dir = -1;
  }
  else if (pos == 0)
  {
    dir = 1;
  }

  // find the left and right average sum
  if (pos > (numStep/2))
    leftSum = 0.3*leftSum + 1.4*distances[pos]/numStep; 
  else if (pos < (numStep/2))
    rightSum = 0.3*rightSum + 1.4*distances[pos]/numStep;

  // find the front average sum
  if ((pos > (numStep/4)) && (pos < (numStep*3/4)))
    frontSum = 0.3*frontSum + 1.4*distances[pos]/numStep;  

RCThr = pulseIn(RCmotpin, HIGH);
RCRud = pulseIn(RCrudpin, HIGH);    

 Lesc = map (Lescs, 100, 700, 900, 2100);
 Resc = map (Rescs, 100, 700, 900, 2100);
 rmix = ((RCThr + RCRud)/2);
 lmix = ((1500 + (RCThr - RCRud)));
 rout = ((rmix + Resc)/2);
 lout = ((lmix + Lesc)/2);

  rudders = ((Lesc + RCRud)/2);

if (lout > rout){   out = rout;}
else { out = rout;}

if (RCThr < 700){
  motorL.writeMicroseconds(1500);
  motorR.writeMicroseconds(1500);
  motor .writeMicroseconds(1500);
  Rudder.writeMicroseconds(1500);
  Serial.println(" throttle failsafe <700us ");
  Serial.println(RCThr);
}
else {
  motorL.writeMicroseconds(lout);
  motorR.writeMicroseconds(rout);
  motor.writeMicroseconds(out);
  Rudder.writeMicroseconds(rudders);
  
  Serial.println(" rudder us ");
  Serial.println(rudders);
  Serial.println(" motor us ");
  Serial.println(out);
   Serial.println(" motorL us ");
  Serial.println(lout);
   Serial.println(" motorR us ");
  Serial.println(rout);
}
}
