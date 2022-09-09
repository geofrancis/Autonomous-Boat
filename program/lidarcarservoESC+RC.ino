/* This robot automatically avoid the obstracles and find the direction by using
  a LiDAR sensor sweeping in front of the robot. The speed of the wheels is
  controled by the PWM signal using the analogWrite() function.
  
  Author: Udom
  Date: 9 Oct 2019
  *** you may need to disconnect the motor control pins before uploading the sketch ***
*/

#include <Wire.h>
#include <VL53L1X.h>
#include <Servo.h>

#define stepAng  30       // step angle
#define numStep 6        // = 180/stepAng 

#define servoPin D2        // GIO2 = D4
#define motrpin D5        // GIO2 = D4
#define motlpin D6        // GIO2 = D4
#define motpin D7        // GIO2 = D4
#define rudpin D8        // GIO2 = D4
// motors control pins


// sampling interval for the motor control @80MHz
#define CtrlIntv  100000        // this gives 0.05 sec or 50ms
#define MinDistance 100     // 100mm

Servo myservo;      // create servo object to control a servo

Servo motorL;
Servo motorR;


int Lesc = 1500;
int Resc = 1500;
int Lescs = 500;
int Rescs = 500;

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
  myservo.attach(servoPin);  
  motorL.attach(motlpin);
  motorR.attach(motrpin);

  pinMode(rudpin, INPUT);
  pinMode(motpin, INPUT);
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
  esc = 50;
}

void turnLeft()
{
  Rescs = 100;
  Lescs = 50;
  esc = 1000;
}

void loop() { 
  pos += dir;
  myservo.write(pos*stepAng);
    
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

RCThr = pulseIn(motpin, HIGH);
RCRud = pulseIn(rudpin, HIGH);    

 Lesc = map (Lescs, 100, 600, 800, 2100);
 Resc = map (Rescs, 100, 600, 800, 2100);
 rmix = ((RCThr + RCRud)/2);
 lmix = ((1500 + (RCThr - RCRud)));
 rout = ((rmix + Resc)/2);
 lout = ((lmix + Lesc)/2);
  motorL.writeMicroseconds(lout);
  motorR.writeMicroseconds(rout);

 
Serial.println(" Lmotor us ");
Serial.println(lout);


Serial.println(" Rmotor us ");
Serial.println(rout);


Serial.println(" lidar ");
Serial.println(val);
}
