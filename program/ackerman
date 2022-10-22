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
#define CtrlIntv  4000000    // this gives 0.05 sec or 50ms
#define TurnDelay 300       // turn for 300ms
#define MinDistance 100     // 100mm
#define MaxDistance 1000     // 100mm


#define scannerPin D0
#define rudderpin D8     
#define motpin D7
//#define motrpin D3        
//#define motlpin D4       
  
//RC inputs     
#define RCmotpin D1        
#define RCrudpin D2     
#define ENpin D3
// sampling interval for the motor control @80MHz




Servo scanner;     
Servo motorL;
Servo motorR;
Servo Rudder;
Servo motor;


VL53L1X sensor;

//servo values

int RCRud;
int RCThr;
int ENABLE;

int Lesc;
int Resc;
int esc;
int rudd;
int LDir;  
int LSpd; 
int RSpd;
int RDir; 
int lmix;
int rmix;
int Lescs;
int Rescs;
int escs;
int rudmix;
int outmix;
//servo outputs us
int lout;
int rout;
int out;
int rudout;


const int numReadings = 12;

int readings[numReadings];  // the readings from the analog input
int readIndex = 0;          // the index of the current reading
int total = 0;              // the running total
int average = 0;            // the average

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

void setup() {
  Serial.begin(115200);

  scanner.attach(scannerPin);  //attach scanner servo
//  motorL.attach(motlpin);      //attach left motor
//  motorR.attach(motrpin);      //attach right motor
  Rudder.attach(rudderpin);    //attach rudder servo
  motor.attach(motpin);        //attach centre motor
  
  pinMode(RCrudpin, INPUT);    //setup rudder PWM servo input
  pinMode(RCmotpin, INPUT);    //setup throttle PWM servo input
  pinMode(ENABLE, INPUT);    //setup throttle PWM servo input
  
 Wire.begin(D5, D6); 
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
   Rescs = 100;
  Lescs = 300;  
  delay(1000);  
}

void turnLeft()
{
  Rescs = 300;
  Lescs = 100;
  delay(1000);  
}





void loop() { 
  pos += dir;
  scanner.write(pos*stepAng);
    
  val = sensor.read();
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  distances[pos] = 0.3*distances[pos] + 0.7*val;
      Serial.println("val ");
      Serial.println(val);



// subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = val;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }
 average = total / numReadings;
 Serial.println(average);

      
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
   //   Serial.println("frontSum ");
  //    Serial.println(frontSum);
      
  //    Serial.println("leftSum ");
  //    Serial.println(leftSum);

 //     Serial.println("rightSum ");
 //     Serial.println(rightSum);



RCThr = pulseIn(RCmotpin, HIGH);
RCRud = pulseIn(RCrudpin, HIGH); 
ENABLE = pulseIn(ENpin, HIGH);


  //    Serial.println("RCThr ");
  //    Serial.println(RCThr);
  //    Serial.println("RCRud ");
 //     Serial.println(RCRud);
 

 Lesc = map (Lescs, 100, 700, 1000, 2000);
 Resc = map (Rescs, 100, 750, 1000, 2000);
 
 rmix = map (RCThr, 1000, 2000, 1000, 2000);
 lmix = map (RCRud, 1000, 2000, 1000, 2000);
 
 rmix = ((RCThr + RCRud)/2);
 lmix = ((1500 + (RCThr - RCRud)));
 
// rout = ((rmix + Resc)/2);
// lout = ((lmix + Lesc)/2);
 
 
////////////////////////////////////////////////////////
if (average < 2000){
outmix = map (average, 500, 2000, 1000, 1500);  
}
else {
outmix =1500;
}
if (outmix > 1500){
outmix = 1500;
}

      Serial.println("outmix");
      Serial.println(outmix);

if (average < 500){
out = 1000;  
}
else {
out =((outmix + RCThr)/2);
}
      
if (average >= 2000){
out = RCThr;  
}
      
if (ENABLE <= 1500){
out = RCThr;  
}

/////////////////////////////////////////////////////////

if (out > 1500){
rudmix = Lesc;
}
else {
rudmix = Lesc;
}

if (ENABLE <= 1500) {
  rudout = RCRud;
}
else {
 rudout = ((rudmix + RCRud)/2);
} 
      


      Serial.println("Rescs ");
      Serial.println(Rescs);
      Serial.println("Lescs ");
      Serial.println(Lescs);

  //    Serial.println("Resc ");
  //    Serial.println(Resc);
  //    Serial.println("Lesc ");
  //    Serial.println(Lesc);

   //   Serial.println("rmix ");
   //   Serial.println(rmix);
   //   Serial.println("lmix ");
   //   Serial.println(lmix);


   //   Serial.println("rout ");
   //   Serial.println(rout);
   //   Serial.println("lout ");
   //   Serial.println(lout);

    //  Serial.println("rudmix ");
   //   Serial.println(rudmix);
   //   Serial.println("rudout ");
    //  Serial.println(rudout);

      Serial.println("outmix % ");
      Serial.println(outmix);
      Serial.println("out us");
      Serial.println(out);




if (RCThr < 700){
 // motorL.writeMicroseconds(1500);
 // motorR.writeMicroseconds(1500);
  motor .writeMicroseconds(1500);
  Rudder.writeMicroseconds(1500);
  Serial.println(" throttle failsafe <700us ");
  Serial.println(RCThr);
}
else {
//  motorL.writeMicroseconds(lout);
//motorR.writeMicroseconds(rout);
  motor.writeMicroseconds(out);
  Rudder.writeMicroseconds(rudout);
  
 // Serial.println(" rudder us ");
 // Serial.println(rudout);
  Serial.println(" motor us ");
  Serial.println(out);
  // Serial.println(" motorL us ");
  //Serial.println(lout);
 //  Serial.println(" motorR us ");
 // Serial.println(rout);
}
}
