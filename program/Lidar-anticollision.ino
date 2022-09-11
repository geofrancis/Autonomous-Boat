
//defines/////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include <VL53L1X.h>
#include <Servo.h>
#define stepAng  30       // step angle
#define numStep 6        // = 180/stepAng 
#define CtrlIntv  100000        // this gives 0.05 sec or 50ms  // sampling interval for the motor control @80MHz
#define MinDistance 500     // 100mm

//PWM outputs
#define rudderpin D1     
#define scannerPin D2       
#define motrpin D5        
#define motlpin D6       
#define motpin D0   
//RC inputs     
#define RCmotpin D7        
#define RCrudpin D8      




//initialise////////////////////////////////////////////////////////////////////

//create lidar object
VL53L1X sensor;

//create servo objects.
Servo scanner;     
Servo motorL;
Servo motorR;
Servo Rudder;
Servo motor;


//rc inputs 
int RCRud;
int RCThr;

//servo values
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
int rudders;

//servo outputs us
int lout;
int rout;
int out;



//lidar values
int pos = 0;          // servo position
int dir = 1;          // servo moving direction: +1/-1
int val;              // LiDAR measured value
float distances[numStep+1]; // array for the distance at each angle, including at 0 and 180
float leftSum, rightSum, frontSum;
volatile unsigned long next;


// software timer interrupt: counts clock cycles at 80MHz////////////////////////////////////////////////////////////////////////
void inline motorCtrl_ISR(void){

//if distance is under critical distance execute hard left or right escape manouver.
  if (frontSum < MinDistance)
  {
    if (leftSum < rightSum)
      turnRight();     
    else
      turnLeft();
  }          
// if steer left or right based on the average of each side 

  int lsp = (int)(1000*rightSum/(leftSum+rightSum));
  int rsp = (int)(1000*leftSum/(leftSum+rightSum));

  
  
  LeftMove(lsp, true);
  RightMove(rsp, true); 

  next=ESP.getCycleCount()+CtrlIntv;
  timer0_write(next);
  
}

void setup() {///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.begin(115200);        //start serial
  scanner.attach(scannerPin);  //attach scanner servo
  motorL.attach(motlpin);      //attach left motor
  motorR.attach(motrpin);      //attach right motor
  Rudder.attach(rudderpin);    //attach rudder servo
  motor.attach(motpin);        //attach centre motor
  
  pinMode(RCrudpin, INPUT);    //setup rudder PWM servo input
  pinMode(RCmotpin, INPUT);    //setup throttle PWM servo input
  delay(1000);
    
  // setup i2c
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
//////////////////////////////////////////////////////////////////////////////////////////////////////////////



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
  Lescs = 500;  
  delay(1000);  
}


void turnLeft()
{
  Rescs = 500;
  Lescs = 100;
  delay(1000);  
}

void averagesum()
{
    // find the left and right average sum
  if (pos > (numStep/2))
    leftSum = 0.3*leftSum + 1.4*distances[pos]/numStep; 
  else if (pos < (numStep/2))
    rightSum = 0.3*rightSum + 1.4*distances[pos]/numStep;

  // find the front average sum
  if ((pos > (numStep/4)) && (pos < (numStep*3/4)))
    frontSum = 0.3*frontSum + 1.4*distances[pos]/numStep; 
  
}

void RCinput()
{
RCThr = pulseIn(RCmotpin, HIGH);
RCRud = pulseIn(RCrudpin, HIGH); 
}

void mixing(){
 Lesc = map (Lescs, 100, 700, 900, 2100);
 Resc = map (Rescs, 100, 700, 900, 2100);
 rmix = ((RCThr + RCRud)/2);
 lmix = ((1500 + (RCThr - RCRud)));
 rout = ((rmix + Resc)/2);
 lout = ((lmix + Lesc)/2);
 
 rudders = ((Lesc + RCRud)/2);

 if (lout > rout){   out = rout;}
 else { out = rout;}
}

void output()
{
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


void loop() { ////////////////////////////////////////////////////////////////////////////////////////
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

 
void averagesum();
void RCinput();
void mixing();
void output();  

}/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
