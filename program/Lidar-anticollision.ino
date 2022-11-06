

#include <Wire.h>
#include <VL53L1X.h>
#include <Servo.h>
#define stepAng  30       // step angle
#define numStep 6        // = 180/stepAng 
#define CtrlIntv  4000000    // this gives 0.05 sec or 50ms
#define TurnDelay 300       // turn for 300ms
#define ReverseDistance 1000    
#define AvoidDistance 3000     
#define MINDistance 300     

#define scannerPin D0
#define rudderpin D8     
#define motpin D7
       
  
//RC inputs     
#define RCmotpin D1        
#define RCrudpin D2     
#define ENpin D3


Servo scanner;     
Servo Rudder;
Servo motor;


VL53L1X sensor;

//servo values

int RCRud;
int RCThr;
int ENABLE;


int esc;
int escs;
int yaw;
int out;
int rudout;

int lmix;
int rmix;

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

RCThr = pulseIn(RCmotpin, HIGH);
RCRud = pulseIn(RCrudpin, HIGH); 
ENABLE = pulseIn(ENpin, HIGH);

if (average < AvoidDistance)
  { 
    yaw = (int)((rightSum-leftSum)+1500); 
  }
  else {(yaw = RCRud);
  }
if (average < ReverseDistance)
  {  
    esc = map (average, ReverseDistance, MINDistance, 1500, 500);
  }
    else{
      esc = RCThr;
  }          


  next=ESP.getCycleCount()+CtrlIntv;
  timer0_write(next);
}
//============================================================================================
void setup() {
  Serial.begin(115200);

  scanner.attach(scannerPin);  //attach scanner servo
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





void loop() { 
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
 Serial.println(" average ");
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




 out = ((esc + RCThr)/2);
 rudout = ((yaw + RCRud)/2);

 
 
/////////////////////////////////////////////////////////
if (ENABLE <= 1500){
out = RCThr;  
}
if (ENABLE <= 1500) {
  rudout = RCRud;
}
/////////////////////////////////////////////////////////

 rmix = ((out + rudout)/2);
 lmix = ((1500 + (out - rudout)));

 
 
if (RCThr < 700){
  motor .writeMicroseconds(1500);
  Rudder.writeMicroseconds(1500);
}
else {
  motor.writeMicroseconds(lmix);
  Rudder.writeMicroseconds(rmix);
}

}
