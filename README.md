# Autonomous-BOAT
This robot automatically avoid the obstracles and find the direction by using
  a LiDAR sensor sweeping in front of the robot.
  
This version has been modified for a boat so it has:
RC input for throttle and rudder, 
outputs for left and right ESC or servo wheels 
outputs for single throttle and rudder


servoPin D2     scanner servo

motpin D0       motor in   

rudderpin D1    rudder in
      
motrpin D5     right motor out 

motlpin D6     left motor out

motpin D7      motor out
rudpin D8      rudder out

Component List:
1. 2x TT motors with wheels
2. 1x front wheel
3. 1x NodeMCU
4. 1x LiDAR sensor (VL53L1X)
5. 1x Dual H-Bridge motor driver
6. 1x 18650 Li-ion battery
7. 1x 18650 battery case
8. 1x 18650 Li-ion battery charging module
9. 1x On/Off switch
10. 1x DC/DC step-up converter
11. 1x SG90 servo motor
12. 1x Rectifier diode
13. 1x 6.3v 820uF Capacitor
