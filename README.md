# Autonomous-BOAT
This robot automatically avoid the obstracles and find the direction by using
  a LiDAR sensor sweeping in front of the Boat.
  
This version has been modified for a boat so it has:

RC input for throttle and rudder for use with an external flight controller

Ibus RC input for RC and function control

IBUS RC output for connection to flight controller

Ported to ESP32-S2 from ESP8266.

lidar new data is used as timer




  https://youtu.be/HKAlPZj2agQ
  
  
  
It has several modes, 

avoidance Mode 1 averages the left and right readings and that is used to calculate the steering input along with a speed scaling factor ideal for avoiding large objects and walls or following a corrodor.

avoidance Mode 2 uses the closest object to determin the steering rate, objects directly infront will cause larger steering inputs that objects off to the side. I use this as a small object avoidance as single returns can get averaged out in mode 1 and insufficent corrective actions will be taken.

Throttle Mode 1 uses an average of the last few returns to use the throttle for avoidance. this will cause the vehicle to keep a minimum distance from objects 

Throttle Mode 2 uses the closest return from the lidar to control the throttle, its better for use with mode 2 as smaller objects can get averaged out.

wall follow mode looks at the outer most readings only to find a wall to the left or right of the vehicle, it will then try and use small corrections to keep the wall a fixed distance away, This Mode is designed for going around a round pond with a well defined brim in a circle without any manual inputs. it should work for other shapes like a square pool.

object follow mode, this is designed to keep an object directly infront of the vehicle, its basically avoidance mode 2 but with inverted steering to turn towrds the object rather than away.

