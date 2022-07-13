# The Osprey Drone Mark I with Osprey Flight Controller Version 1.0
A custom quadcopter build with an Arduino Uno flight controller. 

<p align="center">
<img src="/images/flightTests/flightPhoto.jpg" width="350"/>
</p>

#### Features:
* automated PID stabilization for roll pitch and yaw axes 
* joystick control of roll pitch and yaw axes 
* joystick throttle mapped directly to motor speed
* arduino uno as flight controller with roughly 0.05 second loop time

### Why Drones?
I find drones to be the perfect engineering challenge because they require knowledge from a range of disciplines including physics, electrical engineering, computer science, and math. 
This drone project also provided me experience developing embedded software for a real-time application, designing and constructing circuits and electrical hardware, and implementing an algorithm based on control theory mathematics. Not only are drones a great engineering challenge but they also have innumerous real world applications including in disaster relief, law enforcement, agriculture, and war. 

### The basics of quadcopter flight

### Circuit Diagram
<p align="center">
<img src="/images/fritzing.jpg" width="80%"/>
</p>

### Code explanation 
The arduino runs a continuous loop with the same main functions called each time the loop executes.
<p align="center">
<img src="/images/mainLoop1.jpg" width="350"/>
</p>

### PID tuning 

### Next Steps:
* add altitude control 
* replace arduino uno with multiple arduino nanos
* redesign hardware to be more condensed and centralized on drone frame

### Design flaws 
* maybe wires were too long 

### Photos of construction

<p align="center">
<img src="/images/dronePictures/build1.jpg" width="400"/>
<img src="/images/dronePictures/build2.jpg" width="300"/>
<img src="/images/dronePictures/build3.jpg" width="300"/>

<img src="/images/dronePictures/finished1.jpg" width="300"/>
<img src="/images/dronePictures/finished2.jpg" width="300"/>
<img src="/images/dronePictures/finished3.jpg" width="300"/>
</p>

<p align="center">
<img src="/images/flightTests/test2.gif" width="400"/>
</p>

### Parts list with explanation of each part


