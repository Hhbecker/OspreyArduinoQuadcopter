# The Osprey Drone Mark I with Osprey Flight Controller Version 1.0
A from scratch quadcopter build with an Arduino programmed as a flight controller. 

#### Features:
* automated PID stabilization for roll pitch and yaw axes 
* joystick control of roll pitch and yaw axes 
* joystick throttle mapped directly to motor speed
* arduino uno as flight controller with roughly 0.05 second loop time

## Circuit Diagram
<p align="center">
<img src="/images/fritzing.jpg" width="100%"/>
</p>


## Code explanation 
The arduino runs a continuous loop with the same main functions called each time the loop executes.
<p align="center">
<img src="/images/mainLoop1.jpg" width="300"/>
</p>


#### Why I'm interested in drones
I've really enjoyed this project because it exposed me to several different interesting areas of science and engineering. Drones rely on physics, electrical engineering, computer science, and math in order to fly. I got experience developing embedded software for a real-time application.


* Physics: to understand the forces acting on the drone in flight
* Electrical engineering: circuit design and construction
* Embedded software development: programming on a resource constrained microcontroller 
* Real time programming: programming where the timing of the program is crucial
* Control theory: understanding how to optimize the control algorithm using mathematical techniques from control theory 
* Wireless communication: understanding the transfer of information from the RC transmitter to the reciever


All of this requires a healthy amount of math

Real world applications for drones
Disaster relief 
Law enforcement
War
Agriculture



### Next Steps:
* add altitude control 
* replace arduino uno with multiple arduino nanos
* redesign hardware to be more condensed and centralized on drone frame

### Design flaws 
* maybe wires were too long 


### Basics of Drone flight 

### PID tuning 
### Fritzing diagram
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


