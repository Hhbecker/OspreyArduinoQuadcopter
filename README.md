<h1 align="center">
The Osprey Drone Mark I with Osprey Flight Controller Version 1.0
</h1>

<h5 align="center">
A custom quadcopter build with an Arduino Uno flight controller. 
</h5>

<p align="center">
<img src="/images/flightTests/flightPhoto.jpg" width="350"/>
</p>



#### Features:
* Automated PID stabilization for roll pitch and yaw axes 
* Joystick control of roll pitch and yaw axes 
* Joystick throttle mapped directly to motor speed
* Roughly 20Hz flight controller refresh rate

### Why Drones?
I see drones as the perfect engineering challenge because they require knowledge from a range of disciplines including physics, electrical engineering, computer science, and math. 
This drone project also provided me experience developing embedded software for a real-time application, designing and constructing circuits and electrical hardware, and implementing an algorithm based on control theory mathematics. Not only are drones a great engineering challenge but they also have innumerous real world applications including in disaster relief, law enforcement, agriculture, and war. 

### Circuit Diagram
<p align="center">
<img src="/images/fritzing.jpg" width="80%"/>
</p>

### The basics of quadcopter flight
A drone has three axes of rotation: roll, pitch, and yaw. The primary goal of the Osprey Flight Controller V1.0 is to maintain stability in flight by continously adjusting the speed of each individual motor in order to abruptly and accurately correct any unwanted rotation about any of the three axes.

<p align="center">
<img src="/images/rotation.jpg" width="300"/>
</p>

<p align="center">
<b>Vizualization of the three axes of rotation.</b>
</p>

The onboard accelerometer and gyroscope are used to calculate the rotation of the drone along these three axes in real time. The actual rotation is fed into the control algorithm to calculate the new motor speeds necessary to correct for the current unwanted rotation. The flight controller algorithm runs in a simple loop making these corrections continously throughout the duration of the flight.

The stability of the drone is ultimately a function of how quickly the motors adjust there speed. This timestep is influenced by the speed of the code itself and also the speed and performance of every component in the drone from the battery to the ESCs. Of course, you could have the fastest flight controller in the world but if the ESC only refreshes the motor speed twice every second your drone will not fly.

### The Control Algorithm
Control theory is a branch of Applied Mathematics that deals with the use of feedback to influence the behaviour of a system in order to achieve a desired goal (source 1). In the case of drone flight, the current rotation about each axis must be "fed-back" into the control algorithm and used to determine how much the motor speeds should change to achieve the desired rotation set by the joysticks.

One common method of feedback control is known as PID Control. PID control...

blah blah blah


### State Estimation 
The Osprey Flight Controller incorporates gyroscope and accelerometer data to achieve a 3 degrees of freedom attitude estimation about the roll, pitch, and yaw axes. 


### Code Walkthrough 
The arduino runs a continuous loop with the same main functions called each time the loop executes.
<p align="center">
<img src="/images/mainLoop1.jpg" width="350"/>
</p>

### PID tuning 
* show graphs
* show short roll bar gif 
Ziegler Nicols method 
Tune roll P gain to achieve even oscillations when perturbed
Add D gain to remove overshoot 
Add I gain to compensate for steady state error


### Next Steps:
* Decrease flight controller refresh rate and upgrade hardware
* Replace Arduino Uno with multiple Arduino Nanos
* Add altitude controller for hover
* Add translation controller to prevent wind drift 

### Design Weaknesses  
* There was a lot of unecessary wire between components and so the amount of wire that the signal needed to pass through to get from the arduino to the ESC was probably slowing things down
* The flight controller does not have altitude control or translational control 
* MORE


### Construction Process
<p align="center">
<img src="/images/dronePictures/build1.jpg" width="400"/>
<img src="/images/dronePictures/build2.jpg" width="300"/>
<img src="/images/dronePictures/build3.jpg" width="300"/>
<img src="/images/dronePictures/finished4.jpg" width="400"/>
</p>

<p align="center">
<img src="/images/flightTests/test2.gif" width="400"/>
</p>

### Parts List
Image:
Part: name
Specs:
Include tiny pic and refresh rate of each part 




### Sources

1. https://uwaterloo.ca/applied-mathematics/future-undergraduates/what-you-can-learn-applied-mathematics/control-theory


