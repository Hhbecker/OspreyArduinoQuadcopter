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

### The basics of quadcopter flight
A drone has three axes of rotation: roll, pitch, and yaw. The primary goal of the Osprey Flight Controller V1.0 is to maintain stability in flight by continously adjusting the speed of each individual motor in order to abruptly and accurately correct any unwanted rotation about any of the three axes.

<p align="center">
<img src="/images/rotation.jpg" width="300"/>
</p>

<p align="center">
<b>Vizualization of the three axes of rotation.</b>
</p>




In order to maintain stable flight, the drone must be able to abruptly and accurately correct for any unwanted rotation about each axis. The onboard accelerometer and gyroscope are used to calculate the rotation of the drone along these three axes in real time. The actual rotation is fed into the flight controller algorithm to calculate the new individual motor speeds to correct for the unwanted rotation. The flight controller runs in a loop making these corrections continously throughout the duration of the flight.



### Control Theory

Control theory is a branch of Applied Mathematics dealing with the use of feedback to influence the behaviour of a system in order to achieve a desired goal (source 1).

Control theory is used to keep the drone at the specified angle of rotation along each of rotation. During takeoff and when achieving a stable hover, the desired angle of rotation about each axis is zero. Any nonzero rotation is known as the error term. The actual angle of rotation (the error term) is calculated using the onboard accelerometer and gyroscope. This error term is then fed back into the control loop in order to calculate the new motor speeds to reach an error of zero as quickly as possible without overshooting. 

 and this information is "fed-back" and compared to the speed setting on the cruise control, which causes the engine to accelerate if the speed is too low. The cruise control device is referred to as the controller in the system.


### Circuit Diagram
<p align="center">
<img src="/images/fritzing.jpg" width="80%"/>
</p>

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
* Decrease flight controller refresh rate and upgrade hardware to improve stability 
* Replace arduino uno with multiple arduino nanos
* Redesign hardware to be more condensed and centralized on drone frame
* Add altitude controller for hover
* Add translation controller to prevent wind drift 

### Design Weaknesses  
* 
* 
* 


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


### Sources

1. https://uwaterloo.ca/applied-mathematics/future-undergraduates/what-you-can-learn-applied-mathematics/control-theory


