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
* Joystick throttle mapped directly to motor power
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
<b>Vizualization of the three axes of rotation. Y = roll, X = pitch, and Z = yaw.</b>
</p>

The onboard accelerometer and gyroscope are used to calculate the rotation of the drone along these three axes in real time. The actual rotation is fed into the control algorithm to calculate the new motor speeds necessary to correct for the current unwanted rotation. The flight controller algorithm runs in a simple loop making these corrections continously throughout the duration of the flight. The stability of the drone is ultimately a function of how quickly corrections are calculated and implemented by the motors. Each component on the drone plays a role in accomplishing this task.

### The Control Algorithm
Control theory is a branch of Applied Mathematics that deals with the use of feedback to influence the behaviour of a system in order to achieve a desired goal (source 1). In the case of drone flight, the current rotation about each axis must be "fed-back" into the control algorithm and used to determine how much the motor speeds should change to achieve the desired rotation set by the joysticks.

The method of feedback control used in the Osprey Flight Controller is known as PID Control. PID control is one of the most common control algorithms used in industry because it is simple to understand and implement yet still provides robust performance. As the name suggests, PID algorithms consists of three basic coefficients; proportional, integral and derivative which are varied or "tuned" to get the optimal response. 

To better understand PID control lets use the roll axis of the osprey drone as an example. As previously discussed, the flight controller calculates the drones attitude about the roll axis and compares the current rotation to the desired rotation set by the joystick. Let's say the desired roll is zero degrees but a gust of wind induces a roll of 30º. The error term (the difference between the current and desired rotation) is now 30º.

<p align="center">
<img src="/images/roll.jpg" width="300"/>
</p>

<p align="center">
<b>Drone rolling 30º resulting in an error term of 30.</b>
</p>

This error term is then passed into the PID controller. Inside the PID controller a proportional, integral, and derivative response to this error term are calculated separately and finally summed together. The sum of the proportional, integral, and derivative response to the error term is the output of the controller which is used to calculate the new motor speeds.  

<p align="center">
<img src="/images/pid.jpg" width="300"/>
</p>

<p align="center">
<b>The structure of a PID controller showing the error term passed through the proportional, integral, and derivative paths.</b>
</p>


In this case, because the drone is rolling to the left we would expect our PID controller to reduce the power to the motors on the right and increase the power to the motors on the left. 





So, to recap, basic idea behind a PID controller is to read a sensor, compute the difference between the sensor reading and the desired setpoint, compute the desired change in motor power by calculating proportional, integral, and derivative responses and summing those three components to compute the controller output. 

Closed loop system vs open loop system

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


