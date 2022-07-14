<h1 align="center">
The Osprey Drone Mark I with Osprey Flight Controller Version 1.0
</h1>

<h5 align="center">
A custom quadcopter build with an Arduino Uno flight controller. 
</h5>

<p align="center">
<img src="/images/flightTests/flightPhoto.jpg" width="350"/>
</p>

Mark 2 needs a well thought out software and hardware kill switch


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
<img src="/images/fritzing.jpg" width="75%"/>
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

### Code Structure 
The arduino runs a continuous loop with the same main functions called each time the loop executes. The diagram below shows the main loop functions in the order of their execution. This code is pulled directly from the main loop of the Osprey Flight Controller V1.0.

<p align="center">
<img src="/images/code.jpg" width="600"/>
</p>

### The Control Algorithm
Control theory is a branch of Applied Mathematics that deals with the use of feedback to influence the behaviour of a system in order to achieve a desired goal (source 1). In the case of drone flight, the current rotation about each axis must be "fed-back" into the control algorithm and used to determine how much the motor speeds should change to achieve the desired rotation set by the joysticks.

The method of feedback control used in the Osprey Flight Controller is known as PID Control. PID control is one of the most common control algorithms used in industry because it is simple to understand and implement yet still provides robust performance. PID algorithms consists of three basic coefficients; proportional, integral and derivative which are varied or "tuned" to get the optimal response. 

To better understand PID control lets use the roll axis of the osprey drone as an example. As previously discussed, the flight controller calculates the drones attitude about the roll axis and compares the current rotation to the desired rotation set by the joystick. Let's say the desired roll is zero degrees but a gust of wind induces a roll of 30º. The error term (the difference between the current and desired rotation) is now 30º.

<p align="center">
<img src="/images/roll.jpg" width="300"/>
</p>

<p align="center">
<b>Drone rolling 30º resulting in an error term of 30.</b>
</p>

This error term is then passed into the PID controller. Inside the PID controller a proportional, integral, and derivative response to this error term are calculated separately and finally summed together. The sum of the proportional, integral, and derivative response to the error term is the output of the controller which is combined with the throttle value to calculate the new motor speeds. Notice the gain in front of eacch PID term. This gain is adjusted to increase or decrease the effect of the term on the overall controller output. The PID tuning process is necessary to find the optimal gains for the particular system. 

<p align="center">
<img src="/images/pid.jpg" width="400"/>
</p>

<p align="center">
<b>The structure of a PID controller showing the error term passed through the proportional, integral, and derivative paths.</b>
</p>

 In this case, because the drone is rolling to the left we would expect our PID controller to reduce the power to the motors on the right and increase the power to the motors on the left. How do the proportional, integral, and derivative term each respond to the error to affect the controller output?

##### Proportional term
As you might expect, the proportional term grows proportionally with the size of the error term. A larger error will result in a larger absolute contribution from the proportional term to the controller output. 

##### Integral term
The integral term is the integration of the error over time. With every execution of the loop, the current error is added to the sum of all past errors. The integral term often helps to overcome small errors because when the error is small in magnitude the proprotional term is also small and the contribution of the proportional term alone may not be enough to correct the error.

##### Derivative term
The derivative term takes into account the rate of change of the error term. The derivative term is sometimes called the damping term because its primary responsibility is to prevent the system from overcorrecting due to the contribution of the proportional term. The derivative term is constrained by its tendency to amplify sensor noise. If the derivative gain is too high the system will lose stability because the derivative term will erroneously correct for sensor noise. 

So, to recap, basic idea behind a PID controller is to 1. Read a sensor, 2. Compute the difference between the sensor reading and the desired setpoint, and 3. Compute the desired change in motor power by calculating a proportional, integral, and derivative response to the error. 

Closed loop system vs open loop system?


##### PID Tuning
A flawless flight controller still will not fly without reasonable gains. Testing the overall function of the drone and tuning the gains in a safe and controlled environment before attemtping to fly is the smart approach to drone development (I learned the hard way). To safely tune the gains I built a simple test rig shown below. The roll, pitch, and yaw axes must be tested and tuned individually. For each axis make sure the center of rotation on the test rig is directly through the center of gravity of the drone to best simulate flight.

<p align="center">
<img src="/images/dronePictures/testRig.jpg" width="300"/>
<img src="/images/graph.jpg" width="300"/>
</p>

<p align="center">
<b>Left: handmade roll and pitch test rig. Right: graph of PID output during roll test.</b>
</p>

The Ziegler Nicols Method
1. Balance weight, motor thrustn and motor torques along each axis so that the drone remains level in the absence of perturbations. 
2. Add the proportional term and tune the gain to achieve even, steady oscillations when perturbed. 
4. Add the derivative term and tune the gain to remove overshoot. 
5. Add the integral term and tune the gain to compensate for any steady state error.


### State Estimation 
The Osprey Flight Controller incorporates gyroscope and accelerometer data from the MPU6050 inertial measurement unit to achieve a 3 degrees of freedom attitude estimation about the roll, pitch, and yaw axes. 

<p align="center">
<img src="/images/mpu.jpg" width="600"/>
</p>

The MPU6050 is an I2C running on 3.3v from the 3.3v pin on the arduino. 

The accelerometer measures linear acceleration along the X, Y, and Z axes.

The angular position is calculated from each accelerometer linear acceleration reading using Euler angles. When the drone is upright and level, the vector of gravity points directly downward and trigonometry can be used to calculate the 

The gyroscope measures angular rate along the X, Y, and Z axes.

Every timestep the angular rate reading is integrated with respect to time producing an angular position estimate. This angular position estimate is then combined with the accelerometer based angular position estimate using a high pass (low pass?) filter.

### Next Steps:
Software improvements:
* Decrease flight controller refresh rate 
* Add altitude controller for hover
* Add translation controller to prevent wind drift 

Hardware improvements:
* Replace Arduino Uno with multiple Arduino Nanos
* Minimize length of all wires
* Design an outer shell to protect electrical components

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

Arduino Uno 

MPU6050

Servo control board

Power Distribution board 

ESCs

Motors 

FlySky I6x Transmitter and Reciever

Zeee 3s 1500 Mah Lithium Polymer battery

Air frame 

Propellors 

Battery charger 

Multimeter 


Image:
Part: name
Specs:
Include tiny pic and refresh rate of each part 


### Sources

1. https://uwaterloo.ca/applied-mathematics/future-undergraduates/what-you-can-learn-applied-mathematics/control-theory


