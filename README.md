<h1 align="center">
The Osprey Drone Mark I with Osprey Flight Controller Version 1.0
</h1>

<p align="center">
<img src="/images/dronePictures/finished1.jpg" width="300"/>
</p>

<h5 align="center">
An Arduino based quadcopter build with personally developed flight control software. 
</h5>

### Features:
* Automated PID stabilization for roll pitch and yaw axes 
* Joystick control of roll pitch and yaw axes 
* Joystick throttle mapped directly to motor power
* Approximately 20Hz flight controller refresh rate

### Why Drones?
I see drones as the perfect engineering challenge because they require knowledge from a range of disciplines including physics, electrical engineering, computer science, and math. 
This drone project also provided me experience developing embedded software for a real-time application, designing and constructing circuits and electrical hardware, and implementing an algorithm based on control theory mathematics. Not only are drones a great engineering challenge but they also have innumerous real world applications including in disaster relief, law enforcement, agriculture, and war. 

### Circuit Diagram
<p align="center">

<img src="/images/fritzing.jpg" width="75%"/>
</p>

### The Basics of Quadcopter Flight
A drone has three axes of rotation: roll, pitch, and yaw. The primary goal of the Osprey Flight Controller V1.0 is to maintain stability in flight by continously adjusting the speed of each individual motor in order to abruptly and accurately correct any unwanted rotation about any of the three axes.

<p align="center">
<img src="/images/rotation.jpg" width="270"/>
</p>

<p align="center">
<b>Vizualization of the three axes of rotation. Y = roll, X = pitch, and Z = yaw.</b>
</p>

The onboard accelerometer and gyroscope are used to calculate the rotation of the drone along these three axes in real time. The actual rotation is fed into the control algorithm to calculate the new motor speeds necessary to correct for the current unwanted rotation. The flight controller algorithm runs in a simple loop making these corrections continously throughout the duration of the flight. The stability of the drone is ultimately a function of how quickly corrections are calculated and implemented by the motors. Each component on the drone plays a role in accomplishing this task.

### Code Structure 
The arduino runs a continuous loop with the same main functions called each time the loop executes. The diagram below shows the main loop functions in the order of their execution. This code is pulled directly from the main loop of the Osprey Flight Controller V1.0.

<p align="center">
<img src="/images/code.jpg" width="100%"/>
</p>

### The Control Algorithm
Control theory is a branch of applied mathematics that deals with the use of feedback to influence the behaviour of a system in order to achieve a desired goal. In the case of drone flight, the current rotation about each axis must be "fed-back" into the control algorithm and used to determine how much the motor speeds should change to achieve the desired rotation set by the joysticks.

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

#### Proportional term
As you might expect, the proportional term grows proportionally with the size of the error term. A larger error will result in a larger absolute contribution from the proportional term to the controller output. 

#### Integral term
The integral term is the integration of the error over time. With every execution of the loop, the current error is added to the sum of all past errors. The integral term often helps to overcome small errors because when the error is small in magnitude the proprotional term is also small and the contribution of the proportional term alone may not be enough to correct the error.

#### Derivative term
The derivative term takes into account the rate of change of the error term. The derivative term is sometimes called the damping term because its primary responsibility is to prevent the system from overcorrecting due to the contribution of the proportional term. The derivative term is constrained by its tendency to amplify sensor noise. If the derivative gain is too high the system will lose stability because the derivative term will erroneously correct for sensor noise. 

So, to recap, basic idea behind a PID controller is to 1. Read a sensor, 2. Compute the difference between the sensor reading and the desired setpoint, and 3. Compute the desired change in motor power by calculating a proportional, integral, and derivative response to the error. 

#### PID Tuning
A flawless flight controller still will not fly without reasonable gains. Testing the overall function of the drone and tuning the gains in a safe and controlled environment before attemtping to fly is the smart approach to drone development (I learned the hard way). To safely tune the gains I built a simple test rig shown below. The roll, pitch, and yaw axes must be tested and tuned individually. For each axis, the center of rotation on the test rig should be located directly through the center of gravity of the drone to best simulate flight. Balance weight, motor thrusts, and motor torques along each axis so the drone remains level on the test rig in the absence of perturbations before testing the PID controller.

<br/>

<p align="center">
<img src="/images/dronePictures/testRig.jpg" height="350"/>
<img src="/images/graph.jpg" height="350"/>
</p>

<p align="center">
<b>Left: handmade roll and pitch test rig. Right: graph of PID output during roll test.</b>
</p>


#### The Ziegler Nichols Method
1. Set integral and derivative gains to zero
2. Tune the proportional gain to achieve even, steady oscillations when perturbed. 
4. Add the derivative term and tune the gain to remove overshoot. 
5. Add the integral term and tune the gain to compensate for any steady state error.


### State Estimation 
The Osprey Flight Controller incorporates gyroscope and accelerometer data from the MPU6050 inertial measurement unit to achieve a 3 degrees of freedom attitude estimation about the roll, pitch, and yaw axes. The MPU6050 is a low budget, low power 6-axis microelectromechanical inertial measuremnt unit that can easily interface with Arduino using the I2C protocol. 

<p align="center">
<img src="/images/mpu.jpg" width="200"/>
</p>
<p align="center">
<b>An MPU6050 inertial measurement unit.</b>
</p>

The MPU6050 accelerometer measures linear acceleration along the X, Y, and Z axes. The angular position is calculated from each accelerometer linear acceleration reading using Euler angle based trigonometry. Korneliusz Jarzębski's MPU6050 library was used to calculate the accelerometer and gyroscope based angular position calculations and these calculations were combined using a low pass filter for the accelerometer based state estimation and a high pass filter for the gyroscope based state estimation. 

### Component Timing
The flight controller relies on several different hardware components as well as software to do its job. It's important to understand the time it takes to complete each part of the loop so that optimization efforts can focus on the biggest bottlenecks.

The timestep is often kept constant when integrating sensor data with respect to time. I chose to vary the time step to avoid using the delay() function which is a blocking function and slowed the overall loop time too much. The best solution would be to handle integration of the sensor data on a separate microcontroller to allow for a constant timestep without slowing down the entire flight controller. 

The default operating frequency for Arduino Uno R3 is 16 MHz. The average timestep (total loop time) of the Osprey Flight Controlller V1.0 is approximately 20Hz in Stabilize Mode. This is unfortunately far too slow to allow for the stability required to film or maneuver. Modern F4 flight controllers have loop times of 32KHz which is three orders of magnitude faster than the Osprey Flight Controller V1.0. 

A slower flight controller loop time negatively affects stability because it decreases the accuracy of the state estimation and also does not give the motors as much time to adjust their speed. 

the internal clock of the servo control board can be set to operate at a range of refresh rates from ______ to ______

The SimonK ESCs are capabale of a 400Hz refresh rate and the motors are capable of responding to this refresh rate.

How often do reciever pulses come in?
What is the min and max pulse length and timeout time of the reciever and pulseIn() function?

What is the MPU6050 sampling frequency
The MPU6050 uses an interrupt 
The MPU6050 samples onyl once a loop but you want it to sample as fast as possible and just use the current estimate in the control loop as fast as the loop allows. Sample the gyro as fast as the hardware allows and execute the control loop every nth gyro sampling.

https://www.bestonlinedrones.com/should-your-miniquad-have-the-newest-flight-controller/

### Construction Process
<p align="center">
<img src="/images/dronePictures/build1.jpg" height="280"/>
<img src="/images/dronePictures/build2.jpg" height="280"/>
<img src="/images/dronePictures/build3.jpg" height="280"/>
<img src="/images/dronePictures/finished4.jpg" height="280"/>
</p>

</br>

<h3 align="center">
The Osprey Mark I in Action 
</h3>

<p align="center">
<img src="/images/flightTests/test2.gif" width="370"/>
</p>


### Parts List
* Elegoo Uno R3 - x1
* HiLetgo MPU6050 - x1
* HiLetgo Servo Control Board PCA9685 - x1
* Matek Power Distribution Board 5V and 12V - x1
* Hobbypower SimonK 30A ESC - x4
* Readytosky 2300KV brushless DC motor - x4
* FlySky I6x Transmitter and Reciever - x1
* Zeee 11.1V 120C 1500mAh 3s Lithium Polymer Battery - x1
* Usmile 250mm carbon fiber FPV quadcopter frame - x1
* GEMFAN 5 inch propellor 3 blades - x1
* AstroAI Digital Multimeter - x1
* HTRC 2S-3S LiPo Balance Battery Charger - x1

### Next Steps
Software improvements:
* Decrease flight controller refresh rate 
* Add altitude controller for hover
* Add translation controller to prevent wind drift 
* Represent state estimation using Quaternions instead of Euler Angles

Hardware improvements:
* Replace Arduino Uno with multiple Arduino Nanos
* Minimize length of all wires
* Design an outer shell to protect electrical components

<h4 align="center">
Thank you to Mom for the support, Dad for the support and the tools, and Conor for the inspo and advice!
</h4>


