# The Osprey Drone Mark I with Osprey Flight Controller Version 1.0
A from scratch quadcopter build with an Arduino programmed as a flight controller. 

Features:
* automated PID stabilization for roll pitch and yaw axes 
* joystick control of roll pitch and yaw axes 
* joystick throttle mapped directly to motor speed
* arduino uno as flight controller with roughly 0.05 second loop time

<p align="center">
<img src="/images/flightTests/test2.gif" width="500"/>
</p>




<p align="center">
<img src="/images/fritzing.jpg" width="50%"/>
</p>



### Code explanation 
The arduino runs a continuous loop with the same main functions called each time the loop executes.

Main loop functions: 
Firstly, we need to keep track of the time the loop takes to run in order to integrate the gyroscope data with respect to time. To keep track of the time the loop takes to run we call the micros() function at the beginning and end of the loop 

#### 1. Keep track of the time the loop takes
```
void loop() {

    // The micros() function returns the number of microseconds passed since the arduino  began running the current program.

    timer = micros();

    // All functions go here //
    
    // Call the micros function again and subtract the new microsecond value from the value returned at the beginning of the program.

    timeStep = (float) (micros() - timer)/1000000;
}
```

#### 2. Calculate the drone's position along the X, Y, and Z axes using sensor data
```
getStateEstimation();
```

#### 3. Check abort conditions and turn off motors if any abort conditions are met
```
checkAbortConditions();
```

#### 4. Read the throttle value sent from joystick to onboard reciever
```
readThrottle();
```

#### 5. Read roll pitch and yaw values from joysticks when stabilize mode is off
```
    if(stabilizeMode == false){
        readRoll();
        readPitch();
        readYaw();
    }
```

#### 6. Calculate changes to motor speeds using PID control algorithm to achieve desired roll pitch and yaw angles 
```
    getRollCorrection();
    getPitchCorrection();
    getYawCorrection();
```   
#### 7. Send new motor speeds to servo control board
```
    setFrontRight();
    setFrontLeft();
    setBackRight();
    setBackLeft();
    
```

#### Why I like drones 

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
### Parts list with explanation of each part


