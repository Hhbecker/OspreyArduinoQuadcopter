/*************************************************** 
Flight controller functions
July 2022
Henry Becker
****************************************************/



#include "functions.h"

/////////////////////////////////////////////////// CUSTOM FUNCTIONS /


    // Establish arduino pins to recieve signals from reciever
    void initializeRecieverPins(){
        // CONTROLLER AND RECIEVER SETUP
        pinMode(CH1, INPUT); // roll (aileron)
        pinMode(CH2, INPUT); // pitch (elevator)
        pinMode(CH3, INPUT); // throttle
        pinMode(CH4, INPUT); // yaw (rudder)
        pinMode(CH5, INPUT); // abort switch
        pinMode(CH6, INPUT); // unused switch
    }


    // Set internal clock of servo board for pwm communication with SimonK hobbypower ESCs
    void initializeServoBoard(){
        // SERVO BOARD SETUP
        // The internal oscillator (clock) for the PCA9685 chip is in a range between about 23-27MHz 
        pwm.begin();
        pwm.setOscillatorFrequency(27000000); // 27 MHz
        pwm.setPWMFreq(400);  // Analog servoes run at ~50 Hz updates but we aren't communicating directly with the motor we're communicating with an esc that operates at max of 400hz 
    }


    // Initialize and calibrate mpu6050 sensor
    void initializeIMU(){
        // MPU6050 SETUP
        while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
        {
            if(debug == true){
                Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
            }
            
            delay(100);
        }
    
        // Calibrate gyroscope. Calibration must be at rest.
        mpu.calibrateGyro();
        
        // Set threshold sensivty. Default 3.
        mpu.setThreshold(3);
    }


    // Use an average of 1000 accelerometer readings to set initial gyroscope reading
    void setAccelerometerOffset(){
        // The acceleromter is used to calculate angular position using trigonometry based on the known vector of gravity with an acceleration of 1g downward
        // The position calculated using the accelerometer is then given as the starting point for the gyrsocope
        
        rollOffset = 0.0;
        pitchOffset = 0.0;

        // average 1000 accelerometer readings for a more accurate offset
        for(int i=0; i<1000; i++){
            
            normAccel = mpu.readNormalizeAccel();
            rollOffset = rollOffset + (-(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI);
            pitchOffset = pitchOffset + ( (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI);
        }

        rollOffset = rollOffset/1000;
        pitchOffset = pitchOffset/1000;

        roll = rollOffset;
        pitch = pitchOffset;
        
    }


    // Convert MPU6050 gyroscope and accelerometer readings into accurate attitude estimation 
    void getStateEstimation(){
        // GYRO DATA
        // Read normalized values
        normGyro = mpu.readNormalizeGyro();
    
        // Iintegrate gyroscope data
        gyroRoll = roll + (normGyro.YAxis * timeStep);    // switched gryoRoll for roll
        gyroPitch = pitch + (normGyro.XAxis * timeStep);  // switched gyroPitch for pitch
        gyroYaw = gyroYaw + (normGyro.ZAxis * timeStep);
    
        // ACCELEROMETER DATA
        // Read normalized values 
        normAccel = mpu.readNormalizeAccel();
    
        // Calculate Roll and Pitch from accelerometer readings using Euler angles
        accelRoll = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;

        accelPitch = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
    
        // Sensor fusion using COMPLIMENTARY FILTER
        pitch = (0.995*gyroPitch) + (0.005*accelPitch);
        roll = (0.995*gyroRoll) + (0.005*accelRoll);
        
    }


    // Constrain receiver value to range 1000-2000 (sometiems reciever will read values slightly above or below 1000-2000)
    int mapReceiver(int input){
        
        if(input > 2000){
            return 2000;
        }
        else if(input < 1000){
            return 1000;
        }
        else {
            return input;
        }
    }

    // Map thorttle joystick value to be compatible with the SERVOMIN and SERVOMAX values used in setPWM function
    void readThrottle(){
        
        // read pulse from receiver channel 3 (throttle stick) and map it to range 1000-2000
        int pulse = pulseIn(CH3, HIGH, timeout);

        if(pulse == 0){
            return;
        }
        
        throttle = mapReceiver(pulse);
        
        // map receiver input value range of 0-1023 to a value range of 0-180 which the ESC understands 
        throttle = map(throttle, 1000, 2000, SERVOMIN, SERVOMAX);
        
    }

    // Read desired roll angle from joystick
    void readRoll(){
        
        // read pulse from receiver channel 3 (throttle stick) and map it to range 1000-2000
        int pulse = pulseIn(CH1, HIGH, timeout);

        if(pulse == 0){
            return;
        }
        
        rollValue = mapReceiver(pulse);
        
        // map receiver input value range of 0-1023 to a value range of 0-180 which the ESC understands 
        rollValue = map(rollValue, 1000, 2000, JOYSTICKMIN, JOYSTICKMAX);
        
    }


    // Read desired pitch angle from joystick
    void readPitch(){
        
        // read pulse from receiver channel 3 (throttle stick) and map it to range 1000-2000
        int pulse = pulseIn(CH2, HIGH, timeout);

        if(pulse == 0){
            return;
        }
        
        pitchValue = mapReceiver(pulse);
        
        // map receiver input value range of 0-1023 to a value range of 0-180 which the ESC understands 
        pitchValue = map(pitchValue, 1000, 2000, JOYSTICKMIN, JOYSTICKMAX);
        
    }


    // Read desired yaw angle from joystick
    void readYaw(){
        
        // read pulse from receiver channel 3 (throttle stick) and map it to range 1000-2000
        int pulse = pulseIn(CH1, HIGH, timeout);

        if(pulse == 0){
            return;
        }
        
        yawValue = mapReceiver(pulse);
        
        // map receiver input value range of 0-1023 to a value range of 0-180 which the ESC understands 
        yawValue = map(yawValue, 1000, 2000, JOYSTICKMIN, JOYSTICKMAX);
        
    }

    // Turn off motors if abort conditions are met
    void checkAbortConditions(){

        // if pitch or roll exceeds 80º
        if(abs(pitch) > 80 || abs(roll) > 80){
            killMotors();
        }
        
        // Abort switch on RC controller
        if(pulseIn(CH5, HIGH, timeout) > 1500){
            killMotors();
        }
        
    }


    // Turn off motors
    void killMotors(){
        
        pwm.setPWM(0, 0, SERVOMIN);
        pwm.setPWM(1, 0, SERVOMIN);
        pwm.setPWM(14, 0, SERVOMIN);
        pwm.setPWM(15, 0, SERVOMIN);

        while(true){
             
        }
    }


    // Calculate roll PID value
    void getRollCorrection(){
        // rightside roll -> positive roll degrees
        // leftside roll -> negative roll values 

        rollError = roll - rollValue;

        // Proportional Term
        rollProportional = rollPgain * rollError;

        // Derivative Term
        rollDerivative = rollDgain * normGyro.YAxis;

        rollCorrection = rollProportional + rollDerivative + rollIntegral;
    }


    // Calculate pitch PID value
    void getPitchCorrection(){

        pitchError = pitch - pitchValue;
        
        // Proportional Term
        pitchProportional = pitchPgain * pitchError;

        // Derivative Term
        pitchDerivative = pitchDgain * normGyro.XAxis;
        
        pitchCorrection = pitchProportional + pitchDerivative + pitchIntegral;
    }


    // Calculate yaw PID value
    void getYawCorrection(){

        yawError = gyroYaw - yawValue;

        // Proportional Term
        yawProportional = yawPgain * yawError;

        // Derivative Term
        yawDerivative = yawDgain * normGyro.ZAxis;
        
        yawCorrection = yawProportional + yawDerivative; 
    }


    // Set new front right motor speed
    void setFrontRight(){
        
        frontRight = round(throttle + rollCorrection - pitchCorrection + yawCorrection);
        
        frontRight = limitMotor(frontRight);
    
        pwm.setPWM(1, 0, frontRight);
    }

    // Set new front left motor speed
    void setFrontLeft(){

        frontLeft = round(throttle - rollCorrection - pitchCorrection - yawCorrection); 
        
        frontLeft = limitMotor(frontLeft);
        
        pwm.setPWM(15, 0, frontLeft);
    }


    // Set new back right motor speed
    void setBackRight(){

        backRight = round(throttle + rollCorrection + pitchCorrection - yawCorrection);

        backRight = limitMotor(backRight);

        pwm.setPWM(0, 0, backRight);
    }

    // Set new back left motor speed
    void setBackLeft(){

        int motorOffset = 2;

        backLeft = round(throttle - rollCorrection + pitchCorrection + yawCorrection - motorOffset);

        backLeft = limitMotor(backLeft);

        pwm.setPWM(14, 0, backLeft);
    }


    // Constrain motor speeds to the maximum and minimum possible values recognized by the motor compatible with the setPWM function
    int limitMotor(int motor){
        
        if(motor > SERVOMAX){
            motor = SERVOMAX; 
        }
        
        if(motor < SERVOMIN){
            motor = SERVOMIN; 
        }

        return motor;
    }

    void printData(){
        Serial.print(" Roll = ");
        Serial.print(roll);
        Serial.print(" Pitch = ");
        Serial.print(pitch);
        Serial.print(" Yaw = ");
        Serial.print(gyroYaw);

        Serial.print( " Throttle = ");
        Serial.print(throttle);

        // see wether limits on I and D term are being used
        Serial.print(" Roll Integral = ");
        Serial.print(rollIntegral);
        Serial.print(" Roll Derivative = ");
        Serial.print(rollDerivative);
        
        Serial.print(" Pitch Integral = ");
        Serial.print(pitchIntegral);
        Serial.print(" Pitch Derivative = ");
        Serial.print(pitchDerivative);

        Serial.print(" Last timeStep = ");
        Serial.print(timeStep);
        Serial.print(" seconds");
        Serial.println();
    }
