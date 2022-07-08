/*************************************************** 
Flight Controller for the Osprey Arduino Quadcopter

****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <MPU6050.h>
#include "functions.h"

bool debug = false; 
bool stabilizeMode = true;
bool abortSwitch = false;

float rollPgain = 0.04;
float rollIgain = 0.003; // 0.003 
float rollDgain = 0.06; // 0.06

float pitchPgain = 0.07;
float pitchIgain = 0.007; // 0.007
float pitchDgain = 0.06; // 0.07

float yawPgain = 0.02;
float yawIgain = 0.00;
float yawDgain = 0.03;

// CONTROLLER AND RECIEVER SETUP

int CH1 = 13;
int CH2 = 12;
int CH3 = 11;
int CH4 = 10;
int CH5 = 9;
int CH6 = 8;

//Floating point math is also much slower than integer math in performing calculations, 
//so should be avoided if, for example, a loop has to run at top speed for a critical 
//timing function. Programmers often go to some lengths to convert floating point 
//calculations to integer math to increase speed.

//C++ always truncates aka rounds down to the nearest integer equal to or less than the floating point value.

int ch1Value;
int ch2Value;
int ch3Value;
int ch4Value;
int ch5Value;
unsigned long timeout = 30000; // 1,000,000 = 1 second


int throttle;
int frontRight;
int frontLeft;
int backRight;
int backLeft;

// roll only
float rollValue = 0.0f;
float rollError;
float rollProportional;
float rollIntegral = 0.0f;
float rollDerivative;
float rollCorrection = 0;

float pitchValue = 0.0f;
float pitchError;
float pitchProportional;
float pitchIntegral = 0.0f;
float pitchDerivative;
float pitchCorrection;

float yawValue = 0.0f;
float yawError;
float yawProportional;
float yawDerivative;
float yawCorrection;

// SERVO BOARD SETUP
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int SERVOMIN = 215; // minimum pulse length count (out of 4096) the ESC will recognize
int SERVOMAX = 300; // maximum pulse length count (out of 4096) the ESC will recognize // FLIGHT = 400 ROLL TEST = 350

// MPU6050 SETUP
// Because of the mounting orientation of the chip on the drone: 
// PITCH = X axis
// ROLL = y axis

MPU6050 mpu;

// Timers
unsigned long timer = 0;
float timeStep = 0.0;

// Pitch, Roll and Yaw values
float gyroPitch;
float gyroRoll;
float gyroYaw;
float accelPitch;
float pitchOffset;
float accelRoll;
float rollOffset;
float pitch;
float roll;

Vector normGyro;
Vector normAccel;

/////////////////////////////////////////////////// CUSTOM FUNCTIONS /

    void initializeRecieverPins(){
        // CONTROLLER AND RECIEVER SETUP
        pinMode(CH1, INPUT);
        pinMode(CH2, INPUT);
        pinMode(CH3, INPUT);
        pinMode(CH4, INPUT);
        pinMode(CH5, INPUT);
        pinMode(CH6, INPUT);
    }

    void initializeServoBoard(){
        // SERVO BOARD SETUP
        // The internal oscillator (clock) for the PCA9685 chip is in a range between about 23-27MHz
        // Analog servos run at ~50 Hz updates, 
        pwm.begin();
        pwm.setOscillatorFrequency(27000000); // 27 MHz
        pwm.setPWMFreq(400);  // Analog servos run at ~50 Hz updates  
    }

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

    void setAccelerometerOffset(){
        // LOOP THIS TO GET A MORE ACCURATE OFFSET
        rollOffset = 0.0;
        pitchOffset = 0.0;
    
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
    
    void getStateEstimation(){
        // GYRO DATA
        // Read normalized values
        normGyro = mpu.readNormalizeGyro();
    
        // Calculate Pitch, Roll and Yaw (integrate filtered angle)
        gyroRoll = roll + (normGyro.YAxis * timeStep);    // switched gryoRoll for roll
        gyroPitch = pitch + (normGyro.XAxis * timeStep);  // switched gyroPitch for pitch
        gyroYaw = gyroYaw + (normGyro.ZAxis * timeStep);
    
        // ACCELEROMETER DATA
        // Read normalized values 
        normAccel = mpu.readNormalizeAccel();
    
        // Calculate Pitch & Roll
        accelRoll = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;

        accelPitch = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
    
        // COMPLIMENTARY FILTER
        pitch = (0.995*gyroPitch) + (0.005*accelPitch);
        roll = (0.995*gyroRoll) + (0.005*accelRoll);
        
    }

    
    int mapReceiver(int input){
        // constrain receiver value to range 1000-2000
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

    void killMotors(){
        
        pwm.setPWM(0, 0, SERVOMIN);
        pwm.setPWM(1, 0, SERVOMIN);
        pwm.setPWM(14, 0, SERVOMIN);
        pwm.setPWM(15, 0, SERVOMIN);

        while(true){
             
        }
    }

    void getRollCorrection(){
        // rightside roll -> positive roll degrees
        // leftside roll -> negative roll values 

        rollError = roll - rollValue;

        // Proportional Term
        rollProportional = rollPgain * rollError;

//        // Integral Term
//        if(throttle > 260){
//            rollIntegral = rollIntegral + (rollError * rollIgain);
//        }

        // Derivative Term
        rollDerivative = rollDgain * normGyro.YAxis;

        rollCorrection = rollProportional + rollDerivative + rollIntegral;
    }

    void getPitchCorrection(){

        pitchError = pitch - pitchValue;
        
        // Proportional Term
        pitchProportional = pitchPgain * pitchError;

//        // Integral Term
//        if(throttle > 260){
//            pitchIntegral = pitchIntegral + (pitchError * pitchIgain);
//        }

        // Derivative Term
        pitchDerivative = pitchDgain * normGyro.XAxis;
        
        pitchCorrection = pitchProportional + pitchDerivative + pitchIntegral;
    }

    void getYawCorrection(){

        yawError = gyroYaw - yawValue;

        // Proportional Term
        yawProportional = yawPgain * yawError;

        // Derivative Term
        yawDerivative = yawDgain * normGyro.ZAxis;
        
        yawCorrection = yawProportional + yawDerivative; 
    }

    void setFrontRight(){
        
        frontRight = round(throttle + rollCorrection - pitchCorrection + yawCorrection);
        
        frontRight = limitMotor(frontRight);
    
        pwm.setPWM(1, 0, frontRight);
    }

    void setFrontLeft(){

        frontLeft = round(throttle - rollCorrection - pitchCorrection - yawCorrection); 
        
        frontLeft = limitMotor(frontLeft);
        
        pwm.setPWM(15, 0, frontLeft);
    }

    void setBackRight(){

        backRight = round(throttle + rollCorrection + pitchCorrection - yawCorrection);

        backRight = limitMotor(backRight);

        pwm.setPWM(0, 0, backRight);
    }

    void setBackLeft(){

        int motorOffset = 2;

        backLeft = round(throttle - rollCorrection + pitchCorrection + yawCorrection - motorOffset);

        backLeft = limitMotor(backLeft);

        pwm.setPWM(14, 0, backLeft);
    }

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

/////////////////////////////////////////////////// SETUP FUNCTION /

void setup() {

    if(debug == true){
        Serial.begin(115200);
    }

    initializeRecieverPins();

    initializeServoBoard();

    initializeIMU();

    setAccelerometerOffset();

}

/////////////////////////////////////////////////// LOOP FUNCTION /


void loop() {

    // timer for gyroscope timestep
    // returns the number of milliseconds passed since the arduino board began running the current program
    timer = micros();

    getStateEstimation();
    checkAbortConditions();
    readThrottle();

    getRollCorrection();
    getPitchCorrection();
    getYawCorrection();
   
    setFrontRight();
    setFrontLeft();
    setBackRight();
    setBackLeft();
    
    
    if(debug == true){
        printData();
    }
    
    timeStep = (float) (micros() - timer)/1000000;

    
}
