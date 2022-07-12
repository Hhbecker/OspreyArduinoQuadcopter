/*************************************************** 
Flight Controller for the Osprey Arduino Quadcopter

****************************************************/

/* 
 *  Important Remdiners
 *  -timestep initialized to zero meaning it's zero for the first call to getStateEstimation()
 *  -when reading pulse from arduino pins connected to reciever the policy is to set the reciever value to zero if the timeout is reached
 *  -currently no anti windup scheme or throttle minimum for i term
 */


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

int CH1 = 13; // roll (aileron)
int CH2 = 12; // pitch (elevator)
int CH3 = 11; // throttle
int CH4 = 10; // yaw (rudder)
int CH5 = 9; // abort switch
int CH6 = 8; // unused switch

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

int JOYSTICKMIN = -15;
int JOYSTICKMAX = 15;

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



/////////////////////////////////////////////////// SETUP FUNCTION 

void setup() {

    if(debug == true){
        Serial.begin(115200);
    }

    initializeRecieverPins();

    initializeServoBoard();

    initializeIMU();

    setAccelerometerOffset();

}

/////////////////////////////////////////////////// LOOP FUNCTION 


void loop() {

    // timer for gyroscope timestep
    // returns the number of milliseconds passed since the arduino board began running the current program
    timer = micros();

    getStateEstimation();
    checkAbortConditions();
    readThrottle();

    if(stabilizeMode == false){
        readRoll();
        readPitch();
        readYaw();
    }

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
