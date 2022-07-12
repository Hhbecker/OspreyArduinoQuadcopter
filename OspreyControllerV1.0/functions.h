/*************************************************** 
Flight controller header file
July 2022
Henry Becker
****************************************************/

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <MPU6050.h>

// global variables defined in 

extern bool debug; 
extern bool stabilizeMode;
extern bool abortSwitch;

extern float rollPgain;
extern float rollIgain; 
extern float rollDgain;

extern float pitchPgain;
extern float pitchIgain;
extern float pitchDgain;

extern float yawPgain;
extern float yawIgain;
extern float yawDgain;

// CONTROLLER AND RECIEVER SETUP

extern int CH1;
extern int CH2;
extern int CH3;
extern int CH4;
extern int CH5;
extern int CH6;

extern int ch1Value;
extern int ch2Value;
extern int ch3Value;
extern int ch4Value;
extern int ch5Value;
extern unsigned long timeout;


extern int throttle;
extern int frontRight;
extern int frontLeft;
extern int backRight;
extern int backLeft;

// roll only
extern float rollValue;
extern float rollError;
extern float rollProportional;
extern float rollIntegral;
extern float rollDerivative;
extern float rollCorrection;

extern float pitchValue;
extern float pitchError;
extern float pitchProportional;
extern float pitchIntegral;
extern float pitchDerivative;
extern float pitchCorrection;

extern float yawValue;
extern float yawError;
extern float yawProportional;
extern float yawDerivative;
extern float yawCorrection;

// SERVO BOARD SETUP
// called this way, it uses the default address 0x40
extern Adafruit_PWMServoDriver pwm;

extern int SERVOMIN; // minimum pulse length count (out of 4096) the ESC will recognize
extern int SERVOMAX; // maximum pulse length count (out of 4096) the ESC will recognize // FLIGHT = 400 ROLL TEST = 350

extern int JOYSTICKMIN;
extern int JOYSTICKMAX;

// MPU6050 SETUP
// Because of the mounting orientation of the chip on the drone: 
// PITCH = X axis
// ROLL = y axis

extern MPU6050 mpu;

// Timers
extern unsigned long timer;
extern float timeStep;

// Pitch, Roll and Yaw values
extern float gyroPitch;
extern float gyroRoll;
extern float gyroYaw;
extern float accelPitch;
extern float pitchOffset;
extern float accelRoll;
extern float rollOffset;
extern float pitch;
extern float roll;

extern Vector normGyro;
extern Vector normAccel;


// Setup functions 
void initializeRecieverPins();
void initializeServoBoard();
void initializeServoBoard();
void initializeIMU();
void setAccelerometerOffset();

// Loop functions
void getStateEstimation();
int mapReceiver(int input);
void readThrottle();
void readRoll();
void readPitch();
void readYaw();
void checkAbortConditions();
void killMotors();
void getRollCorrection();
void getPitchCorrection();
void getYawCorrection();
void setFrontRight();
void setFrontLeft();
void setBackRight();
void setBackLeft();
int limitMotor(int motor);
void printData();


#endif
