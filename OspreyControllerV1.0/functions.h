#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <MPU6050.h>

// global variables defined in 

//extern bool debug = false; 
//extern bool stabilizeMode = true;
//extern bool abortSwitch = false;
//
//extern float rollPgain = 0.04;
//extern float rollIgain = 0.003; // 0.003 
//extern float rollDgain = 0.06; // 0.06
//
//extern float pitchPgain = 0.07;
//extern float pitchIgain = 0.007; // 0.007
//extern float pitchDgain = 0.06; // 0.07
//
//extern float yawPgain = 0.02;
//extern float yawIgain = 0.00;
//extern float yawDgain = 0.03;
//
//// CONTROLLER AND RECIEVER SETUP
//
//int CH1 = 13;
//int CH2 = 12;
//int CH3 = 11;
//int CH4 = 10;
//int CH5 = 9;
//int CH6 = 8;
//
//int ch1Value;
//int ch2Value;
//int ch3Value;
//int ch4Value;
//int ch5Value;
//unsigned long timeout = 30000; // 1,000,000 = 1 second
//
//
//int throttle;
//int frontRight;
//int frontLeft;
//int backRight;
//int backLeft;
//
//// roll only
//float rollValue = 0.0f;
//float rollError;
//float rollProportional;
//float rollIntegral = 0.0f;
//float rollDerivative;
//float rollCorrection = 0;
//
//float pitchValue = 0.0f;
//float pitchError;
//float pitchProportional;
//float pitchIntegral = 0.0f;
//float pitchDerivative;
//float pitchCorrection;
//
//float yawValue = 0.0f;
//float yawError;
//float yawProportional;
//float yawDerivative;
//float yawCorrection;
//
//// SERVO BOARD SETUP
//// called this way, it uses the default address 0x40
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
//
//int SERVOMIN = 215; // minimum pulse length count (out of 4096) the ESC will recognize
//int SERVOMAX = 300; // maximum pulse length count (out of 4096) the ESC will recognize // FLIGHT = 400 ROLL TEST = 350
//
//// MPU6050 SETUP
//// Because of the mounting orientation of the chip on the drone: 
//// PITCH = X axis
//// ROLL = y axis
//
//MPU6050 mpu;
//
//// Timers
//unsigned long timer = 0;
//float timeStep = 0.0;
//
//// Pitch, Roll and Yaw values
//float gyroPitch;
//float gyroRoll;
//float gyroYaw;
//float accelPitch;
//float pitchOffset;
//float accelRoll;
//float rollOffset;
//float pitch;
//float roll;
//
//Vector normGyro;
//Vector normAccel;






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
