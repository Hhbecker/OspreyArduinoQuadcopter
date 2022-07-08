#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <MPU6050.h>

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
