/*************************************************** 
Flight Controller for the Osprey Arduino Quadcopter


****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050.h>

//Floating point math is also much slower than integer math in performing calculations, 
//so should be avoided if, for example, a loop has to run at top speed for a critical 
//timing function. Programmers often go to some lengths to convert floating point 
//calculations to integer math to increase speed.

//C++ always truncates aka rounds down to the nearest integer equal to or less than the floating point value.

bool stabilizeMode = true;
int throttle;
int frontRight;
int frontLeft;
int backRight;
int backLeft;

// roll only
int proportional;
int integral;
int derivative;

float previousRoll;
int rollCorrection;
float rollErrorIntegrated;
int pitchCorrection;
float pitchErrorIntegrated;
int yawCorrection;
float yawErrorIntegrated;

float rollPgain = 1;
float rollIgain;
float rollDgain = 0.1;

float pitchPgain;
float pitchIgain;
float pitchDgain;

float yawPgain;
float yawIgain;
float yawDgain;


// MPU6050 SETUP
// Because of the mounting orientation of the chip on the drone: 
// PITCH = X axis
// ROLL = y axis

MPU6050 mpu;

// Timers
unsigned long millisecondsPassed = 0;
float timeStep = 0.05f;

// Pitch, Roll and Yaw values
float gyroPitch = 0.0;
float gyroRoll = 0.0;
float gyroYaw = 0;
float accelPitch;
float pitchOffset;
float accelRoll;
float rollOffset;
float pitch;
float roll;

Vector normGyro;
Vector normAccel;

/////////////////////////////////////////////////// SETUP FUNCTION /

void setup() {

    // set serial monitor baud rate
    Serial.begin(115200);

    // MPU6050 SETUP
    while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
    {
        Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
        delay(500);
    }

    // Calibrate gyroscope. Calibration must be at rest.
    mpu.calibrateGyro();
    
    // Set threshold sensivty. Default 3.
    mpu.setThreshold(3);

    // LOOP THIS TO GET A MORE ACCURATE OFFSET
    normAccel = mpu.readNormalizeAccel();

    rollOffset = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
    pitchOffset = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;

//    Serial.println(" Roll offset = ");
//    Serial.println(rollOffset);
//    Serial.println(" Pitch offset = ");
//    Serial.println(pitchOffset);

    int previousRoll = 0.0;

}

/////////////////////////////////////////////////// LOOP FUNCTION /


void loop() {

    ///////// 3 DOF state estimation //////////

    // timer for gyroscope timestep
    // returns the number of milliseconds passed since the arduino board began running the current program
    millisecondsPassed = millis();

    // GYRO DATA
    // Read normalized values
    normGyro = mpu.readNormalizeGyro();

    // Calculate Pitch, Roll and Yaw
    gyroRoll = gyroRoll + normGyro.YAxis * timeStep;    // switched gryoRoll for roll
    gyroPitch = gyroPitch + normGyro.XAxis * timeStep;  // switched gyroPitch for pitch
    gyroYaw = gyroYaw + normGyro.ZAxis * timeStep;

    // ACCELEROMETER DATA
    // Read normalized values 
    normAccel = mpu.readNormalizeAccel();

    // Calculate Pitch & Roll
    accelRoll = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
    accelRoll = accelRoll - rollOffset;
    accelPitch = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
    accelPitch = accelPitch - pitchOffset;

//    Serial.print(" gyroPitch = ");
//    Serial.print(gyroPitch);
//    Serial.print(" gyroRoll = ");
//    Serial.print(gyroRoll);
//    Serial.print(" accelRoll = ");
//    Serial.print(accelRoll);
//    Serial.print(" accelPitch = ");
//    Serial.print(accelPitch);


    // COMPLIMENTARY FILTER

    pitch = (0.98*gyroPitch) + (0.02*accelPitch);
    roll = (0.98*gyroRoll) + (0.02*accelRoll);


//    // Output
    Serial.print(" Pitch = ");
    Serial.print(pitch);
    Serial.print(" Roll = ");
    Serial.print(roll);
    Serial.print(" Yaw = ");
    Serial.print(gyroYaw);
    Serial.println();

    ///////// state estimation complete ///////////

    // PID COMMANDS ////////////////////////////////////////////////
 // if (stabilize mode) {      
        
    // ROLL
    // rightside roll -> positive roll degrees
    // leftside roll -> negative roll values 

    // Proportional Term
    proportional = rollPgain * roll;

    // Integral Term
    integral = integral + (roll * rollIgain);

    if(integral > 10){
        integral = 10;
    }

    // Derivative Term
    derivative = rollDgain * (roll - previousRoll)/timeStep; 
    if(derivative > 40){
        derivative = 40;
    }
    if(derivative < -40){
        derivative = -40;
    }

    previousRoll = roll;

    rollCorrection = proportional + integral + derivative;
   
    Serial.print( " proportional = ");
    Serial.print(proportional);
    Serial.print( " integral = ");
    Serial.print(integral);
    Serial.print( " derivative = ");
    Serial.print(derivative);
    Serial.print( " rollCorrection = ");
    Serial.print(rollCorrection);


    // SATURATION POSSIBLE 
    frontRight = throttle + rollCorrection; // - pitchCorrection + yawCorrection;
    
    frontLeft = throttle - rollCorrection; // - pitchCorrection - yawCorrection;    
    
    backRight = throttle + rollCorrection; // + pitchCorrection - yawCorrection;          
    
    backLeft = throttle - rollCorrection; // + pitchCorrection + yawCorrection;             


//        Serial.print( " throttle = ");
//        Serial.print(throttle);
//        Serial.print( " rollError = ");
//        Serial.print(rollError);
//        Serial.print( " frontRight = ");
//        Serial.print(frontRight);
//        Serial.print( " frontLeft = ");
//        Serial.print(frontLeft);


    // WAIT FULL TIME STEP BEFORE CONTINUING FOR SAKE OF GYRO READINGS

    // if this number is negative the timeStep is too short
    Serial.print(" Time to delay in milliseconds = ");
    Serial.print((timeStep*1000) - (millis() - millisecondsPassed));
    

    // Wait to full timeStep period

    // millis() - millisecondsPassed = the time in milliseconds spent running the loop 
    // delay the timestep - time already spent running this script
    // delay command takes an input in milliseconds and delays for that amount of milliseconds
    delay((timeStep*1000) - (millis() - millisecondsPassed));
    
  

    Serial.println();
    Serial.println();
  
}
