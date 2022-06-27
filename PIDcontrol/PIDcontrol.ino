/*************************************************** 
Flight Controller for the Osprey Arduino Quadcopter


****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050.h>

// CONTROLLER AND RECIEVER SETUP

#define CH1 13
#define CH2 12
#define CH3 11
#define CH4 10
#define CH5 9
#define CH6 8


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
bool abortSwitch = false;
bool stabilizeMode = true;
int throttle;
int frontRight;
int frontLeft;
int backRight;
int backLeft;

// roll only
float rollProportional;
float rollIntegral = 0.0f;
float rollDerivative;
float previousRoll;
int rollCorrection;


float pitchProportional;
float pitchIntegral = 0.0f;
float pitchDerivative;
float previousPitch;
int pitchCorrection;



// got better and better performance by continually dropping D gain, I think D gain could be lower and I might try raising P gain back up

float rollPgain = 0.14; // 0.18
float rollIgain = 0.06; 
float rollDgain = 0.055; // 0.055

float pitchPgain;
float pitchIgain;
float pitchDgain;

float yawPgain;
float yawIgain;
float yawDgain;


// SERVO BOARD SETUP
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  210 // minimum pulse length count (out of 4096) the ESC will recognize
#define SERVOMAX  340 // maximum pulse length count (out of 4096) the ESC will recognize // FLIGHT = 400 ROLL TEST = 350

// MPU6050 SETUP
// Because of the mounting orientation of the chip on the drone: 
// PITCH = X axis
// ROLL = y axis

MPU6050 mpu;

// Timers
unsigned long millisecondsPassed = 0;
float timeStep = 0.06f;
float tDelay;

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

    // CONTROLLER AND RECIEVER SETUP
    pinMode(CH1, INPUT);
    pinMode(CH2, INPUT);
    pinMode(CH3, INPUT);
    pinMode(CH4, INPUT);
    pinMode(CH5, INPUT);
    pinMode(CH6, INPUT);

    // SERVO BOARD SETUP
    // The internal oscillator (clock) for the PCA9685 chip is in a range between about 23-27MHz
    // Analog servos run at ~50 Hz updates, 
 
    pwm.begin();
    pwm.setOscillatorFrequency(27000000); // 27 MHz
    pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
        
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

    float previousRoll = 0.0;
    float previousPitch = 0.0;

}

/////////////////////////////////////////////////// CUSTOM FUNCTIONS /

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

    if(abs(pitch) > 80 || abs(roll) > 80){
        abortSwitch = true;
    }


    ///////// state estimation complete ///////////

    // THROTTLE
    // read pulse from receiver channel 3 (throttle stick) and map it to range 1000-2000
    ch3Value = mapReceiver(pulseIn(CH3, HIGH));
    
    // map receiver input value range of 0-1023 to a value range of 0-180 which the ESC understands 
    throttle = map(ch3Value, 1000, 2000, SERVOMIN, SERVOMAX);

//    Serial.print( " abort value = ");
//    Serial.print(pulseIn(CH5, HIGH));
    
    // ABORT SWITCH - turn all motors off
    if(pulseIn(CH5, HIGH) > 1500){
        abortSwitch = true;
    }

    Serial.print(abortSwitch);
    Serial.print(" ");
    
    if(abortSwitch == true){
        pwm.setPWM(0, 0, SERVOMIN);
        pwm.setPWM(3, 0, SERVOMIN);
        pwm.setPWM(4, 0, SERVOMIN);
        pwm.setPWM(7, 0, SERVOMIN);
    } 
    // PID COMMANDS ////////////////////////////////////////////////
    else { // if (stabilize mode) {      
        
        // ROLL /////////////////////////////////////////////////////////////////////////
        // rightside roll -> positive roll degrees
        // leftside roll -> negative roll values 

    

        // Proportional Term
        rollProportional = rollPgain * roll;

        // Integral Term
        if(throttle > 310){
            rollIntegral = rollIntegral + (roll * rollIgain);
        }

        if(rollIntegral > 10){
            rollIntegral = 10;
        }
        if(rollIntegral < -10){
            rollIntegral = -10;
        }

        // Derivative Term
        rollDerivative = (rollDgain * (roll - previousRoll)/timeStep); 
        // derivative = rollDgain * normGyro.YAxis;

        //derivative = rollDgain * ((previousAngularRate + normGyro.YAxis)/2); 
        //previousAngularRate = normGyro.YAxis; 
        //prevPreviousAngularRate = previousAngularRate;

        
        if(rollDerivative > 10){
            rollDerivative = 10;
        }
        if(rollDerivative < -10){
            rollDerivative = -10;
        }

        previousRoll = roll;

        rollCorrection = rollProportional + rollDerivative + rollIntegral;

        ///////////// PITCH //////////////////////////////////////////////////////////////////


        // Proportional Term
        pitchProportional = pitchPgain * pitch;

        // Integral Term
        if(throttle > 310){
            pitchIntegral = pitchIntegral + (pitch * pitchIgain);
        }

        if(pitchIntegral > 10){
            pitchIntegral = 10;
        }
        if(pitchIntegral < -10){
            pitchIntegral = -10;
        }

        // Derivative Term
        pitchDerivative = (pitchDgain * (pitch - previousPitch)/timeStep); 
        // derivative = rollDgain * normGyro.YAxis;

        //derivative = rollDgain * ((previousAngularRate + normGyro.YAxis)/2); 
        //previousAngularRate = normGyro.YAxis; 
        //prevPreviousAngularRate = previousAngularRate;

        
        if(pitchDerivative > 10){
            pitchDerivative = 10;
        }
        if(pitchDerivative < -10){
            pitchDerivative = -10;
        }

        previousPitch = pitch;

        pitchCorrection = pitchProportional + pitchDerivative + pitchIntegral;






        ////////////////////////////////////


        frontRight = throttle + rollCorrection - pitchCorrection; // - pitchCorrection + yawCorrection;
        if(frontRight > SERVOMAX) { frontRight = SERVOMAX;}
        if(frontRight < SERVOMIN) { frontRight = SERVOMIN;}  
        
        frontLeft = throttle - rollCorrection - pitchCorrection;// - pitchCorrection - yawCorrection;    
        if(frontLeft > SERVOMAX) { frontLeft = SERVOMAX; }
        if(frontLeft < SERVOMIN) { frontLeft = SERVOMIN; }
        
        backRight = throttle + rollCorrection + pitchCorrection; // + pitchCorrection - yawCorrection;          
        if(backRight > SERVOMAX) { backRight = SERVOMAX; }
        if(backRight < SERVOMIN) { backRight = SERVOMIN; }
        
        backLeft = throttle - rollCorrection + pitchCorrection; // + pitchCorrection + yawCorrection;             
        if(backLeft > SERVOMAX) { backLeft = SERVOMAX; }
        if(backLeft < SERVOMIN) { backLeft = SERVOMIN; }

    
        pwm.setPWM(0, 0, frontRight);
        pwm.setPWM(3, 0, frontLeft);
        pwm.setPWM(4, 0, backRight);
        pwm.setPWM(7, 0, backLeft);
    
    
        // WAIT FULL TIME STEP BEFORE CONTINUING FOR SAKE OF GYRO READINGS

        tDelay = (timeStep*1000) - (millis() - millisecondsPassed);

                    // Output
////      Serial.print(" Roll = ");
//        Serial.print(roll);
////      Serial.print(" Pitch = ");
//        Serial.print(" ");
//        Serial.print(pitch);
////      Serial.print(" Yaw = ");
//        Serial.print(" ");
//        Serial.print(gyroYaw);
////      Serial.print( " proportional = ");
//        Serial.print(" ");
//        Serial.print(proportional);
////      Serial.print( " integral = ");
//        Serial.print(" ");
//        Serial.print(integral);
////      Serial.print( " derivative = ");
//        Serial.print(" ");
//        Serial.print(derivative);
////      Serial.print( " rollCorrection = ");
//        Serial.print(" ");
//        Serial.print(rollCorrection);
//        Serial.print( " throttle = ");
        Serial.print(" ");
        Serial.print(throttle);
        Serial.print(" ");
//        Serial.print( " rollError = ");
//        Serial.print(rollError);
//        Serial.print( " frontRight = ");
        Serial.print(frontRight);
        Serial.print(" ");
//        Serial.print( " frontLeft = ");
        Serial.print(frontLeft);
        Serial.print(" ");
        Serial.print(tDelay);
        Serial.print(" ");
        Serial.print("END");
        
        if(tDelay < 1){
            abortSwitch = true;
            
            pwm.setPWM(0, 0, SERVOMIN);
            pwm.setPWM(3, 0, SERVOMIN);
            pwm.setPWM(4, 0, SERVOMIN);
            pwm.setPWM(7, 0, SERVOMIN);
            
        }
        // millis() - millisecondsPassed = the time in milliseconds spent running the loop 
        // delay the timestep - time already spent running this script
        // delay command takes an input in milliseconds and delays for that amount of milliseconds
        delay((timeStep*1000) - (millis() - millisecondsPassed));
        
    }

//    else{  // not stabilize mode 
//        desiredPitch = map(inputPitch, 1000, 2000, -10, 10);
//        desiredRoll = map(inputRoll, 1000, 2000, -10, 10);
//        desiredYaw = map(inputYaw, 1000, 2000, -10, 10);
//    }

    Serial.println();
  
}
