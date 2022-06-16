/*************************************************** 
Flight Controller for the Osprey Arduino Quadcopter

****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
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
int rollCorrection = 0;


float pitchProportional;
float pitchIntegral = 0.0f;
float pitchDerivative;
float previousPitch;
int pitchCorrection = 0;

int yawCorrection = 0;

// got better and better performance by continually dropping D gain, I think D gain could be lower and I might try raising P gain back up

float rollPgain = 0.47; // 0.18
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
        pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates  
    }

    void initializeIMU(){
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
    }

    void setAccelerometerOffset(){
        // LOOP THIS TO GET A MORE ACCURATE OFFSET
        normAccel = mpu.readNormalizeAccel();
    
        rollOffset = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
        pitchOffset = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
    }
    
    void getStateEstimation(){
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
    
        // COMPLIMENTARY FILTER
        pitch = (0.98*gyroPitch) + (0.02*accelPitch);
        roll = (0.98*gyroRoll) + (0.02*accelRoll);
        
    }

    int readThrottle(){
        
        // read pulse from receiver channel 3 (throttle stick) and map it to range 1000-2000
        ch3Value = mapReceiver(pulseIn(CH3, HIGH));
        
        // map receiver input value range of 0-1023 to a value range of 0-180 which the ESC understands 
        throttle = map(ch3Value, 1000, 2000, SERVOMIN, SERVOMAX);

    }

    void checkAbortConditions(){

        if(abortSwitch == true){
            killMotors();
        }

        // if pitch or roll exceeds 80º
        if(abs(pitch) > 80 || abs(roll) > 80){
            killMotors();
        }
        
        // Abort switch on RC controller
        if(pulseIn(CH5, HIGH) > 1500){
            killMotors();
        }
        
    }

    void killMotors(){
        Serial.print(true);
        Serial.print(" ");
        pwm.setPWM(0, 0, SERVOMIN);
        pwm.setPWM(3, 0, SERVOMIN);
        pwm.setPWM(4, 0, SERVOMIN);
        pwm.setPWM(7, 0, SERVOMIN);

        while(true){
             
        }
    }

    void getRollCorrection(){
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
        //rollDerivative = (rollDgain * (roll - previousRoll)/timeStep); 
        rollDerivative = rollDgain * normGyro.YAxis;
        
        if(rollDerivative > 15){
            rollDerivative = 15;
        }
        if(rollDerivative < -15){
            rollDerivative = -15;
        }

       previousRoll = roll;

        rollCorrection = rollProportional + rollDerivative; //  + rollIntegral;
    }

    void getPitchCorrection(){
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
    }

    void getYawCorrection(){
        
    }

    void setFrontRight(){
        
        frontRight = throttle + rollCorrection - pitchCorrection + yawCorrection;

        // frontRight = pow(frontRight, 1.01) - 10;

        frontRight = (frontRight*1.028) - 7;
        
        if(frontRight > SERVOMAX) { frontRight = SERVOMAX;}
        if(frontRight < SERVOMIN) { frontRight = SERVOMIN;} 
    
        pwm.setPWM(0, 0, frontRight);
    }

    void setFrontLeft(){

        frontLeft = throttle - rollCorrection - pitchCorrection - yawCorrection; 

        // frontLeft = pow(frontLeft, 1.011) - 11;
        frontLeft = (frontLeft*1.035) - 7;
        
        if(frontLeft > SERVOMAX) { frontLeft = SERVOMAX; }
        if(frontLeft < SERVOMIN) { frontLeft = SERVOMIN; }
        pwm.setPWM(3, 0, frontLeft);
    }

    void setBackRight(){

        backRight = throttle + rollCorrection + pitchCorrection - yawCorrection;

        // backRight = pow(backRight, 1.011) - 11;
        //backRight = (1.03 * backRight) - 7;
        
        if(backRight > SERVOMAX) { backRight = SERVOMAX; }
        if(backRight < SERVOMIN) { backRight = SERVOMIN; }

        pwm.setPWM(4, 0, backRight);
    }

    void setBackLeft(){

        backLeft = throttle - rollCorrection + pitchCorrection + yawCorrection;
        if(backLeft > SERVOMAX) { backLeft = SERVOMAX; }
        if(backLeft < SERVOMIN) { backLeft = SERVOMIN; }

        pwm.setPWM(7, 0, backLeft);
    }

    void printData(){
        Serial.print(false);
        Serial.print(" ");
//        Serial.print(" Roll = ");
        Serial.print(roll);
//        Serial.print(" Pitch = ");
        Serial.print(" ");
        Serial.print(pitch);
//           Serial.print(" Yaw = ");
        Serial.print(" ");
        Serial.print(gyroYaw);
//        Serial.print( " proportional = ");
        Serial.print(" ");
        Serial.print(rollProportional);
//        Serial.print( " integral = ");
        Serial.print(" ");
        Serial.print(rollIntegral);
//        Serial.print( " derivative = ");
        Serial.print(" ");
        Serial.print(rollDerivative);
//        Serial.print( " rollCorrection = ");
        Serial.print(" ");
        Serial.print(rollCorrection);
//        Serial.print( " throttle = ");
        Serial.print(" ");
        Serial.print(throttle);
        Serial.print(" ");
        Serial.print(frontRight);
        Serial.print(" ");
        Serial.print(frontLeft);
        Serial.print(" ");
//        Serial.print( " backRight = ");
        Serial.print(backRight);
        Serial.print(" ");
//        Serial.print( " backLeft = ");
        Serial.print(backLeft);
        Serial.print(" ");
        Serial.print(tDelay);
        Serial.print(" ");
        Serial.print("END");
        Serial.println();
    }

/////////////////////////////////////////////////// SETUP FUNCTION /

void setup() {

    // set serial monitor baud rate
    Serial.begin(115200);

    initializeRecieverPins();

    initializeServoBoard();

    initializeIMU();

    setAccelerometerOffset();

    float previousRoll = 0.0;
    float previousPitch = 0.0;


}

/////////////////////////////////////////////////// LOOP FUNCTION /


void loop() {

    // timer for gyroscope timestep
    // returns the number of milliseconds passed since the arduino board began running the current program
    millisecondsPassed = millis();

    getStateEstimation();

    checkAbortConditions();

    readThrottle();

    // calculate PID corrections for roll, pitch, and yaw
    getRollCorrection();
    
    //getPitchCorrection();
    
    //getYawCorrection();
   
    setFrontRight();
    
    setFrontLeft();
    
    setBackRight();
    
    setBackLeft();
    
    // MUST WAIT FULL TIME STEP BEFORE CONTINUING FOR SAKE OF GYRO READINGS
    tDelay = (timeStep*1000) - (millis() - millisecondsPassed);

    if(tDelay < 5){
        killMotors();
    }
        
    printData();

    

    // delay command takes an input in milliseconds and delays for that amount of milliseconds
    // time to delay = timestep - time already spent running this script
    // millis() - millisecondsPassed = the time in milliseconds spent running the loop 
    delay((timeStep*1000) - (millis() - millisecondsPassed));


//        not stabilize mode 
//        desiredPitch = map(inputPitch, 1000, 2000, -10, 10);
//        desiredRoll = map(inputRoll, 1000, 2000, -10, 10);
//        desiredYaw = map(inputYaw, 1000, 2000, -10, 10);
    

    
}
