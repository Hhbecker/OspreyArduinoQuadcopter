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
bool debug = false; 
bool stabilizeMode = false;
bool abortSwitch = false;
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
float rollValue;
float rollError;
float rollProportional;
float rollIntegral = 0.0f;
float rollDerivative;
float previousRoll = 0.0f;
int rollCorrection = 0;

float pitchValue = 0.0f;
float pitchError;
float pitchProportional;
float pitchIntegral = 0.0f;
float pitchDerivative;
float previousPitch = 0.0f;
int pitchCorrection;

float yawValue = 0.0f;
float yawError;
float yawProportional;
float yawIntegral = 0.0f;
float yawDerivative;
float previousYaw = 0.0f;
int yawCorrection;

// got better and better performance by continually dropping D gain, I think D gain could be lower and I might try raising P gain back up

float rollPgain = 0.225;
float rollIgain = 0.001; 
float rollDgain = 0.047;

float pitchPgain = 0.3;
float pitchIgain = 0.00;
float pitchDgain = 0.1;

float yawPgain = 0.00;
float yawIgain = 0.00;
float yawDgain = 0.00;


// SERVO BOARD SETUP
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  210 // minimum pulse length count (out of 4096) the ESC will recognize
#define SERVOMAX  340 // maximum pulse length count (out of 4096) the ESC will recognize // FLIGHT = 400 ROLL TEST = 350

int ROLLMIN = -10;
int ROLLMAX = 10;
int PITCHMIN = -10;
int PITCHMAX = 10;
int YAWMIN = -10;
int YAWMAX = 10;


// MPU6050 SETUP
// Because of the mounting orientation of the chip on the drone: 
// PITCH = X axis
// ROLL = y axis

MPU6050 mpu;

// Timers
unsigned long millisecondsPassed = 0;
float timeStep;
float tDelay;

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
            
            delay(500);
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
//        pitch = (0.98*gyroPitch) + (0.02*accelPitch);
//        roll = (0.98*gyroRoll) + (0.02*accelRoll);
//        pitch = (0.99*gyroPitch) + (0.01*accelPitch);
//        roll = (0.99*gyroRoll) + (0.01*accelRoll);
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

    void readRoll(){

        if(stabilizeMode == true){
            
            rollValue = 0.00;
            
        }
        else{

            int pulse = pulseIn(CH1, HIGH, timeout);

            if(pulse == 0){
                Serial.print("Roll in = 0");
            }
            
            rollValue = mapReceiver(pulse);
            
            rollValue = map(rollValue, 1000, 2000, ROLLMIN, ROLLMAX);

            rollValue++; // to counter offset value of -1 when joystick is at rest 
        }
    }
    
    void readPitch(){
        if(stabilizeMode == true){
            
            pitchValue = 0.00;
            
        }
        else{

            int pulse = pulseIn(CH2, HIGH, timeout);

            if(pulse == 0){
                Serial.print("Pitch in = 0");
            }
            
            pitchValue = mapReceiver(pulse);

            pitchValue = map(pitchValue, 1000, 2000, PITCHMIN, PITCHMAX);
        }
    }

    void readYaw(){
        if(stabilizeMode == true){
            
            yawValue = 0.00;
            
        }
        else{

            int pulse = pulseIn(CH4, HIGH, timeout);

            if(pulse == 0){
                Serial.print("Yaw in = 0");
            }
            
            yawValue = mapReceiver(pulse);

            yawValue = map(yawValue, 1000, 2000, YAWMIN, YAWMAX);
        }
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
        if(pulseIn(CH5, HIGH, timeout) > 1500){
            killMotors();
        }
        
    }

    void killMotors(){

        if(debug == true){
            Serial.print(true);
            Serial.print(" ");
        }
        
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

        // Integral Term
        if(throttle > 250){
            rollIntegral = rollIntegral + (rollError * rollIgain);
        }

        if(rollIntegral > 5){
            rollIntegral = 5;
        }
        if(rollIntegral < -5){
            rollIntegral = -5;
        }

        // Derivative Term
        //rollDerivative = (rollDgain * (rollError - previousRoll)/timeStep); 
        rollDerivative = rollDgain * normGyro.YAxis;
        
        if(rollDerivative > 10){
            rollDerivative = 10;
        }
        if(rollDerivative < -10){
            rollDerivative = -10;
        }

       previousRoll = rollError;

        rollCorrection = rollProportional + rollDerivative + rollIntegral;
    }

    void getPitchCorrection(){

        pitchError = pitch - pitchValue;
        
                // Proportional Term
        pitchProportional = pitchPgain * pitchError;

        // Integral Term
        if(throttle > 250){
            pitchIntegral = pitchIntegral + (pitchError * pitchIgain);
        }

        if(pitchIntegral > 5){
            pitchIntegral = 5;
        }
        if(pitchIntegral < -5){
            pitchIntegral = -5;
        }

        // Derivative Term
        // pitchDerivative = (pitchDgain * (pitch - previousPitch)/timeStep); 
        pitchDerivative = pitchDgain * normGyro.YAxis;
        
        if(pitchDerivative > 10){
            pitchDerivative = 10;
        }
        if(pitchDerivative < -10){
            pitchDerivative = -10;
        }

        previousPitch = pitchError;

        pitchCorrection = pitchProportional + pitchDerivative + pitchIntegral;
    }

    void getYawCorrection(){

        yawError = gyroYaw - yawValue;

        // Proportional Term
        yawProportional = yawPgain * yawError;

        // Derivative Term
        //yawDerivative = (yawDgain * (yawError - previousYaw)/timeStep); 
        yawDerivative = yawDgain * normGyro.YAxis;
        
        if(yawDerivative > 10){
            yawDerivative = 10;
        }
        if(yawDerivative < -10){
            yawDerivative = -10;
        }

       previousYaw = yawError;

        yawCorrection = yawProportional + yawDerivative; // + yawIntegral;
    }

    void setFrontRight(){
        
        frontRight = throttle + rollCorrection - pitchCorrection + yawCorrection;

        // frontRight = pow(frontRight, 1.01) - 10;
        // frontRight = (frontRight*1.028) - 6;
        
        if(frontRight > SERVOMAX) { frontRight = SERVOMAX;}
        if(frontRight < SERVOMIN) { frontRight = SERVOMIN;} 
    
        pwm.setPWM(1, 0, frontRight);
    }

    void setFrontLeft(){

        frontLeft = throttle - rollCorrection - pitchCorrection - yawCorrection; 

        // frontLeft = pow(frontLeft, 1.011) - 11;
        // frontLeft = (frontLeft*1.035) - 7;
        
        if(frontLeft > SERVOMAX) { frontLeft = SERVOMAX; }
        if(frontLeft < SERVOMIN) { frontLeft = SERVOMIN; }
        
        pwm.setPWM(15, 0, frontLeft);
    }

    void setBackRight(){

        backRight = throttle + rollCorrection + pitchCorrection - yawCorrection;

        // backRight = pow(backRight, 1.011) - 11;
        //backRight = (1.03 * backRight) - 7;
        
        if(backRight > SERVOMAX) { backRight = SERVOMAX; }
        if(backRight < SERVOMIN) { backRight = SERVOMIN; }

        pwm.setPWM(0, 0, backRight);
    }

    void setBackLeft(){

        backLeft = throttle - rollCorrection + pitchCorrection + yawCorrection;

        backLeft = backLeft; 
        
        if(backLeft > SERVOMAX) { backLeft = SERVOMAX; }
        if(backLeft < SERVOMIN) { backLeft = SERVOMIN; }

        pwm.setPWM(14, 0, backLeft);
    }

    void printData(){
//        Serial.print(false);
//        Serial.print(" ");
////        Serial.print(" Roll = ");
//        Serial.print(roll);
////        Serial.print(" Pitch = ");
//        Serial.print(" ");
//        Serial.print(pitch);
////        Serial.print(" Yaw = ");
//        Serial.print(" ");
//        Serial.print(gyroYaw);

        Serial.print(" rollValue = ");
        Serial.print(rollValue);
        Serial.print(" rollError = ");
        Serial.print(rollError);
        Serial.print(" rollCorrection = ");
        Serial.print(rollCorrection);
        

        Serial.print(" pitchValue = ");
        Serial.print(pitchValue);
        Serial.print(" pitchError = ");
        Serial.print(pitchError);
        Serial.print(" pitchCorrection = ");
        Serial.print(pitchCorrection);

        Serial.print(" yawValue = ");
        Serial.print(yawValue);
        Serial.print(" yawError = ");
        Serial.print(yawError);
        Serial.print(" yawCorrection = ");
        Serial.print(yawCorrection);

        
////        Serial.print( " proportional = ");
//        Serial.print(" ");
//        Serial.print(rollProportional);
////        Serial.print( " integral = ");
//        Serial.print(" ");
//        Serial.print(rollIntegral);
////        Serial.print( " derivative = ");
//        Serial.print(" ");
//        Serial.print(rollDerivative);
////        Serial.print( " rollCorrection = ");
//        Serial.print(" ");
//        Serial.print(rollCorrection);
////
//        Serial.print(" ");
//        Serial.print(pitchProportional);
////        Serial.print( " integral = ");
//        Serial.print(" ");
//        Serial.print(pitchIntegral);
////        Serial.print( " derivative = ");
//        Serial.print(" ");
//        Serial.print(pitchDerivative);
////        Serial.print( " rollCorrection = ");
//        Serial.print(" ");
//        Serial.print(pitchCorrection);
//
//
//        Serial.print(" ");
        Serial.print( " throttle = ");
        Serial.print(throttle);
//
//        
//        Serial.print(" ");
//        Serial.print(frontRight);
//        Serial.print(" ");
//        Serial.print(frontLeft);
//        Serial.print(" ");
////        Serial.print( " backRight = ");
//        Serial.print(backRight);
//        Serial.print(" ");
////        Serial.print( " backLeft = ");
//        Serial.print(backLeft);
//        Serial.print(" ");
        Serial.print("time delay");
        Serial.print(tDelay);
//        Serial.print(" ");
//        Serial.print("END");
        Serial.println();
    }

/////////////////////////////////////////////////// SETUP FUNCTION /

void setup() {

    if(debug == true){
        // set serial monitor baud rate
        Serial.begin(115200);
        timeStep = 0.07f; 
    }
    else{
        timeStep = 0.07f;
    }


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

    readRoll();
    //readPitch();
    //readYaw();

    getRollCorrection();
    getPitchCorrection();
    getYawCorrection();
   
    setFrontRight();
    
    setFrontLeft();
    
    setBackRight();
    
    setBackLeft();
    
    
    if(debug == true){
        tDelay = (timeStep*1000) - (millis() - millisecondsPassed);
        printData();
    }

    // MUST WAIT FULL TIME STEP BEFORE CONTINUING FOR SAKE OF GYRO READINGS
    tDelay = (timeStep*1000) - (millis() - millisecondsPassed);

    if(tDelay < 6){
        killMotors();
    }
    

    // delay command takes an input in milliseconds and delays for that amount of milliseconds
    // time to delay = timestep - time already spent running this script
    // millis() - millisecondsPassed = the time in milliseconds spent running the loop 
    delay((timeStep*1000) - (millis() - millisecondsPassed));


    
}
