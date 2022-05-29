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

int rollError;
int rollErrorIntegrated;

int ROLLPITCHERRORMIN = -180;
int ROLLPITCHERRORMAX = 180;


// SERVO BOARD SETUP
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  210 // minimum pulse length count (out of 4096) the ESC will recognize
#define SERVOMAX  400 // maximum pulse length count (out of 4096) the ESC will recognize

// MPU6050 SETUP
// Because of the mounting orientation of the chip on the drone: 
// PITCH = X axis
// ROLL = y axis

MPU6050 mpu;

// Timers
unsigned long millisecondsPassed = 0;
float timeStep = 0.05;

// Pitch, Roll and Yaw values
float gyroPitch = 0;
float gyroRoll = 0;
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

    pitch = (0.99*gyroPitch) + (0.01*accelPitch);
    roll = (0.99*gyroRoll) + (0.01*accelRoll);

    if(abs(pitch) > 180 || abs(roll) > 180){
        abortSwitch = true;
    }

//    // Output
//    Serial.print(" Pitch = ");
//    Serial.print(pitch);
//    Serial.print(" Roll = ");
//    Serial.print(roll);
//    Serial.print(" Yaw = ");
//    Serial.print(gyroYaw);

    ///////// state estimation complete ///////////

    // THROTTLE
    // read pulse from receiver channel 3 (throttle stick) and map it to range 1000-2000
    ch3Value = mapReceiver(pulseIn(CH3, HIGH));
    
    // map receiver input value range of 0-1023 to a value range of 0-180 which the ESC understands 
    throttle = map(ch3Value, 1000, 2000, SERVOMIN, SERVOMAX);
    
    
    // ABORT SWITCH - turn all motors off
    if(pulseIn(CH5, HIGH) > 1500 || throttle < 215){
        abortSwitch = true;
    }
    else{
        abortSwitch = false;
    }
    
    if(abortSwitch == true){
        pwm.setPWM(0, 0, SERVOMIN);
        pwm.setPWM(3, 0, SERVOMIN);
        pwm.setPWM(4, 0, SERVOMIN);
        pwm.setPWM(7, 0, SERVOMIN);
    } 
    
    else { // if (stabilize mode) {      
        
        // calculate error term
        rollErrorIntegrated = rollErrorIntegrated + roll;

        rollError = (0.5 * roll) + (0.05 * rollErrorIntegrated); // proportional term plus integral term
    
        // PITCH
        // forward pitch -> negative pitch values 
        // backward pitch -> positive pitch values 
        // ROLL 
        // right side roll -> positive roll degrees
        // leftside roll -> negative roll values 
        // YAW
        // clockwise yaw (right spin) -> negative yaw values
        // counterclockwise yaw (left spin) -> positive yaw values


        // SATURATION POSSIBLE HERE
        frontRight = throttle + rollError;           // + frontPitchAdjust - rightRollAdjust + clockwiseYawAdjust;
        if(frontRight > SERVOMAX) { frontRight = SERVOMAX; }
        
        frontLeft = throttle - rollError;            // + frontPitchAdjust + leftRollAdjust - counterclockwiseYawAdjust;
        if(frontLeft > SERVOMAX) { frontLeft = SERVOMAX; }
        
        backRight = throttle + rollError;            // + backPitchAdjust - rightRollAdjust + clockwiseYawAdjust;
        if(backRight > SERVOMAX) { backRight = SERVOMAX; }
        
        backLeft = throttle - rollError;             // + backPitchAdjust + leftRollAdjust - counterclockwiseYawAdjust;  //BACK LEFT STOPS BEFORE OTHER MOTORS TURN OFF
        if(backLeft > SERVOMAX) { backLeft = SERVOMAX; }
        
//
//        Serial.print( " throttle = ");
//        Serial.print(throttle);
//        Serial.print( " rollError = ");
//        Serial.print(rollError);
//        Serial.print( " frontRight = ");
//        Serial.print(frontRight);
//        Serial.print( " frontLeft = ");
//        Serial.print(frontLeft);

    
        pwm.setPWM(0, 0, frontRight);
        pwm.setPWM(3, 0, frontLeft);
        pwm.setPWM(4, 0, backRight);
        pwm.setPWM(7, 0, backLeft);
    
    
        // WAIT FULL TIME STEP BEFORE CONTINUING FOR SAKE OF GYRO READINGS
    
        // if this number is negative the timeStep is too short
//        Serial.print(" Time to delay in milliseconds = ");
//        Serial.print((timeStep*1000) - (millis() - millisecondsPassed));
        
    
        // Wait to full timeStep period
    
        delay((timeStep*1000) - (millis() - millisecondsPassed));
        
    }

    //Serial.println();
  
}
