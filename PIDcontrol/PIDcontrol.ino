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

int ch1Value;
int ch2Value;
int ch3Value;
int ch4Value;
int ch5Value;
bool abortSwitch = false;
int throttle;
int frontRight;
int frontLeft;
int backRight;
int backLeft;


// SERVO BOARD SETUP
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you have!

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// motor PWM frequency variable
uint16_t pulselen;

// MPU6050 SETUP
// Because of the mounting orientation of the chip on the drone: 
// PITCH = X axis
// ROLL = y axis

MPU6050 mpu;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float gyroPitch = 0;
float gyroRoll = 0;
float gyroYaw = 0;
float accelPitch;
float accelRoll;
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

    // SERVO BOARD SETUP
    // The internal oscillator (clock) for the PCA9685 chip is in a range between about 23-27MHz
    // Analog servos run at ~50 Hz updates, 
 
    pwm.begin();
    pwm.setOscillatorFrequency(27000000); // 27 MHz
    pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
    
    delay(10); 
    
    // MPU6050 SETUP
    Serial.println("Initialize MPU6050");

    while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
    {
        Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
        delay(500);
    }

    // Calibrate gyroscope. The calibration must be at rest.
    // If you don't want calibrate, comment this line.
    mpu.calibrateGyro();
    
    // Set threshold sensivty. Default 3.
    // If you don't want use threshold, comment this line or set 0.
    mpu.setThreshold(3);
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

    // timer for gyroscope timestep
    timer = millis();
    
    // ABORT SWITCH - turn all motors off
    if(pulseIn(CH5, HIGH) > 1500){
    
        pwm.setPWM(0, 0, SERVOMIN);
        pwm.setPWM(3, 0, SERVOMIN);
        pwm.setPWM(4, 0, SERVOMIN);
        pwm.setPWM(7, 0, SERVOMIN);
    }
  
  else {

    // GYRO DATA
    // Read normalized values
    normGyro = mpu.readNormalizeGyro();

    // Calculate Pitch, Roll and Yaw
    gyroRoll = gyroRoll + normGyro.YAxis * timeStep;
    gyroPitch = gyroPitch + normGyro.XAxis * timeStep;
    gyroYaw = gyroYaw + normGyro.ZAxis * timeStep;

    // ACCELEROMETER DATA
    // Read normalized values 
    normAccel = mpu.readNormalizeAccel();

    // Calculate Pitch & Roll
    accelRoll = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
    accelPitch = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;

    // COMPLIMENTARY FILTER

    pitch = (0.98*gyroPitch) + (0.02*accelPitch);
    roll = (0.98*gyroRoll) + (0.02*accelRoll);


    // Output
    Serial.print(" Pitch = ");
    Serial.print(pitch);
    Serial.print(" Roll = ");
    Serial.print(roll);
    Serial.print(" Yaw = ");
    Serial.println(gyroYaw);

    // SET NEW MOTOR SPEED THROUGH SERVO BOARD


    // THROTTLE
    // read pulse from receiver channel 3 (throttle stick) and map it to range 1000-2000
    ch3Value = mapReceiver(pulseIn(CH3, HIGH));

    // map receiver input value range of 0-1023 to a value range of 0-180 which the ESC understands 
    throttle = map(ch3Value, 1000, 2000, 150, 600);

    Serial.print( " throttle = ");
    Serial.print(throttle);


    // PITCH
    // forward pitch -> negative pitch values 
    // backward pitch -> positive pitch values 
    // ROLL 
    // right side roll -> positive roll degrees
    // leftside roll -> negative roll values 
    // YAW
    // clockwise yaw (right spin) -> negative yaw values
    // counterclockwise yaw (left spin) -> positive yaw values

    frontRight = throttle + frontPitchAdjust - rightRollAdjust + clockwiseYawAdjust;
    frontLeft = throttle + frontPitchAdjust + leftRollAdjust - counterclockwiseYawAdjust;
    backRight = throttle + backPitchAdjust - rightRollAdjust + clockwiseYawAdjust;
    backLeft = throttle + backPitchAdjust + leftRollAdjust - counterclockwiseYawAdjust;  //BACK LEFT STOP AND STUTTERS BEFORE OTHER MOTORS TURN OFF

  
    pwm.setPWM(0, 0, frontRight);
    pwm.setPWM(3, 0, frontLeft);
    pwm.setPWM(4, 0, backRight);
    pwm.setPWM(7, 0, backLeft);


    // WAIT FULL TIME STEP BEFORE CONTINUING FOR SAKE OF GYRO READINGS

    // make sure time isn't running over
    Serial.print(" Time to delay in milliseconds = ");
    Serial.print((timeStep*1000) - (millis() - timer));
    Serial.println();

    // Wait to full timeStep period

    // delay((timeStep*1000) - (millis() - timer));

    
  }

  
}
