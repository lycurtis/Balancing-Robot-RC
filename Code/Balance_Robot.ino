#include <Wire.h>
#include "MPU6050.h"
#include <IntervalTimer.h>

//Tasks Declaration
IntervalTimer pidTimer;
IntervalTimer motorTimer;

//creating object type MPU6050 named "mpu"
MPU6050 mpu;

// Motor direction and step pins
#define dirPinR 2
#define stepPinR 3
#define dirPinL 4
#define stepPinL 5

// PID scalar constants
float Kp = 7.0;
float Ki = 0.0;
float Kd = 0.0;

float setPoint  = 0.0; // Desired angle (upright default)

// Global variables necessary for PID control system (for later calculations)
float currentAngle, error, prevError, proportional, integral, derivative, motorSpeedL, motorSpeedR;

int stepDelay = 0; //neded to control the speed of motor
int direction = 0; //later used to check if robot is falling forward or backward

//KNOWN OFFSET VALUES aX:-1582  aY:3043  aZ:1220 | gX:102 gY:52 gZ:62
const int aXOffset = -1582;
const int aYOffset = 3043;
const int aZOffset = 1220;
const int gXOffset = 102;
const int gYOffset = 52;
const int gZOffset = 62;


void setup() {
  Serial.begin(115200); //baud rate 115200 for faster serial com
  Wire.begin();
  
  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  // Verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  //compensate for offset (set offset)
  mpu.setXAccelOffset(aXOffset);
  mpu.setYAccelOffset(aYOffset);
  mpu.setZAccelOffset(aZOffset);
  mpu.setXGyroOffset(gXOffset);
  mpu.setYGyroOffset(gYOffset);
  mpu.setZGyroOffset(gZOffset);

  // Motor control pins
  pinMode(dirPinR, OUTPUT);
  pinMode(stepPinR, OUTPUT);
  pinMode(dirPinL, OUTPUT);
  pinMode(stepPinL, OUTPUT);

  // Start the timer to call the balance function every 10 000 microseconds = 10 ms (10 Hz)
  pidTimer.begin(calcSpeedPID, 10000); 
  motorTimer.begin(motorControlPID, 1000);
}


void calcSpeedPID() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Convert raw values to G-force
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;

  // Calculate tilt angles
  float roll  = atan2(accelY, accelZ) * 180 / PI;
  float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;

  currentAngle = pitch;

  // PID controller calculations
  error = currentAngle - setPoint; // or setPoint - currentAngle (whichever gives you the correct interpretation of positive/negative)
  proportional = error * Kp;
  integral += error * Ki; // integral = integral + error * Ki
  derivative = (error - prevError) * Kd;

  // Output (left and right motor speed)
  motorSpeedR = proportional + integral + derivative;
  motorSpeedL = proportional + integral + derivative;

  prevError = error; // important to set prevError so on the next loop it remembers the previous error

  direction = motorSpeedR;
  stepDelay = map(abs(motorSpeedR), 0, 300, 1000, 400);
  //motorTimer.update(stepDelay);
}

enum STATES_MOTORR {STEPHIGHR, STEPLOWR} rState = STEPHIGHR;
enum STATES_MOTORL {STEPHIGHL, STEPLOWL} lState = STEPHIGHL;

void motorControlPID(){
  //right motor
  switch(rState){
    case STEPHIGHR: //PULSE HIGH
      if(1){
        rState = STEPLOWR;
      }
      break;
    case STEPLOWR: //PULSE LOW
      if(1){
        rState = STEPHIGHR;
      }
    default:
      rState = STEPHIGHR;
      break;
  }
  switch(rState){
    case STEPHIGHR:
      if(direction < 0){ //falling forward
        digitalWrite(dirPinR, HIGH); //drive right motor CW (forward dir)
      }
      else if(direction > 0){ //falling backward
        digitalWrite(dirPinR, LOW); //drive right motor CCW (backward dir)
      }
      digitalWrite(stepPinR, HIGH); //pulse HIGH
      break;
    case STEPLOWR:
      digitalWrite(stepPinR, LOW); //pulse LOW
      break;
  }

  //left motor
  switch(lState){
    case STEPHIGHL:
      if(1){
        lState = STEPLOWL;
      }
      break;
    case STEPLOWL:
      if(1){
        lState = STEPHIGHL;
      }
    default:
      lState = STEPHIGHL;
      break;
  }
  switch(lState){
    case STEPHIGHL:
      if(direction < 0){ //falling forward
        digitalWrite(dirPinL, LOW); //drive left motor CCW (forward dir)
      }
      else if(direction > 0){ //falling backward
        digitalWrite(dirPinL, HIGH); //drive left motor CW
      }
      digitalWrite(stepPinL, HIGH); //pulse HIGH
      break;
    case STEPLOWL:
      digitalWrite(stepPinL, LOW); //pulse LOW
      break;
  }
}


void loop() {
  Serial.println(currentAngle);
  //Serial.println(motorSpeedR);
  //Serial.println(stepDelay);
  delay(500);
}
