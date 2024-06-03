#include <Wire.h>
#include "MPU6050.h"
#include <IntervalTimer.h>

IntervalTimer pidTimer;
IntervalTimer motorRTimer;

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

float setPoint  = 0.0; // Desired angle (upright)

// Global variables necessary for PID control system (for later calculations)
float currentAngle, error, prevError, proportional, integral, derivative, motorSpeedL, motorSpeedR;

int stepDelay = 0;
int direction = 0;

const int numReadings = 500;
float rollOffset = 0.0;
float pitchOffset = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  // Verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // Calibrate the MPU6050
  Serial.println("Calibrating MPU6050...");
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();

  // Measure offsets
  measureOffsets();

  // Motor control pins
  pinMode(dirPinR, OUTPUT);
  pinMode(stepPinR, OUTPUT);
  pinMode(dirPinL, OUTPUT);
  pinMode(stepPinL, OUTPUT);

  // Start the timer to call the balance function every 10 000 microseconds = 10 ms (10 Hz)
  pidTimer.begin(calcSpeedPID, 10000); 
  motorRTimer.begin(motorControlPID, 1000);
}

void measureOffsets() {
  float rollSum = 0.0;
  float pitchSum = 0.0;

  for (int i = 0; i < numReadings; i++) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);

    // Convert raw values to G-force
    float accelX = ax / 16384.0;
    float accelY = ay / 16384.0;
    float accelZ = az / 16384.0;

    // Calculate tilt angles
    float roll  = atan2(accelY, accelZ) * 180 / PI;
    float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;

    rollSum += roll;
    pitchSum += pitch;
  }

  rollOffset = rollSum / numReadings;
  pitchOffset = pitchSum / numReadings;

  Serial.print("Measured roll offset: "); Serial.println(rollOffset);
  Serial.print("Measured pitch offset: "); Serial.println(pitchOffset);
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

  // Apply offsets
  roll -= rollOffset;
  pitch -= pitchOffset;

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
  //motorRTimer.update(stepDelay);
}

enum STATES_MOTORR {STEPHIGHR, STEPLOWR} rState = STEPHIGHR;
enum STATES_MOTORL {STEPHIGHL, STEPLOWL} lState = STEPHIGHL;

void motorControlPID(){
  //right motor
  switch(rState){
    case STEPHIGHR:
      if(1){
        rState = STEPLOWR;
      }
      break;
    case STEPLOWR:
      if(1){
        rState = STEPHIGHR;
      }
    default:
      rState = STEPHIGHR;
      break;
  }
  switch(rState){
    case STEPHIGHR:
      if(direction < 0){
        digitalWrite(dirPinR, HIGH);
      }
      else if(direction > 0){
        digitalWrite(dirPinR, LOW);
      }
      digitalWrite(stepPinR, HIGH);
      break;
    case STEPLOWR:
      digitalWrite(stepPinR, LOW);
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
      if(direction < 0){
        digitalWrite(dirPinL, LOW);
      }
      else if(direction > 0){
        digitalWrite(dirPinL, HIGH);
      }
      digitalWrite(stepPinL, HIGH);
      break;
    case STEPLOWL:
      digitalWrite(stepPinL, LOW);
      break;
  }
}


void loop() {
  //Serial.println(currentAngle);
  //Serial.println(motorSpeedR);
  Serial.println(stepDelay);
}
