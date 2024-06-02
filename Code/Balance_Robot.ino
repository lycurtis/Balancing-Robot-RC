#include <Wire.h>
#include "MPU6050.h"
#include <IntervalTimer.h>

IntervalTimer myTimer;

MPU6050 mpu;

// Motor direction and step pins
#define dirPinR 2
#define stepPinR 3
#define dirPinL 4
#define stepPinL 5


// PID scalar constants
float Kp = 10.0;
float Ki = 0.0;
float Kd = 0.0;

float setPoint  = 0.0; // Desired angle (upright)

// Global variables necessary for PID control system (for later calculations)
float currentAngle, error, prevError, proportional, integral, derivative, motorSpeedL, motorSpeedR;

int takenStepR = 0;

const int numReadings = 500;
float rollOffset = 0.0;
float pitchOffset = 0.0;

void setup() {
  Serial.begin(115200);
  myTimer.begin(balance, 20);
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

    //delay(10);
  }

  rollOffset = rollSum / numReadings;
  pitchOffset = pitchSum / numReadings;

  Serial.print("Measured roll offset: "); Serial.println(rollOffset);
  Serial.print("Measured pitch offset: "); Serial.println(pitchOffset);
}

void balance(){
  if(motorSpeedR < 0){
    //forward pitch
    digitalWrite(dirPinR, HIGH); //CW (forward dir)
    if(takenStepR == 0){
      digitalWrite(stepPinR, HIGH);
      takenStepR++;
    }
    else if(takenStepR == 1){
      digitalWrite(stepPinR, LOW);
      takenStepR = 0;
    }
  }
  else if(motorSpeedR > 0){
    //backward pitch
    digitalWrite(dirPinR, LOW); //CCW (backward dir)
    if(takenStepR == 0){
      digitalWrite(stepPinR, HIGH);
      takenStepR++;
    }
    else if(takenStepR == 1){
      digitalWrite(stepPinR, LOW);
      takenStepR = 0;
    }
  }
}

void loop() {
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

  // Print tilt angles
  Serial.print(" Roll: "); Serial.print(roll); // Right (-) Left(+)
  Serial.print(" | Pitch: "); Serial.println(pitch); // Forward (-) Backward (+)
  delay(100);

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

  /*
  Serial.print("Right Speed: ");
  Serial.println(motorSpeedR);
  Serial.print("Left Speed");
  Serial.println(motorSpeedL);

  delay(100);
  */
}
