#include <Wire.h>
#include "MPU6050.h"
#include "IntervalTimer.h"

MPU6050 mpu;

#define TOTAL_TASK 2
#define PERIOD_GCD 1000
#define PERIOD_MOTORR 1000
#define PERIOD_MOTORL 1000

// Motor direction and step pins
#define dirPinR 2
#define stepPinR 3
#define dirPinL 4
#define stepPinL 5

#define stepsPerRevolution 200

// PID scalar constants
float Kp = 10.0;
float Ki = 0.0;
float Kd = 0.0;

float setPoint  = 0.0; // Desired angle (upright)

// Global variables necessary for PID control system (for later calculations)
float currentAngle, error, prevError, proportional, integral, derivative, motorSpeedL, motorSpeedR;

const int numReadings = 500;
float rollOffset = 0.0;
float pitchOffset = 0.0;

int takenStepR = 0;
int takenStepL = 0;

IntervalTimer myTimer;

typedef struct task {
  unsigned short period;
  unsigned short timeElapsed;
  void (*tick)(void);
} task;

static task gTaskSet[TOTAL_TASK];

void initializeTask(void) {
  gTaskSet[0].period = PERIOD_MOTORR;
  gTaskSet[0].timeElapsed = 0;
  gTaskSet[0].tick = tickControlR;

  gTaskSet[1].period = PERIOD_MOTORL;
  gTaskSet[1].timeElapsed = 0;
  gTaskSet[1].tick = tickControlL;
}

void scheduleTask() {
  for (int i = 0; i < TOTAL_TASK; i++) {
    gTaskSet[i].timeElapsed += PERIOD_GCD;
    if (gTaskSet[i].timeElapsed >= gTaskSet[i].period) {
      gTaskSet[i].tick();
      gTaskSet[i].timeElapsed = 0;
    }
  }
}

enum STATES_CONTROLR {INIT, IDLE, FORWARD, BACKWARD, TURNL, TURNR, QUIT} rState = INIT;
enum STATES_CONTROLL {INIT1, IDLE1, FORWARD1, BACKWARD1, TURNL1, TURNR1, QUIT1} lState = INIT1;

void tickControlR(void) {
  switch (rState) {
    case INIT:
      if (1) {
        rState = IDLE;
      }
      break;
    case IDLE:
      break;
    case FORWARD:
      break;
    case BACKWARD:
      break;
    case TURNL:
      break;
    case TURNR:
      break;
    case QUIT:
      break;
    // Add other states and their actions
    default:
      rState = IDLE;
      break;
  }

  switch(rState){
    case INIT:
      break;
    case IDLE:
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
      break;
    case FORWARD:
      break;
    case BACKWARD:
      break;
    case TURNL:
      break;
    case TURNR:
      break;
    case QUIT:
      break;
  }
}

void tickControlL(void) {
  switch (lState) {
    case INIT1:
      if (1) {
        lState = IDLE1;
      }
      break;
    case IDLE1:
      break;
    case FORWARD1:
      break;
    case BACKWARD1:
      break;
    case TURNL1:
      break;
    case TURNR1:
      break;
    case QUIT1:
      break;
    // Add other states and their actions
  }

  switch(rState){
    case INIT1:
      break;
    case IDLE1:
      break;
    case FORWARD1:
      break;
    case BACKWARD1:
      break;
    case TURNL1:
      break;
    case TURNR1:
      break;
    case QUIT1:
      break;
  }
}

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

  // Initialize tasks
  initializeTask();

  // Start the timer
  myTimer.begin(scheduleTask, PERIOD_GCD);
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
  //Serial.print(" Roll: "); Serial.print(roll); // Right (-) Left(+)
  //Serial.print(" | Pitch: "); Serial.println(pitch); // Forward (-) Backward (+)

  // delay(100);

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

  scheduleTask();
  //while(!TimerFlag){}
  //TimerFlag = 0;
}
