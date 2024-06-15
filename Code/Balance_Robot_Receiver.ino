//NRF24L01 library
#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>
#include <SPI.h>
#include <stdlib.h>
#include <string.h>

#include <Wire.h>
#include "MPU6050.h"
#include <IntervalTimer.h>

//Tasks Declaration
IntervalTimer pidTimer;
IntervalTimer motorTimer;

//creating object type MPU6050 named "mpu"
MPU6050 mpu;

// Motor dirThreshold and step pins
#define dirPinR 2
#define stepPinR 3
#define dirPinL 4
#define stepPinL 5

// Define the nRF24L01 radio and the CE, CSN pins
RF24 radio(7, 9); // CE, CSN pins

// Define the pipe address for the communication
const byte address[6] = "00001";

// PID scalar constants
float Kp = 150.0;
float Ki = 0.0;
float Kd = 0.0;

float setPoint  = 0.0; // Desired angle (upright default)

// Global variables necessary for PID control system (for later calculations)
float currentAngle, error, prevError, proportional, integral, derivative, motorSpeedL, motorSpeedR;

int stepDelay = 0; //neded to control the speed of motor
int dirThreshold = 0; //later used to check if robot is falling forward or backward

//KNOWN OFFSET VALUES aX:-1582  aY:3043  aZ:1220 | gX:102 gY:52 gZ:62
const int aXOffset = -1582;
const int aYOffset = 3043;
const int aZOffset = 1220;
const int gXOffset = 102;
const int gYOffset = 52;
const int gZOffset = 62;


void setup() {
  Serial.begin(115200); //baud rate 115200 for faster serial com

  // Initialize the radio
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();

  pinMode(MISO, OUTPUT); // Have to send on master in so it set as output
  SPCR |= _BV(SPE); // Turn on SPI in slave mode
  indx = 0; // Buffer empty
  process = false;
  SPI.attachInterrupt(); // Turn on interrupt

  //setup code for the balancing robot
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

  dirThreshold = motorSpeedR;
  if(abs(motorSpeedR) > 1500){
    stepDelay = abs(motorSpeedR) - 1500;
  }
  else{
    stepDelay = 1500 - abs(motorSpeedR);
  }
  //stepDelay = map(abs(motorSpeedR), 0, 500, 1500, 400);
  if(stepDelay > 1000){
    stepDelay = 1200;
    
  }
  else if(stepDelay > 800){
    stepDelay = 800;
    
  }
  else{
    stepDelay = 500;
  }
  motorTimer.update(stepDelay);
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
      if(dirThreshold < -650){ //falling forward
        digitalWrite(dirPinR, HIGH); //drive right motor CW (forward dir)
      }
      else if(dirThreshold > 650){ //falling backward
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
      if(dirThreshold < -650){ //falling forward
        digitalWrite(dirPinL, LOW); //drive left motor CCW (forward dir)
      }
      else if(dirThreshold > 650){ //falling backward
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
  if (radio.available()) {
    // Read the data from the radio
    radio.read(&setPoint, sizeof(setPoint));
    Serial.print("Received setPoint: ");
    Serial.println(setPoint);
  }
  //Serial.println(currentAngle);
  //Serial.println(motorSpeedR);
  //Serial.println(stepDelay);
  //delay(500);
}
