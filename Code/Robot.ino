#include "mpu.h"
#include "pid.h"

const int stepsPerRevolution = 200; 

const int leftMotorStep = 4;
const int leftMotorDir = 5;
const int rightMotorStep = 2;
const int rightMotorDir = 3;

void control_motor(int dirPin, int stepPin, float pidValue){
  //determine the direction 
  if(pidValue > 0){
    digitalWrite(dirPin, HIGH); //Set direction forward
  }
  else{
    digitalWrite(dirPin, LOW); //Set direction backward
  }

  //calculate delay between steps based on pidValue to control speed
  float speed = abs(pidValue); 
  long stepDelay = map(speed, 0, 400, 2000, 500);; //Map pid value

  // Generate step pulses based on the calculated speed
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(stepDelay);  // Delay determines speed
  digitalWrite(stepPin, LOW);
  delayMicroseconds(stepDelay);  // Delay between steps
}

void setup(){
  Serial.begin(115200);
  mpu_begin();

  pinMode(leftMotorStep, OUTPUT);
  pinMode(leftMotorDir, OUTPUT);
  pinMode(rightMotorStep, OUTPUT);
  pinMode(rightMotorDir, OUTPUT);
}

void loop(){
  run_mpu();
  float pidValue = calculate_PID(kalmanAnglePitch);
  control_motor(rightMotorDir, rightMotorStep, pidValue);

  /////////////////////////////////////
  //Print Statements 
  //Serial.println(pidValue);
  //print_kalmanPlotter();
  //print_kalmanMonitor();
  ////////////////////////////////////

}
