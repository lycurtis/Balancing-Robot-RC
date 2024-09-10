#include "mpu.h"

float setPoint = 0.0; //Desired angle (upright default)

void setup(){
  Serial.begin(115200);
  mpu_begin();
}

void loop(){
  run_mpu();
}
