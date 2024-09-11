#include "mpu.h"
#include "pid.h"

void setup(){
  Serial.begin(115200);
  mpu_begin();
}

void loop(){
  run_mpu();
  //Serial.println(calculate_PID(kalmanAnglePitch));
  print_kalmanPlotter();
}
