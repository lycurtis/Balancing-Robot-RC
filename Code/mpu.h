#ifndef MPU_H
#define MPU_H

#include <Arduino.h>
#include <Wire.h>

//declar global kalman variables as extern so they are available across files
extern float kalmanAngleRoll;
extern float kalmanAnglePitch;

void kalman_1d(float &kalmanState, float &kalmanUncertainty, float kalmanInput, float kalmanMeasurement);
void gyro_signals(void);
void mpu_begin();
void print_accXYZ(void);
void print_gyroXYZ(void);
void print_unfilteredAngles(void);
void print_kalmanMonitor(void);
void print_kalmanPlotter(void);
void run_mpu(void);

#endif
