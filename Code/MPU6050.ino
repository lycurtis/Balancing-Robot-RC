#include <Wire.h>

float rateRoll, ratePitch, rateYaw;
float offsetRoll, offsetPitch, offsetYaw;
int calibrationNum;
float accX, accY, accZ;
float angleRoll, anglePitch;

//Define the predicted angles and uncertanties 
float kalmanAngleRoll = 0, kalmanUncertaintyAngleRoll = 2*2; //1st assume that initial state is on a leveled surface so 0 degs
float kalmanAnglePitch = 0, kalmanUncertaintyAnglePitch = 2*2; //however there are uncertanties therefore we set the standard dev. to be 2 deg

float kalman1DOutput[]={0,0}; //Initialize the output of the filter 

//function that calculates the predicted angle and uncertainty using the Kalman equations
void kalman_1d(float kalmanState, float kalmanUncertainty, float kalmanInput, float kalmanMeasurement){
  kalmanState = kalmanState + 0.004*kalmanInput;
  kalmanUncertainty = kalmanUncertainty + 0.004*0.004*4*4;
  float kalmanGain = kalmanUncertainty*1/(1*kalmanUncertainty + 3*3);
  kalmanState = kalmanState + kalmanGain*(kalmanMeasurement-kalmanState);
  kalmanUncertainty = (1 - kalmanGain)*kalmanUncertainty;
  //kalman filter output
  kalman1DOutput[0] = kalmanState;
  kalman1DOutput[1] = kalmanUncertainty;
}

void gyro_signals(void){
  Wire.beginTransmission(0x68); //Start I2C communication 
  
  //Switch on the low pass filter
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  //Configure accelerometer output
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  //Pull the accelerometer measurement from sensor
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t accXLSB = Wire.read() << 8 | Wire.read();
  int16_t accYLSB = Wire.read() << 8 | Wire.read();
  int16_t accZLSB = Wire.read() << 8 | Wire.read();

  //Configure the gyroscope output and pull rotation rate measurements from the sensor
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); //Set sensitivity scale factor
  Wire.write(0x8); //set senstivity scale factor
  Wire.endTransmission();
  //Access registers storing gyro measurements
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();
  int16_t gyroZ = Wire.read() << 8 | Wire.read();
  rateRoll = (float)gyroX/65.5;
  ratePitch = (float)gyroY/65.5;
  rateYaw = (float)gyroZ/65.5;

  //Convert the acceleratometer measurements into physical values
  accX = (float)accXLSB/4096 - 0.04;
  accY = (float)accYLSB/4096 + 0.01;
  accZ = (float)accZLSB/4096 + 0.18; 

  //Using geoemtry to calculate angle of roll and pitch in degrees
  angleRoll = atan(accY/sqrt(accX*accX+accZ*accZ))*1/(3.142/180);
  anglePitch = -atan(accX/sqrt(accY*accY+accZ*accZ))*1/(3.142/180);
}
void setup(){
  Serial.begin(115200);
  Wire.setClock(400000); //Set clock speed of I2C to 400kHz (comes from component spec)
  Wire.begin();
  delay(250); //must give the mpu6050 time to start
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); //Start the gyro in power mode
  Wire.write(0x00);
  Wire.endTransmission();

  //Calibrate the gyroscope to minimize offset errors
  for(calibrationNum = 0; calibrationNum < 2000; calibrationNum++){
    gyro_signals();
    offsetRoll += rateRoll;
    offsetPitch += ratePitch;
    offsetYaw += rateYaw;
    delay(1);
  }
  offsetRoll /= 2000;
  offsetPitch /= 2000;
  offsetYaw /= 2000;
}

void loop(){
  gyro_signals();

  /*
  Serial.print("Acc X [g] = ");
  Serial.print(accX);
  Serial.print(" Acc Y [g] = ");
  Serial.print(accY);
  Serial.print(" Acc Z [g] = ");
  Serial.println(accZ);
  */

  rateRoll -= offsetRoll;
  ratePitch -= offsetPitch;
  rateYaw -= offsetYaw;

  /*
  Serial.print("Roll rate [deg/s] = ");
  Serial.print(rateRoll);
  Serial.print(" Pitch rate [deg/s] = ");
  Serial.print(ratePitch);
  Serial.print(" Yaw Rate [deg/s] = ");
  Serial.println(rateYaw);
  delay(50);
  */

  /*
  Serial.print("Roll angle [deg] = ");
  Serial.print(angleRoll);
  Serial.print(" Pitch angle [deg] = ");
  Serial.println(anglePitch);
  */
  Serial.print(-10); // To freeze the lower limit
  Serial.print(" ");
  Serial.print(10); // To freeze the upper limit
  Serial.print(" ");

  //Start the Kalman Filter
  kalman_1d(kalmanAngleRoll, kalmanUncertaintyAngleRoll, rateRoll, angleRoll); //Roll
  kalmanAngleRoll = kalman1DOutput[0];
  kalmanUncertaintyAngleRoll = kalman1DOutput[1];
  kalman_1d(kalmanAnglePitch, kalmanUncertaintyAnglePitch, ratePitch, anglePitch); //Pitch
  kalmanAnglePitch = kalman1DOutput[0];
  kalmanUncertaintyAnglePitch = kalman1DOutput[1];

  /*
  //Serial Monitor View
  Serial.print("Roll angle [deg] = ");
  Serial.print(kalmanAngleRoll);
  Serial.print(" Pitch angle [deg] = ");
  Serial.println(kalmanAnglePitch);
  */

  //Serial Plotter View
  Serial.print(kalmanAngleRoll);
  Serial.print(" ");
  Serial.println(kalmanAnglePitch);
}
