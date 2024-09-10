#include <Wire.h>

float rateRoll, ratePitch, rateYaw;

float accX, accY, accZ;
float angleRoll, anglePitch;

void gyro_signals(void){
  //Switch on the low pass filter
  Wire.beginTransmission(0x68);
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
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
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
  Serial.begin(9600);
  Wire.setClock(400000);
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
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

  Serial.print("Roll angle [deg] = ");
  Serial.print(angleRoll);
  Serial.print(" Pitch angle [deg] = ");
  Serial.println(anglePitch);

}
