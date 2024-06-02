#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu; //create object "mpu"

//Motor direction and step pins
#define dirPinR 2
#define stepPinR 3
#define dirPinL 4
#define stepPinL 5

#define stepsPerRevolution 200

//Offset values
#define accXOffset 0.32
#define accYOffset -0.19
#define accZOffset 7.76 
#define gyroXOffset -0.06 
#define gyroYOffset -0.03 
#define gyroZOffset -0.03 

//PID scalar constants
float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;

float setPoint  = 0.0; //Desired angle (upright)

//global variables necessary for PID control system (for later calculations)
float currentAngle, error, prevError, proportional, integral, derivative, motorSpeedL, motorSpeedR;

void setup() {
  Serial.begin(9600);
  Wire.begin(); //Initializes the I2C bus as a master

  //mpu.initialize();
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  //motor control pins
  pinMode(dirPinR, OUTPUT);
  pinMode(stepPinR, OUTPUT);
  pinMode(dirPinL, OUTPUT);
  pinMode(stepPinL, OUTPUT);


}

void loop() {

  // reading IMU data
  sensors_event_t a, g, temp;

  mpu.getEvent(&a, &g, &temp); //read raw acceleration and gyro measurements (dereference)

  // Display data
  
  Serial.print("a/g:\t");
  Serial.print(a.acceleration.x - accXOffset); Serial.print("\t");
  Serial.print(a.acceleration.y - accYOffset); Serial.print("\t");
  Serial.print(a.acceleration.z - accZOffset); Serial.print("\t");
  Serial.print(g.gyro.x - gyroXOffset); Serial.print("\t");
  Serial.print(g.gyro.y - gyroYOffset); Serial.print("\t");
  Serial.println(g.gyro.z - gyroZOffset);

  delay(100);
  

  //calculate currentAngle (simplified example: currentAngle = atan(ay, az) * 180 / PI;)

  //PID controller calculations
  error = currentAngle - setPoint; //or setPoint - currentAngle (whichever gives you the correct interpretation of positive/negative)
  proportional = error * Kp;
  integral += error * Ki; //integral = integral + error * Ki
  derivative = (error - prevError) * Kd;

  //Output (left and right motor speed)
  motorSpeedR =  proportional + integral + derivative;
  motorSpeedL = proportional + integral + derivative;

  prevError = error; //important to set prevError so on the next loop it remembers the previous error
}
