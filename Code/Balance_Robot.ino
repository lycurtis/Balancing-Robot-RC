#include <MPU6050.h>
#include <Wire.h>

MPU6050 mpu; //create object "mpu"

//Motor direction and step pins
#define dirPinR 2; 
#define stepPinR 3;
#define dirPinL 4;
#define stepPinL 5;

#define stepsPerRevolution 200

//PID scalar constants
float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;

float setPoint  = 0.0 //Desired angle (upright)

//global variables necessary for PID control system (for later calculations)
float currentAngle, error, prevError, proportional, integral, derivative, motorSpeedL, motorSpeedR;

void setup() {
  Serial.begin(9600);
  Wire.begin(); //Initializes the I2C bus as a master
  mpu.initialize();

  //motor control pins
  pinMode(dirPinR, OUTPUT);
  pinMode(stepPinR, OUTPUT);
  pinMode(dirPinL, OUTPUT);
  pinMode(stepPinL, OUTPUT);
}

void loop() {

  // reading IMU data
  int16_t ax, ay, az; //accerlation addresses
  int16_t gx, gy, gz; //gyroscope addresses

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //read raw accerlation and gyro measurements (dereference)

  // Display data
  Serial.print("a/g:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);

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

