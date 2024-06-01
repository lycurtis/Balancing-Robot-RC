// Define the connections for the right motor
#define dirPinRight 2
#define stepPinRight 3

// Define the connections for the left motor
#define dirPinLeft 4
#define stepPinLeft 5

// Define the steps per revolution (200 for standard NEMA 17 with 1.8° step angle)
#define stepsPerRevolution 200

void setup() {
  // Set the motor control pins as outputs
  pinMode(stepPinRight, OUTPUT);
  pinMode(dirPinRight, OUTPUT);
  pinMode(stepPinLeft, OUTPUT);
  pinMode(dirPinLeft, OUTPUT);

  // Set initial direction for both motors
  digitalWrite(dirPinRight, HIGH); // Set the right motor direction to forward
  digitalWrite(dirPinLeft, LOW);  // Set the left motor direction to forward
}

void loop() {
  // Spin the right motor
  for (int i = 0; i < stepsPerRevolution; i++) {
    digitalWrite(stepPinRight, HIGH);
    delayMicroseconds(1000); // Adjust delay to control speed
    digitalWrite(stepPinRight, LOW);
    delayMicroseconds(1000); // Adjust delay to control speed
  }

  // Spin the left motor
  for (int i = 0; i < stepsPerRevolution; i++) {
    digitalWrite(stepPinLeft, HIGH);
    delayMicroseconds(1000); // Adjust delay to control speed
    digitalWrite(stepPinLeft, LOW);
    delayMicroseconds(1000); // Adjust delay to control speed
  }
}
