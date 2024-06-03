#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>
#include <SPI.h>

#define CE_PIN 9 // teensy 4.1
#define CSN_PIN 10 // teensy 4.1

RF24 radio(CE_PIN, CSN_PIN); // CE, CSN

const byte address[6] = "00001";

const int forwardLED1 = 32;
const int forwardLED2 = 31;
const int backwardLED1 = 30;
const int backwardLED2 = 29;
const int leftLED = 40;
const int rightLED = 41;

IntervalTimer pitchTimer;
IntervalTimer yawTimer; //rotation timer

void setup() {
  Serial.begin(115200);
  Serial.println("Starting receiver setup");

  pitchTimer.begin(joyX, 100000);
  yawTimer.begin(joyY, 100000);

  Serial.println("Initializing radio...");
  if (radio.begin()) {
    Serial.println("Radio initialized successfully");
    radio.printDetails(); // Print details for debugging
  } else {
    Serial.println("Radio initialization failed");
    while (true); // Halt execution if initialization fails
  }

  radio.setChannel(90); // Change channel to avoid interference
  radio.setPALevel(RF24_PA_LOW); // Set power level to minimum
  radio.openReadingPipe(0, address);
  radio.startListening();
  Serial.println("Receiver setup done");

  pinMode(forwardLED1, OUTPUT);
  pinMode(forwardLED2, OUTPUT);
  pinMode(backwardLED1, OUTPUT);
  pinMode(backwardLED2, OUTPUT);
  pinMode(leftLED, OUTPUT);
  pinMode(rightLED, OUTPUT);

  digitalWrite(forwardLED1, LOW);
  digitalWrite(forwardLED2, LOW);
  digitalWrite(backwardLED1, LOW);
  digitalWrite(backwardLED2, LOW);
  digitalWrite(leftLED, LOW);
  digitalWrite(rightLED, LOW);
}

enum STATES_joyY {IDLEY, FORWARD_MIN, FORWARD_MAX, BACKWARD_MIN, BACKWARD_MAX} jState = IDLEY;
enum STATES_joyX {IDLEX, LEFT_LED, RIGHT_LED} gState = IDLEX;

struct JoystickData {
  int forwardBackward;
  int leftRight;
}data;
    

void joyY(){
    radio.read(&data, sizeof(data));

    int fbValue = map(data.forwardBackward, 0, 1023, 0, 100);

    // Thresholds for joystick detection
    int thresholdUp = 60;
    int thresholdDown = 40;
    int secondLEDThresholdUp = 98; 
    int secondLEDThresholdDown = 2;

    // Forward/Backward LEDs
    //transitions
    switch (jState) {
      case IDLEY: //No actuation
        if (fbValue > thresholdUp) {
          jState = FORWARD_MIN;
        }
        else if (fbValue < thresholdDown) {
          jState = BACKWARD_MIN;
        }
        break;

      case FORWARD_MIN:
        if (fbValue > secondLEDThresholdUp) {
          jState = FORWARD_MAX;
        }
        else if (fbValue <= thresholdDown) {
          jState = IDLEY;
        }
        break;

      case FORWARD_MAX:
        if (fbValue <= secondLEDThresholdUp && fbValue > thresholdDown) {
          jState = FORWARD_MIN;
        }
        else if (fbValue <= thresholdUp) {
          jState = IDLEY;
        }
        break;
      
      case BACKWARD_MIN:
        if (fbValue <= secondLEDThresholdDown) {
          jState = BACKWARD_MAX;
        }
        else if (fbValue >= thresholdUp) {
          jState = IDLEY;
        }
        break;

      case BACKWARD_MAX:
        if (fbValue > secondLEDThresholdDown) {
          jState = BACKWARD_MIN;
        }
        else if (fbValue > thresholdDown) {
          jState = IDLEY;
        }
        break;
      default:
        jState = IDLEY;
        break;
    }
    //actions
    switch(gState) {
      case IDLEY:
        digitalWrite (forwardLED1, LOW);
        digitalWrite (forwardLED2, LOW);
        digitalWrite (backwardLED1, LOW);
        digitalWrite (backwardLED2, LOW);
        break;
      case FORWARD_MIN:
        digitalWrite (forwardLED1, HIGH);
        digitalWrite (forwardLED2, LOW);
        digitalWrite (backwardLED1, LOW);
        digitalWrite (backwardLED2, LOW);
        break;
      case FORWARD_MAX:
        digitalWrite (forwardLED1, HIGH);
        digitalWrite (forwardLED2, HIGH);
        digitalWrite (backwardLED1, LOW);
        digitalWrite (backwardLED2, LOW);
        break;
      case BACKWARD_MIN: 
        digitalWrite (forwardLED1, LOW);
        digitalWrite (forwardLED2, LOW);
        digitalWrite (backwardLED1, HIGH);
        digitalWrite (backwardLED2, LOW);
        break;
      case BACKWARD_MAX: 
        digitalWrite (forwardLED1, LOW);
        digitalWrite (forwardLED2, LOW);
        digitalWrite (backwardLED1, HIGH);
        digitalWrite (backwardLED2, HIGH);
        break;
    }
}


void joyX(){
    radio.read(&data, sizeof(data));

    int lrValue = map(data.leftRight, 0, 1023, 0, 100);

    //transitions
    switch(gState) {
      case IDLEX:
        if (lrValue >= 49 && lrValue <= 51) {
          gState = IDLEX;
        }
        else if(lrValue > 51){
          gState = RIGHT_LED:
        }
        else if (lrValue < 49) {
          gState = LEFT_LED;
        }
        break;
      case LEFT_LED:
        if (lrValue <= 51) {
          gState = LEFT_LED;
        }
        else {
          gState = IDLEX;
        }
        break;
      case RIGHT_LED:
        if (lrValue >= 51) {
          gState = RIGHT_LED;
        }
        else {
          gState = IDLEX;
        }
        break;
    }
    //actions
    switch(gState) {
      case LEFT_LED:
        digitalWrite(leftLED, HIGH);
        digitalWrite(rightLED, LOW);
      case RIGHT_LED:
        digitalWrite(rightLED, HIGH);
        digitalWrite(leftLED, LOW);
    }
}

void loop() {
}
