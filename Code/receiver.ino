#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define ledForwardPin 2
#define ledBackwardPin 3
#define ledRightPin 4
#define ledLeftPin 5

RF24 radio(9,10);
const byte address[6] = "00001";

char dataReceived[4];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  pinMode(ledForwardPin, OUTPUT);
  pinMode(ledBackwardPin, OUTPUT);
  pinMode(ledRightPin, OUTPUT);
  pinMode(ledLeftPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(radio.available()){
    radio.read(&dataReceived, sizeof(dataReceived));
    Serial.print("Received: ");
    Serial.print(dataReceived[0]);
    Serial.print(dataReceived[1]);
    Serial.print(dataReceived[2]);
    Serial.print(dataReceived[3]);
    Serial.println();
  }
  if (dataReceived[0] == 1 && dataReceived[1] == 0) {
    digitalWrite(ledForwardPin, HIGH);
    digitalWrite(ledBackwardPin, LOW);
  } else if (dataReceived[0] == 0 && dataReceived[1] == 1) {
    digitalWrite(ledForwardPin, LOW);
    digitalWrite(ledBackwardPin, HIGH);
  } else {
    digitalWrite(ledForwardPin, LOW);
    digitalWrite(ledBackwardPin, LOW);
  }

  if (dataReceived[2] == 1 && dataReceived[3] == 0) {
    digitalWrite(ledLeftPin, HIGH);
    digitalWrite(ledRightPin, LOW);
  } else if (dataReceived[2] == 0 && dataReceived[3] == 1) {
    digitalWrite(ledLeftPin, LOW);
    digitalWrite(ledRightPin, HIGH);
  } else {
    digitalWrite(ledLeftPin, LOW);
    digitalWrite(ledRightPin, LOW);
  }
}
