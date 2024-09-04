#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define ledForwardPin 2
#define ledBackwardPin 3
#define ledRightPin 4
#define ledLeftPin 5 

RF24 radio(9, 10); //CE and CSN
const byte address[6] = "00001";

volatile int joyFB = 0;
volatile int joyLR = 0;

volatile char dataToSend[4] = {0, 0, 0, 0}; //UP DOWN LEFT RIGHT

void setup() {
  Serial.begin(115200);

  //Timer1 Initialize
  TCCR1A = 0; //reset entire TCCR1A (Timer/Counter Control Register A) to 0
  TCCR1B = 0; //reset entire TCCR1B (Timer/Counter Control Register B) to 0
  TCNT1 = 0;
  //Set the prescaler to the deseired value by changing the CS10 CS11 and CS12 bits
  TCCR1B |= (1 << CS12) | (1 << WGM12); //Set CS12 to 1 so we get prescaler 256
  //enable compare match mode on register A
  TIMSK1 |= B00000010; //Set OCIE1A to 1 so we enable compare match A
  //OCR1A = (16e6 / (256*10)) - 1 = 6249 for 100ms interval
  OCR1A = 6249; //Set the value of register A 
  sei(); //enable back the interrupt 

  //Transmitter
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  //Led Pins
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

  //Joysticks
  pinMode(A0, INPUT); //forward/backward
  pinMode(A1, INPUT); //left/right
}

void tickJoy(void){
  if (joyFB > 512 + 50) { // Forward
    digitalWrite(ledForwardPin, HIGH);
    digitalWrite(ledBackwardPin, LOW);
    dataToSend[0] = 1; // UP
    dataToSend[1] = 0; // DOWN
  } else if (joyFB < 512 - 50) { // Backward
    digitalWrite(ledForwardPin, LOW);
    digitalWrite(ledBackwardPin, HIGH);
    dataToSend[0] = 0; // UP
    dataToSend[1] = 1; // DOWN
  } else { // Center/Default
    digitalWrite(ledForwardPin, LOW);
    digitalWrite(ledBackwardPin, LOW);
    dataToSend[0] = 0; // UP
    dataToSend[1] = 0; // DOWN
  }

  if (joyLR > 512 + 50) { // Right
    digitalWrite(ledRightPin, HIGH);
    digitalWrite(ledLeftPin, LOW);
    dataToSend[2] = 0; // LEFT
    dataToSend[3] = 1; // RIGHT
  } else if (joyLR < 512 - 50) { // Left
    digitalWrite(ledRightPin, LOW);
    digitalWrite(ledLeftPin, HIGH);
    dataToSend[2] = 1; // LEFT
    dataToSend[3] = 0; // RIGHT
  } else { // Center/Default
    digitalWrite(ledRightPin, LOW);
    digitalWrite(ledLeftPin, LOW);
    dataToSend[2] = 0; // LEFT
    dataToSend[3] = 0; // RIGHT
  }
}
void loop() {
  tickJoy();

  //radio.write(&dataToSend, sizeof(dataToSend));
  int success = radio.write(&dataToSend, sizeof(dataToSend));
  if (success) {
    Serial.println("Data sent successfully");
  } else {
    Serial.println("Data sending failed");
  }

}

//keep ISR short and minimal 
ISR(TIMER1_COMPA_vect) {
  TCNT1 = 0;
  joyFB = analogRead(A0);
  joyLR = analogRead(A1);
}
