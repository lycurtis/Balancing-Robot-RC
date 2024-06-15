#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>
#include <SPI.h>
#include <stdlib.h>
#include <string.h>

#define CE_PIN 7
#define CSN_PIN 9
#define SS_K64F 10 // Use pin 10 for K-64F SS

RF24 radio(CE_PIN, CSN_PIN);

// Define the addresses for the nRF24L01 communication
const byte address[6] = "00001";

char buff[64];
volatile byte indx;
volatile boolean process;

float setPointL = 0.0;
float setPointR = 0.0;
int joyL = 0;
int joyR = 0;

void setup(void) {
  Serial.begin(115200);
  pinMode(MISO, OUTPUT); // Have to send on master in so it set as output
  SPCR |= _BV(SPE); // Turn on SPI in slave mode
  indx = 0; // Buffer empty
  process = false;
  SPI.attachInterrupt(); // Turn on interrupt

  // Initialize the nRF24L01 radio
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  // Set SS pin for K-64F as output and deselect it
  pinMode(SS_K64F, OUTPUT);
  digitalWrite(SS_K64F, HIGH);

  // Set CSN_PIN (pin 9) as output for nRF24L01
  pinMode(CSN_PIN, OUTPUT);
  digitalWrite(CSN_PIN, HIGH);
}

ISR(SPI_STC_vect) // SPI interrupt routine
{
  byte c = SPDR; // Read byte from SPI Data Register

  if (indx < sizeof(buff) - 1) {
    buff[indx++] = c; // Save data in the next index in the array buff
    if (c == '\n') {
      buff[indx - 1] = 0; // Replace newline ('\n') with end of string
      process = true;
    }
  }
}

void loop(void) {
  if (process) {
    process = false; // Reset the process

    // Ensure buffer is null-terminated
    buff[indx] = '\0';

    //Print the buffer content
    Serial.print("Buffer content: ");
    Serial.println(buff);

    //Print each character received
    Serial.print("Received characters: ");
    for (int i = 0; i < indx; i++) {
      Serial.print(buff[i]);
      Serial.print(" ");
    }
    Serial.println();

    // Parse the labeled data
    char* joyLStr = strstr(buff, "joyL: ");
    char* joyRStr = strstr(buff, "joyR: ");
    if (joyLStr != NULL && joyRStr != NULL) {
      joyLStr += 6; // Move past "joyL: "
      joyRStr += 6; // Move past "joyR: "

      joyL = atoi(joyLStr); // Convert joyL value
      joyR = atoi(joyRStr); // Convert joyR value

      // Debugging information
      Serial.print("Parsed joyL: ");
      Serial.println(joyL);
      Serial.print("Parsed joyR: ");
      Serial.println(joyR);

      // Determine the setPointL variable based on the joyL integer value
      if (joyL > -16900 && joyL < -16000) {
        setPointL = 0.0;
      } else if (joyL == -1) {
        setPointL = -20.0;
      } else if (joyL >= 0 && joyL < 150) {
        setPointL = 20.0;
      } else {
        setPointL = 0.0; // Default case 
      }

      
      // Determine the setPointR variable based on the joyR integer value
      if (joyR > -16900 && joyR < -16000) {
        setPointR = 0.0;
      } else if (joyR == -1) {
        setPointR = -20.0;
      } else if (joyR >= 0 && joyR < 150) {
        setPointR = 20.0;
      } else {
        setPointR = 0.0; // Default case
      }

      // Print the setPoint values
      Serial.print("setPointL: ");
      Serial.print(setPointL);
      Serial.print(" | setPointR: ");
      Serial.println(setPointR);

      // Send the setPoint values via RF24
      sendSetPoint(setPointL, setPointR);
    } else {
      Serial.println("Labeled data not found in buffer.");
    }

    indx = 0; // Reset buffer to zero
  }
}

// Function to send setPoint values
void sendSetPoint(float valueL, float valueR) {
  float setPoints[2] = {valueL, valueR};

  // Deselect K-64F
  digitalWrite(SS_K64F, HIGH);

  // Send data with nRF24L01
  radio.write(&setPoints, sizeof(setPoints));

  // reselect K-64F here if needed
  //digitalWrite(SS_K64F, LOW);
}
// functionality to send data using the RF24 library not fully included in this code as it was causing errors with SPI data communication with K64F and Processor Expert.
//i.e. radio.write/read(), radio.openWritingPipe(addresss), etc. 
