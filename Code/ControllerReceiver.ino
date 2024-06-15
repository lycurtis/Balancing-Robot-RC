//This file is to be combined onto the Teensy 4.1 in combination with the Balance_Robot.ino to receive the processed setPoint from the joystick input.

#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>

#include <SPI.h>
#include <stdlib.h>
#include <string.h>

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
}

ISR(SPI_STC_vect) // SPI interrupt routine
{
  byte c = SPDR; // Read byte from SPI Data Register

  if (indx < sizeof(buff) - 1) {
    buff[indx++] = c; // Save data in the next index in the array buff
    if (c == '\n') {
      buff[indx - 1] = 0; // Replace newline ('\n') with end of string (0)
      process = true;
    }
  }
}

void loop(void) {
  if (process) {
    process = false; // Reset the process

    // Ensure buffer is null-terminated
    buff[indx] = '\0';

    // Print the buffer content for debugging
    //Serial.print("Buffer content: ");
    //Serial.println(buff);

    //Print each character received
    /*Serial.print("Received characters: ");
    for (int i = 0; i < indx; i++) {
      Serial.print(buff[i]);
      Serial.print(" ");
    }
    Serial.println();*/

    // Parse the labeled data
    char* joyLStr = strstr(buff, "joyL: ");
    char* joyRStr = strstr(buff, "joyR: ");
    if (joyLStr != NULL && joyRStr != NULL) {
      joyLStr += 6; // Move past "joyL: "
      joyRStr += 6; // Move past "joyR: "

      joyL = atoi(joyLStr); // Convert joyL value
      joyR = atoi(joyRStr); // Convert joyR value

      /*
      Serial.print("Parsed joyL: ");
      Serial.println(joyL);
      Serial.print("Parsed joyR: ");
      Serial.println(joyR);
      */
      
      /*
      // Determine the setPointL variable based on the joyL integer value
      if (joyL > -16900 && joyL < -16000) {
        setPointL = 0.0;
      } else if (joyL == -1) {
        setPointL = -20.0;
      } else if (joyL >= 0 && joyL < 150) {
        setPointL = 20.0;
      } else {
        setPointL = 0.0; // Default case if none of the conditions are met
      }

      // Determine the setPointR variable based on the joyR integer value
      if (joyR > -16900 && joyR < -16000) {
        setPointR = 0.0;
      } else if (joyR == -1) {
        setPointR = -20.0;
      } else if (joyR >= 0 && joyR < 150) {
        setPointR = 20.0;
      } else {
        setPointR = 0.0; // Default case if none of the conditions are met
      }

      // Print the setPoint values for debugging

      
      Serial.print("setPointL: ");
      Serial.print(setPointL);
      Serial.print(" | setPointR: ");
      Serial.println(setPointR);
      */

    } else {
      Serial.println("Labeled data not found in buffer.");
    }

    indx = 0; // Reset buffer to zero
  }
}

//void sendSetPoint(float valueL, float valueR) {
//}
