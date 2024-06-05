//#include <TimerOne.h>
#include "Timer.h"
#include <SPI.h>
#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>


// Define the nRF24L01 CE and CSN pins
#define CE_PIN 9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);
// Define the addresses for the nRF24L01 communication
const byte address[6] = "00001";

#define led1 2
#define led2 3
#define led3 4
#define led4 5
#define led5 6 //right 
#define led6 7 //mostright
#define led7 8//left
#define led8 9

#define TOTAL_TASK 3
#define PERIOD_GCD 10
#define PERIOD_FBCONTROL 10
#define PERIOD_RCONTROL 10

const int fullThrottleTH = -1; //analog range
const int fullReverseTH = 150;

int joyL;
int joyR;

// Buffer to hold incoming data from SPI, needs to be volatile
char buff[32]; //
volatile boolean checkBuff = false;
volatile byte indx = 0; //tracks the buffer index

typedef struct task {
  unsigned short period;
  unsigned short timeElapsed;

  void(*tick) (void);
};

static struct task gTaskSet[TOTAL_TASK];

ISR (SPI_STC_vect) // SPI interrupt routine 
{ 
   byte c = SPDR; // read byte from SPI Data Register
   
   if (indx < sizeof(buff)) {
      buff[indx++] = c; // save data in the next index in the array buff
      if (c == '\n') { 
        buff[indx - 1] = 0; // replace newline ('\n') with end of string (0)
        checkBuff = true;
      }
   }   
}

void initializeTask(void){
  gTaskSet[0].period = PERIOD_FBCONTROL;
  gTaskSet[0].timeElapsed = 0;
  gTaskSet[0].tick = tickFB;
  
  
  gTaskSet[1].period = PERIOD_RCONTROL;
  gTaskSet[1].timeElapsed = 0;
  gTaskSet[1].tick = tickR;

  gTaskSet[2].period = PERIOD_RCONTROL;
  gTaskSet[2].timeElapsed = 0;
  gTaskSet[2].tick = tickR;
   
}

void scheduleTask(){
  for(int i = 0; i < TOTAL_TASK; i++){
    gTaskSet[i].timeElapsed += PERIOD_GCD;
    if(gTaskSet[i].timeElapsed >= gTaskSet[i].period){
      gTaskSet[i].tick();
      gTaskSet[i].timeElapsed = 0;
    }
  }
}

enum STATES_FBCONTROL {INIT, READ, UP, UPMAX, DOWN, DOWNMAX} lState = INIT;
enum STATES_RCONTROL {INIT1, READ1, RIGHT, RIGHTMAX, LEFT, LEFTMAX} rState = INIT1;

void tickFB(void){ //[-16900, -16000] rest value (0 down, -1 up)
//transitions
  switch(lState){
    case INIT:
      lState = READ;
    break;

    case READ:
      if(joyL > -16200 && joyL < fullThrottleTH){
        lState = UP;
      }
      else if(joyL == fullThrottleTH){
        lState = UPMAX;
      }
      else if((joyL < -16900 && joyL > -30000) || (joyL < 21000 && joyL > fullReverseTH)){
        lState = DOWN;
      }
      else if(joyL <= fullReverseTH){
        lState = DOWNMAX;
      }
      else if (joyL >= -16999 && joyL <= -16000){
        lState = READ;
      }
    break;

    case UP:
      if(joyL > -16200 && joyL < fullThrottleTH){
        lState = UP;
      }
      else if(joyL == fullThrottleTH){
        lState = UPMAX;
      }
      else if((joyL < -16900 && joyL > -30000) || (joyL < 21000 && joyL > fullReverseTH)){
        lState = DOWN;
      }
      else if(joyL <= fullReverseTH){
        lState = DOWNMAX;
      }
      else if (joyL >= -16999 && joyL <= -16000){
        lState = READ;
      }
    break;

    case UPMAX:
      if(joyL > -16200 && joyL < fullThrottleTH){
        lState = UP;
      }
      else if(joyL == fullThrottleTH){
        lState = UPMAX;
      }
      else if((joyL < -16900 && joyL > -30000) || (joyL < 21000 && joyL > fullReverseTH)){
        lState = DOWN;
      }
      else if(joyL <= fullReverseTH){
        lState = DOWNMAX;
      }
      else if (joyL >= -16999 && joyL <= -16000){
        lState = READ;
      }
    break;

    case DOWN:
      if(joyL > -16200 && joyL < fullThrottleTH){
        lState = UP;
      }
      else if(joyL == fullThrottleTH){
        lState = UPMAX;
      }
      else if((joyL < -16900 && joyL > -30000) || (joyL < 21000 && joyL > fullReverseTH)){
        lState = DOWN;
      }
      else if(joyL <= fullReverseTH){
        lState = DOWNMAX;
      }
      else if (joyL >= -16999 && joyL <= -16000){
        lState = READ;
      }
    break;

    case DOWNMAX:
      if(joyL > -16200 && joyL < fullThrottleTH){
        lState = UP;
      }
      else if(joyL == fullThrottleTH){
        lState = UPMAX;
      }
      else if((joyL < -16900 && joyL > -30000) || (joyL < 21000 && joyL > fullReverseTH)){
        lState = DOWN;
      }
      else if(joyL <= fullReverseTH){
        lState = DOWNMAX;
      }
      else if (joyL >= -16999 && joyL <= -16000){
        lState = READ;
      }
    break;

    default:
      lState = READ;
    break;
  }

  //actions
  switch(lState){
    case INIT:
    break;

    case READ:
      
      digitalWrite(led1, LOW);
      digitalWrite(led2, LOW);
      digitalWrite(led3, LOW);
      digitalWrite(led4, LOW);
    break;

    case UP:
      
      digitalWrite(led1, HIGH);
      digitalWrite(led2, LOW);
      digitalWrite(led3, LOW);
      digitalWrite(led4, LOW);
      

    break;

    case UPMAX:
      digitalWrite(led1, HIGH);
      digitalWrite(led2, HIGH);
      digitalWrite(led3, LOW);
      digitalWrite(led4, LOW);
    break;

    case DOWN:
      
      digitalWrite(led1, LOW);
      digitalWrite(led2, LOW);
      digitalWrite(led3, HIGH);
      digitalWrite(led4, LOW);
      
    break;

    case DOWNMAX:
      
      digitalWrite(led1, LOW);
      digitalWrite(led2, LOW);
      digitalWrite(led3, HIGH);
      digitalWrite(led4, HIGH);

    break;
  }
}

void tickR(void){ //501 rest val (left 0, right 1023)
//transitions
  switch(rState){
    case INIT1:
      rState = READ1;
    break;

    case READ1:
      if(joyR > 502 && joyR < fullThrottleTH){
        rState = RIGHT;
      }
      else if(joyR >= fullThrottleTH){
        rState = RIGHTMAX;
      }
      else if(joyR < 500 && joyR > fullReverseTH){
        rState = LEFT;
      }
      else if(joyR <= fullReverseTH){
        rState = LEFTMAX;
      }
      else if (joyR >= 500 && joyR <=502){
        rState = READ1;
      }
    break;

    case RIGHT:
      if(joyR > 502 && joyR < fullThrottleTH){
        rState = RIGHT;
      }
      else if(joyR >= fullThrottleTH){
        rState = RIGHTMAX;
      }
      else if(joyR < 500 && joyR > fullReverseTH){
        rState = LEFT;
      }
      else if(joyR <= fullReverseTH){
        rState = LEFTMAX;
      }
      else if (joyR >= 500 && joyR <=502){
        rState = READ1;
      }
    break;

    case RIGHTMAX:
      if(joyR > 502 && joyR < fullThrottleTH){
        rState = RIGHT;
      }
      else if(joyR >= fullThrottleTH){
        rState = RIGHTMAX;
      }
      else if(joyR < 500 && joyR > fullReverseTH){
        rState = LEFT;
      }
      else if(joyR <= fullReverseTH){
        rState = LEFTMAX;
      }
      else if (joyR >= 500 && joyR <=502){
        rState = READ1;
      }
    break;

    case LEFT:
      if(joyR > 502 && joyR < fullThrottleTH){
        rState = RIGHT;
      }
      else if(joyR >= fullThrottleTH){
        rState = RIGHTMAX;
      }
      else if(joyR < 500 && joyR > fullReverseTH){
        rState = LEFT;
      }
      else if(joyR <= fullReverseTH){
        rState = LEFTMAX;
      }
      else if (joyR >= 500 && joyR <=502){
        rState = READ1;
      }
    break;

    case LEFTMAX:
      if(joyR > 502 && joyR < fullThrottleTH){
        rState = RIGHT;
      }
      else if(joyR >= fullThrottleTH){
        rState = RIGHTMAX;
      }
      else if(joyR < 500 && joyR > fullReverseTH){
        rState = LEFT;
      }
      else if(joyR <= fullReverseTH){
        rState = LEFTMAX;
      }
      else if (joyR >= 500 && joyR <=502){
        rState = READ1;
      }
    break;

    default:
      rState = READ1;
    break;
  }

  //actions
  switch(rState){
    case INIT1:
    break;

    case READ1:
      
      digitalWrite(led5, LOW);
      digitalWrite(led6, LOW);
      digitalWrite(led7, LOW);
      digitalWrite(led8, LOW);
    break;

    case RIGHT:
      
      digitalWrite(led5, HIGH);
      digitalWrite(led6, LOW);
      digitalWrite(led7, LOW);
      digitalWrite(led8, LOW);
      

    break;

    case RIGHTMAX:
      digitalWrite(led5, HIGH);
      digitalWrite(led6, HIGH);
      digitalWrite(led7, LOW);
      digitalWrite(led8, LOW);
    break;

    case LEFT:
      
      digitalWrite(led5, LOW);
      digitalWrite(led6, LOW);
      digitalWrite(led7, HIGH);
      digitalWrite(led8, LOW);
      
    break;

    case LEFTMAX:
      
      digitalWrite(led5, LOW);
      digitalWrite(led6, LOW);
      digitalWrite(led7, HIGH);
      digitalWrite(led8, HIGH);

    break;
  }
}

void TimerISR() {
  scheduleTask();
}

void setup(void) {
    Serial.begin(115200);

    // Initialize SPI as a slave
    pinMode(MISO, OUTPUT);
    //SPI.begin();
    SPCR |= _BV(SPE); // Enable SPI in slave mode
    indx = 0; // Buffer empty
    checkBuff = 0;
    SPI.attachInterrupt(); // Enable SPI interrupt

    pinMode(CE_PIN, OUTPUT);

    // Initialize the nRF24L01
    radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN); //Min value for now
    radio.stopListening(); //standby mode

    pinMode(led1, OUTPUT);
    pinMode(led2, OUTPUT);
    pinMode(led3, OUTPUT);
    pinMode(led4, OUTPUT);
    pinMode(led5, OUTPUT);
    pinMode(led6, OUTPUT);
    pinMode(led7, OUTPUT);
    pinMode(led8, OUTPUT);

    //Timer1.initialize(PERIOD_GCD); //initialize timer with specific period
    //Timer1.attachInterrupt(TimerISR);
    TimerSet(PERIOD_GCD);
    TimerOn();
    //Initialize the tasks
    initializeTask();
}

void loop(void) {
  if (checkBuff) {
    noInterrupts(); //Disable interrupts
    checkBuff = false; // Reset the flag variable once it knows check buff has received both joystick datasets

    //extracts the upper and lower 16 bits from the buffer into variables.
    joyL = buff[0];
    joyR = buff[1];

    interrupts(); //enable interrutps

    //send the data using the nRF24L01
    /*
    radio.stopListening(); //standby mode
    digitalWrite(CE_PIN, HIGH); //CE pin high to start transmission.
    radio.write(&joyL, sizeof(joyL)); //true or false
    radio.write(&joyR, sizeof(joyR));
    digitalWrite(CE_PIN, LOW); //CE pin to low to stop the transmission.
    */

    //Print the buffer
    Serial.print("Sent data: ");
    Serial.print(joyL);
    Serial.print(", ");
    Serial.println(joyR);

    indx = 0;

    //radio.startListening();
    }
    scheduleTask(); //task scheduling for the state machines

    noInterrupts();
    while(!TimerFlag) {}
    TimerFlag = 0;
    interrupts();
}
