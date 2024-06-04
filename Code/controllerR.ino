#include "Timer.h"

#define TOTAL_TASK 2
#define PERIOD_GCD 10
#define PERIOD_FBCONTROL 10
#define PERIOD_RCONTROL 10
#define led1 5
#define led2 6
#define led3 4
#define led4 3

#define led5 2 //right 
#define led6 9 //mostright
#define led7 10 //left

// Define the analog pins for the joystick
const int joystickLPin = A0;
const int joystickRPin = A1;

const int fullThrottleTH = 1022;
const int fullReverseTH = 1;

int joyL;
int joyR;


typedef struct task{
  unsigned short period;
  unsigned short timeElapsed;

  void(*tick) (void);
};

static struct task gTaskSet[TOTAL_TASK];

void initializeTask(void){
  gTaskSet[0].period = PERIOD_FBCONTROL;
  gTaskSet[0].timeElapsed = 0;
  gTaskSet[0].tick = tickFB;
  
  
  gTaskSet[1].period = PERIOD_RCONTROL;
  gTaskSet[1].timeElapsed = 0;
  gTaskSet[1].tick = tickR;
  
  
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

void tickFB(void){ //528 rest value (0 down, 1023 up)
  switch(lState){
    case INIT:
      lState = READ;
    break;

    case READ:
      if(joyL > 529 && joyL < fullThrottleTH){
        lState = UP;
      }
      else if(joyL >= fullThrottleTH){
        lState = UPMAX;
      }
      else if(joyL < 527 && joyL > fullReverseTH){
        lState = DOWN;
      }
      else if(joyL <= fullReverseTH){
        lState = DOWNMAX;
      }
      else if (joyL >= 527 && joyL <=529){
        lState = READ;
      }
    break;

    case UP:
      if(joyL > 529 && joyL < fullThrottleTH){
        lState = UP;
      }
      else if(joyL >= fullThrottleTH){
        lState = UPMAX;
      }
      else if(joyL < 527 && joyL > fullReverseTH){
        lState = DOWN;
      }
      else if(joyL <= fullReverseTH){
        lState = DOWNMAX;
      }
      else if (joyL >= 527 && joyL <=529){
        lState = READ;
      }
    break;

    case UPMAX:
      if(joyL > 529 && joyL < fullThrottleTH){
        lState = UP;
      }
      else if(joyL >= fullThrottleTH){
        lState = UPMAX;
      }
      else if(joyL < 527 && joyL > fullReverseTH){
        lState = DOWN;
      }
      else if(joyL <= fullReverseTH){
        lState = DOWNMAX;
      }
      else if (joyL >= 527 && joyL <=529){
        lState = READ;
      }
    break;

    case DOWN:
      if(joyL > 529 && joyL < fullThrottleTH){
        lState = UP;
      }
      else if(joyL >= fullThrottleTH){
        lState = UPMAX;
      }
      else if(joyL < 527 && joyL > fullReverseTH){
        lState = DOWN;
      }
      else if(joyL <= fullReverseTH){
        lState = DOWNMAX;
      }
      else if (joyL >= 527 && joyL <=529){
        lState = READ;
      }
    break;

    case DOWNMAX:
      if(joyL > 529 && joyL < fullThrottleTH){
        lState = UP;
      }
      else if(joyL >= fullThrottleTH){
        lState = UPMAX;
      }
      else if(joyL < 527 && joyL > fullReverseTH){
        lState = DOWN;
      }
      else if(joyL <= fullReverseTH){
        lState = DOWNMAX;
      }
      else if (joyL >= 527 && joyL <=529){
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
      digitalWrite(A4, LOW);
    break;

    case RIGHT:
      
      digitalWrite(led5, HIGH);
      digitalWrite(led6, LOW);
      digitalWrite(led7, LOW);
      digitalWrite(A4, LOW);
      

    break;

    case RIGHTMAX:
      digitalWrite(led5, HIGH);
      digitalWrite(led6, HIGH);
      digitalWrite(led7, LOW);
      digitalWrite(A4, LOW);
    break;

    case LEFT:
      
      digitalWrite(led5, LOW);
      digitalWrite(led6, LOW);
      digitalWrite(led7, HIGH);
      digitalWrite(A4, LOW);
      
    break;

    case LEFTMAX:
      
      digitalWrite(led5, LOW);
      digitalWrite(led6, LOW);
      digitalWrite(led7, HIGH);
      digitalWrite(A4, HIGH);

    break;
  }
}

void setup() {
  // Initialize serial communication at 9600 bits per second
  Serial.begin(9600);
  TimerSet(PERIOD_GCD);
  TimerOn();
  initializeTask();

  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);

  pinMode(led5, OUTPUT);
  pinMode(led6, OUTPUT);
  pinMode(led7, OUTPUT); //left
  pinMode(A4, OUTPUT); //most left
}

void loop() {
  // Read the raw values from the joystick
  joyL = analogRead(joystickLPin);
  joyR = analogRead(joystickRPin);

  // Print the raw values to the Serial Monitor

  scheduleTask();
  while(!TimerFlag){}
  TimerFlag = 0;
}
