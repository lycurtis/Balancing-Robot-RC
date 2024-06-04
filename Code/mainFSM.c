//#include "MK64F12.h"
#include "fsl_device_registers.h"
#include <stdint.h>

/*////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////*/
//GLOBAL VARIABLES AND DEFINITIONS
#define PIT_PERIOD_US 10000U //10 ms period (10,000 us = 10ms)
//define joystick pins

float setPoint = 0.0;

const int fullThrottleTH = 1022;
const int fullReverseTH = 1;

int joyL;
int joyR;

/*////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////*/
// Define states for state machine 1 (Task 1)
typedef enum STATES_FBCONTROL {INIT, READ, UP, UPMAX, DOWN, DOWNMAX} STATES_FBCONTROL;
volatile STATES_FBCONTROL lState = INIT; //current state
volatile bool timerInterruptFlag1 = false; // Interrupt flags for state machine 1
volatile bool gpioInterruptFlag1 = false;

// ISR for timer interrupt for state machine 1
void PIT0_IRQHandler(void) {
    PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK; // Clear interrupt flag
    timerInterruptFlag1 = true;
}

// ISR for GPIO interrupt for state machine 1
/*
void PORTA_IRQHandler(void) {
    PORTA->ISFR = 0xFFFFFFFF; // Clear interrupt flags
    gpioInterruptFlag1 = true;
}
*/

// State transition function for state machine 1
void tickFB(void) {
    // Transitions
    switch(lState) {
        case INIT:
            lState = READ;
            break;
        case READ:
            if(joyL > 529 && joyL < fullThrottleTH) {
                lState = UP;
            } else if(joyL >= fullThrottleTH) {
                lState = UPMAX;
            } else if(joyL < 527 && joyL > fullReverseTH) {
                lState = DOWN;
            } else if(joyL <= fullReverseTH) {
                lState = DOWNMAX;
            } else if (joyL >= 527 && joyL <= 529) {
                lState = READ;
            }
            break;
        case UP:
            if(joyL > 529 && joyL < fullThrottleTH) {
                lState = UP;
            } else if(joyL >= fullThrottleTH) {
                lState = UPMAX;
            } else if(joyL < 527 && joyL > fullReverseTH) {
                lState = DOWN;
            } else if(joyL <= fullReverseTH) {
                lState = DOWNMAX;
            } else if (joyL >= 527 && joyL <= 529) {
                lState = READ;
            }
            break;
        case UPMAX:
            if(joyL > 529 && joyL < fullThrottleTH) {
                lState = UP;
            } else if(joyL >= fullThrottleTH) {
                lState = UPMAX;
            } else if(joyL < 527 && joyL > fullReverseTH) {
                lState = DOWN;
            } else if(joyL <= fullReverseTH) {
                lState = DOWNMAX;
            } else if (joyL >= 527 && joyL <= 529) {
                lState = READ;
            }
            break;
        case DOWN:
            if(joyL > 529 && joyL < fullThrottleTH) {
                lState = UP;
            } else if(joyL >= fullThrottleTH) {
                lState = UPMAX;
            } else if(joyL < 527 && joyL > fullReverseTH) {
                lState = DOWN;
            } else if(joyL <= fullReverseTH) {
                lState = DOWNMAX;
            } else if (joyL >= 527 && joyL <= 529) {
                lState = READ;
            }
            break;
        case DOWNMAX:
            if(joyL > 529 && joyL < fullThrottleTH) {
                lState = UP;
            } else if(joyL >= fullThrottleTH) {
                lState = UPMAX;
            } else if(joyL < 527 && joyL > fullReverseTH) {
                lState = DOWN;
            } else if(joyL <= fullReverseTH) {
                lState = DOWNMAX;
            } else if (joyL >= 527 && joyL <= 529) {
                lState = READ;
            }
            break;
        default:
            lState = READ;
            break;
    }

    // Actions
    switch(lState) {
        case INIT:
            break;
        case READ:
            setPoint = 0.0;
            break;
        case UP:
            setPoint = -10.0;
            break;
        case UPMAX:
            setPoint = -20.0;
            break;
        case DOWN:
            setPoint = 10.0;
            break;
        case DOWNMAX:
            setPoint = 20.0;
            break;
    }
}

/*////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////*/
//Define states for state machine 2 (Task 2)
typedef enum STATES_LRCONTROL {INIT1, READ1, RIGHT, RIGHTMAX, LEFT, LEFTMAX} STATES_LRCONTROL;
volatile STATES_LRCONTROL rState = INIT1; //current state
volatile bool timerInterruptFlag2 = false; // Interrupt flags for state machine 2
volatile bool gpioInterruptFlag2 = false;

// ISR for timer interrupt for state machine 2
void PIT1_IRQHandler(void) {
    PIT->CHANNEL[1].TFLG = PIT_TFLG_TIF_MASK; // Clear interrupt flag
    timerInterruptFlag2 = true;
}

// ISR for GPIO interrupt for state machine 2
/*
void PORTB_IRQHandler(void) {
    PORTB->ISFR = 0xFFFFFFFF; // Clear interrupt flags
    gpioInterruptFlag2 = true;
}
*/

// State transition function for state machine 2
void tickLR(void){ //501 rest val (left 0, right 1023)
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
      /*
      digitalWrite(led5, LOW);
      digitalWrite(led6, LOW);
      digitalWrite(led7, LOW);
      digitalWrite(A4, LOW);
	  */
    break;

    case RIGHT:
      /*
      digitalWrite(led5, HIGH);
      digitalWrite(led6, LOW);
      digitalWrite(led7, LOW);
      digitalWrite(A4, LOW);
      */

    break;

    case RIGHTMAX:
		/*
      digitalWrite(led5, HIGH);
      digitalWrite(led6, HIGH);
      digitalWrite(led7, LOW);
      digitalWrite(A4, LOW);
	  */
    break;

    case LEFT:
      /*
      digitalWrite(led5, LOW);
      digitalWrite(led6, LOW);
      digitalWrite(led7, HIGH);
      digitalWrite(A4, LOW);
      */
    break;

    case LEFTMAX:
      /*
      digitalWrite(led5, LOW);
      digitalWrite(led6, LOW);
      digitalWrite(led7, HIGH);
      digitalWrite(A4, HIGH);
	  */
    break;
  }
}

/*////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////*/
void PIT_Init(void) {
    // Enable clock for PIT module
    SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;

    // Enable PIT module and enable timers to run in debug mode
    PIT->MCR = 0x00;

    // Configure PIT channel 0 for 1ms period
    uint32_t pit0_load_value = (CLOCK_GetFreq(kCLOCK_BusClk) / 1000000U) * PIT_PERIOD_US;
    PIT->CHANNEL[0].LDVAL = PIT_LDVAL_TSV(pit0_load_value - 1);

    // Configure PIT channel 1 for 1ms period
    uint32_t pit1_load_value = (CLOCK_GetFreq(kCLOCK_BusClk) / 1000000U) * PIT_PERIOD_US;
    PIT->CHANNEL[1].LDVAL = PIT_LDVAL_TSV(pit1_load_value - 1);

    // Enable interrupts for PIT channels
    PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK;
    PIT->CHANNEL[1].TCTRL |= PIT_TCTRL_TIE_MASK;

    // Enable timers
    PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;
    PIT->CHANNEL[1].TCTRL |= PIT_TCTRL_TEN_MASK;

    // Enable PIT interrupts in the NVIC
    NVIC_EnableIRQ(PIT0_IRQn);
    NVIC_EnableIRQ(PIT1_IRQn);
}

/*////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////*/
uint16_t ADC_ReadFB(){
	ADC0_SC1A = 0x00; //Write to SC1A to start conversion
	while(ADC0_SC2 & ADC_SC2_ADACT_MASK); //conversion in process
	while(!(ADC0_SC1A & ADC_SC1_COCO_MASK)); //Until conversion is complete
	return ADC0_RA; //ADC conversion result for ADC0
}

uint16_t ADC_ReadLR(){
	ADC1_SC1A = 0x00; //Write to SC1A to start conversion
	while(ADC1_SC2 & ADC_SC2_ADACT_MASK); //conversion in process
	while(!(ADC1_SC1A & ADC_SC1_COCO_MASK)); //Until conversion is complete
	return ADC1_RA; //ADC conversion result for ADC0
}

/*////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////*/
// Main function
int main(void) {
    // Initialize hardware
    // ... (clock settings, GPIO initialization, timer initialization, etc.)
	PIT_Init(); //Initialize PIT
	
    // Enable interrupts
    NVIC_EnableIRQ(PIT0_IRQn);
    //NVIC_EnableIRQ(PORTA_IRQn);
    NVIC_EnableIRQ(PIT1_IRQn);
    //NVIC_EnableIRQ(PORTB_IRQn);

    while (1) { //loop function
		// Read the raw values from the joystick
		/*
		joyL = analogRead(joystickLPin);
		joyR = analogRead(joystickRPin);
		*/
	joyL = ADC_ReadFB();
        joyR = ADC_ReadLR();
		
		// Perform tasks based on current states
        tickFB();
        tickLR();
        
    }
	return 0;
}
