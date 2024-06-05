	/* ###################################################################
**     Filename    : main.c
**     Project     : K64F_Joystick_ADC
**     Processor   : MK64FN1M0VLL12
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2024-06-04, 16:21, # CodeGen: 0
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file main.c
** @version 01.01
** @brief
**         Main module.
**         This module contains user's application code.
*/         
/*!
**  @addtogroup main_module main module documentation
**  @{
*/         
/* MODULE main */


/* Including needed modules to compile this module/procedure */
#include "Cpu.h"
#include "Events.h"
#include "Pins1.h"
#include "SM1.h"
#include "ADC0.h"
#include "ADC1.h"
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "PDD_Includes.h"
#include "Init_Config.h"
#include "MK64F12.h"
/* User includes (#include below this line is not maintained by Processor Expert) */
unsigned char buffer[32]; //default from lab 6 is write[512];

unsigned short ADC_ReadFB(void){
    ADC0_SC1A = 0x00; //Write to SC1A to start conversion
    while(ADC0_SC2 & ADC_SC2_ADACT_MASK); //conversion in process
    while(!(ADC0_SC1A & ADC_SC1_COCO_MASK)); //Until conversion is complete
    return ADC0_RA; //ADC conversion result for ADC0
}

unsigned short ADC_ReadLR(void){
    ADC1_SC1A = 0x00; //Write to SC1A to start conversion
    while(ADC1_SC2 & ADC_SC2_ADACT_MASK); //conversion in process
    while(!(ADC1_SC1A & ADC_SC1_COCO_MASK)); //Until conversion is complete
    return ADC1_RA; //ADC conversion result for ADC1
}

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  /* Write your local variable definition here */
    
  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */
  int16_t joyL;
  int16_t joyR;
  
  SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK; /*Enable Port ADC0_DP0 Clock Gate Control*/
  ADC0_CFG1 = 0x0C; //16 bits ADC; bus CLOCK
  ADC0_SC1A = 0x1F; //Disable the module, ADCH = 11111
  
  SIM_SCGC3 |= SIM_SCGC3_ADC1_MASK; /*Enable Port ADC1_DP1 Clock Gate Control*/
  ADC1_CFG1 = 0x0C; //16 bits ADC; bus CLOCK
  ADC1_SC1A = 0x1F; //Disable the module, ADCH = 11111
  
  uint32_t delay;
  int len;
  LDD_TDeviceData *SM1_DeviceData;
  SM1_DeviceData = SM1_Init(NULL);
  
  /* For example: for(;;) { } */
  for(;;){
      joyL = ADC_ReadFB();
      joyR = ADC_ReadLR();
	  
	  printf("joyL: %d\t joyR:%d\n", joyL, joyR);
	  len = sprintf(buffer, "joyL: %d\t joyR: %d\n", joyL, joyR);
	  SM1_SendBlock(SM1_DeviceData, &buffer, len);
	  for(delay = 0; delay < 300000; delay++); //delay
	  
  }
  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;){}
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.5 [05.21]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
