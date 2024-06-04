/*
**     Filename    : main.c
**     Project     : PID balancing robot
**     Processor   : MK64FN1M0VLL12
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2024/6/3, 17:50, # CodeGen: 0
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
*/
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
#include "FX1.h"
#include "GI2C1.h"
#include "WAIT1.h"
#include "CI2C1.h"
#include "CsIO1.h"
#include "IO1.h"
#include "MCUC1.h"
#include "SM1.h"
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "PDD_Includes.h"
#include "Init_Config.h"
#include "fsl_device_registers.h"
#include <stdint.h>
/* User includes (#include below this line is not maintained by Processor Expert) */

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */

#define BUFFER_SIZE 2

int joystickFB, joystickLR;

uint8_t tx_buf[BUFFER_SIZE];
void SPI(void);
void joystick_rs(void);

int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
    //clock gating 
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; //Enable Port A Clock Gate Control
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; /*Enable Port B Clock Gate Control*/
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; /*Enable Port C Clock Gate Control*/
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK; /*Enable Port D Clock Gate Control*/
	//ADC initialization and clock gating
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK; //Enable ADC0 Clock (A0 Forward Backward Joystick)
	SIM_SCGC6 |= SIM_SCGC6_ADC1_MASK; //Enable ADC1 Clock (A1 Left Right Joystick)
		
	ADC0_CFG1 = 0x0C; //16 bits ADC, bus CLOCK
	ADC0_SC1A = 0x1F; //Disable the module, ADCH = 11111
	
	
	//configurations & initialization
	PORTD_GPCLR = 0xFF0100; //Configure pins 0-7 on PORT D to be GPIO
	PORTC_GPCLR = 0x1BF0100; //Configure pins 0-5, 7-8 on PORT C to be GPIO (skip PTC 6 because that is reserved by the K64F for interrupt)
	PORTB_GPCLR = 0x000C0100; //configures pins 2,3, on port B to be GPIO 
	
	
    /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
    PE_low_level_init();
    SPI_init();
    /*** End of Processor Expert internal initialization.                    ***/
    /* Write your code here */

    for(;;) {
        joystick_rs();
        //probably need a timer or delay somewhere around here.
		printf(joystickFB);
    }


    //for(;;) {}
    return 0;
}
void SPI(void) {
    //Initialize SPI1 moduel
    SPI1_init(NULL);
}

void joystick_rs(void) {
    // Read joystick values and store them in tx_buf
    //FOR EXAMPLE: how to doesnt actually work add our logic
    joystickFB = readJoystickFB();
    joystickLR = readJoystickLR();
    
    //map joystick values to tx_buf
    tx_buf[0] = joystickFB;
    tx_buf[0] = joystickLR;
    
    //send the data via SPI
    SPI1_sendBlock(SPI1_DeviceData, tx_buf, BUFFER_SIZE);
    WAIT1_Waitms(50); //dont know if necessary but this is mean to wait for the transmission to be complete (can be adjusted)
}

int readjoystickFB(void) {
	ADC0_SC1A = 0x00; //Write to SC1A to start conversion
	while(ADC0_SC2 & ADC_SC2_ADACT_MASK); //conversion in process
	while(!(ADC0_SC1A & ADC_SC1_COCO_MASK)); //Until conversion is complete
	return ADC0_RA; //ADC conversion result for ADC0
}

int readjoystickLR(void) {
	
    return 0;
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
 **     This file was created by Processor Expert 10.4 [05.11]
 **     for the Freescale Kinetis series of microcontrollers.
 **
 ** ###################################################################
 */
