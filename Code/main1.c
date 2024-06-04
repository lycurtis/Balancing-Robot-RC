/* ###################################################################
**     Filename    : main.c
**     Project     : K64F_ProjData
**     Processor   : MK64FN1M0VLL12
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2024-06-04, 07:18, # CodeGen: 0
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
#include "ADC1.h"
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "PDD_Includes.h"
#include "Init_Config.h"
#inclue "MK64F12.h"
/* User includes (#include below this line is not maintained by Processor Expert) */

uint16_t readJoystick(uint8_t channelGroup, uint8_t channel) {
	uint16_t result;
	ADC1_Measure(True);
}
/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  /* Write your local variable definition here */
  LDD_TDeviceData* adcData;
  LDD_TError error;
  ADC1_TResultData adcValue[ADC1_CHANNEL_COUNT]; //array to store ADC values
  LDD_TData spiData[2] //Array to store SPI Data
  
  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */
  adcData = ADC1_Init(NULL);
  for(;;){
	  /* Start ADC measurement */
    error = ADC1_StartSingleMeasurement(adcData);
    if (error != ERR_OK) {
      // Handle error
    }

    /* Wait for the measurement to complete */
    while (ADC1_GetMeasurementCompleteStatus(adcData) == FALSE) {
      // Busy wait
    }

    /* Get ADC values */
    error = ADC1_GetMeasuredValues(adcData, (LDD_TData *)adcValue);
    if (error != ERR_OK) {
      // Handle error
    }

    /* Prepare data for SPI transmission */
    spiData[0] = (adcValue[0] >> 8) & 0xFF; // High byte of first channel
    spiData[1] = adcValue[0] & 0xFF;        // Low byte of first channel

    /* Send data via SPI */
    error = SPI0_SendBlock(spiData, sizeof(spiData)); //sends the data via SPI
    if (error != ERR_OK) {
      // Handle error
    }

    /* Add delay if necessary */
    Cpu_Delay100US(10); // Delay for 1 ms
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
