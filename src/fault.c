/*
 * fault.c
 *
 *  Created on: 15 feb. 2025
 *      Author: Ludo
 */

#ifndef STM32G4XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32g4xx_drivers_flags.h"
#endif

#ifndef STM32G4XX_DRIVERS_DISABLE

#include "pwr.h"

/*******************************************************************/
void __attribute__((optimize("-O0"))) NMI_Handler(void) {
    // Trigger software reset.
    PWR_software_reset();
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) HardFault_Handler(void) {
    // Trigger software reset.
    PWR_software_reset();
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) MemManage_Handler(void) {
    // Trigger software reset.
    PWR_software_reset();
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) BusFault_Handler(void) {
    // Trigger software reset.
    PWR_software_reset();
}

#endif /* STM32G4XX_DRIVERS_DISABLE */
