/*
 * nvic.c
 *
 *  Created on: 15 feb. 2024
 *      Author: Ludo
 */

#ifndef STM32G4XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32g4xx_drivers_flags.h"
#endif

#ifndef STM32G4XX_DRIVERS_DISABLE

#include "nvic.h"

#include "nvic_registers.h"
#include "scb_registers.h"
#include "types.h"

/*** NVIC linker generated symbols ***/

extern uint32_t __Vectors;

/*** NVIC local macros ***/

#define NVIC_PRIORITY_MIN   15

/*** NVIC local functions ***/

/*******************************************************************/
static void _NVIC_set_primask_bit(uint32_t primask_bit)  {
    // Set core bit.
    __asm volatile ("MSR primask, %0" : : "r" (primask_bit) : "memory");
}

/*******************************************************************/
static uint32_t _NVIC_get_primask_bit(void)  {
    // Local variables.
    uint32_t result;
    // Read core bit.
    __asm volatile ("MRS %0, primask" : "=r" (result));
    return result;
}

/*** NVIC functions ***/

/*******************************************************************/
void NVIC_init(void) {
    // Init vector table address.
    SCB->VTOR = (uint32_t) &__Vectors;
}

/*******************************************************************/
void NVIC_set_global_interrupts(uint8_t enable) {
    // Set PRIMASK bit.
    _NVIC_set_primask_bit((enable == 0) ? 1 : 0);
}

/*******************************************************************/
uint8_t NVIC_get_global_interrupts(void) {
    // Read PRIMASK bit.
    return (((_NVIC_get_primask_bit() & 0x00000001) == 0) ? 1 : 0);
}

/*******************************************************************/
void NVIC_enable_interrupt(NVIC_interrupt_t irq_index) {
    // Check index.
    if (irq_index >= NVIC_INTERRUPT_LAST) goto errors;
    // Enable interrupt.
    NVIC->ISER[irq_index >> 5] = (0b1 << (irq_index % 32));
errors:
    return;
}

/*******************************************************************/
void NVIC_disable_interrupt(NVIC_interrupt_t irq_index) {
    // Check index.
    if (irq_index >= NVIC_INTERRUPT_LAST) goto errors;
    // Disable interrupt.
    NVIC->ICER[irq_index >> 5] = (0b1 << (irq_index % 32));
errors:
    return;
}

/*******************************************************************/
void NVIC_set_priority(NVIC_interrupt_t irq_index, uint8_t priority) {
    // Check index.
    if (irq_index >= NVIC_INTERRUPT_LAST) goto errors;
    // Clamp parameter.
    if (priority > NVIC_PRIORITY_MIN) {
        priority = NVIC_PRIORITY_MIN;
    }
    // Reset bits.
    NVIC->IPR[irq_index >> 2] &= ~(0xFF << ((irq_index % 4) << 3));
    // Set priority.
    NVIC->IPR[irq_index >> 2] |= ((priority << 4) << ((irq_index % 4) << 3));
errors:
    return;
}

#endif /* STM32G4XX_DRIVERS_DISABLE */
