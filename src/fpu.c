/*
 * fpu.c
 *
 *  Created on: 15 feb. 2025
 *      Author: Ludo
 */

#include "fpu.h"

#include "fpu_registers.h"

/*** FPU functions ***/

/*******************************************************************/
void FPU_init(void) {
    // Enable FPU access for all cores.
    FPU->CPACR |= (0b11 << 22) | (0b11 << 20);
}

/*******************************************************************/
void FPU_de_init(void) {
    // Disable FPU access for all cores.
    FPU->CPACR &= ~((0b11 << 22) | (0b11 << 20));
}
