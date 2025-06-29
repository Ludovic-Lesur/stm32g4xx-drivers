/*
 * pwr.c
 *
 *  Created on: 15 feb. 2025
 *      Author: Ludo
 */

#ifndef STM32G4XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32g4xx_drivers_flags.h"
#endif

#ifndef STM32G4XX_DRIVERS_DISABLE

#include "pwr.h"

#include "nvic_registers.h"
#include "pwr_registers.h"
#include "rcc_registers.h"
#include "scb_registers.h"
#include "types.h"

/*** PWR local macros ***/

#define PWR_TIMEOUT_COUNT   1000000

/*** PWR local functions ***/

/*******************************************************************/
static void __attribute__((optimize("-O0"))) _PWR_reset_backup_domain(void) {
    // Local variables.
    uint8_t count = 0;
    // Unlock back-up registers.
    PWR->CR1 |= (0b1 << 8); // DBP='1'.
    // Perform manual reset and delay.
    RCC->BDCR |= (0b1 << 16); // BDRST='1'.
    for (count = 0; count < 100; count++);
    RCC->BDCR &= ~(0b1 << 16); // BDRST='0'.
}

/*******************************************************************/
static void _PWR_set_regulator_state(PWR_sleep_mode_t sleep_mode) {
    // Local variables.
    uint32_t loop_count = 0;
    uint32_t reglpf = 0;
    // Check mode.
    switch (sleep_mode) {
    case PWR_SLEEP_MODE_NORMAL:
        // Regulator in normal mode by default.
        PWR->CR1 &= ~(0b1 << 14); // LPR='0'.
        reglpf = 0;
        break;
    case PWR_SLEEP_MODE_LOW_POWER:
        // Regulator in low power mode.
        PWR->CR1 |= (0b1 << 14); // LPR='1'.
        reglpf = 1;
        break;
    default:
        goto errors;
    }
    // Wait for the regulator to be ready.
    while ((((PWR->SR2) >> 9) & 0x00000001) != reglpf) {
        loop_count++;
        if (loop_count > PWR_TIMEOUT_COUNT) break;
    }
errors:
    return;
}

/*** PWR functions ***/

/*******************************************************************/
void PWR_init(void) {
    // Enable power interface clock.
    RCC->APB1ENR1 |= (0b1 << 28); // PWREN='1'.
    // Reset backup domain.
    _PWR_reset_backup_domain();
    // Never return in low power sleep mode after wake-up.
    SCB->SCR &= ~(0b1 << 1); // SLEEPONEXIT='0'.
    // Disable UCPD pull-down resistors.
    PWR->CR3 |= (0b1 << 14);
}

/*******************************************************************/
void PWR_de_init(void) {
    // Disable power interface clock.
    RCC->APB1ENR1 &= ~(0b1 << 28); // PWREN='0'.
}

/*******************************************************************/
void PWR_enter_sleep_mode(PWR_sleep_mode_t sleep_mode) {
    // Configure regulator.
    _PWR_set_regulator_state(sleep_mode);
    // Enter sleep mode.
    SCB->SCR &= ~(0b1 << 2); // SLEEPDEEP='0'.
    // Wait For Interrupt core instruction.
    __asm volatile ("wfi");
    // Restore regulator in main mode.
    _PWR_set_regulator_state(PWR_SLEEP_MODE_NORMAL);
}

/*******************************************************************/
void PWR_enter_deepsleep_mode(PWR_deepsleep_mode_t deepsleep_mode) {
    // Reset LPMS field.
    PWR->CR1 &= ~(0b111 << 0);
    // Select low power mode.
    switch (deepsleep_mode) {
    case PWR_DEEPSLEEP_MODE_STOP_0:
        // Nothing to do.
        break;
    case PWR_DEEPSLEEP_MODE_STOP_1:
        PWR->CR1 |= (0b001 << 0); // LPMS='001'.
        break;
    case PWR_DEEPSLEEP_MODE_STANDBY:
        PWR->CR1 |= (0b011 << 0); // LPMS='011'.
        break;
    case PWR_DEEPSLEEP_MODE_SHUTDOWN:
        PWR->CR1 |= (0b100 << 0); // LPMS='100'.
        break;
    default:
        break;
    }
    // Clear wake-up flags.
    PWR->SCR |= (0b11111 << 0); // CWUFX='1'.
    // Clear all interrupt pending bits.
    NVIC->ICPR[0] = 0xFFFFFFFF;
    NVIC->ICPR[1] = 0xFFFFFFFF;
    NVIC->ICPR[2] = 0xFFFFFFFF;
    // Enter deep sleep mode.
    SCB->SCR |= (0b1 << 2); // SLEEPDEEP='1'.
    // Wait For Interrupt core instruction.
    __asm volatile ("wfi");
}

/*******************************************************************/
void PWR_software_reset(void) {
    // Trigger software reset.
    SCB->AIRCR = 0x05FA0000 | ((SCB->AIRCR) & 0x0000FFFF) | (0b1 << 2);
}

/*******************************************************************/
uint8_t PWR_get_reset_flags(void) {
    // Local variables.
    return ((uint8_t) (((RCC->CSR) >> 24) & 0xFF));
}

/*******************************************************************/
void PWR_clear_reset_flags(void) {
    RCC->CSR |= (0b1 << 23);
}

#endif /* STM32G4XX_DRIVERS_DISABLE */
