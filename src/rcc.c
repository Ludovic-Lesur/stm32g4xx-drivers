/*
 * rcc.c
 *
 *  Created on: 16 feb. 2025
 *      Author: Ludo
 */

#ifndef STM32G4XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32g4xx_drivers_flags.h"
#endif

#ifndef STM32G4XX_DRIVERS_DISABLE

#include "rcc.h"

#include "error.h"
#include "flash.h"
#include "gpio.h"
#include "nvic.h"
#include "pwr.h"
#include "rcc_registers.h"
#include "tim.h"
#include "types.h"

/*** RCC local macros ***/

#define RCC_TIMEOUT_COUNT                   1000000

#define RCC_LSI_FREQUENCY_TYPICAL_HZ        38000
#define RCC_LSI_FREQUENCY_ACCURACY_PERCENT  48

#define RCC_HSI_FREQUENCY_TYPICAL_HZ        16000000
#define RCC_HSI_FREQUENCY_ACCURACY_PERCENT  6

#define RCC_WAIT_STATES_TABLE_SIZE          5

#define RCC_PLL_M_MIN                       1
#define RCC_PLL_M_MAX                       16
#define RCC_PLL_INPUT_CLOCK_HZ_MIN          3000000
#define RCC_PLL_INPUT_CLOCK_HZ_MAX          16000000
#define RCC_PLL_N_MIN                       8
#define RCC_PLL_N_MAX                       127
#define RCC_PLL_VCO_CLOCK_HZ_MIN            96000000
#define RCC_PLL_VCO_CLOCK_HZ_MAX            344000000
#define RCC_PLL_P_MIN                       2
#define RCC_PLL_P_MAX                       31
#define RCC_PLL_OUTPUT_CLOCK_MAX_HZ         170000000

/*** RCC local structures ***/

/*******************************************************************/
typedef struct {
    RCC_clock_t source;
#ifdef STM32G4XX_DRIVERS_RCC_HSE_ENABLE
    RCC_hse_mode_t hse_mode;
#endif
    RCC_pll_configuration_t pll_configuration;
} RCC_system_clock_t;

/*******************************************************************/
typedef struct {
    RCC_system_clock_t current_sysclk;
    RCC_system_clock_t previous_sysclk;
    uint32_t clock_frequency[RCC_CLOCK_LAST];
} RCC_context_t;

/*** RCC local global variables ***/

static const uint32_t RCC_WAIT_STATES_THRESHOLDS[RCC_WAIT_STATES_TABLE_SIZE] = { 34000000, 68000000, 102000000, 136000000, 170000000 };

static const uint8_t RCC_PLL_RQ_DIVIDER[RCC_PLL_RQ_LAST] = { 2, 4, 6, 8 };

static const uint8_t RCC_MCOSEL[RCC_CLOCK_LAST] = { 0b0000, 0b0001, 0b0011, 0b1000, 0b0100, 0b0101, 0b0000, 0b0110, 0b0111 };

static RCC_context_t rcc_ctx;

/*** RCC local functions ***/

/*******************************************************************/
void __attribute__((optimize("-O0"))) RCC_IRQHandler(void) {
    // Clear flags.
    RCC->CICR = (0b11 << 0);
}

/*******************************************************************/
static void _RCC_enable_lsi(void) {
    // Enable LSI.
    RCC->CSR |= (0b1 << 0); // LSION='1'.
    // Enable interrupt.
    RCC->CIER |= (0b1 << 0);
    NVIC_enable_interrupt(NVIC_INTERRUPT_RCC);
    // Wait for LSI to be stable.
    while (((RCC->CSR) & (0b1 << 1)) == 0) {
        PWR_enter_sleep_mode(PWR_SLEEP_MODE_NORMAL);
    }
    NVIC_disable_interrupt(NVIC_INTERRUPT_RCC);
}

#if (STM32G4XX_DRIVERS_RCC_LSE_MODE == 1)
/*******************************************************************/
static RCC_status_t _RCC_enable_lse(void) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    uint32_t loop_count = 0;
    // Enable LSE (32.768kHz crystal).
    RCC->BDCR |= (0b1 << 0); // LSEON='1'.
    // Wait for LSE to be stable.
    while (((RCC->BDCR) & (0b1 << 1)) == 0) {
        // Wait for LSERDY='1' ready flag or timeout.
        loop_count++;
        if (loop_count > RCC_TIMEOUT_COUNT) {
            // Switch LSE off.
            RCC->BDCR &= ~(0b1 << 0); // LSEON='0'.
            // Exit loop.
            status = RCC_ERROR_LSE_READY;
            goto errors;
        }
    }
errors:
    return status;
}
#endif

#if (STM32G4XX_DRIVERS_RCC_LSE_MODE == 2)
/*******************************************************************/
static void _RCC_enable_lse(void) {
    // Enable LSE (32.768kHz crystal).
    RCC->BDCR |= (0b1 << 0); // LSEON='1'.
    // Enable interrupt.
    RCC->CIER |= (0b1 << 1);
    NVIC_enable_interrupt(NVIC_INTERRUPT_RCC);
    // Wait for LSE to be stable.
    while (((RCC->BDCR) & (0b1 << 1)) == 0) {
        PWR_enter_sleep_mode(PWR_SLEEP_MODE_NORMAL);
    }
    NVIC_disable_interrupt(NVIC_INTERRUPT_RCC);
}
#endif

/*******************************************************************/
static void _RCC_save_system_clock(void) {
    // Copy current settings in previous structure.
    rcc_ctx.previous_sysclk.source = rcc_ctx.current_sysclk.source;
#ifdef STM32G4XX_DRIVERS_RCC_HSE_ENABLE
    rcc_ctx.previous_sysclk.hse_mode = rcc_ctx.current_sysclk.hse_mode;
#endif
    rcc_ctx.previous_sysclk.pll_configuration.source = rcc_ctx.current_sysclk.pll_configuration.source;
#ifdef STM32G4XX_DRIVERS_RCC_HSE_ENABLE
    rcc_ctx.previous_sysclk.pll_configuration.hse_mode = rcc_ctx.current_sysclk.pll_configuration.hse_mode;
#endif
    rcc_ctx.previous_sysclk.pll_configuration.m = rcc_ctx.current_sysclk.pll_configuration.m;
    rcc_ctx.previous_sysclk.pll_configuration.n = rcc_ctx.current_sysclk.pll_configuration.n;
    rcc_ctx.previous_sysclk.pll_configuration.r = rcc_ctx.current_sysclk.pll_configuration.r;
    rcc_ctx.previous_sysclk.pll_configuration.p = rcc_ctx.current_sysclk.pll_configuration.p;
    rcc_ctx.previous_sysclk.pll_configuration.q = rcc_ctx.current_sysclk.pll_configuration.q;
}

/*******************************************************************/
static RCC_status_t _RCC_update_flash_latency(uint32_t target_sysclk_frequency_hz, uint8_t post_update_flag) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    FLASH_status_t flash_status = FLASH_SUCCESS;
    uint8_t current_latency = 0;
    uint8_t new_latency = 0;
    // Get current latency.
    flash_status = FLASH_get_latency(&current_latency);
    FLASH_exit_error(RCC_ERROR_BASE_FLASH);
    // Compute required number of wait states.
    for (new_latency = 0; new_latency < RCC_WAIT_STATES_TABLE_SIZE; new_latency++) {
        if (target_sysclk_frequency_hz <= RCC_WAIT_STATES_THRESHOLDS[new_latency]) {
            break;
        }
    }
    // Check step.
    if (((post_update_flag == 0) && (new_latency > current_latency)) || ((post_update_flag != 0) && (new_latency < current_latency))) {
        // Set new latency.
        flash_status = FLASH_set_latency(new_latency);
        FLASH_exit_error(RCC_ERROR_BASE_FLASH);
    }
errors:
    return status;
}

/*******************************************************************/
static RCC_status_t _RCC_wait_for_clock_ready(RCC_clock_t clock, RCC_status_t timeout_error) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    uint8_t clock_is_ready = 0;
    uint32_t loop_count = 0;
    // Wait for clock to be stable.
    do {
        // Read status.
        status = RCC_get_status(clock, &clock_is_ready);
        if (status != RCC_SUCCESS) goto errors;
        // Exit if timeout.
        loop_count++;
        if (loop_count > RCC_TIMEOUT_COUNT) {
            status = timeout_error;
            goto errors;
        }
    }
    while (clock_is_ready == 0);
errors:
    return status;
}

/*******************************************************************/
static RCC_status_t _RCC_switch_system_clock(RCC_clock_t system_clock, RCC_status_t switch_error) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    uint32_t reg_cfgr = 0;
    uint32_t switch_value = 0;
    uint32_t loop_count = 0;
    // Read register.
    reg_cfgr = (RCC->CFGR);
    reg_cfgr &= ~(0b11 << 0); // Reset bits 0-1.
    // Check system clock.
    switch (system_clock) {
    case RCC_CLOCK_HSI:
        switch_value = 0b01;
        break;
    case RCC_CLOCK_HSE:
        switch_value = 0b10;
        break;
    case RCC_CLOCK_PLL:
        switch_value = 0b11;
        break;
    default:
        status = RCC_ERROR_CLOCK;
        goto errors;
    }
    reg_cfgr |= (switch_value << 0);
    // Perform switch.
    RCC->CFGR = reg_cfgr;
    // Wait for clock switch.
    while (((RCC->CFGR) & (0b11 << 2)) != (switch_value << 2)) {
        // Wait for SWS='switch_value' or timeout.
        loop_count++;
        if (loop_count > RCC_TIMEOUT_COUNT) {
            status = switch_error;
            goto errors;
        }
    }
errors:
    return status;
}

#if ((STM32G4XX_DRIVERS_TIM_MODE_MASK & 0x04) != 0)
/*******************************************************************/
static RCC_status_t _RCC_check_frequency_range(uint32_t default_value, uint32_t accuracy_percent, uint32_t measured_value, RCC_status_t calibration_error) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    uint32_t frequency_min = (default_value - ((default_value * accuracy_percent) / (100)));
    uint32_t frequency_max = (default_value + ((default_value * accuracy_percent) / (100)));
    // Check range.
    if ((measured_value < frequency_min) || (measured_value > frequency_max)) {
        status = calibration_error;
    }
    return status;
}
#endif

/*** RCC functions ***/

/*******************************************************************/
RCC_status_t RCC_init(uint8_t nvic_priority) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    uint8_t idx = 0;
    // Set boot configuration.
    rcc_ctx.current_sysclk.source = RCC_CLOCK_HSI;
#ifdef STM32G4XX_DRIVERS_RCC_HSE_ENABLE
    rcc_ctx.current_sysclk.hse_mode = RCC_HSE_MODE_OSCILLATOR;
#endif
    rcc_ctx.current_sysclk.pll_configuration.source = RCC_CLOCK_NONE;
#ifdef STM32G4XX_DRIVERS_RCC_HSE_ENABLE
    rcc_ctx.current_sysclk.pll_configuration.hse_mode = RCC_HSE_MODE_OSCILLATOR;
#endif
    rcc_ctx.current_sysclk.pll_configuration.m = 0;
    rcc_ctx.current_sysclk.pll_configuration.n = 0;
    rcc_ctx.current_sysclk.pll_configuration.r = 0;
    rcc_ctx.current_sysclk.pll_configuration.p = 0;
    rcc_ctx.current_sysclk.pll_configuration.q = 0;
    _RCC_save_system_clock();
    // Set default frequencies.
    for (idx = 0; idx < RCC_CLOCK_LAST; idx++) {
        rcc_ctx.clock_frequency[idx] = 0;
    }
    rcc_ctx.clock_frequency[RCC_CLOCK_HSI] = RCC_HSI_FREQUENCY_TYPICAL_HZ;
    rcc_ctx.clock_frequency[RCC_CLOCK_HSE] = STM32G4XX_DRIVERS_RCC_HSE_FREQUENCY_HZ;
    rcc_ctx.clock_frequency[RCC_CLOCK_LSI] = RCC_LSI_FREQUENCY_TYPICAL_HZ;
    rcc_ctx.clock_frequency[RCC_CLOCK_LSE] = STM32G4XX_DRIVERS_RCC_LSE_FREQUENCY_HZ;
    rcc_ctx.clock_frequency[RCC_CLOCK_SYSTEM] = rcc_ctx.clock_frequency[rcc_ctx.current_sysclk.source];
    // Set interrupt priority.
    NVIC_set_priority(NVIC_INTERRUPT_RCC, nvic_priority);
    // Start low speed oscillators.
    _RCC_enable_lsi();
#if (STM32G4XX_DRIVERS_RCC_LSE_MODE == 1)
    status = _RCC_enable_lse();
#endif
#if (STM32G4XX_DRIVERS_RCC_LSE_MODE == 2)
    _RCC_enable_lse();
#endif
    return status;
}

/*******************************************************************/
RCC_status_t RCC_switch_to_hsi(void) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    // Save current configuration.
    _RCC_save_system_clock();
    // Set flash latency.
    status = _RCC_update_flash_latency(rcc_ctx.clock_frequency[RCC_CLOCK_HSI], 0);
    if (status != RCC_SUCCESS) goto errors;
    // Enable HSI.
    RCC->CR |= (0b1 << 8); // HSI16ON='1'.
    // Wait for HSI to be stable.
    status = _RCC_wait_for_clock_ready(RCC_CLOCK_HSI, RCC_ERROR_HSI_READY);
    if (status != RCC_SUCCESS) goto errors;
    // Switch system clock.
    status = _RCC_switch_system_clock(RCC_CLOCK_HSI, RCC_ERROR_HSI_SWITCH);
    if (status != RCC_SUCCESS) goto errors;
    // Update flash latency.
    status = _RCC_update_flash_latency(rcc_ctx.clock_frequency[RCC_CLOCK_HSI], 1);
    if (status != RCC_SUCCESS) goto errors;
    // Turn PLL and HSE off.
    RCC->CR &= ~(0b1 << 24); // PLLON='0'.
    RCC->CR &= ~(0b1 << 16); // HSEON='0'.
    // Update clocks context.
    rcc_ctx.current_sysclk.source = RCC_CLOCK_HSI;
errors:
    // Update system clock frequency.
    rcc_ctx.clock_frequency[RCC_CLOCK_SYSTEM] = rcc_ctx.clock_frequency[rcc_ctx.current_sysclk.source];
    return status;
}

#ifdef STM32G4XX_DRIVERS_RCC_HSE_ENABLE
/*******************************************************************/
RCC_status_t RCC_switch_to_hse(RCC_hse_mode_t hse_mode) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    // Save current configuration.
    _RCC_save_system_clock();
    // Set flash latency.
    status = _RCC_update_flash_latency(STM32G4XX_DRIVERS_RCC_HSE_FREQUENCY_HZ, 0);
    if (status != RCC_SUCCESS) goto errors;
    // Set mode.
    if (hse_mode == RCC_HSE_MODE_BYPASS) {
        RCC->CR |= (0b1 << 18); // HSEBYP='1'.
    }
    // Enable HSE.
    RCC->CR |= (0b1 << 16); // HSEON='1'.
    // Wait for HSE to be stable.
    status = _RCC_wait_for_clock_ready(RCC_CLOCK_HSE, RCC_ERROR_HSE_READY);
    if (status != RCC_SUCCESS) goto errors;
    // Switch system clock.
    status = _RCC_switch_system_clock(RCC_CLOCK_HSE, RCC_ERROR_HSE_SWITCH);
    if (status != RCC_SUCCESS) goto errors;
    // Update flash latency.
    status = _RCC_update_flash_latency(STM32G4XX_DRIVERS_RCC_HSE_FREQUENCY_HZ, 1);
    if (status != RCC_SUCCESS) goto errors;
    // Turn PLL off.
    RCC->CR &= ~(0b1 << 24); // PLLON='0'.
    // Update clocks context.
    rcc_ctx.current_sysclk.source = RCC_CLOCK_HSE;
    rcc_ctx.current_sysclk.hse_mode = hse_mode;
errors:
    // Update system clock frequency.
    rcc_ctx.clock_frequency[RCC_CLOCK_SYSTEM] = rcc_ctx.clock_frequency[rcc_ctx.current_sysclk.source];
    return status;
}
#endif

/*******************************************************************/
RCC_status_t RCC_switch_to_pll(RCC_pll_configuration_t* pll_configuration) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    uint8_t pllsrc = 0;
    uint32_t pll_in_hz = 0;
    uint32_t pll_vco_hz = 0;
    uint32_t pll_r_hz = 0;
    uint32_t pll_p_hz = 0;
    uint32_t pll_q_hz = 0;
    // Check parameter.
    if (pll_configuration == NULL) {
        status = RCC_ERROR_NULL_PARAMETER;
        goto errors;
    }
    if (((pll_configuration->m) < RCC_PLL_M_MIN) || ((pll_configuration->m) > RCC_PLL_M_MAX)) {
        status = RCC_ERROR_PLL_M;
        goto errors;
    }
    if (((pll_configuration->n) < RCC_PLL_N_MIN) || ((pll_configuration->n) > RCC_PLL_N_MAX)) {
        status = RCC_ERROR_PLL_N;
        goto errors;
    }
    if ((pll_configuration->r) >= RCC_PLL_RQ_LAST) {
        status = RCC_ERROR_PLL_R;
        goto errors;
    }
    if (((pll_configuration->p) < RCC_PLL_P_MIN) || ((pll_configuration->p) > RCC_PLL_P_MAX)) {
        status = RCC_ERROR_PLL_P;
        goto errors;
    }
    if ((pll_configuration->q) >= RCC_PLL_RQ_LAST) {
        status = RCC_ERROR_PLL_Q;
        goto errors;
    }
    // Save current configuration.
    _RCC_save_system_clock();
    // Configure clock source.
    switch (pll_configuration->source) {
    case RCC_CLOCK_HSI:
        // Enable HSI.
        RCC->CR |= (0b1 << 8); // HSI16ON='1'.
        pllsrc = 0b10;
        break;
    case RCC_CLOCK_HSE:
        // Set mode.
        if ((pll_configuration->hse_mode) == RCC_HSE_MODE_BYPASS) {
            RCC->CR |= (0b1 << 18); // HSEBYP='1'.
        }
        // Enable HSE.
        RCC->CR |= (0b1 << 16); // HSEON='1'.
        pllsrc = 0b11;
        break;
    default:
        status = RCC_ERROR_PLL_CLOCK_SOURCE;
        goto errors;
    }
    // Wait for source to be stable.
    status = _RCC_wait_for_clock_ready((pll_configuration->source), RCC_ERROR_PLL_CLOCK_SOURCE_READY);
    if (status != RCC_SUCCESS) goto errors;
    // Reset PLL configuration.
    RCC->PLLCFGR = 0;
    // Input clock divider.
    pll_in_hz = (rcc_ctx.clock_frequency[pll_configuration->source] / (pll_configuration->m));
    if (pll_in_hz < RCC_PLL_INPUT_CLOCK_HZ_MIN) {
        status = RCC_ERROR_PLL_INPUT_CLOCK_FREQUENCY_UNDERFLOW;
        goto errors;
    }
    if (pll_in_hz > RCC_PLL_INPUT_CLOCK_HZ_MAX) {
        status = RCC_ERROR_PLL_INPUT_CLOCK_FREQUENCY_OVERFLOW;
        goto errors;
    }
    RCC->PLLCFGR |= (((pll_configuration->m) - 1) << 4) | (pllsrc << 0);
    // VCO.
    pll_vco_hz = (pll_in_hz * (pll_configuration->n));
    if (pll_vco_hz < RCC_PLL_VCO_CLOCK_HZ_MIN) {
        status = RCC_ERROR_PLL_VCO_CLOCK_FREQUENCY_UNDERFLOW;
        goto errors;
    }
    if (pll_vco_hz > RCC_PLL_VCO_CLOCK_HZ_MAX) {
        status = RCC_ERROR_PLL_VCO_CLOCK_FREQUENCY_OVERFLOW;
        goto errors;
    }
    RCC->PLLCFGR |= ((pll_configuration->n) << 8);
    // System clock prescaler.
    pll_r_hz = (pll_vco_hz / RCC_PLL_RQ_DIVIDER[pll_configuration->r]);
    if (pll_r_hz > RCC_PLL_OUTPUT_CLOCK_MAX_HZ) {
        status = RCC_ERROR_PLL_OUTPUT_CLOCK_OVERFLOW;
        goto errors;
    }
    RCC->PLLCFGR |= ((pll_configuration->r) << 25) | (0b1 << 24);
    // ADC clock divider.
    pll_p_hz = (pll_vco_hz / (pll_configuration->p));
    if (pll_p_hz > RCC_PLL_OUTPUT_CLOCK_MAX_HZ) {
        status = RCC_ERROR_PLL_OUTPUT_CLOCK_OVERFLOW;
        goto errors;
    }
    RCC->PLLCFGR |= ((pll_configuration->p) << 27) | (0b1 << 16);
    // Q clock prescaler.
    pll_q_hz = (pll_vco_hz / RCC_PLL_RQ_DIVIDER[pll_configuration->q]);
    if (pll_q_hz > RCC_PLL_OUTPUT_CLOCK_MAX_HZ) {
        status = RCC_ERROR_PLL_OUTPUT_CLOCK_OVERFLOW;
        goto errors;
    }
    RCC->PLLCFGR |= ((pll_configuration->q) << 21) | (0b1 << 20);
    // Turn PLL on.
    RCC->CR |= (0b1 << 24);
    // Wait for PLL to be stable.
    status = _RCC_wait_for_clock_ready(RCC_CLOCK_PLL, RCC_ERROR_PLL_READY);
    if (status != RCC_SUCCESS) goto errors;
    // Set flash latency.
    status = _RCC_update_flash_latency(pll_r_hz, 0);
    if (status != RCC_SUCCESS) goto errors;
    // Switch system clock.
    status = _RCC_switch_system_clock(RCC_CLOCK_PLL, RCC_ERROR_PLL_SWITCH);
    if (status != RCC_SUCCESS) goto errors;
    // Update flash latency.
    status = _RCC_update_flash_latency(pll_r_hz, 1);
    if (status != RCC_SUCCESS) goto errors;
    // Save PLL configuration.
    rcc_ctx.current_sysclk.pll_configuration.source = (pll_configuration->source);
#ifdef STM32G4XX_DRIVERS_RCC_HSE_ENABLE
    rcc_ctx.current_sysclk.pll_configuration.hse_mode = (pll_configuration->hse_mode);
#endif
    rcc_ctx.current_sysclk.pll_configuration.m = (pll_configuration->m);
    rcc_ctx.current_sysclk.pll_configuration.n = (pll_configuration->n);
    rcc_ctx.current_sysclk.pll_configuration.r = (pll_configuration->r);
    rcc_ctx.current_sysclk.pll_configuration.p = (pll_configuration->p);
    rcc_ctx.current_sysclk.pll_configuration.q = (pll_configuration->q);
    // Update clocks context.
    rcc_ctx.current_sysclk.source = RCC_CLOCK_PLL;
    rcc_ctx.clock_frequency[RCC_CLOCK_PLL] = pll_r_hz;
    rcc_ctx.clock_frequency[RCC_CLOCK_ADC] = pll_p_hz;
errors:
    // Update system clock.
    rcc_ctx.clock_frequency[RCC_CLOCK_SYSTEM] = rcc_ctx.clock_frequency[rcc_ctx.current_sysclk.source];
    return status;
}

/*******************************************************************/
RCC_status_t RCC_restore_previous_system_clock(void) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    // Check previous configuration.
    switch (rcc_ctx.previous_sysclk.source) {
    case RCC_CLOCK_HSI:
        status = RCC_switch_to_hsi();
        break;
#ifdef STM32G4XX_DRIVERS_RCC_HSE_ENABLE
    case RCC_CLOCK_HSE:
        status = RCC_switch_to_hse(rcc_ctx.previous_sysclk.hse_mode);
        break;
#endif
    case RCC_CLOCK_PLL:
        status = RCC_switch_to_pll(&(rcc_ctx.previous_sysclk.pll_configuration));
        break;
    default:
        status = RCC_ERROR_CLOCK;
        break;
    }
    return status;
}

#if ((STM32G4XX_DRIVERS_TIM_MODE_MASK & 0x04) != 0)
/*******************************************************************/
RCC_status_t RCC_calibrate_internal_clocks(uint8_t nvic_priority) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    TIM_status_t tim_status = TIM_SUCCESS;
    int32_t ref_clock_pulse_count = 0;
    int32_t mco_pulse_count = 0;
    uint64_t temp_u64 = 0;
    uint32_t clock_frequency_hz = 0;
#if (STM32G4XX_DRIVERS_RCC_LSE_MODE > 0)
    uint8_t lse_status = 0;
#endif
#ifdef STM32G4XX_DRIVERS_RCC_HSE_ENABLE
    uint8_t restore_done = 0;
#endif
    // Switch to HSI.
    status = RCC_switch_to_hsi();
    if (status != RCC_SUCCESS) goto errors;
    // Init measurement timer.
    tim_status = TIM_CAL_init(TIM_INSTANCE_TIM17, nvic_priority);
    if (tim_status != TIM_SUCCESS) {
        status = RCC_ERROR_CALIBRATION_TIMER;
        goto errors;
    }
#if (STM32G4XX_DRIVERS_RCC_LSE_MODE > 0)
    // Check LSE status.
    RCC_get_status(RCC_CLOCK_LSE, &lse_status);
    // HSI calibration is not possible without LSE.
    if (lse_status == 0) goto lsi_calibration;
    // Connect MCO to LSE clock.
    RCC_set_mco(RCC_CLOCK_LSE, RCC_MCO_PRESCALER_1, NULL);
    // HSI calibration.
    tim_status = TIM_CAL_mco_capture(TIM_INSTANCE_TIM17, &ref_clock_pulse_count, &mco_pulse_count);
    if (tim_status != TIM_SUCCESS) {
        status = RCC_ERROR_CALIBRATION_TIMER;
        goto errors;
    }
    // Compute HSI frequency.
    temp_u64 = ((uint64_t) STM32G4XX_DRIVERS_RCC_LSE_FREQUENCY_HZ * (uint64_t) ref_clock_pulse_count);
    clock_frequency_hz = (uint32_t) ((temp_u64) / ((uint64_t) mco_pulse_count));
    // Check range.
    status = _RCC_check_frequency_range(RCC_HSI_FREQUENCY_TYPICAL_HZ, RCC_HSI_FREQUENCY_ACCURACY_PERCENT, clock_frequency_hz, RCC_ERROR_CALIBRATION_HSI);
    if (status != RCC_SUCCESS) goto errors;
    // Store calibration value.
    rcc_ctx.clock_frequency[RCC_CLOCK_HSI] = clock_frequency_hz;
lsi_calibration:
#endif
    // LSI calibration.
#ifdef STM32G4XX_DRIVERS_RCC_HSE_ENABLE
    // Check if HSE is available for better precision.
    if (rcc_ctx.previous_sysclk.source == RCC_CLOCK_HSE) {
        // Restore HSE.
        status = RCC_restore_previous_system_clock();
        if (status != RCC_SUCCESS) goto errors;
        // Update local flag.
        restore_done = 1;
    }
#endif
    // Connect MCO to LSI clock.
    RCC_set_mco(RCC_CLOCK_LSI, RCC_MCO_PRESCALER_1, NULL);
    // Perform measurement.
    tim_status = TIM_CAL_mco_capture(TIM_INSTANCE_TIM17, &ref_clock_pulse_count, &mco_pulse_count);
    if (tim_status != TIM_SUCCESS) {
        status = RCC_ERROR_CALIBRATION_TIMER;
        goto errors;
    }
    // Compute LSI frequency.
    temp_u64 = ((uint64_t) rcc_ctx.clock_frequency[RCC_CLOCK_SYSTEM] * (uint64_t) mco_pulse_count);
    clock_frequency_hz = (uint32_t) ((temp_u64) / ((uint64_t) ref_clock_pulse_count));
    // Check range.
    status = _RCC_check_frequency_range(RCC_LSI_FREQUENCY_TYPICAL_HZ, RCC_LSI_FREQUENCY_ACCURACY_PERCENT, clock_frequency_hz, RCC_ERROR_CALIBRATION_LSI);
    if (status != RCC_SUCCESS) goto errors;
    // Update local data.
    rcc_ctx.clock_frequency[RCC_CLOCK_LSI] = clock_frequency_hz;
errors:
    // Release timer and MCO.
    TIM_CAL_de_init(TIM_INSTANCE_TIM17);
    RCC_set_mco(RCC_CLOCK_NONE, RCC_MCO_PRESCALER_1, NULL);
    // Restore system clock.
#ifdef STM32G4XX_DRIVERS_RCC_HSE_ENABLE
    if (restore_done == 0) {
        status = RCC_restore_previous_system_clock();
        if (status != RCC_SUCCESS) goto errors;
    }
#else
    status = RCC_restore_previous_system_clock();
    if (status != RCC_SUCCESS) goto errors;
#endif
    // Update system clock frequency.
    rcc_ctx.clock_frequency[RCC_CLOCK_SYSTEM] = rcc_ctx.clock_frequency[rcc_ctx.current_sysclk.source];
    return status;
}
#endif

/*******************************************************************/
RCC_clock_t RCC_get_system_clock(void) {
    return (rcc_ctx.current_sysclk.source);
}

/*******************************************************************/
RCC_status_t RCC_get_frequency_hz(RCC_clock_t clock, uint32_t* frequency_hz) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    // Check parameters.
    if (clock >= RCC_CLOCK_LAST) {
        status = RCC_ERROR_CLOCK;
        goto errors;
    }
    if (frequency_hz == NULL) {
        status = RCC_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Read frequency.
    (*frequency_hz) = rcc_ctx.clock_frequency[clock];
errors:
    return status;
}

/*******************************************************************/
RCC_status_t RCC_get_status(RCC_clock_t clock, uint8_t* clock_is_ready) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    // Check parameters.
    if (clock_is_ready == NULL) {
        status = RCC_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Reset result.
    (*clock_is_ready) = 0;
    // Check clock.
    switch (clock) {
    case RCC_CLOCK_SYSTEM:
        (*clock_is_ready) = 1;
        break;
    case RCC_CLOCK_HSI:
        (*clock_is_ready) = (((RCC->CR) >> 10) & 0b1);
        break;
    case RCC_CLOCK_HSI48:
        (*clock_is_ready) = (((RCC->CRRCR) >> 1) & 0b1);
        break;
    case RCC_CLOCK_HSE:
        (*clock_is_ready) = (((RCC->CR) >> 17) & 0b1);
        break;
    case RCC_CLOCK_PLL:
    case RCC_CLOCK_ADC:
        (*clock_is_ready) = (((RCC->CR) >> 25) & 0b1);
        break;
    case RCC_CLOCK_LSI:
        (*clock_is_ready) = (((RCC->CSR) >> 1) & 0b1);
        break;
    case RCC_CLOCK_LSE:
        (*clock_is_ready) = (((RCC->BDCR) >> 1) & 0b1);
        break;
    default:
        status = RCC_ERROR_CLOCK;
        goto errors;
    }
errors:
    return status;
}

/*******************************************************************/
RCC_status_t RCC_set_mco(RCC_clock_t mco_clock, RCC_mco_prescaler_t mco_prescaler, const GPIO_pin_t* mco_gpio) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    uint32_t cfgr = 0;
    // Check parameters.
    if (mco_clock >= RCC_CLOCK_LAST) {
        status = RCC_ERROR_CLOCK;
        goto errors;
    }
    if (mco_prescaler >= RCC_MCO_PRESCALER_LAST) {
        status = RCC_ERROR_MCO_PRESCALER;
        goto errors;
    }
    // Configure clock and prescaler.
    cfgr = (RCC->CFGR);
    cfgr &= 0x80FFFFFF;
    cfgr |= (mco_prescaler << 28) | (RCC_MCOSEL[mco_clock] << 24);
    RCC->CFGR = cfgr;
    // Configure GPIO if needed.
    if (mco_gpio != NULL) {
        // Check clock selection.
        if (mco_clock == RCC_CLOCK_NONE) {
            GPIO_configure(mco_gpio, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
        }
        else {
            GPIO_configure(mco_gpio, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
        }
    }
errors:
    return status;
}

#endif /* STM32G4XX_DRIVERS_DISABLE */
