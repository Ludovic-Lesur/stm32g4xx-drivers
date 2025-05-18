/*
 * adc.c
 *
 *  Created on: 02 feb. 2025
 *      Author: Ludo
 */

#ifndef STM32G4XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32g4xx_drivers_flags.h"
#endif

#ifndef STM32G4XX_DRIVERS_DISABLE

#include "adc.h"

#include "adc_registers.h"
#include "error.h"
#include "gpio.h"
#include "lptim.h"
#include "maths.h"
#include "rcc_registers.h"
#include "rcc.h"
#include "types.h"

/*** ADC local macros ***/

#define ADC_TIMEOUT_COUNT           1000000

#define ADC_SEQUENCE_LENGTH_MAX     16

#define ADC_SQX_ERROR               0xFF
#define ADC_EXTSEL_ERROR            0xFF

#define ADC_NUMBER_OF_OFFSETS       4

#define ADC_SMPX_RANGE              8
#define ADC_SMPR_CHANNEL_THRESHOLD  10

#define ADC_VBAT_VOLTAGE_DIVIDER    3

#define ADC_MEDIAN_FILTER_SIZE      9
#define ADC_CENTER_AVERAGE_SIZE     3

#if (STM32G4XX_REGISTERS_MCU_CATEGORY == 3)
#define ADC3_IS_MASTER              1
#define ADC3_SLAVE_INSTANCE         ADC_INSTANCE_ADC4
#else
#define ADC3_IS_MASTER              0
#define ADC3_SLAVE_INSTANCE         ADC_INSTANCE_LAST
#endif

#define ADC_REGISTER_MASK_SQR1      0x1F7DF7CF
#define ADC_REGISTER_MASK_SQR2      0x1F7DF7DF
#define ADC_REGISTER_MASK_SQR3      0x1F7DF7DF
#define ADC_REGISTER_MASK_SQR4      0x000007DF
#define ADC_REGISTER_MASK_OFR       0xFF000FFF

/*** ADC local structures ***/

/*******************************************************************/
typedef struct {
    ADC_registers_t* peripheral;
    ADCCR_registers_t* common_peripheral;
    uint8_t is_master;
    ADC_instance_t slave_instance;
    volatile uint32_t* rcc_enr;
    volatile uint32_t* rcc_reset;
    uint32_t rcc_mask;
    uint32_t rcc_ccipr_shift;
    uint8_t* sqx;
    uint8_t* extsel;
} ADC_descriptor_t;

/*******************************************************************/
typedef enum {
    ADC_MODE_NONE = 0,
    ADC_MODE_INDEPENDENT_SINGLE,
    ADC_MODE_INDEPENDENT_SEQUENCE,
    ADC_MODE_DUAL_SEQUENCE,
    ADC_MODE_LAST
} ADC_mode_t;

#if ((STM32G4XX_DRIVERS_ADC_MODE_MASK & ADC_MODE_MASK_SEQUENCE) != 0)
/*******************************************************************/
typedef struct {
    ADC_channel_configuration_t* sequence;
    uint8_t length;
    uint32_t sampling_frequency_hz;
    ADC_trigger_t trigger;
    ADC_trigger_detection_t trigger_detection;
} ADC_sequence_t;
#endif

/*******************************************************************/
typedef struct {
    ADC_mode_t mode;
} ADC_context_t;

/*** ADC local global variables ***/

//@formatter:off
#if ((STM32G4XX_DRIVERS_ADC_MODE_MASK & ADC_MODE_MASK_SEQUENCE) != 0)
static const uint32_t ADC_SMPX_CLOCK_CYCLE[ADC_SMPX_RANGE] = { 3, 7, 13, 25, 48, 93, 248, 641 };
#endif

#if ((STM32G4XX_DRIVERS_ADC_MODE_MASK & ADC_MODE_MASK_SEQUENCE) != 0)
static const uint32_t ADC_PRESCALER[ADC_CLOCK_PRESCALER_LAST] = { 1, 2, 4, 6, 8, 10, 12, 16, 32, 64, 128, 256 };
#endif

static const uint8_t ADC1_SQX[ADC_CHANNEL_LAST] = {
    ADC_SQX_ERROR, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, ADC_SQX_ERROR, 14, 15, ADC_SQX_ERROR, ADC_SQX_ERROR, ADC_SQX_ERROR,
    0, 17, 18, 16, 13, ADC_SQX_ERROR, ADC_SQX_ERROR, ADC_SQX_ERROR, ADC_SQX_ERROR, ADC_SQX_ERROR
};

static const uint8_t ADC2_SQX[ADC_CHANNEL_LAST] = {
    ADC_SQX_ERROR, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, ADC_SQX_ERROR, 17, ADC_SQX_ERROR,
    0, ADC_SQX_ERROR, ADC_SQX_ERROR, ADC_SQX_ERROR, ADC_SQX_ERROR, 16, 18, ADC_SQX_ERROR, ADC_SQX_ERROR, ADC_SQX_ERROR
};

#if ((STM32G4XX_REGISTERS_MCU_CATEGORY == 3) || (STM32G4XX_REGISTERS_MCU_CATEGORY == 4))
static const uint8_t ADC3_SQX[ADC_CHANNEL_LAST] = {
    ADC_SQX_ERROR, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, ADC_SQX_ERROR, 14, 15, 16, ADC_SQX_ERROR, ADC_SQX_ERROR,
#if (STM32G4XX_REGISTERS_MCU_CATEGORY == 3)
    0, 17, 18, ADC_SQX_ERROR, ADC_SQX_ERROR, ADC_SQX_ERROR, 13, ADC_SQX_ERROR, ADC_SQX_ERROR, ADC_SQX_ERROR
#endif
#if (STM32G4XX_REGISTERS_MCU_CATEGORY == 4)
    0, ADC_SQX_ERROR, 18, ADC_SQX_ERROR, ADC_SQX_ERROR, ADC_SQX_ERROR, 13, ADC_SQX_ERROR, ADC_SQX_ERROR, 17
#endif
};
#endif

#if (STM32G4XX_REGISTERS_MCU_CATEGORY == 3)
static const uint8_t ADC4_SQX[ADC_CHANNEL_LAST] = {
    ADC_SQX_ERROR, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, ADC_SQX_ERROR, ADC_SQX_ERROR,
    0, ADC_SQX_ERROR, 18, ADC_SQX_ERROR, ADC_SQX_ERROR, ADC_SQX_ERROR, ADC_SQX_ERROR, ADC_SQX_ERROR, ADC_SQX_ERROR, 17
};
#endif

#if (STM32G4XX_REGISTERS_MCU_CATEGORY == 3)
static const uint8_t ADC5_SQX[ADC_CHANNEL_LAST] = {
    ADC_SQX_ERROR, 1, 2, ADC_SQX_ERROR, ADC_SQX_ERROR, ADC_SQX_ERROR, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, ADC_SQX_ERROR, ADC_SQX_ERROR,
    0, 17, 18, 16, 4, ADC_SQX_ERROR, ADC_SQX_ERROR, 5, 3, ADC_SQX_ERROR
};
#endif

static const uint8_t ADC12_EXTSEL[ADC_TRIGGER_LAST] = {
    ADC_EXTSEL_ERROR,
    ADC_EXTSEL_ERROR, 0b00110,
    0b00000, 0b00001, 0b00010, 0b01001, 0b01010,
    ADC_EXTSEL_ERROR, 0b00011, ADC_EXTSEL_ERROR, 0b01011,
    ADC_EXTSEL_ERROR, ADC_EXTSEL_ERROR, 0b01111, 0b00100,
    ADC_EXTSEL_ERROR, 0b00101, 0b01100,
    0b01101,
    0b11110,
    0b00111, 0b01000,
    0b01110,
    0b10010, 0b10011, 0b10100, 0b10000, 0b10001,
    0b11101,
    0b10101, ADC_EXTSEL_ERROR, 0b10110, ADC_EXTSEL_ERROR, 0b10111, 0b11000, 0b11001, 0b11010, 0b11011, 0b11100,
};

#if ((STM32G4XX_REGISTERS_MCU_CATEGORY == 3) || (STM32G4XX_REGISTERS_MCU_CATEGORY == 4))
static const uint8_t ADC345_EXTSEL[ADC_TRIGGER_LAST] = {
    ADC_EXTSEL_ERROR,
    0b00101, ADC_EXTSEL_ERROR,
    ADC_EXTSEL_ERROR, ADC_EXTSEL_ERROR, 0b00010, 0b01001, 0b01010,
    0b01111, ADC_EXTSEL_ERROR, 0b00001, 0b01011,
    0b00000, ADC_EXTSEL_ERROR, 0b01011, 0b00100,
    0b00110, ADC_EXTSEL_ERROR, 0b01100,
    0b01101,
    0b11110,
    0b00111, 0b01000,
    0b01110,
    0b10010, ADC_EXTSEL_ERROR, ADC_EXTSEL_ERROR, 0b10000, 0b10001,
    0b11101,
    0b10101, 0b10011, 0b10110, 0b10100, 0b10111, 0b11000, 0b11001, 0b11010, 0b11011, 0b11100,
};
#endif

static const ADC_descriptor_t ADC_DESCRIPTOR[ADC_INSTANCE_LAST] = {
    { ADC1, ADCCR12, 1, ADC_INSTANCE_ADC2, &(RCC->AHB2ENR), &(RCC->AHB2RSTR), (0b1 << 13), 28, (uint8_t*) ADC1_SQX, (uint8_t*) ADC12_EXTSEL },
    { ADC2, ADCCR12, 0, ADC_INSTANCE_LAST, &(RCC->AHB2ENR), &(RCC->AHB2RSTR), (0b1 << 13), 28, (uint8_t*) ADC2_SQX, (uint8_t*) ADC12_EXTSEL },
#if ((STM32G4XX_REGISTERS_MCU_CATEGORY == 3) || (STM32G4XX_REGISTERS_MCU_CATEGORY == 4))
    { ADC3, ADCCR345, ADC3_IS_MASTER, ADC3_SLAVE_INSTANCE, &(RCC->AHB2ENR), &(RCC->AHB2RSTR), (0b1 << 14), 30, (uint8_t*) ADC3_SQX, (uint8_t*) ADC345_EXTSEL },
#endif
#if (STM32G4XX_REGISTERS_MCU_CATEGORY == 3)
    { ADC4, ADCCR345, 0, ADC_INSTANCE_LAST, &(RCC->AHB2ENR), &(RCC->AHB2RSTR), (0b1 << 14), 30, (uint8_t*) ADC4_SQX, (uint8_t*) ADC345_EXTSEL },
    { ADC5, ADCCR345, 0, ADC_INSTANCE_LAST, &(RCC->AHB2ENR), &(RCC->AHB2RSTR), (0b1 << 14), 30, (uint8_t*) ADC5_SQX, (uint8_t*) ADC345_EXTSEL }
#endif
};

static ADC_context_t adc_ctx[ADC_INSTANCE_LAST] = {
    [0 ... (ADC_INSTANCE_LAST - 1)] = {
        .mode = ADC_MODE_NONE
    }
};

/*** ADC local functions ***/

/*******************************************************************/
#define _ADC_check_instance(instance) { \
    /* Check instance */ \
    if (instance >= ADC_INSTANCE_LAST) { \
        status = ADC_ERROR_INSTANCE; \
        goto errors; \
    } \
}

/*******************************************************************/
#define _ADC_check_mode(instance, expected_mode) { \
    /* Check timer mode */ \
    if (adc_ctx[instance].mode != expected_mode) { \
        status = ADC_ERROR_MODE; \
        goto errors; \
    } \
}

/*******************************************************************/
static void __attribute__((optimize("-O0"))) _ADC_reset(ADC_instance_t instance) {
    // Local variables.
    uint8_t count = 0;
    // Perform manual reset and delay.
    (*ADC_DESCRIPTOR[instance].rcc_reset) |= (ADC_DESCRIPTOR[instance].rcc_mask);
    for (count = 0; count < 100; count++);
    (*ADC_DESCRIPTOR[instance].rcc_reset) &= ~(ADC_DESCRIPTOR[instance].rcc_mask);
}

#if ((STM32G4XX_DRIVERS_ADC_MODE_MASK & ADC_MODE_MASK_SEQUENCE) != 0)
/*******************************************************************/
static ADC_status_t _ADC_get_clock(ADC_instance_t instance, uint32_t *adc_clock_hz) {
    // Local variables.
    ADC_status_t status = ADC_SUCCESS;
    uint32_t system_clock_hz = 0;
    uint32_t pll_adc_clock_hz = 0;
    uint8_t ck_mode = ((ADC_DESCRIPTOR[instance].common_peripheral->CCR) >> 16) & 0x03;
    uint8_t presc = ((ADC_DESCRIPTOR[instance].common_peripheral->CCR) >> 18) & 0x0F;
    uint8_t adcxsel = ((RCC->CCIPR) >> ADC_DESCRIPTOR[instance].rcc_ccipr_shift) & 0x03;
    // Get RCC clocks frequency.
    RCC_get_frequency_hz(RCC_CLOCK_SYSTEM, &system_clock_hz);
    RCC_get_frequency_hz(RCC_CLOCK_ADC, &pll_adc_clock_hz);
    // Reset result.
    (*adc_clock_hz) = 0;
    // Check prescaler.
    if (presc >= ADC_CLOCK_PRESCALER_LAST) {
        status = ADC_ERROR_PRESC_VALUE;
        goto errors;
    }
    // Check clock mode.
    switch (ck_mode) {
    case 0b00:
        // Check RCC bits.
        switch (adcxsel) {
        case 0b01:
            (*adc_clock_hz) = pll_adc_clock_hz;
            break;
        case 0b10:
            (*adc_clock_hz) = system_clock_hz;
            break;
        default:
            status = ADC_ERROR_ADCXSEL_VALUE;
            goto errors;
        }
        break;
    case 0b01:
        (*adc_clock_hz) = (system_clock_hz >> 1);
        break;
    case 0b10:
        (*adc_clock_hz) = (system_clock_hz >> 2);
        break;
    case 0b11:
        (*adc_clock_hz) = system_clock_hz;
        break;
    default:
        status = ADC_ERROR_CK_MODE_VALUE;
        goto errors;
    }
    (*adc_clock_hz) /= ADC_PRESCALER[presc];
errors:
    return status;
}
#endif

#if ((STM32G4XX_DRIVERS_ADC_MODE_MASK & ADC_MODE_MASK_SEQUENCE) != 0)
/*******************************************************************/
static ADC_status_t _ADC_compute_sampling_time(ADC_instance_t instance, uint32_t sampling_frequency_hz, uint8_t sequence_length, uint8_t* smpx) {
    // Local variables.
    ADC_status_t status = ADC_SUCCESS;
    uint32_t adc_clock_hz = 0;
    uint32_t total_conversion_time_us = 0;
    uint32_t sampling_period_us = (MATH_POWER_10[6] / sampling_frequency_hz);
    uint8_t smpx_found = 0;
    uint8_t loop = 0;
    uint8_t idx = 0;
    // Update ADC clock frequency.
    status = _ADC_get_clock(instance, &adc_clock_hz);
    if (status != ADC_SUCCESS) goto errors;
    // Compute the highest SMPx value to reach the given sampling frequency regarding sequence length.
    for (loop = 0; loop < ADC_SMPX_RANGE; loop++) {
        // Read table in reverse order to get the highest value.
        idx = (ADC_SMPX_RANGE - loop - 1);
        // Compute conversion time.
        total_conversion_time_us = (sequence_length * ((((ADC_SMPX_CLOCK_CYCLE[idx] + ADC_RESOLUTION_BITS) * MATH_POWER_10[6]) / (adc_clock_hz)) + 1));
        // Check if conversion time is under the sampling period.
        if (total_conversion_time_us < sampling_period_us) {
            // Update output.
            (*smpx) = idx;
            smpx_found = 1;
            break;
        }
    }
    // Update status.
    if (smpx_found == 0) {
        status = ADC_ERROR_OVERRUN;
        goto errors;
    }
errors:
    return status;
}
#endif

/*******************************************************************/
static void _ADC_set_sampling_time(ADC_instance_t instance, uint8_t sqx, uint8_t smpx) {
    // Select register.
    if (sqx < ADC_SMPR_CHANNEL_THRESHOLD) {
        ADC_DESCRIPTOR[instance].peripheral->SMPR1 |= (smpx << (3 * sqx));
    }
    else {
        ADC_DESCRIPTOR[instance].peripheral->SMPR2 |= (smpx << (3 * (sqx - ADC_SMPR_CHANNEL_THRESHOLD)));
    }
}

/*******************************************************************/
static ADC_status_t _ADC_disable(ADC_instance_t instance) {
    // Local variables.
    ADC_status_t status = ADC_SUCCESS;
    uint32_t loop_count = 0;
    // Check ADC state.
    if (((ADC_DESCRIPTOR[instance].peripheral->CR) & (0b1 << 0)) == 0) goto errors;
    // Disable ADC.
    ADC_DESCRIPTOR[instance].peripheral->CR |= (0b1 << 1); // ADDIS='1'.
    // Wait for ADC to be disabled.
    while (((ADC_DESCRIPTOR[instance].peripheral->CR) & (0b1 << 0)) != 0) {
        // Exit if timeout.
        loop_count++;
        if (loop_count > ADC_TIMEOUT_COUNT) {
            status = ADC_ERROR_DISABLE_TIMEOUT;
            break;
        }
    }
errors:
    // Disable voltage regulator.
    ADC_DESCRIPTOR[instance].peripheral->CR &= ~(0b1 << 28);
    return status;
}

/*******************************************************************/
static ADC_status_t _ADC_enable_regulator(ADC_instance_t instance) {
    // Local variables.
    ADC_status_t status = ADC_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    // Exit deep power down.
    ADC_DESCRIPTOR[instance].peripheral->CR &= ~(0b1 << 29);
    // Enable voltage regulator.
    ADC_DESCRIPTOR[instance].peripheral->CR |= (0b1 << 28);
    // Wait for regulator startup time.
    lptim_status = LPTIM_delay_milliseconds(ADC_INIT_DELAY_MS_REGULATOR, LPTIM_DELAY_MODE_ACTIVE);
    LPTIM_exit_error(ADC_ERROR_BASE_LPTIM);
errors:
    return status;
}

/*******************************************************************/
static ADC_status_t _ADC_calibrate(ADC_instance_t instance) {
    // Local variables.
    ADC_status_t status = ADC_SUCCESS;
    uint32_t loop_count = 0;
    // Start calibration in single-ended mode.
    ADC_DESCRIPTOR[instance].peripheral->CR &= ~(0b1 << 30);
    ADC_DESCRIPTOR[instance].peripheral->CR |= (0b1 << 31);
    // Wait for calibration to complete.
    while (((ADC_DESCRIPTOR[instance].peripheral->CR) & (0b1 << 31)) != 0) {
        // Wait until calibration is done or timeout.
        loop_count++;
        if (loop_count > ADC_TIMEOUT_COUNT) {
            status = ADC_ERROR_CALIBRATION;
            goto errors;
        }
    }
errors:
    return status;
}

/*******************************************************************/
static ADC_status_t _ADC_init(ADC_instance_t instance, ADC_mode_t mode, ADC_clock_t adc_clock, ADC_clock_prescaler_t adc_clock_prescaler, const ADC_gpio_t* pins) {
    // Local variables.
    ADC_status_t status = ADC_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    uint8_t common_settings_update_flag = 1;
    uint8_t ck_mode = 0;
    uint8_t idx = 0;
    // Check mode.
    _ADC_check_mode(instance, ADC_MODE_NONE);
    // Check parameters.
    if (adc_clock_prescaler >= ADC_CLOCK_PRESCALER_LAST) {
        status = ADC_ERROR_CLOCK_PRESCALER;
        goto errors;
    }
    // Do not allow tightly coupled ADC to be independent at the same time.
    if ((mode == ADC_MODE_INDEPENDENT_SINGLE) || (mode == ADC_MODE_INDEPENDENT_SEQUENCE)) {
        // Check instance.
        switch (instance) {
        case ADC_INSTANCE_ADC1:
            _ADC_check_mode(ADC_INSTANCE_ADC2, ADC_MODE_NONE);
            break;
        case ADC_INSTANCE_ADC2:
            _ADC_check_mode(ADC_INSTANCE_ADC1, ADC_MODE_NONE);
            break;
#if (STM32G4XX_REGISTERS_MCU_CATEGORY == 3)
        case ADC_INSTANCE_ADC3:
            _ADC_check_mode(ADC_INSTANCE_ADC4, ADC_MODE_NONE);
            _ADC_check_mode(ADC_INSTANCE_ADC5, ADC_MODE_NONE);
            break;
        case ADC_INSTANCE_ADC4:
            _ADC_check_mode(ADC_INSTANCE_ADC3, ADC_MODE_NONE);
            _ADC_check_mode(ADC_INSTANCE_ADC5, ADC_MODE_NONE);
            break;
        case ADC_INSTANCE_ADC5:
            _ADC_check_mode(ADC_INSTANCE_ADC3, ADC_MODE_NONE);
            _ADC_check_mode(ADC_INSTANCE_ADC4, ADC_MODE_NONE);
            break;
#endif
        default:
            status = ADC_ERROR_INSTANCE;
            goto errors;
        }
    }
    // Do not reconfigure clock and do not reset peripheral in case of dual mode and slave instance.
    if ((mode == ADC_MODE_DUAL_SEQUENCE) && (ADC_DESCRIPTOR[instance].is_master == 0)) {
        common_settings_update_flag = 0;
    }
    // Init GPIOs.
    if (pins != NULL) {
        for (idx = 0; idx < (pins->list_size); idx++) {
            GPIO_configure(pins->list[idx], GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
        }
    }
    // Select clock source.
    if (common_settings_update_flag != 0) {
        // Disable peripheral clock.
        (*ADC_DESCRIPTOR[instance].rcc_enr) &= ~(ADC_DESCRIPTOR[instance].rcc_mask);
        // Select clock source.
        RCC->CCIPR &= ~(0b11 << ADC_DESCRIPTOR[instance].rcc_ccipr_shift);
        switch (adc_clock) {
        case ADC_CLOCK_SYSCLK:
            RCC->CCIPR |= (0b10 << ADC_DESCRIPTOR[instance].rcc_ccipr_shift);
            break;
        case ADC_CLOCK_PLL:
            RCC->CCIPR |= (0b01 << ADC_DESCRIPTOR[instance].rcc_ccipr_shift);
            break;
        case ADC_CLOCK_HCLK_DIV_1:
            ck_mode = 0b11;
            break;
        case ADC_CLOCK_HCLK_DIV_2:
            ck_mode = 0b01;
            break;
        case ADC_CLOCK_HCLK_DIV_4:
            ck_mode = 0b10;
            break;
        default:
            status = ADC_ERROR_CLOCK;
            goto errors;
        }
        // Reset peripheral.
        _ADC_reset(instance);
    }
    // Enable peripheral clock.
    (*ADC_DESCRIPTOR[instance].rcc_enr) |= ADC_DESCRIPTOR[instance].rcc_mask;
    // Clear ready flag.
    if (((ADC_DESCRIPTOR[instance].peripheral->ISR) & (0b1 << 0)) != 0) {
        ADC_DESCRIPTOR[instance].peripheral->ISR |= (0b1 << 0);
    }
    // Ensure ADC is disabled.
    status = _ADC_disable(instance);
    if (status != ADC_SUCCESS) goto errors;
    // Select peripheral clock.
    if (common_settings_update_flag != 0) {
        ADC_DESCRIPTOR[instance].common_peripheral->CCR &= ~(0b11 << 16);
        ADC_DESCRIPTOR[instance].common_peripheral->CCR |= (ck_mode << 16);
        ADC_DESCRIPTOR[instance].common_peripheral->CCR &= ~(0b1111 << 18);
        ADC_DESCRIPTOR[instance].common_peripheral->CCR |= (adc_clock_prescaler << 18);
    }
    // Single conversion mode.
    ADC_DESCRIPTOR[instance].peripheral->CFGR &= ~(0b1 << 13);
    ADC_DESCRIPTOR[instance].peripheral->CFGR &= ~(0b1 << 16);
    // 12-bits resolution and right alignment.
    ADC_DESCRIPTOR[instance].peripheral->CFGR &= ~(0b1 << 15);
    ADC_DESCRIPTOR[instance].peripheral->CFGR &= ~(0b11 << 3);
    // Overwrite register in case of overrun.
    ADC_DESCRIPTOR[instance].peripheral->CFGR |= (0b1 << 12);
    // Specific settings for each mode.
    switch (mode) {
    case ADC_MODE_INDEPENDENT_SINGLE:
        // ADC mode.
        ADC_DESCRIPTOR[instance].common_peripheral->CCR &= ~(0b11111 << 0);
        break;
    case ADC_MODE_INDEPENDENT_SEQUENCE:
        // ADC mode.
        ADC_DESCRIPTOR[instance].common_peripheral->CCR &= ~(0b11111 << 0);
        // Enable circular DMA.
        ADC_DESCRIPTOR[instance].peripheral->CFGR |= (0b11 << 0);
        break;
    case ADC_MODE_DUAL_SEQUENCE:
        // Enable circular DMA.
        ADC_DESCRIPTOR[instance].peripheral->CFGR |= (0b11 << 0);
        // ADC mode.
        ADC_DESCRIPTOR[instance].common_peripheral->CCR &= ~(0b11111 << 0);
        ADC_DESCRIPTOR[instance].common_peripheral->CCR |= (0b00110 << 0);
        // Use independent circular DMA channel for each ADC.
        ADC_DESCRIPTOR[instance].common_peripheral->CCR &= ~(0b11 << 14);
        ADC_DESCRIPTOR[instance].common_peripheral->CCR |= (0b1 << 13);
        break;
    default:
        status = ADC_ERROR_MODE;
        goto errors;
    }
    // Software trigger by default.
    ADC_DESCRIPTOR[instance].peripheral->CFGR &= ~(0b1111111 << 5);
    // Enable regulator.
    status = _ADC_enable_regulator(instance);
    if (status != ADC_SUCCESS) goto errors;
    // Calibration.
    status = _ADC_calibrate(instance);
    if (status != ADC_SUCCESS) goto errors;
    // Wake-up VREFINT, VBAT and temperature sensor.
    ADC_DESCRIPTOR[instance].common_peripheral->CCR |= (0b111 << 22);
    // Wait for startup.
    lptim_status = LPTIM_delay_milliseconds(ADC_INIT_DELAY_MS_VBAT_VREF_TS, LPTIM_DELAY_MODE_ACTIVE);
    LPTIM_exit_error(ADC_ERROR_BASE_LPTIM);
    // Update mode.
    adc_ctx[instance].mode = mode;
errors:
    return status;
}

/*******************************************************************/
static ADC_status_t _ADC_de_init(ADC_instance_t instance) {
    // Local variables.
    ADC_status_t status = ADC_SUCCESS;
    // Disable ADC.
    status = _ADC_disable(instance);
    // Turn internal channels off.
    ADC_DESCRIPTOR[instance].common_peripheral->CCR &= ~(0b111 << 22);
    // Disable peripheral clock.
    (*ADC_DESCRIPTOR[instance].rcc_enr) &= ~(ADC_DESCRIPTOR[instance].rcc_mask);
    // Update mode.
    adc_ctx[instance].mode = ADC_MODE_NONE;
    return status;
}

#if ((STM32G4XX_DRIVERS_ADC_MODE_MASK & ADC_MODE_MASK_SEQUENCE) != 0)
/*******************************************************************/
static ADC_status_t _ADC_configure_sequence(ADC_instance_t instance, ADC_sequence_t* adc_sequence) {
    // Local variables.
    ADC_status_t status = ADC_SUCCESS;
    uint8_t sqx = 0;
    uint8_t sqx_list[ADC_SEQUENCE_LENGTH_MAX];
    uint8_t smpx = 0;
    int32_t offset = 0;
    uint8_t ofr_idx = 0;
    uint8_t idx = 0;
    // Sequence loop.
    for (idx = 0; idx < (adc_sequence->length); idx++) {
        // Compute channels indexes.
        sqx = ADC_DESCRIPTOR[instance].sqx[(adc_sequence->sequence[idx]).channel];
        if (sqx >= ADC_CHANNEL_LAST) {
            status = ADC_ERROR_CHANNEL;
            goto errors;
        }
        sqx_list[idx] = sqx;
    }
    // Compute sampling time.
    status = _ADC_compute_sampling_time(instance, (adc_sequence->sampling_frequency_hz), (adc_sequence->length), &smpx);
    if (status != ADC_SUCCESS) goto errors;
    // Update sampling time on selected channels.
    for (idx = 0; idx < (adc_sequence->length); idx++) {
        _ADC_set_sampling_time(instance, sqx_list[idx], smpx);
    }
    // Regular sequence definition.
    ADC_DESCRIPTOR[instance].peripheral->SQR1 &= (~ADC_REGISTER_MASK_SQR1);
    ADC_DESCRIPTOR[instance].peripheral->SQR2 &= (~ADC_REGISTER_MASK_SQR2);
    ADC_DESCRIPTOR[instance].peripheral->SQR3 &= (~ADC_REGISTER_MASK_SQR3);
    ADC_DESCRIPTOR[instance].peripheral->SQR4 &= (~ADC_REGISTER_MASK_SQR4);
    // Sequence length.
    ADC_DESCRIPTOR[instance].peripheral->SQR1 |= (((adc_sequence->length) - 1) & 0x0000000F);
    // Channels sequence.
    for (idx = 0; idx < (adc_sequence->length); idx++) {
        ADC_DESCRIPTOR[instance].peripheral->SQR[idx >> 2] |= (sqx_list[idx] << (6 * ((idx % 4) + 1)));
    }
    // Configure offset.
    for (idx = 0; idx < (adc_sequence->length); idx++) {
        // Get offset.
        offset = (adc_sequence->sequence[idx]).offset_12bits;
        // Check offset.
        if (offset != 0) {
            // Check range.
            if ((offset > ADC_FULL_SCALE) || (offset < ((-1) * ADC_FULL_SCALE))) {
                status = ADC_ERROR_OFFSET;
                goto errors;
            }
            // Check if there is remaining register.
            if (ofr_idx >= ADC_NUMBER_OF_OFFSETS) {
                status = ADC_ERROR_OFFSET_NOT_CONFIGURABLE;
                goto errors;
            }
            // Program offset.
            ADC_DESCRIPTOR[instance].peripheral->OFR[ofr_idx] &= (~ADC_REGISTER_MASK_OFR);
            ADC_DESCRIPTOR[instance].peripheral->OFR[ofr_idx] |= (offset & 0x00000FFF);
            ADC_DESCRIPTOR[instance].peripheral->OFR[ofr_idx] |= sqx_list[idx] << 26;
            // Set sign.
            if (offset > 0) {
                ADC_DESCRIPTOR[instance].peripheral->OFR[ofr_idx] |= (0b1 << 24);
            }
            else {
                ADC_DESCRIPTOR[instance].peripheral->OFR[ofr_idx] &= ~(0b1 << 24);
            }
            ADC_DESCRIPTOR[instance].peripheral->OFR[ofr_idx] |= (0b1 << 31);
            ofr_idx++;
        }
    }
    // Check external trigger.
    if ((adc_sequence->trigger) != ADC_TRIGGER_SOFTWARE) {
        // Set detection.
        ADC_DESCRIPTOR[instance].peripheral->CFGR |= ((adc_sequence->trigger_detection) << 10);
        ADC_DESCRIPTOR[instance].peripheral->CFGR |= (ADC_DESCRIPTOR[instance].extsel[adc_sequence->trigger] << 5);
    }
errors:
    return status;
}
#endif

/*******************************************************************/
static ADC_status_t _ADC_start(ADC_instance_t instance) {
    // Local variables.
    ADC_status_t status = ADC_SUCCESS;
    uint32_t loop_count = 0;
    // Enable ADC.
    ADC_DESCRIPTOR[instance].peripheral->CR |= (0b1 << 0);
    // Wait for ADC to be ready.
    while (((ADC_DESCRIPTOR[instance].peripheral->ISR) & (0b1 << 0)) == 0) {
        // Wait for ADRDY='1' or timeout.
        loop_count++;
        if (loop_count > ADC_TIMEOUT_COUNT) {
            status = ADC_ERROR_READY_TIMEOUT;
            goto errors;
        }
    }
    // Start regular conversions.
    ADC_DESCRIPTOR[instance].peripheral->CR |= (0b1 << 2);
errors:
    return status;
}

#if ((STM32G4XX_DRIVERS_ADC_MODE_MASK & ADC_MODE_MASK_SEQUENCE) != 0)
/*******************************************************************/
static ADC_status_t _ADC_stop(ADC_instance_t instance) {
    // Local variables.
    ADC_status_t status = ADC_SUCCESS;
    uint32_t loop_count = 0;
    // Stop ongoing conversions.
    ADC_DESCRIPTOR[instance].peripheral->CR |= (0b1 << 4);
    // Wait for ongoing conversions to complete.
    while (((ADC_DESCRIPTOR[instance].peripheral->CR) & (0b1 << 4)) != 0) {
        // Wait command completion or timeout.
        loop_count++;
        if (loop_count > ADC_TIMEOUT_COUNT) break;
    }
    // Disable ADC.
    if (((ADC_DESCRIPTOR[instance].peripheral->CR) & (0b1 << 0)) != 0) {
        ADC_DESCRIPTOR[instance].peripheral->CR |= (0b1 << 1); // ADDIS='1'.
    }
    return status;
}
#endif

/*******************************************************************/
static ADC_status_t _ADC_single_conversion(ADC_instance_t instance, uint8_t sqx, int32_t* adc_data_12bits) {
    // Local variables.
    ADC_status_t status = ADC_SUCCESS;
    uint32_t loop_count = 0;
    // Maximum sampling time.
    _ADC_set_sampling_time(instance, sqx, (ADC_SMPX_RANGE - 1));
    // Regular sequence definition.
    ADC_DESCRIPTOR[instance].peripheral->SQR1 &= (~ADC_REGISTER_MASK_SQR1);
    ADC_DESCRIPTOR[instance].peripheral->SQR1 |= (sqx << 6);
    // Clear conversion flags.
    ADC_DESCRIPTOR[instance].peripheral->ISR |= (0b1111 << 1);
    // Start conversion.
    status = _ADC_start(instance);
    if (status != ADC_SUCCESS) goto errors;
    // Wait for conversion to complete.
    while ((ADC_DESCRIPTOR[instance].peripheral->ISR & (0b1 << 3)) == 0) {
        // Exit if timeout.
        loop_count++;
        if (loop_count > ADC_TIMEOUT_COUNT) {
            status = ADC_ERROR_CONVERSION_TIMEOUT;
            goto errors;
        }
    }
    (*adc_data_12bits) = (int32_t) ((ADC_DESCRIPTOR[instance].peripheral->DR) & ADC_FULL_SCALE);
errors:
    return status;
}

/*******************************************************************/
ADC_status_t _ADC_compute_tmcu(int32_t vref_mv, int32_t tmcu_12bits, int32_t* tmcu_degrees) {
    // Local variables.
    ADC_status_t status = ADC_SUCCESS;
    int32_t raw_temp_calib_mv = 0;
    int32_t temp_calib_degrees = 0;
    // Check parameters.
    if (tmcu_12bits > ADC_FULL_SCALE) {
        status = ADC_ERROR_DATA;
        goto errors;
    }
    if (tmcu_degrees == NULL) {
        status = ADC_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Compute temperature according to MCU factory calibration.
    raw_temp_calib_mv = ((tmcu_12bits * vref_mv) / (ADC_TS_VCC_CALIB_MV)) - ADC_TS_CAL1;
    temp_calib_degrees = raw_temp_calib_mv * (ADC_TS_CAL2_TEMP - ADC_TS_CAL1_TEMP);
    temp_calib_degrees = (temp_calib_degrees) / (ADC_TS_CAL2 - ADC_TS_CAL1);
    (*tmcu_degrees) = temp_calib_degrees + ADC_TS_CAL1_TEMP;
errors:
    return status;
}

/*** ADC functions ***/

#if ((STM32G4XX_DRIVERS_ADC_MODE_MASK & ADC_MODE_MASK_SINGLE) != 0)
/*******************************************************************/
ADC_status_t ADC_SGL_init(ADC_instance_t instance, const ADC_gpio_t* pins, ADC_SGL_configuration_t* adc_configuration) {
    // Local variables.
    ADC_status_t status = ADC_SUCCESS;
    // Check instance.
    _ADC_check_instance(instance);
    // Check parameters.
    if (adc_configuration == NULL) {
        status = ADC_ERROR_NULL_PARAMETER;
        goto errors;
    }
    status = _ADC_init(instance, ADC_MODE_INDEPENDENT_SINGLE, (adc_configuration->clock), (adc_configuration->clock_prescaler), pins);
    if (status != ADC_SUCCESS) goto errors;
errors:
    return status;
}
#endif

#if ((STM32G4XX_DRIVERS_ADC_MODE_MASK & ADC_MODE_MASK_SINGLE) != 0)
/*******************************************************************/
ADC_status_t ADC_SGL_de_init(ADC_instance_t instance) {
    // Local variables.
    ADC_status_t status = ADC_SUCCESS;
    // Check instance and mode.
    _ADC_check_instance(instance);
    // Release ADC.
    status = _ADC_de_init(instance);
errors:
    return status;
}
#endif

#if ((STM32G4XX_DRIVERS_ADC_MODE_MASK & ADC_MODE_MASK_SINGLE) != 0)
/*******************************************************************/
ADC_status_t ADC_SGL_convert_channel(ADC_instance_t instance, ADC_channel_t channel, int32_t* adc_data_12bits) {
    // Local variables.
    ADC_status_t status = ADC_SUCCESS;
    MATH_status_t math_status = MATH_SUCCESS;
    int32_t adc_sample_buf[ADC_MEDIAN_FILTER_SIZE] = { 0x00 };
    uint8_t idx = 0;
    // Check mode.
    _ADC_check_mode(instance, ADC_MODE_INDEPENDENT_SINGLE);
    // Check parameters.
    if (instance >= ADC_INSTANCE_LAST) {
        status = ADC_ERROR_INSTANCE;
        goto errors;
    }
    if (channel >= ADC_CHANNEL_LAST) {
        status = ADC_ERROR_CHANNEL;
        goto errors;
    }
    if (adc_data_12bits == NULL) {
        status = ADC_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Perform all conversions.
    for (idx = 0; idx < ADC_MEDIAN_FILTER_SIZE; idx++) {
        status = _ADC_single_conversion(instance, ADC_DESCRIPTOR[instance].sqx[channel], &(adc_sample_buf[idx]));
        if (status != ADC_SUCCESS) goto errors;
    }
    // Apply median filter.
    math_status = MATH_median_filter(adc_sample_buf, ADC_MEDIAN_FILTER_SIZE, ADC_CENTER_AVERAGE_SIZE, adc_data_12bits);
    MATH_exit_error(ADC_ERROR_BASE_MATH);
errors:
    return status;
}
#endif

#if ((STM32G4XX_DRIVERS_ADC_MODE_MASK & ADC_MODE_MASK_SEQUENCE) != 0)
/*******************************************************************/
ADC_status_t ADC_SQC_init(ADC_instance_t instance, const ADC_gpio_t* pins, ADC_SQC_configuration_t* adc_configuration) {
    // Local variables.
    ADC_status_t status = ADC_SUCCESS;
    ADC_mode_t adc_mode = ADC_MODE_LAST;
    ADC_sequence_t adc_sequence;
    // Check instance and mode.
    _ADC_check_instance(instance);
    // Check parameters.
    if (adc_configuration == NULL) {
        status = ADC_ERROR_NULL_PARAMETER;
        goto errors;
    }
    if (adc_configuration->master_sequence == NULL) {
        status = ADC_ERROR_NULL_PARAMETER;
        goto errors;
    }
    if ((adc_configuration->slave_sequence != NULL) && (ADC_DESCRIPTOR[instance].is_master == 0)) {
        status = ADC_ERROR_INSTANCE_NOT_MASTER;
        goto errors;
    }
    if (((adc_configuration->sequence_length) == 0) || ((adc_configuration->sequence_length) > ADC_SEQUENCE_LENGTH_MAX)) {
        status = ADC_ERROR_SEQUENCE_LENGTH;
        goto errors;
    }
    if ((adc_configuration->trigger) >= ADC_TRIGGER_LAST) {
        status = ADC_ERROR_TRIGGER;
        goto errors;
    }
    if ((adc_configuration->trigger_detection) >= ADC_TRIGGER_DETECTION_LAST) {
        status = ADC_ERROR_TRIGGER_DETECTION;
        goto errors;
    }
    // Update mode.
    adc_mode = (adc_configuration->slave_sequence == NULL) ? ADC_MODE_INDEPENDENT_SEQUENCE : ADC_MODE_DUAL_SEQUENCE;
    // Init ADC.
    status = _ADC_init(instance, adc_mode, (adc_configuration->clock), (adc_configuration->clock_prescaler), pins);
    if (status != ADC_SUCCESS) goto errors;
    // Build sequence configuration.
    adc_sequence.sequence = (adc_configuration->master_sequence);
    adc_sequence.length = (adc_configuration->sequence_length);
    adc_sequence.sampling_frequency_hz = (adc_configuration->sampling_frequency_hz);
    adc_sequence.trigger = (adc_configuration->trigger);
    adc_sequence.trigger_detection = (adc_configuration->trigger_detection);
    // Init master sequence.
    status = _ADC_configure_sequence(instance, &adc_sequence);
    if (status != ADC_SUCCESS) goto errors;
    // Init slave if needed.
    if (adc_mode == ADC_MODE_DUAL_SEQUENCE) {
        // Init slave ADC.
        status = _ADC_init(ADC_DESCRIPTOR[instance].slave_instance, adc_mode, (adc_configuration->clock), (adc_configuration->clock_prescaler), NULL);
        if (status != ADC_SUCCESS) goto errors;
        // Build sequence configuration.
        adc_sequence.sequence = (adc_configuration->slave_sequence);
        // Init slave sequence.
        status = _ADC_configure_sequence(ADC_DESCRIPTOR[instance].slave_instance, &adc_sequence);
        if (status != ADC_SUCCESS) goto errors;
    }
errors:
    return status;
}
#endif

#if ((STM32G4XX_DRIVERS_ADC_MODE_MASK & ADC_MODE_MASK_SEQUENCE) != 0)
/*******************************************************************/
ADC_status_t ADC_SQC_de_init(ADC_instance_t instance) {
    // Local variables.
    ADC_status_t status = ADC_SUCCESS;
    // Check instance.
    _ADC_check_instance(instance);
    // Release slave ADC if needed.
    if (adc_ctx[instance].mode == ADC_MODE_DUAL_SEQUENCE) {
        status = _ADC_de_init(ADC_DESCRIPTOR[instance].slave_instance);
    }
    // Release ADC.
    status = _ADC_de_init(instance);
errors:
    return status;
}
#endif

#if ((STM32G4XX_DRIVERS_ADC_MODE_MASK & ADC_MODE_MASK_SEQUENCE) != 0)
/*******************************************************************/
ADC_status_t ADC_SQC_start(ADC_instance_t instance) {
    // Local variables.
    ADC_status_t status = ADC_SUCCESS;
    // Check instance.
    _ADC_check_instance(instance);
    // Check mode.
    switch (adc_ctx[instance].mode) {
    case ADC_MODE_INDEPENDENT_SEQUENCE:
        status = _ADC_start(instance);
        if (status != ADC_SUCCESS) goto errors;
        break;
    case ADC_MODE_DUAL_SEQUENCE:
        status = _ADC_start(instance);
        if (status != ADC_SUCCESS) goto errors;
        status = _ADC_start(ADC_DESCRIPTOR[instance].slave_instance);
        if (status != ADC_SUCCESS) goto errors;
        break;
    default:
        status = ADC_ERROR_MODE;
        goto errors;
    }
errors:
    return status;
}
#endif

#if ((STM32G4XX_DRIVERS_ADC_MODE_MASK & ADC_MODE_MASK_SEQUENCE) != 0)
/*******************************************************************/
ADC_status_t ADC_SQC_stop(ADC_instance_t instance) {
    // Local variables.
    ADC_status_t status = ADC_SUCCESS;
    // Check instance.
    _ADC_check_instance(instance);
    // Check mode.
    switch (adc_ctx[instance].mode) {
    case ADC_MODE_INDEPENDENT_SEQUENCE:
        status = _ADC_stop(instance);
        if (status != ADC_SUCCESS) goto errors;
        break;
    case ADC_MODE_DUAL_SEQUENCE:
        status = _ADC_stop(instance);
        if (status != ADC_SUCCESS) goto errors;
        status = _ADC_stop(ADC_DESCRIPTOR[instance].slave_instance);
        if (status != ADC_SUCCESS) goto errors;
        break;
    default:
        status = ADC_ERROR_MODE;
        goto errors;
    }
errors:
    return status;
}
#endif

#ifdef STM32G4XX_DRIVERS_ADC_VREF_MV
/*******************************************************************/
ADC_status_t ADC_compute_vmcu(int32_t vbat_12bits, int32_t* vmcu_mv) {
    // Local variables.
    ADC_status_t status = ADC_SUCCESS;
    // Check parameters.
    if (vbat_12bits > ADC_FULL_SCALE) {
        status = ADC_ERROR_DATA;
        goto errors;
    }
    if (vmcu_mv == NULL) {
        status = ADC_ERROR_NULL_PARAMETER;
        goto errors;
    }
    (*vmcu_mv) = (vbat_12bits * STM32G4XX_DRIVERS_ADC_VREF_MV * ADC_VBAT_VOLTAGE_DIVIDER) / (ADC_FULL_SCALE);
errors:
    return status;
}
#else
/*******************************************************************/
ADC_status_t ADC_compute_vmcu(int32_t ref_voltage_12bits, int32_t ref_voltage_mv, int32_t* vmcu_mv) {
    // Local variables.
    ADC_status_t status = ADC_SUCCESS;
    // Check parameters.
    if (ref_voltage_12bits > ADC_FULL_SCALE) {
        status = ADC_ERROR_DATA;
        goto errors;
    }
    if (vmcu_mv == NULL) {
        status = ADC_ERROR_NULL_PARAMETER;
        goto errors;
    }
    (*vmcu_mv) = (ref_voltage_mv * ADC_FULL_SCALE) / (ref_voltage_12bits);
errors:
    return status;
}
#endif

#ifdef STM32G4XX_DRIVERS_ADC_VREF_MV
/*******************************************************************/
ADC_status_t ADC_compute_tmcu(int32_t tmcu_12bits, int32_t* tmcu_degrees) {
    return _ADC_compute_tmcu(STM32G4XX_DRIVERS_ADC_VREF_MV, tmcu_12bits, tmcu_degrees);
}
#else
/*******************************************************************/
ADC_status_t ADC_compute_tmcu(int32_t vref_mv, int32_t tmcu_12bits, int32_t* tmcu_degrees) {
    return _ADC_compute_tmcu(vref_mv, tmcu_12bits, tmcu_degrees);
}
#endif

/*******************************************************************/
int32_t ADC_get_vrefint_voltage_mv(void) {
    // Local variables.
    int32_t vrefint_mv = ((ADC_VREFINT_CAL * ADC_VREFINT_VCC_CALIB_MV) / (ADC_FULL_SCALE));
    return vrefint_mv;
}

/*******************************************************************/
uint32_t ADC_get_master_dr_register_address(ADC_instance_t instance) {
    // Local variables.
    ADC_status_t status = ADC_SUCCESS;
    uint32_t master_dr_address = 0;
    // Check instance.
    _ADC_check_instance(instance);
    // Update address.
    master_dr_address = ((uint32_t) &(ADC_DESCRIPTOR[instance].peripheral->DR));
errors:
    UNUSED(status);
    return master_dr_address;
}
/*******************************************************************/
uint32_t ADC_get_slave_dr_register_address(ADC_instance_t instance) {
    // Local variables.
    ADC_status_t status = ADC_SUCCESS;
    uint32_t slave_dr_address = 0;
    // Check instance.
    _ADC_check_instance(instance);
    _ADC_check_instance(ADC_DESCRIPTOR[instance].slave_instance);
    // Update address.
    slave_dr_address = ((uint32_t) &(ADC_DESCRIPTOR[ADC_DESCRIPTOR[instance].slave_instance].peripheral->DR));
errors:
    UNUSED(status);
    return slave_dr_address;
}

#endif /* STM32G4XX_DRIVERS_DISABLE */
