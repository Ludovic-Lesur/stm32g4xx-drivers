/*
 * adc.h
 *
 *  Created on: 02 feb. 2025
 *      Author: Ludo
 */

#ifndef STM32G4XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32g4xx_drivers_flags.h"
#endif

#ifndef STM32G4XX_DRIVERS_DISABLE

#ifndef __ADC_H__
#define __ADC_H__

#ifndef STM32G4XX_REGISTERS_DISABLE_FLAGS_FILE
#include "stm32g4xx_registers_flags.h"
#endif
#include "error.h"
#include "gpio.h"
#include "lptim.h"
#include "maths.h"
#include "types.h"

/*** ADC macros ***/

#define ADC_RESOLUTION_BITS             12
#define ADC_FULL_SCALE                  ((1 << ADC_RESOLUTION_BITS) - 1)

#define ADC_INIT_DELAY_MS_REGULATOR     5
#define ADC_INIT_DELAY_MS_VBAT_VREF_TS  10

#define ADC_INIT_DELAY_MS               (ADC_INIT_DELAY_MS_REGULATOR + ADC_INIT_DELAY_MS_VBAT_VREF_TS)

#define ADC_MODE_MASK_SINGLE            0x01
#define ADC_MODE_MASK_SEQUENCE          0x02

#define ADC_MODE_MASK_ALL               0x03

/*** ADC structures ***/

/*!******************************************************************
 * \enum ADC_status_t
 * \brief ADC driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    ADC_SUCCESS = 0,
    ADC_ERROR_NULL_PARAMETER,
    ADC_ERROR_INSTANCE,
    ADC_ERROR_INSTANCE_NOT_MASTER,
    ADC_ERROR_MODE,
    ADC_ERROR_DISABLE_TIMEOUT,
    ADC_ERROR_CALIBRATION,
    ADC_ERROR_READY_TIMEOUT,
    ADC_ERROR_CHANNEL,
    ADC_ERROR_INTERNAL_CHANNEL,
    ADC_ERROR_CLOCK,
    ADC_ERROR_CLOCK_PRESCALER,
    ADC_ERROR_ADCXSEL_VALUE,
    ADC_ERROR_CK_MODE_VALUE,
    ADC_ERROR_PRESC_VALUE,
    ADC_ERROR_SEQUENCE_LENGTH,
    ADC_ERROR_SAMPLING_FREQUENCY,
    ADC_ERROR_OVERRUN,
    ADC_ERROR_OFFSET,
    ADC_ERROR_OFFSET_NOT_CONFIGURABLE,
    ADC_ERROR_TRIGGER,
    ADC_ERROR_TRIGGER_SOFTWARE_REQUIRED,
    ADC_ERROR_TRIGGER_DETECTION,
    ADC_ERROR_DATA,
    ADC_ERROR_CONVERSION_TIMEOUT,
    // Low level drivers errors.
    ADC_ERROR_BASE_LPTIM = ERROR_BASE_STEP,
    ADC_ERROR_BASE_MATH = (ADC_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST),
    // Last base value.
    ADC_ERROR_BASE_LAST = (ADC_ERROR_BASE_MATH + MATH_ERROR_BASE_LAST)
} ADC_status_t;

/*!******************************************************************
 * \enum ADC_instance_t
 * \brief ADC instances list.
 *******************************************************************/
typedef enum {
    ADC_INSTANCE_ADC1 = 0,
    ADC_INSTANCE_ADC2,
#if ((STM32G4XX_REGISTERS_MCU_CATEGORY == 3) || (STM32G4XX_REGISTERS_MCU_CATEGORY == 4))
    ADC_INSTANCE_ADC3,
#endif
#if (STM32G4XX_REGISTERS_MCU_CATEGORY == 3)
    ADC_INSTANCE_ADC4,
    ADC_INSTANCE_ADC5,
#endif
    ADC_INSTANCE_LAST
} ADC_instance_t;

/*!******************************************************************
 * \enum ADC_channel_t
 * \brief ADC channels list.
 *******************************************************************/
typedef enum {
    // External channels.
    ADC_CHANNEL_IN0 = 0,
    ADC_CHANNEL_IN1,
    ADC_CHANNEL_IN2,
    ADC_CHANNEL_IN3,
    ADC_CHANNEL_IN4,
    ADC_CHANNEL_IN5,
    ADC_CHANNEL_IN6,
    ADC_CHANNEL_IN7,
    ADC_CHANNEL_IN8,
    ADC_CHANNEL_IN9,
    ADC_CHANNEL_IN10,
    ADC_CHANNEL_IN11,
    ADC_CHANNEL_IN12,
    ADC_CHANNEL_IN13,
    ADC_CHANNEL_IN14,
    ADC_CHANNEL_IN15,
    ADC_CHANNEL_IN16,
    ADC_CHANNEL_IN17,
    ADC_CHANNEL_IN18,
    // Internal channels.
    ADC_CHANNEL_VSSA,
    ADC_CHANNEL_VBAT,
    ADC_CHANNEL_VREFINT,
    ADC_CHANNEL_TEMPERATURE_SENSOR,
    ADC_CHANNEL_OPAMP1,
    ADC_CHANNEL_OPAMP2,
    ADC_CHANNEL_OPAMP3,
    ADC_CHANNEL_OPAMP4,
    ADC_CHANNEL_OPAMP5,
    ADC_CHANNEL_OPAMP6,
    // Last index.
    ADC_CHANNEL_LAST
} ADC_channel_t;

/*!******************************************************************
 * \struct ADC_DM_channel_t
 * \brief ADC dual mode channel configuration.
 *******************************************************************/
typedef struct {
    ADC_channel_t channel;
    int32_t offset_12bits;
} ADC_channel_configuration_t;

/*!******************************************************************
 * \struct ADC_gpio_t
 * \brief ADC GPIO pins list.
 *******************************************************************/
typedef struct {
    const GPIO_pin_t** list;
    uint8_t list_size;
} ADC_gpio_t;

/*!******************************************************************
 * \enum ADC_clock_t
 * \brief ADC clock sources list.
 *******************************************************************/
typedef enum {
    ADC_CLOCK_SYSCLK = 0,
    ADC_CLOCK_PLL,
    ADC_CLOCK_HCLK_DIV_1,
    ADC_CLOCK_HCLK_DIV_2,
    ADC_CLOCK_HCLK_DIV_4,
    ADC_CLOCK_LAST
} ADC_clock_t;

/*!******************************************************************
 * \enum ADC_clock_prescaler_t
 * \brief ADC clock prescaler list.
 *******************************************************************/
typedef enum {
    ADC_CLOCK_PRESCALER_NONE = 0,
    ADC_CLOCK_PRESCALER_2,
    ADC_CLOCK_PRESCALER_4,
    ADC_CLOCK_PRESCALER_6,
    ADC_CLOCK_PRESCALER_8,
    ADC_CLOCK_PRESCALER_10,
    ADC_CLOCK_PRESCALER_12,
    ADC_CLOCK_PRESCALER_16,
    ADC_CLOCK_PRESCALER_32,
    ADC_CLOCK_PRESCALER_64,
    ADC_CLOCK_PRESCALER_128,
    ADC_CLOCK_PRESCALER_256,
    ADC_CLOCK_PRESCALER_LAST
} ADC_clock_prescaler_t;

/*!******************************************************************
 * \enum ADC_channel_t
 * \brief ADC channels list.
 *******************************************************************/
typedef enum {
    ADC_TRIGGER_SOFTWARE = 0,
    ADC_TRIGGER_EXTI2,
    ADC_TRIGGER_EXTI11,
    ADC_TRIGGER_TIM1_CC1,
    ADC_TRIGGER_TIM1_CC2,
    ADC_TRIGGER_TIM1_CC3,
    ADC_TRIGGER_TIM1_TRGO,
    ADC_TRIGGER_TIM1_TRGO2,
    ADC_TRIGGER_TIM2_CC1,
    ADC_TRIGGER_TIM2_CC2,
    ADC_TRIGGER_TIM2_CC3,
    ADC_TRIGGER_TIM2_TRGO,
    ADC_TRIGGER_TIM3_CC1,
    ADC_TRIGGER_TIM3_CC3,
    ADC_TRIGGER_TIM3_CC4,
    ADC_TRIGGER_TIM3_TRGO,
    ADC_TRIGGER_TIM4_CC1,
    ADC_TRIGGER_TIM4_CC4,
    ADC_TRIGGER_TIM4_TRGO,
    ADC_TRIGGER_TIM6_TRGO,
    ADC_TRIGGER_TIM7_TRGO,
    ADC_TRIGGER_TIM8_TRGO,
    ADC_TRIGGER_TIM8_TRGO2,
    ADC_TRIGGER_TIM15_TRGO,
    ADC_TRIGGER_TIM20_CC1,
    ADC_TRIGGER_TIM20_CC2,
    ADC_TRIGGER_TIM20_CC3,
    ADC_TRIGGER_TIM20_TRGO,
    ADC_TRIGGER_TIM20_TRGO2,
    ADC_TRIGGER_LPTIMOUT,
    ADC_TRIGGER_HRTIM_ADC_TRG1,
    ADC_TRIGGER_HRTIM_ADC_TRG2,
    ADC_TRIGGER_HRTIM_ADC_TRG3,
    ADC_TRIGGER_HRTIM_ADC_TRG4,
    ADC_TRIGGER_HRTIM_ADC_TRG5,
    ADC_TRIGGER_HRTIM_ADC_TRG6,
    ADC_TRIGGER_HRTIM_ADC_TRG7,
    ADC_TRIGGER_HRTIM_ADC_TRG8,
    ADC_TRIGGER_HRTIM_ADC_TRG9,
    ADC_TRIGGER_HRTIM_ADC_TRG10,
    ADC_TRIGGER_LAST
} ADC_trigger_t;

/*!******************************************************************
 * \enum ADC_channel_t
 * \brief ADC channels list.
 *******************************************************************/
typedef enum {
    ADC_TRIGGER_DETECTION_NONE = 0,
    ADC_TRIGGER_DETECTION_RISING_EDGE,
    ADC_TRIGGER_DETECTION_FALLING_EDGE,
    ADC_TRIGGER_DETECTION_ANY_EDGE,
    ADC_TRIGGER_DETECTION_LAST
} ADC_trigger_detection_t;

#if ((STM32G4XX_DRIVERS_ADC_MODE_MASK & ADC_MODE_MASK_SINGLE) != 0)
/*!******************************************************************
 * \struct ADC_SGL_configuration_t
 * \brief ADC configuration structure for single mode.
 *******************************************************************/
typedef struct {
    ADC_clock_t clock;
    ADC_clock_prescaler_t clock_prescaler;
} ADC_SGL_configuration_t;
#endif

#if ((STM32G4XX_DRIVERS_ADC_MODE_MASK & ADC_MODE_MASK_SEQUENCE) != 0)
/*!******************************************************************
 * \struct ADC_SQC_configuration_t
 * \brief ADC configuration structure for sequence mode.
 *******************************************************************/
typedef struct {
    ADC_clock_t clock;
    ADC_clock_prescaler_t clock_prescaler;
    ADC_channel_configuration_t* master_sequence;
    ADC_channel_configuration_t* slave_sequence;
    uint8_t sequence_length;
    uint32_t sampling_frequency_hz;
    ADC_trigger_t trigger;
    ADC_trigger_detection_t trigger_detection;
} ADC_SQC_configuration_t;
#endif

/*** ADC functions ***/

#if ((STM32G4XX_DRIVERS_ADC_MODE_MASK & ADC_MODE_MASK_SINGLE) != 0)
/*!******************************************************************
 * \fn ADC_status_t ADC_SGL_init(ADC_instance_t instance, const ADC_gpio_t* pins, ADC_SGL_configuration_t* adc_configuration)
 * \brief Init ADC peripheral in single mode.
 * \param[in]   instance: ADC instance to initialize.
 * \param[in]   pins: List of ADC pins to use.
 * \param[in]   adc_configuration: Pointer to the ADC configuration.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ADC_status_t ADC_SGL_init(ADC_instance_t instance, const ADC_gpio_t* pins, ADC_SGL_configuration_t* adc_configuration);
#endif

#if ((STM32G4XX_DRIVERS_ADC_MODE_MASK & ADC_MODE_MASK_SINGLE) != 0)
/*!******************************************************************
 * \fn ADC_status_t ADC_SGL_de_init(ADC_instance_t instance)
 * \brief Release ADC peripheral.
 * \param[in]   instance: ADC instance to release.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ADC_status_t ADC_SGL_de_init(ADC_instance_t instance);
#endif

#if ((STM32G4XX_DRIVERS_ADC_MODE_MASK & ADC_MODE_MASK_SINGLE) != 0)
/*!******************************************************************
 * \fn ADC_status_t ADC_SGL_convert_channel(ADC_instance_t instance, ADC_channel_t channel, int32_t* adc_data_12bits)
 * \brief Perform a single channel conversion.
 * \param[in]   instance: ADC instance to use.
 * \param[in]   channel: Channel to convert.
 * \param[out]  adc_data_12bits: Pointer to integer that will contain the 12-bits ADC data.
 * \retval      Function execution status.
 *******************************************************************/
ADC_status_t ADC_SGL_convert_channel(ADC_instance_t instance, ADC_channel_t channel, int32_t* adc_data_12bits);
#endif

#if ((STM32G4XX_DRIVERS_ADC_MODE_MASK & ADC_MODE_MASK_SEQUENCE) != 0)
/*!******************************************************************
 * \fn ADC_status_t ADC_SQC_init(ADC_instance_t instance, const ADC_gpio_t* pins, ADC_SQC_configuration_t* adc_configuration)
 * \brief Init ADC peripheral in sequence mode.
 * \param[in]   instance: ADC instance to initialize (master in case of dual mode).
 * \param[in]   pins: List of ADC pins to use.
 * \param[in]   adc_configuration: Pointer to the ADC configuration.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ADC_status_t ADC_SQC_init(ADC_instance_t instance, const ADC_gpio_t* pins, ADC_SQC_configuration_t* adc_configuration);
#endif

#if ((STM32G4XX_DRIVERS_ADC_MODE_MASK & ADC_MODE_MASK_SEQUENCE) != 0)
/*!******************************************************************
 * \fn ADC_status_t ADC_SQC_de_init(ADC_instance_t instance)
 * \brief Release ADC peripheral (master in case of dual mode).
 * \param[in]   instance: ADC instance to release.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ADC_status_t ADC_SQC_de_init(ADC_instance_t instance);
#endif

#if ((STM32G4XX_DRIVERS_ADC_MODE_MASK & ADC_MODE_MASK_SEQUENCE) != 0)
/*!******************************************************************
 * \fn ADC_status_t ADC_SQC_start(ADC_instance_t instance)
 * \brief Start ADC continuous conversions.
 * \param[in]   instance: ADC instance to start (master in case of dual mode).
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ADC_status_t ADC_SQC_start(ADC_instance_t instance);
#endif

#if ((STM32G4XX_DRIVERS_ADC_MODE_MASK & ADC_MODE_MASK_SEQUENCE) != 0)
/*!******************************************************************
 * \fn ADC_status_t ADC_SQC_stop(ADC_instance_t instance)
 * \brief Stop ADC continuous conversions.
 * \param[in]   instance: ADC instance to stop (master in case of dual mode).
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ADC_status_t ADC_SQC_stop(ADC_instance_t instance);
#endif

#ifdef STM32G4XX_DRIVERS_ADC_VREF_MV
/*!******************************************************************
 * \fn ADC_status_t ADC_compute_vmcu(int32_t vbat_12bits, int32_t* vmcu_mv)
 * \brief Compute MCU voltage.
 * \param[in]   vbat_12bits_12bits: VBAT channel 12-bits raw data from ADC.
 * \param[out]  vmcu_mv: Pointer to integer that will contain the MCU voltage in mV.
 * \retval      Function execution status.
 *******************************************************************/
ADC_status_t ADC_compute_vmcu(int32_t vbat_12bits, int32_t* vmcu_mv);
#else
/*!******************************************************************
 * \fn ADC_status_t ADC_compute_vmcu(int32_t_t ref_voltage_12bits, int32_t_t ref_voltage_mv, int32_t* vmcu_mv)
 * \brief Compute MCU voltage.
 * \param[in]   ref_voltage_12bits: Reference voltage 12-bits raw data from ADC.
 * \param[in]   ref_voltage_mv: Reference voltage in mV.
 * \param[out]  vmcu_mv: Pointer to integer that will contain the MCU voltage in mV.
 * \retval      Function execution status.
 *******************************************************************/
ADC_status_t ADC_compute_vmcu(int32_t ref_voltage_12bits, int32_t ref_voltage_mv, int32_t* vmcu_mv);
#endif

#ifdef STM32G4XX_DRIVERS_ADC_VREF_MV
/*!******************************************************************
 * \fn ADC_status_t ADC_compute_tmcu(int32_t_t tmcu_12bits, int32_t* tmcu_degrees)
 * \brief Compute MCU temperature.
 * \param[in]   tmcu_12bits: Temperature sensor 12-bits raw data from ADC.
 * \param[out]  tmcu_degrees: Pointer to integer that will contain MCU temperature in 2's complement format.
 * \retval      Function execution status.
 *******************************************************************/
ADC_status_t ADC_compute_tmcu(int32_t tmcu_12bits, int32_t* tmcu_degrees);
#else
/*!******************************************************************
 * \fn ADC_status_t ADC_compute_tmcu(int32_t vref_mv, int32_t_t tmcu_12bits, int32_t* tmcu_degrees)
 * \brief Compute MCU temperature.
 * \param[in]   vref_mv: VREF+ voltage in mV.
 * \param[in]   tmcu_12bits: Temperature sensor 12-bits raw data from ADC.
 * \param[out]  tmcu_degrees: Pointer to integer that will contain MCU temperature in 2's complement format.
 * \retval      Function execution status.
 *******************************************************************/
ADC_status_t ADC_compute_tmcu(int32_t vref_mv, int32_t tmcu_12bits, int32_t* tmcu_degrees);
#endif

/*!******************************************************************
 * \fn int32_t ADC_get_vrefint_voltage_mv(void)
 * \brief Get internal reference voltage.
 * \param[in]   none
 * \param[out]  none
 * \retval      Internal reference voltage in mV.
 *******************************************************************/
int32_t ADC_get_vrefint_voltage_mv(void);

/*!******************************************************************
 * \fn uint32_t ADC_get_master_dr_register_address(ADC_instance_t instance)
 * \brief Get master ADC data register address.
 * \param[in]   instance: ADC instance to read.
 * \param[out]  none
 * \retval      DR register address.
 *******************************************************************/
uint32_t ADC_get_master_dr_register_address(ADC_instance_t instance);

/*!******************************************************************
 * \fn uint32_t ADC_get_slave_dr_register_address(ADC_instance_t instance)
 * \brief Get slave ADC data register address.
 * \param[in]   instance: ADC instance to read.
 * \param[out]  none
 * \retval      DR register address.
 *******************************************************************/
uint32_t ADC_get_slave_dr_register_address(ADC_instance_t instance);

/*******************************************************************/
#define ADC_exit_error(base) { ERROR_check_exit(adc_status, ADC_SUCCESS, base) }

/*******************************************************************/
#define ADC_stack_error(base) { ERROR_check_stack(adc_status, ADC_SUCCESS, base) }

/*******************************************************************/
#define ADC_stack_exit_error(base, code) { ERROR_check_stack_exit(adc_status, ADC_SUCCESS, base, code) }

#endif /* __ADC_H__ */

#endif /* STM32G4XX_DRIVERS_DISABLE */
