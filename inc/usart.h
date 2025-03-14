/*
 * usart.h
 *
 *  Created on: 19 feb. 2025
 *      Author: Ludo
 */

#ifndef __USART_H__
#define __USART_H__

#ifndef STM32G4XX_REGISTERS_DISABLE_FLAGS_FILE
#include "stm32g4xx_registers_flags.h"
#endif
#ifndef STM32G4XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32g4xx_drivers_flags.h"
#endif
#include "error.h"
#include "gpio.h"
#include "rcc.h"
#include "types.h"

/*** USART structures ***/

/*!******************************************************************
 * \enum USART_status_t
 * \brief USART driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    USART_SUCCESS = 0,
    USART_ERROR_NULL_PARAMETER,
    USART_ERROR_INSTANCE,
    USART_ERROR_ALREADY_INITIALIZED,
    USART_ERROR_UNINITIALIZED,
    USART_ERROR_CLOCK,
    USART_ERROR_PARITY,
    USART_ERROR_BAUD_RATE,
    USART_ERROR_TX_TIMEOUT,
    USART_ERROR_RS485_MODE,
    USART_ERROR_TC_TIMEOUT,
    // Last base value.
    USART_ERROR_BASE_LAST = ERROR_BASE_STEP
} USART_status_t;

/*!******************************************************************
 * \enum USART_instance_t
 * \brief USART instances list.
 *******************************************************************/
typedef enum {
    USART_INSTANCE_USART1 = 0,
    USART_INSTANCE_USART2,
    USART_INSTANCE_USART3,
    USART_INSTANCE_UART4,
#if (STM32G4XX_REGISTERS_MCU_CATEGORY == 3) || (STM32G4XX_REGISTERS_MCU_CATEGORY == 4)
    USART_INSTANCE_UART5,
#endif
    USART_INSTANCE_LAST
} USART_instance_t;

/*!******************************************************************
 * \struct USART_gpio_t
 * \brief USART GPIO pins list.
 *******************************************************************/
typedef struct {
    const GPIO_pin_t* tx;
    const GPIO_pin_t* rx;
#ifdef STM32G4XX_DRIVERS_USART_RS485
    const GPIO_pin_t* de;
    const GPIO_pin_t* nre;
#endif
} USART_gpio_t;

/*!******************************************************************
 * \fn USART_rx_irq_cb_t
 * \brief USART RX interrupt callback.
 *******************************************************************/
typedef void (*USART_rx_irq_cb_t)(uint8_t data);

/*!******************************************************************
 * \fn USART_character_match_irq_cb_t
 * \brief USART character match interrupt callback.
 *******************************************************************/
typedef void (*USART_character_match_irq_cb_t)(void);

/*!******************************************************************
 * \enum USART_parity_t
 * \brief USART parities list.
 *******************************************************************/
typedef enum {
    USART_PARITY_NONE = 0,
    USART_PARITY_EVEN,
    USART_PARITY_ODD,
    USART_PARITY_LAST
} USART_parity_t;

#ifdef STM32G4XX_DRIVERS_USART_RS485
/*!******************************************************************
 * \enum USART_rs485_mode_t
 * \brief USART RS485 modes list.
 *******************************************************************/
typedef enum {
    USART_RS485_MODE_DISABLED = 0,
    USART_RS485_MODE_ADDRESSED,
    USART_RS485_MODE_DIRECT,
    USART_RS485_MODE_LAST
} USART_rs485_mode_t;
#endif

/*!******************************************************************
 * \struct USART_configuration_t
 * \brief USART configuration structure.
 *******************************************************************/
typedef struct {
    RCC_clock_t clock;
    uint32_t baud_rate;
    USART_parity_t parity;
    uint8_t nvic_priority;
    USART_rx_irq_cb_t rxne_irq_callback;
    USART_character_match_irq_cb_t cm_irq_callback;
    char_t match_character;
#ifdef STM32G4XX_DRIVERS_USART_RS485
    USART_rs485_mode_t rs485_mode;
    uint8_t self_address;
#endif
} USART_configuration_t;

/*** USART functions ***/

/*!******************************************************************
 * \fn USART_status_t USART_init(USART_instance_t instance, const USART_gpio_t* pins, USART_configuration_t* configuration)
 * \brief Init USART peripheral.
 * \param[in]   instance: Peripheral instance to initialize.
 * \param[in]   pins: Pointer to the USART pins to use.
 * \param[in]   configuration: Pointer to the USART configuration structure.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
USART_status_t USART_init(USART_instance_t instance, const USART_gpio_t* pins, USART_configuration_t* configuration);

/*!******************************************************************
 * \fn USART_status_t USART_de_init(USART_instance_t instance, const USART_gpio_t* pins)
 * \brief Release USART peripheral.
 * \param[in]   instance: Peripheral instance to release.
 * \param[in]   pins: Pointer to the USART pins to release.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
USART_status_t USART_de_init(USART_instance_t instance, const USART_gpio_t* pins);

/*!******************************************************************
 * \fn USART_status_t USART_enable_rx(USART_instance_t instance)
 * \brief Enable USART RX operation.
 * \param[in]   instance: Peripheral instance to control.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
USART_status_t USART_enable_rx(USART_instance_t instance);

/*!******************************************************************
 * \fn USART_status_t USART_disable_rx(USART_instance_t instance)
 * \brief Disable USART RX operation.
 * \param[in]   instance: Peripheral instance to control.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
USART_status_t USART_disable_rx(USART_instance_t instance);

/*!******************************************************************
 * \fn USART_status_t USART_write(USART_instance_t instance, uint8_t* data, uint32_t data_size_bytes)
 * \brief Send data over USART.
 * \param[in]   instance: Peripheral instance to use.
 * \param[in]   data: Byte array to send.
 * \param[in]   data_size_bytes: Number of bytes to send.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
USART_status_t USART_write(USART_instance_t instance, uint8_t* data, uint32_t data_size_bytes);

/*!******************************************************************
 * \fn uint32_t USART_get_rdr_register_address(USART_instance_t instance)
 * \brief Get USART RDR register address.
 * \param[in]   instance: USART instance to read.
 * \param[out]  none
 * \retval      RDR register address.
 *******************************************************************/
uint32_t USART_get_rdr_register_address(USART_instance_t instance);

/*******************************************************************/
#define USART_exit_error(base) { ERROR_check_exit(usart_status, USART_SUCCESS, base) }

/*******************************************************************/
#define USART_stack_error(base) { ERROR_check_stack(usart_status, USART_SUCCESS, base) }

/*******************************************************************/
#define USART_stack_exit_error(base, code) { ERROR_check_stack_exit(usart_status, USART_SUCCESS, base, code) }

#endif /* __USART_H__ */
