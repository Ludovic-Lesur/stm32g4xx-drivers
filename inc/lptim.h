/*
 * lptim.h
 *
 *  Created on: 15 feb. 2025
 *      Author: Ludo
 */

#ifndef STM32G4XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32g4xx_drivers_flags.h"
#endif

#ifndef STM32G4XX_DRIVERS_DISABLE

#ifndef __LPTIM_H__
#define __LPTIM_H__

#include "error.h"
#include "types.h"

/*** LPTIM structures ***/

/*!******************************************************************
 * \enum LPTIM_status_t
 * \brief LPTIM driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    LPTIM_SUCCESS = 0,
    LPTIM_ERROR_ALREADY_INITIALIZED,
    LPTIM_ERROR_UNINITIALIZED,
    LPTIM_ERROR_DELAY_UNDERFLOW,
    LPTIM_ERROR_DELAY_OVERFLOW,
    LPTIM_ERROR_DELAY_MODE,
    LPTIM_ERROR_ARR_TIMEOUT,
    LPTIM_ERROR_CLOCK_SOURCE,
    LPTIM_ERROR_ALREADY_RUNNING,
    // Last base value.
    LPTIM_ERROR_BASE_LAST = ERROR_BASE_STEP
} LPTIM_status_t;

/*!******************************************************************
 * \enum LPTIM_delay_mode_t
 * \brief LPTIM delay waiting modes.
 *******************************************************************/
typedef enum {
    LPTIM_DELAY_MODE_ACTIVE = 0,
    LPTIM_DELAY_MODE_SLEEP,
    LPTIM_DELAY_MODE_STOP,
    LPTIM_DELAY_MODE_LAST
} LPTIM_delay_mode_t;

/*** LPTIM functions ***/

/*!******************************************************************
 * \fn LPTIM_status_t LPTIM_init(uint8_t nvic_priority)
 * \brief Init LPTIM peripheral for delay operation.
 * \param[in]   nvic_priority: Interrupt priority.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
LPTIM_status_t LPTIM_init(uint8_t nvic_priority);

/*!******************************************************************
 * \fn LPTIM_status_t LPTIM_de_init(void)
 * \brief Release LPTIM peripheral.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
LPTIM_status_t LPTIM_de_init(void);

/*!******************************************************************
 * \fn LPTIM_status_t LPTIM_delay_milliseconds(uint32_t delay_ms, LPTIM_delay_mode_t delay_mode)
 * \brief Delay function.
 * \param[in]   delay_ms: Delay to wait in ms.
 * \param[in]   delay_mode: Delay waiting mode.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
LPTIM_status_t LPTIM_delay_milliseconds(uint32_t delay_ms, LPTIM_delay_mode_t delay_mode);

/*******************************************************************/
#define LPTIM_exit_error(base) { ERROR_check_exit(lptim_status, LPTIM_SUCCESS, base) }

/*******************************************************************/
#define LPTIM_stack_error(base) { ERROR_check_stack(lptim_status, LPTIM_SUCCESS, base) }

/*******************************************************************/
#define LPTIM_stack_exit_error(base, code) { ERROR_check_stack_exit(lptim_status, LPTIM_SUCCESS, base, code) }

#endif /* __LPTIM_H__ */

#endif /* STM32G4XX_DRIVERS_DISABLE */
