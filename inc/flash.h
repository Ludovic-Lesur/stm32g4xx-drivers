/*
 * flash.h
 *
 *  Created on: 15 feb. 2025
 *      Author: Ludo
 */

#ifndef __FLASH_H__
#define __FLASH_H__

#include "error.h"
#include "types.h"

/*** FLASH macros ***/

#define FLASH_PAGE_SIZE_BYTES   2048
#define FLASH_WORD_SIZE_BYTES   8

/*** FLASH structures ***/

/*!******************************************************************
 * \enum FLASH_status_t
 * \brief FLASH driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    FLASH_SUCCESS = 0,
    FLASH_ERROR_NULL_PARAMETER,
    FLASH_ERROR_LATENCY,
    FLASH_ERROR_TIMEOUT,
    FLASH_ERROR_ADDRESS_UNDERFLOW,
    FLASH_ERROR_ADDRESS_OVERFLOW,
    FLASH_ERROR_ADDRESS_ALIGNMENT,
    FLASH_ERROR_PAGE_INDEX,
    FLASH_ERROR_UNLOCK,
    FLASH_ERROR_LOCK,
    FLASH_ERROR_ERASE,
    FLASH_ERROR_READ,
    FLASH_ERROR_WRITE,
    // Last base value.
    FLASH_ERROR_BASE_LAST = ERROR_BASE_STEP
} FLASH_status_t;

/*** FLASH functions ***/

/*!******************************************************************
 * \fn FLASH_status_t FLASH_set_latency(uint8_t wait_states)
 * \brief Set FLASH latency.
 * \param[in]   wait_states: Number of wait states to set.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
FLASH_status_t FLASH_set_latency(uint8_t wait_states);

/*!******************************************************************
 * \fn FLASH_status_t FLASH_get_latency(uint8_t* wait_states)
 * \brief Get FLASH latency.
 * \param[in]   none
 * \param[out]  wait_states: Pointer to the current number of wait states.
 * \retval      Function execution status.
 *******************************************************************/
FLASH_status_t FLASH_get_latency(uint8_t* wait_states);

/*!******************************************************************
 * \fn FLASH_status_t FLASH_read_double_word(uint32_t absolute_address, uint64_t* data)
 * \brief Read 64-bits value in flash.
 * \param[in]   absolute_address: Absolute address to read.
 * \param[out]  data: Pointer to double-word that will contain the read value.
 * \retval      Function execution status.
 *******************************************************************/
FLASH_status_t FLASH_read_double_word(uint32_t absolute_address, uint64_t* data);

/*!******************************************************************
 * \fn FLASH_status_t FLASH_write_double_word(uint32_t absolute_address, uint64_t data)
 * \brief Write a 64-bits value in flash.
 * \param[in]   absolute_address: Absolute address to write.
 * \param[out]  data: Double-word to write.
 * \retval      Function execution status.
 *******************************************************************/
FLASH_status_t FLASH_write_double_word(uint32_t absolute_address, uint64_t data);

/*!******************************************************************
 * \fn FLASH_status_t FLASH_erase_page(uint32_t page_index)
 * \brief Erase FLASH page.
 * \param[in]   page_index: Index of the page to erase.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
FLASH_status_t FLASH_erase_page(uint32_t page_index);

/*******************************************************************/
#define FLASH_exit_error(base) { ERROR_check_exit(flash_status, FLASH_SUCCESS, base) }

/*******************************************************************/
#define FLASH_stack_error(base) { ERROR_check_stack(flash_status, FLASH_SUCCESS, base) }

/*******************************************************************/
#define FLASH_stack_exit_error(base, code) { ERROR_check_stack_exit(flash_status, FLASH_SUCCESS, base, code) }

#endif /* __FLASH_H__ */
