/*
 * nvm.h
 *
 *  Created on: 15 feb. 2025
 *      Author: Ludo
 */

#ifndef __NVM_H__
#define __NVM_H__

#ifndef STM32G4XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32g4xx_drivers_flags.h"
#endif
#include "flash.h"
#include "types.h"

/*** NVM structures ***/

/*!******************************************************************
 * \enum NVM_status_t
 * \brief NVM driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    NVM_SUCCESS = 0,
    NVM_ERROR_NULL_PARAMETER,
    NVM_ERROR_OVERFLOW,
    NVM_ERROR_ADDRESS,
    NVM_ERROR_RECURSIVE_CALL,
    // Low level drivers errors.
    NVM_ERROR_BASE_FLASH = 0x0100,
    // Last base value.
    NVM_ERROR_BASE_LAST = (NVM_ERROR_BASE_FLASH + FLASH_ERROR_BASE_LAST)
} NVM_status_t;

/*** NVM functions ***/

/*!******************************************************************
 * \fn NVM_status_t NVM_erase(void)
 * \brief Erase all NVM.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
NVM_status_t NVM_erase(void);

/*!******************************************************************
 * \fn NVM_status_t NVM_read_word(uint32_t address, uint32_t* data)
 * \brief Read a 32-bits value in NVM.
 * \param[in]   address: Relative address to read (starting from 0).
 * \param[out]  data: Pointer to word that will contain the read value.
 * \retval      Function execution status.
 *******************************************************************/
NVM_status_t NVM_read_word(uint32_t address, uint32_t* data);

/*!******************************************************************
 * \fn NVM_status_t NVM_write_word(uint32_t address, uint32_t data)
 * \brief Write a 32-bits value in NVM.
 * \param[in]   address: Relative address to write (starting from 0).
 * \param[out]  data: Word to write.
 * \retval      Function execution status.
 *******************************************************************/
NVM_status_t NVM_write_word(uint32_t address, uint32_t data);

/*******************************************************************/
#define NVM_exit_error(base) { ERROR_check_exit(nvm_status, NVM_SUCCESS, base) }

/*******************************************************************/
#define NVM_stack_error(base) { ERROR_check_stack(nvm_status, NVM_SUCCESS, base) }

/*******************************************************************/
#define NVM_stack_exit_error(base, code) { ERROR_check_stack_exit(nvm_status, NVM_SUCCESS, base, code) }

#endif /* __NVM_H__ */
