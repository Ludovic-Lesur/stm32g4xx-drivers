/*
 * nvm.c
 *
 *  Created on: 15 feb. 2025
 *      Author: Ludo
 */

#ifndef STM32G4XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32g4xx_drivers_flags.h"
#endif

#ifndef STM32G4XX_DRIVERS_DISABLE

#include "nvm.h"

#include "error.h"
#include "flash.h"
#include "maths.h"
#include "types.h"

/*** NVM linker generated symbols ***/

extern uint32_t __flash_address__;
extern uint32_t __eeprom_page_address__;

/*** NVM local macros ***/

#define NVM_WORD_BLANK          0xFFFFFFFF

#define NVM_FLASH_PAGE_ADDRESS  ((uint32_t) (&__flash_address__))
#define NVM_PAGE_ADDRESS        ((uint32_t) (&__eeprom_page_address__))

#define NVM_PAGE_SIZE_WORDS     (FLASH_PAGE_SIZE_BYTES / FLASH_WORD_SIZE_BYTES)

#define NVM_PAGE_INDEX          ((NVM_PAGE_ADDRESS - NVM_FLASH_PAGE_ADDRESS) / (FLASH_PAGE_SIZE_BYTES))

/*** NVM local structures ***/

/*******************************************************************/
typedef enum {
    NVM_MEMORY_STATUS_EMPTY = 0,
    NVM_MEMORY_STATUS_NOT_EMPTY,
    NVM_MEMORY_STATUS_FULL
} NVM_memory_status_t;

/*******************************************************************/
typedef union {
    uint64_t double_word;
    struct {
        uint32_t virtual_address :32;
        uint32_t value :32;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} NVM_record_t;

/*******************************************************************/
typedef struct {
    NVM_memory_status_t memory_status;
    uint8_t flash_erase_count;
} NVM_context_t;

/*** NVM local global variables ***/

static NVM_record_t nvm_backup[NVM_PAGE_SIZE_WORDS] __attribute__((section(".bss_sram2")));

static NVM_context_t nvm_ctx = {
    .memory_status = NVM_MEMORY_STATUS_FULL,
    .flash_erase_count = 0
};

/*** NVM local functions ***/

/*******************************************************************/
static NVM_status_t _NVM_update_memory_status(uint32_t* free_absolute_address) {
    // Local variables.
    NVM_status_t status = NVM_SUCCESS;
    FLASH_status_t flash_status = FLASH_SUCCESS;
    uint32_t absolute_address = 0;
    NVM_record_t nvm_record;
    uint32_t idx = 0;
    // Reset status.
    nvm_ctx.memory_status = NVM_MEMORY_STATUS_FULL;
    // Search first blank word.
    for (idx = 0; idx < NVM_PAGE_SIZE_WORDS; idx++) {
        // Compute absolute address.
        absolute_address = (NVM_PAGE_ADDRESS + (idx * FLASH_WORD_SIZE_BYTES));
        // Read word.
        flash_status = FLASH_read_double_word(absolute_address, &(nvm_record.double_word));
        FLASH_exit_error(NVM_ERROR_BASE_FLASH);
        // Check if address is free.
        if (nvm_record.virtual_address == NVM_WORD_BLANK) {
            // Save address.
            (*free_absolute_address) = absolute_address;
            // Update status.
            nvm_ctx.memory_status = (idx == 0) ? NVM_MEMORY_STATUS_EMPTY : NVM_MEMORY_STATUS_NOT_EMPTY;
            break;
        }
    }
errors:
    return status;
}

/*******************************************************************/
static NVM_status_t _NVM_search_last_record(uint32_t address, uint8_t* record_found, NVM_record_t* nvm_record) {
    // Local variables.
    NVM_status_t status = NVM_SUCCESS;
    FLASH_status_t flash_status = FLASH_SUCCESS;
    NVM_record_t local_nvm_record;
    uint32_t idx = 0;
    // Reset result.
    (*record_found) = 0;
    // Read all page to retrieve the last value of the given address.
    for (idx = 0; idx < NVM_PAGE_SIZE_WORDS; idx++) {
        // Read word.
        flash_status = FLASH_read_double_word((NVM_PAGE_ADDRESS + (idx * FLASH_WORD_SIZE_BYTES)), &(local_nvm_record.double_word));
        FLASH_exit_error(NVM_ERROR_BASE_FLASH);
        // Check address match.
        if (local_nvm_record.virtual_address == address) {
            // Update flag.
            (*record_found) = 1;
            // Update output value.
            nvm_record->double_word = local_nvm_record.double_word;
            // Note: do not break here since we want the last value stored in flash.
        }
    }
errors:
    return status;
}

/*** NVM functions ***/

/*******************************************************************/
NVM_status_t NVM_erase(void) {
    // Local variables.
    NVM_status_t status = NVM_SUCCESS;
    FLASH_status_t flash_status = FLASH_SUCCESS;
    // Erase EEPROM page.
    flash_status = FLASH_erase_page(NVM_PAGE_INDEX);
    FLASH_exit_error(NVM_ERROR_BASE_FLASH);
errors:
    return status;
}

/*******************************************************************/
NVM_status_t NVM_read_word(uint32_t address, uint32_t* data) {
    // Local variables.
    NVM_status_t status = NVM_SUCCESS;
    NVM_record_t nvm_record;
    uint8_t record_found = 0;
    // Check parameters.
    if (address >= NVM_PAGE_SIZE_WORDS) {
        status = NVM_ERROR_OVERFLOW;
        goto errors;
    }
    if (data == NULL) {
        status = NVM_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Reset output data.
    (*data) = NVM_WORD_BLANK;
    // Search data.
    status = _NVM_search_last_record(address, &record_found, &nvm_record);
    if (status != NVM_SUCCESS) goto errors;
    // Check flag.
    if (record_found != 0) {
        (*data) = ((uint32_t) nvm_record.value);
    }
errors:
    return status;
}

/*******************************************************************/
NVM_status_t NVM_write_word(uint32_t address, uint32_t data) {
    // Local variables.
    NVM_status_t status = NVM_SUCCESS;
    FLASH_status_t flash_status = FLASH_SUCCESS;
    uint32_t free_absolute_address = 0;
    NVM_record_t new_nvm_record;
    uint32_t backup_size = 0;
    uint32_t data_word = 0;
    uint32_t idx = 0;
    // Check parameters.
    if (address >= NVM_PAGE_SIZE_WORDS) {
        status = NVM_ERROR_OVERFLOW;
        goto errors;
    }
    // Build word to record.
    new_nvm_record.virtual_address = address;
    new_nvm_record.value = data;
    // Search free address.
    status = _NVM_update_memory_status(&free_absolute_address);
    if (status != NVM_SUCCESS) goto errors;
    // Check status.
    if (nvm_ctx.memory_status == NVM_MEMORY_STATUS_FULL) {
        // Prevent from recursive call error (memory should not be full two consecutive times).
        if (nvm_ctx.flash_erase_count > 0) {
            status = NVM_ERROR_RECURSIVE_CALL;
            goto errors;
        }
        // Backup all virtual addresses value.
        for (idx = 0; idx < NVM_PAGE_SIZE_WORDS; idx++) {
            // Read word.
            status = NVM_read_word(idx, &data_word);
            if (status != NVM_SUCCESS) goto errors;
            // Check word.
            if (data_word != NVM_WORD_BLANK) {
                // Store in RAM backup.
                nvm_backup[backup_size].virtual_address = idx;
                nvm_backup[backup_size].value = (idx == address) ? data : data_word;
                backup_size++;
            }
        }
        // Erase page.
        flash_status = FLASH_erase_page(NVM_PAGE_INDEX);
        FLASH_exit_error(NVM_ERROR_BASE_FLASH);
        // Update erase count.
        nvm_ctx.flash_erase_count++;
        // Restore backup and new value.
        for (idx = 0; idx < backup_size; idx++) {
            // Write byte.
            status = NVM_write_word(nvm_backup[idx].virtual_address, nvm_backup[idx].value);
            if (status != NVM_SUCCESS) goto errors;
        }
    }
    else {
        // Write word at free location.
        flash_status = FLASH_write_double_word(free_absolute_address, new_nvm_record.double_word);
        FLASH_exit_error(NVM_ERROR_BASE_FLASH);
    }
errors:
    nvm_ctx.flash_erase_count = 0;
    return status;
}

#endif /* STM32G4XX_DRIVERS_DISABLE */
