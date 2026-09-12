/*
 * flash.c
 *
 *  Created on: 15 feb. 2025
 *      Author: Ludo
 */

#ifndef STM32G4XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32g4xx_drivers_flags.h"
#endif

#ifndef STM32G4XX_DRIVERS_DISABLE

#include "flash.h"

#include "flash_registers.h"
#include "nvic.h"
#include "types.h"

/*** FLASH linker generated symbols ***/

extern uint32_t __flash_address__;
extern uint32_t __flash_size__;

/*** FLASH local macros ***/

#define FLASH_WAIT_STATES_MAX   15
#define FLASH_TIMEOUT_COUNT     100000

#define FLASH_ADDRESS           ((uint32_t) (&__flash_address__))
#define FLASH_SIZE_BYTES        ((uint32_t) (&__flash_size__))

#define FLASH_ERROR_FLAGS_MASK  0x0000C3FB

/*** FLASH local functions ***/

/*******************************************************************/
#define _FLASH_check_address(address) { \
    /* Check range */ \
    if (address < FLASH_ADDRESS) { \
        status = FLASH_ERROR_ADDRESS_UNDERFLOW; \
        goto end; \
    } \
    if (address >= (FLASH_ADDRESS + FLASH_SIZE_BYTES)) { \
        status = FLASH_ERROR_ADDRESS_OVERFLOW; \
        goto end; \
    } \
    /* Check alignment */ \
    if ((address % FLASH_WORD_SIZE_BYTES) != 0) { \
        status = FLASH_ERROR_ADDRESS_ALIGNMENT; \
        goto end; \
    } \
}

/*******************************************************************/
static FLASH_status_t _FLASH_check_busy(FLASH_status_t timeout_error_code) {
    // Local variables.
    FLASH_status_t status = FLASH_SUCCESS;
    uint32_t loop_count = 0;
    // Check no write/erase operation is running.
    while (((FLASH->SR) & (0b1 << 16)) != 0) {
        // Wait till BSY='1' or timeout.
        loop_count++;
        if (loop_count > FLASH_TIMEOUT_COUNT) {
            status = timeout_error_code;
        }
    }
    // Clear all status flags.
    FLASH->SR = FLASH_ERROR_FLAGS_MASK;
    // Return status.
    return status;
}

/*******************************************************************/
static FLASH_status_t __attribute__((optimize("-O0"))) _FLASH_unlock(void) {
    // Local variables.
    FLASH_status_t status = FLASH_SUCCESS;
    // Check memory is ready.
    status = _FLASH_check_busy(FLASH_ERROR_UNLOCK_READY);
    if (status != FLASH_SUCCESS) goto errors;
    // Check the memory is not already unlocked.
    if (((FLASH->CR) & (0b1 << 31)) != 0) {
        // Perform unlock sequence.
        FLASH->KEYR = 0x45670123;
        FLASH->KEYR = 0xCDEF89AB;
    }
    // Check if unlock sequence completed successfully.
    if (((FLASH->CR) & (0b1 << 31)) != 0) {
       status = FLASH_ERROR_UNLOCK_SEQUENCE;
       goto errors;
    }
errors:
    return status;
}

/*******************************************************************/
static void __attribute__((optimize("-O0"))) _FLASH_lock(void) {
    // Lock sequence.
    FLASH->CR |= (0b1 << 31);
}

/*******************************************************************/
static void __attribute__((optimize("-O0"))) _FLASH_flush_caches(void) {
    // Flush instruction cache.
    FLASH->ACR &= ~(0b1 << 9);
    FLASH->ACR |= (0b1 << 11);
    FLASH->ACR &= ~(0b1 << 11);
    FLASH->ACR |= (0b1 << 9);
    // Flush data cache.
    FLASH->ACR &= ~(0b1 << 10);
    FLASH->ACR |= (0b1 << 12);
    FLASH->ACR &= ~(0b1 << 12);
    FLASH->ACR |= (0b1 << 10);
}

/*** FLASH functions ***/

/*******************************************************************/
FLASH_status_t FLASH_set_latency(uint8_t wait_states) {
    // Local variables.
    FLASH_status_t status = FLASH_SUCCESS;
    uint32_t loop_count = 0;
    // Check parameter.
    if (wait_states > FLASH_WAIT_STATES_MAX) {
        status = FLASH_ERROR_LATENCY;
        goto errors;
    }
    // Configure number of wait states.
    FLASH->ACR &= ~(0b1111 << 0); // Reset bits.
    FLASH->ACR |= ((wait_states & 0b1111) << 0); // Set latency.
    // Wait until configuration is done.
    while (((FLASH->ACR) & (0b1111 << 0)) != ((wait_states & 0b1111) << 0)) {
        loop_count++;
        if (loop_count > FLASH_TIMEOUT_COUNT) {
            status = FLASH_ERROR_TIMEOUT;
            goto errors;
        }
    }
errors:
    return status;
}

/*******************************************************************/
FLASH_status_t FLASH_get_latency(uint8_t* wait_states) {
    // Local variables.
    FLASH_status_t status = FLASH_SUCCESS;
    // Check parameter.
    if (wait_states == NULL) {
        status = FLASH_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Read latency.
    (*wait_states) = (uint8_t) ((FLASH->ACR) & 0x000000FF);
errors:
    return status;
}

/*******************************************************************/
FLASH_status_t __attribute__((optimize("-O0"))) FLASH_read_double_word(uint32_t absolute_address, uint64_t* data) {
    // Local variables.
    FLASH_status_t status = FLASH_SUCCESS;
    uint8_t global_interrupts = NVIC_get_global_interrupts();
    // Check parameters.
    _FLASH_check_address(absolute_address);
    if (data == NULL) {
        status = FLASH_ERROR_NULL_PARAMETER;
        goto end;
    }
    // Disable all interrupts.
    NVIC_set_global_interrupts(0);
    // Check there is no pending operation.
    status = _FLASH_check_busy(FLASH_ERROR_READ_READY);
    if (status != FLASH_SUCCESS) goto errors;
    // Read data.
    (*data) = *((uint64_t*) (absolute_address));
errors:
    // Restore interrupts.
    NVIC_set_global_interrupts(global_interrupts);
end:
    return status;
}

/*******************************************************************/
FLASH_status_t __attribute__((optimize("-O0"))) FLASH_write_double_word(uint32_t absolute_address, uint64_t data) {
    // Local variables.
    FLASH_status_t status = FLASH_SUCCESS;
    uint8_t global_interrupts = NVIC_get_global_interrupts();
    uint64_t read_data = 0;
    // Check parameters.
    _FLASH_check_address(absolute_address);
    // Disable all interrupts.
    NVIC_set_global_interrupts(0);
    // Unlock memory.
    status = _FLASH_unlock();
    if (status != FLASH_SUCCESS) goto errors;
    // Check there is no pending operation.
    status = _FLASH_check_busy(FLASH_ERROR_WRITE_READY);
    if (status != FLASH_SUCCESS) goto errors;
    // Disable data cache.
    FLASH->ACR &= ~(0b1 << 10);
    // Set programming bit.
    FLASH->CR |= (0b1 << 0);
    // Write first word.
    *((uint32_t*) (absolute_address + 0)) = (uint32_t) (data & 0xFFFFFFFF);
    __asm volatile ("isb");
    *((uint32_t*) (absolute_address + 4)) = (uint32_t) ((data >> 32) & 0xFFFFFFFF);
    // Wait the end of operation.
    status = _FLASH_check_busy(FLASH_ERROR_WRITE_COMPLETION);
    if (status != FLASH_SUCCESS) goto errors;
    // Verify write operation.
    read_data = *((uint64_t*) (absolute_address));
    if (read_data != data) {
        status = FLASH_ERROR_WRITE_VERIFY;
        goto errors;
    }
errors:
    // Reset programming bit.
    FLASH->CR &= ~(0b1 << 0);
    // Flush instruction and data caches.
    _FLASH_flush_caches();
    // Lock memory.
    _FLASH_lock();
    // Restore interrupts.
    NVIC_set_global_interrupts(global_interrupts);
end:
    return status;
}

/*******************************************************************/
FLASH_status_t __attribute__((optimize("-O0"))) FLASH_erase_page(uint32_t page_index) {
    // Local variables.
    FLASH_status_t status = FLASH_SUCCESS;
    uint8_t global_interrupts = NVIC_get_global_interrupts();
    // Check parameter.
    if (page_index >= (FLASH_SIZE_BYTES / FLASH_PAGE_SIZE_BYTES)) {
        status = FLASH_ERROR_PAGE_INDEX;
        goto end;
    }
    // Disable all interrupts.
    NVIC_set_global_interrupts(0);
    // Unlock memory.
    status = _FLASH_unlock();
    if (status != FLASH_SUCCESS) goto errors;
    // Select page.
    FLASH->CR &= ~(0x7F << 3);
    FLASH->CR |= ((page_index & 0x7F) << 3);
    // Set page erase bit.
    FLASH->CR |= (0b1 << 1);
    // Start operation.
    FLASH->CR |= (0b1 << 16);
    // Wait the end of operation.
    status = _FLASH_check_busy(FLASH_ERROR_ERASE);
    if (status != FLASH_SUCCESS) goto errors;
errors:
    // Reset page erase bit.
    FLASH->CR &= ~(0b1 << 1);
    // Flush instruction and data caches.
    _FLASH_flush_caches();
    // Lock memory.
    _FLASH_lock();
    // Restore interrupts.
    NVIC_set_global_interrupts(global_interrupts);
end:
    return status;
}

#endif /* STM32G4XX_DRIVERS_DISABLE */
