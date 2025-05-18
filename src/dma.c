/*
 * dma.c
 *
 *  Created on: 18 jun. 2023
 *      Author: Ludo
 */

#ifndef STM32G4XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32g4xx_drivers_flags.h"
#endif

#ifndef STM32G4XX_DRIVERS_DISABLE

#include "dma.h"

#ifndef STM32G4XX_REGISTERS_DISABLE_FLAGS_FILE
#include "stm32g4xx_registers_flags.h"
#endif
#include "dma_registers.h"
#include "dmamux_registers.h"
#include "dmamux.h"
#include "nvic.h"
#include "rcc_registers.h"
#include "types.h"

#if ((STM32G4XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_ALL) != 0)

/*** DMA local macros ***/

#define DMA_REGISTER_MASK_CCR       0x00007FFF
#define DMA_REGISTER_MASK_CNDTR     0x0000FFFF
#define DMAMUX_REGISTER_MASK_CCR    0x1FFF037F

/*** DMA local structures ***/

/*******************************************************************/
typedef struct {
    DMA_registers_t* peripheral;
    volatile uint32_t* rcc_enr;
    uint32_t rcc_mask;
    NVIC_interrupt_t nvic_interrupt[DMA_CHANNEL_LAST];
} DMA_descriptor_t;

/*******************************************************************/
typedef struct {
    uint16_t number_of_data;
    DMA_transfer_complete_irq_cb_t tc_irq_callback;
} DMA_channel_context_t;

/*******************************************************************/
typedef struct {
    uint8_t enabled_channels_mask;
    DMA_channel_context_t channel_ctx[DMA_CHANNEL_LAST];
} DMA_context_t;

/*******************************************************************/
typedef struct {
    uint8_t enabled_dma_mask;
} DMA_global_context_t;

/*** DMA local global variables ***/

#if (STM32G4XX_REGISTERS_MCU_CATEGORY > 2)
#define DMA_DESCRIPTOR_NVIC(instance) { \
    NVIC_INTERRUPT_DMA##instance##_CH1, \
    NVIC_INTERRUPT_DMA##instance##_CH2, \
    NVIC_INTERRUPT_DMA##instance##_CH3, \
    NVIC_INTERRUPT_DMA##instance##_CH4, \
    NVIC_INTERRUPT_DMA##instance##_CH5, \
    NVIC_INTERRUPT_DMA##instance##_CH6, \
    NVIC_INTERRUPT_DMA##instance##_CH7, \
    NVIC_INTERRUPT_DMA##instance##_CH8, \
}
#else
#define DMA_DESCRIPTOR_NVIC(instance) { \
    NVIC_INTERRUPT_DMA##instance##_CH1, \
    NVIC_INTERRUPT_DMA##instance##_CH2, \
    NVIC_INTERRUPT_DMA##instance##_CH3, \
    NVIC_INTERRUPT_DMA##instance##_CH4, \
    NVIC_INTERRUPT_DMA##instance##_CH5, \
    NVIC_INTERRUPT_DMA##instance##_CH6, \
}
#endif

static const DMA_descriptor_t DMA_DESCRIPTOR[DMA_INSTANCE_LAST] = {
    { DMA1, &(RCC->AHB1ENR), (0b1 << 0), DMA_DESCRIPTOR_NVIC(1) },
    { DMA2, &(RCC->AHB1ENR), (0b1 << 1), DMA_DESCRIPTOR_NVIC(2) },
};

static DMA_context_t dma_ctx[DMA_INSTANCE_LAST] = {
    [0 ... (DMA_INSTANCE_LAST - 1)] = {
        .enabled_channels_mask = 0,
        .channel_ctx = {
            [0 ... (DMA_CHANNEL_LAST - 1)] {
                .number_of_data = 0,
                .tc_irq_callback = NULL
            }
        }
    }
};

static DMA_global_context_t dma_global_ctx = {
    .enabled_dma_mask = 0
};

/*** DMA local functions ***/

/*******************************************************************/
#define _DMA_check_instance(instance) { \
    if (instance >= DMA_INSTANCE_LAST) { \
        status = DMA_ERROR_INSTANCE; \
        goto errors; \
    } \
}

/*******************************************************************/
#define _DMA_check_channel(channel) { \
    if (channel >= DMA_CHANNEL_LAST) { \
        status = DMA_ERROR_CHANNEL; \
        goto errors; \
    } \
}

/*******************************************************************/
#define _DMA_check_channel_state(instance, channel) { \
    if (((dma_ctx[instance].enabled_channels_mask) & (0b1 << channel)) == 0) { \
        status = DMA_ERROR_UNINITIALIZED; \
        goto errors; \
    } \
}

/*******************************************************************/
#define _DMA_irq_handler(instance, channel) { \
    /* Check flag */ \
    if (((DMA_DESCRIPTOR[instance].peripheral->ISR) & (0b1 << ((channel << 2) + 1))) != 0) { \
        /* Check mask and callback */ \
        if ((((DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CCR) & (0b1 << 1)) != 0) && (dma_ctx[instance].channel_ctx[channel].tc_irq_callback != NULL)) { \
            /* Execute callback */ \
            dma_ctx[instance].channel_ctx[channel].tc_irq_callback(); \
        } \
        /* Clear flags */ \
        DMA_DESCRIPTOR[instance].peripheral->IFCR |= (0b1111 << (channel << 2)); \
    } \
}

#if ((STM32G4XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_DMA1_CH1) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) DMA1_CH1_IRQHandler(void) {
    _DMA_irq_handler(DMA_INSTANCE_DMA1, DMA_CHANNEL_1);
}
#endif

#if ((STM32G4XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_DMA1_CH2) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) DMA1_CH2_IRQHandler(void) {
    _DMA_irq_handler(DMA_INSTANCE_DMA1, DMA_CHANNEL_2);
}
#endif

#if ((STM32G4XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_DMA1_CH3) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) DMA1_CH3_IRQHandler(void) {
    _DMA_irq_handler(DMA_INSTANCE_DMA1, DMA_CHANNEL_3);
}
#endif

#if ((STM32G4XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_DMA1_CH4) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) DMA1_CH4_IRQHandler(void) {
    _DMA_irq_handler(DMA_INSTANCE_DMA1, DMA_CHANNEL_4);
}
#endif

#if ((STM32G4XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_DMA1_CH5) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) DMA1_CH5_IRQHandler(void) {
    _DMA_irq_handler(DMA_INSTANCE_DMA1, DMA_CHANNEL_5);
}
#endif

#if ((STM32G4XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_DMA1_CH6) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) DMA1_CH6_IRQHandler(void) {
    _DMA_irq_handler(DMA_INSTANCE_DMA1, DMA_CHANNEL_6);
}
#endif

#if (((STM32G4XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_DMA1_CH7) != 0) && (STM32G4XX_REGISTERS_MCU_CATEGORY > 2))
/*******************************************************************/
void __attribute__((optimize("-O0"))) DMA1_CH7_IRQHandler(void) {
    _DMA_irq_handler(DMA_INSTANCE_DMA1, DMA_CHANNEL_7);
}
#endif

#if (((STM32G4XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_DMA1_CH8) != 0) && (STM32G4XX_REGISTERS_MCU_CATEGORY > 2))
/*******************************************************************/
void __attribute__((optimize("-O0"))) DMA1_CH8_IRQHandler(void) {
    _DMA_irq_handler(DMA_INSTANCE_DMA1, DMA_CHANNEL_8);
}
#endif

#if ((STM32G4XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_DMA2_CH1) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) DMA2_CH1_IRQHandler(void) {
    _DMA_irq_handler(DMA_INSTANCE_DMA2, DMA_CHANNEL_1);
}
#endif

#if ((STM32G4XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_DMA2_CH2) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) DMA2_CH2_IRQHandler(void) {
    _DMA_irq_handler(DMA_INSTANCE_DMA2, DMA_CHANNEL_2);
}
#endif

#if ((STM32G4XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_DMA2_CH3) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) DMA2_CH3_IRQHandler(void) {
    _DMA_irq_handler(DMA_INSTANCE_DMA2, DMA_CHANNEL_3);
}
#endif

#if ((STM32G4XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_DMA2_CH4) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) DMA2_CH4_IRQHandler(void) {
    _DMA_irq_handler(DMA_INSTANCE_DMA2, DMA_CHANNEL_4);
}
#endif

#if ((STM32G4XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_DMA2_CH5) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) DMA2_CH5_IRQHandler(void) {
    _DMA_irq_handler(DMA_INSTANCE_DMA2, DMA_CHANNEL_5);
}
#endif

#if ((STM32G4XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_DMA2_CH6) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) DMA2_CH6_IRQHandler(void) {
    _DMA_irq_handler(DMA_INSTANCE_DMA2, DMA_CHANNEL_6);
}
#endif

#if (((STM32G4XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_DMA2_CH7) != 0) && (STM32G4XX_REGISTERS_MCU_CATEGORY > 2))
/*******************************************************************/
void __attribute__((optimize("-O0"))) DMA2_CH7_IRQHandler(void) {
    _DMA_irq_handler(DMA_INSTANCE_DMA2, DMA_CHANNEL_7);
}
#endif

#if (((STM32G4XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_DMA2_CH8) != 0) && (STM32G4XX_REGISTERS_MCU_CATEGORY > 2))
/*******************************************************************/
void __attribute__((optimize("-O0"))) DMA2_CH8_IRQHandler(void) {
    _DMA_irq_handler(DMA_INSTANCE_DMA2, DMA_CHANNEL_8);
}
#endif

/*** DMA functions ***/

/*******************************************************************/
DMA_status_t DMA_init(DMA_instance_t instance, DMA_channel_t channel, DMA_configuration_t* configuration) {
    // Local variables.
    DMA_status_t status = DMA_SUCCESS;
    uint32_t cndtr = 0;
    // Check instance and channel.
    _DMA_check_instance(instance);
    _DMA_check_channel(channel);
    // Check parameters.
    if (configuration == NULL) {
        status = DMA_ERROR_NULL_PARAMETER;
        goto errors;
    }
    if (((configuration->request_id) == DMAMUX_PERIPHERAL_REQUEST_NONE) || ((configuration->request_id) >= DMAMUX_PERIPHERAL_REQUEST_LAST)) {
        status = DMA_ERROR_REQUEST_ID;
        goto errors;
    }
    // Enable DMAMUX clock.
    RCC->AHB1ENR |= (0b1 << 2);
    // Enable peripheral clock.
    (*DMA_DESCRIPTOR[instance].rcc_enr) |= DMA_DESCRIPTOR[instance].rcc_mask;
    // Reset configuration register.
    DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CCR &= (~DMA_REGISTER_MASK_CCR);
    // Direction.
    switch (configuration->direction) {
    case DMA_DIRECTION_PERIPHERAL_TO_MEMORY:
        // Nothing to do.
        break;
    case DMA_DIRECTION_MEMORY_TO_PERIPHERAL:
        DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CCR |= (0b1 << 4);
        break;
    default:
        status = DMA_ERROR_DIRECTION;
        goto errors;
    }
    // Flags.
    DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CCR |= ((configuration->flags).circular_mode << 5);
    DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CCR |= ((configuration->flags).peripheral_increment << 6);
    DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CCR |= ((configuration->flags).memory_increment << 7);
    // Memory data size.
    switch (configuration->memory_data_size) {
    case DMA_DATA_SIZE_8_BITS:
        // Nothing to do.
        break;
    case DMA_DATA_SIZE_16_BITS:
        DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CCR |= (0b01 << 10);
        break;
    case DMA_DATA_SIZE_32_BITS:
        DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CCR |= (0b10 << 10);
        break;
    default:
        status = DMA_ERROR_MEMORY_DATA_SIZE;
        goto errors;
    }
    // Peripheral data size.
    switch (configuration->peripheral_data_size) {
    case DMA_DATA_SIZE_8_BITS:
        // Nothing to do.
        break;
    case DMA_DATA_SIZE_16_BITS:
        DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CCR |= (0b01 << 8);
        break;
    case DMA_DATA_SIZE_32_BITS:
        DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CCR |= (0b10 << 8);
        break;
    default:
        status = DMA_ERROR_PERIPHERAL_DATA_SIZE;
        goto errors;
    }
    // Priority.
    switch (configuration->priority) {
    case DMA_PRIORITY_LOW:
        // Nothing to do.
        break;
    case DMA_PRIORITY_MEDIUM:
        DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CCR |= (0b01 << 12);
        break;
    case DMA_PRIORITY_HIGH:
        DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CCR |= (0b10 << 12);
        break;
    case DMA_PRIORITY_VERY_HIGH:
        DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CCR |= (0b11 << 12);
        break;
    default:
        status = DMA_ERROR_PRIORITY;
        goto errors;
    }
    // Number of data.
    cndtr = ((DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CNDTR) & (~DMA_REGISTER_MASK_CNDTR));
    cndtr |= ((configuration->number_of_data) & DMA_REGISTER_MASK_CNDTR);
    DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CNDTR = cndtr;
    dma_ctx[instance].channel_ctx[channel].number_of_data = (configuration->number_of_data);
    // Memory address.
    DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CMAR = (configuration->memory_address);
    // Peripheral address.
    DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CPAR = (configuration->peripheral_address);
    // Configure DMA multiplexer.
    DMAMUX->CxCR[channel] &= (~DMAMUX_REGISTER_MASK_CCR);
    DMAMUX->CxCR[channel] |= ((configuration->request_id) & 0x0000007F);
    // Set interrupt priority.
    NVIC_set_priority(DMA_DESCRIPTOR[instance].nvic_interrupt[channel], (configuration->nvic_priority));
    // Enable transfer complete interrupt.
    if ((configuration->tc_irq_callback) != NULL) {
        DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CCR |= (0b1 << 1);
    }
    // Register callback.
    dma_ctx[instance].channel_ctx[channel].tc_irq_callback = (configuration->tc_irq_callback);
    // Update masks.
    dma_global_ctx.enabled_dma_mask |= (0b1 << instance);
    dma_ctx[instance].enabled_channels_mask |= (0b1 << channel);
errors:
    return status;
}

/*******************************************************************/
DMA_status_t DMA_de_init(DMA_instance_t instance, DMA_channel_t channel) {
    // Local variables.
    DMA_status_t status = DMA_SUCCESS;
    // Check instance and channel.
    _DMA_check_instance(instance);
    _DMA_check_channel(channel);
    // Disable channel.
    status = DMA_stop(instance, channel);
    // Reset channel.
    dma_ctx[instance].enabled_channels_mask &= ~(0b1 << channel);
    dma_ctx[instance].channel_ctx[channel].number_of_data = 0;
    dma_ctx[instance].channel_ctx[channel].tc_irq_callback = NULL;
    // Check if all channel are stopped.
    if (dma_ctx[instance].enabled_channels_mask == 0) {
        // Disable peripheral clock.
        (*DMA_DESCRIPTOR[instance].rcc_enr) &= ~(DMA_DESCRIPTOR[instance].rcc_mask);
        // Update mask.
        dma_global_ctx.enabled_dma_mask &= ~(0b1 << instance);
    }
    // Disable DMAMUX if all DMA are disabled.
    if (dma_global_ctx.enabled_dma_mask == 0) {
        RCC->AHB1ENR &= ~(0b1 << 2);
    }
errors:
    return status;
}

/*******************************************************************/
DMA_status_t DMA_start(DMA_instance_t instance, DMA_channel_t channel) {
    // Local variables.
    DMA_status_t status = DMA_SUCCESS;
    // Check instance and channel.
    _DMA_check_instance(instance);
    _DMA_check_channel(channel);
    _DMA_check_channel_state(instance, channel);
    // Clear all flags.
    DMA_DESCRIPTOR[instance].peripheral->IFCR |= (0b1111 << (channel << 2));
    // Enable interrupt.
    NVIC_enable_interrupt(DMA_DESCRIPTOR[instance].nvic_interrupt[channel]);
    // Start transfer.
    DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CCR |= (0b1 << 0);
errors:
    return status;
}

/*******************************************************************/
DMA_status_t DMA_stop(DMA_instance_t instance, DMA_channel_t channel) {
    // Local variables.
    DMA_status_t status = DMA_SUCCESS;
    // Check instance and channel.
    _DMA_check_instance(instance);
    _DMA_check_channel(channel);
    _DMA_check_channel_state(instance, channel);
    // Stop transfer.
    DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CCR &= ~(0b1 << 0);
    // Disable interrupt.
    NVIC_disable_interrupt(DMA_DESCRIPTOR[instance].nvic_interrupt[channel]);
    // Clear all flags.
    DMA_DESCRIPTOR[instance].peripheral->IFCR |= (0b1111 << (channel << 2));
errors:
    return status;
}

/*******************************************************************/
DMA_status_t DMA_set_memory_address(DMA_instance_t instance, DMA_channel_t channel, uint32_t memory_addr, uint16_t number_of_data) {
    // Local variables.
    DMA_status_t status = DMA_SUCCESS;
    uint32_t cndtr = 0;
    // Check instance and channel.
    _DMA_check_instance(instance);
    _DMA_check_channel(channel);
    _DMA_check_channel_state(instance, channel);
    // Set memory address and transfer size.
    DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CMAR = memory_addr;
    cndtr = ((DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CNDTR) & (~DMA_REGISTER_MASK_CNDTR));
    cndtr |= (number_of_data & DMA_REGISTER_MASK_CNDTR);
    DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CNDTR = cndtr;
    dma_ctx[instance].channel_ctx[channel].number_of_data = number_of_data;
errors:
    return status;
}

/*******************************************************************/
DMA_status_t DMA_set_peripheral_address(DMA_instance_t instance, DMA_channel_t channel, uint32_t peripheral_addr, uint16_t number_of_data) {
    // Local variables.
    DMA_status_t status = DMA_SUCCESS;
    uint32_t cndtr = 0;
    // Check instance and channel.
    _DMA_check_instance(instance);
    _DMA_check_channel(channel);
    _DMA_check_channel_state(instance, channel);
    // Set memory address and transfer size.
    DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CPAR = peripheral_addr;
    cndtr = ((DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CNDTR) & (~DMA_REGISTER_MASK_CNDTR));
    cndtr |= (number_of_data & DMA_REGISTER_MASK_CNDTR);
    DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CNDTR = cndtr;
    dma_ctx[instance].channel_ctx[channel].number_of_data = number_of_data;
errors:
    return status;
}

/*******************************************************************/
DMA_status_t DMA_get_number_of_transfered_data(DMA_instance_t instance, DMA_channel_t channel, uint16_t* number_of_transfered_data) {
    // Local variables.
    DMA_status_t status = DMA_SUCCESS;
    uint16_t number_of_data = 0;
    uint16_t cndtr = 0;
    // Check instance and channel.
    _DMA_check_instance(instance);
    _DMA_check_channel(channel);
    // Check parameter.
    if (number_of_transfered_data == NULL) {
        status = DMA_ERROR_NULL_PARAMETER;
        goto errors;
    }
    number_of_data = dma_ctx[instance].channel_ctx[channel].number_of_data;
    cndtr = DMA_DESCRIPTOR[instance].peripheral->CHx[channel].CNDTR;
    // Compute number of transfered data.
    (*number_of_transfered_data) = (number_of_data > cndtr) ? (number_of_data - cndtr) : 0;
errors:
    return status;
}

#endif /* STM32G4XX_DRIVERS_DMA_CHANNEL_MASK */

#endif /* STM32G4XX_DRIVERS_DISABLE */
