/*
 * dma.h
 *
 *  Created on: 13 feb. 2025
 *      Author: Ludo
 */

#ifndef STM32G4XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32g4xx_drivers_flags.h"
#endif

#ifndef STM32G4XX_DRIVERS_DISABLE

#ifndef __DMA_H__
#define __DMA_H__

#ifndef STM32G4XX_REGISTERS_DISABLE_FLAGS_FILE
#include "stm32g4xx_registers_flags.h"
#endif
#ifndef STM32G4XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32g4xx_drivers_flags.h"
#endif
#include "dmamux.h"
#include "error.h"
#include "types.h"

/*** DMA macros ***/

#define DMA_CHANNEL_MASK_DMA1_CH1   0x0001
#define DMA_CHANNEL_MASK_DMA1_CH2   0x0002
#define DMA_CHANNEL_MASK_DMA1_CH3   0x0004
#define DMA_CHANNEL_MASK_DMA1_CH4   0x0008
#define DMA_CHANNEL_MASK_DMA1_CH5   0x0010
#define DMA_CHANNEL_MASK_DMA1_CH6   0x0020
#define DMA_CHANNEL_MASK_DMA1_CH7   0x0040
#define DMA_CHANNEL_MASK_DMA1_CH8   0x0080
#define DMA_CHANNEL_MASK_DMA2_CH1   0x0100
#define DMA_CHANNEL_MASK_DMA2_CH2   0x0200
#define DMA_CHANNEL_MASK_DMA2_CH3   0x0400
#define DMA_CHANNEL_MASK_DMA2_CH4   0x0800
#define DMA_CHANNEL_MASK_DMA2_CH5   0x1000
#define DMA_CHANNEL_MASK_DMA2_CH6   0x2000
#define DMA_CHANNEL_MASK_DMA2_CH7   0x4000
#define DMA_CHANNEL_MASK_DMA2_CH8   0x8000

#define DMA_CHANNEL_MASK_ALL        0xFFFF

/*** DMA structures ***/

/*!******************************************************************
 * \enum DMA_status_t
 * \brief DMA driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    DMA_SUCCESS = 0,
    DMA_ERROR_NULL_PARAMETER,
    DMA_ERROR_INSTANCE,
    DMA_ERROR_CHANNEL,
    DMA_ERROR_DIRECTION,
    DMA_ERROR_MEMORY_DATA_SIZE,
    DMA_ERROR_PERIPHERAL_DATA_SIZE,
    DMA_ERROR_PRIORITY,
    DMA_ERROR_REQUEST_ID,
    DMA_ERROR_UNINITIALIZED,
    // Last base value.
    DMA_ERROR_BASE_LAST = ERROR_BASE_STEP
} DMA_status_t;

#if ((STM32G4XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_ALL) != 0)

/*!******************************************************************
 * \enum DMA_instance_t
 * \brief DMA instances list.
 *******************************************************************/
typedef enum {
    DMA_INSTANCE_DMA1 = 0,
    DMA_INSTANCE_DMA2,
    DMA_INSTANCE_LAST
} DMA_instance_t;

/*!******************************************************************
 * \enum DMA_channel_t
 * \brief DMA channels list.
 *******************************************************************/
typedef enum {
    DMA_CHANNEL_1 = 0,
    DMA_CHANNEL_2,
    DMA_CHANNEL_3,
    DMA_CHANNEL_4,
    DMA_CHANNEL_5,
    DMA_CHANNEL_6,
#if (STM32G4XX_REGISTERS_MCU_CATEGORY > 2)
    DMA_CHANNEL_7,
    DMA_CHANNEL_8,
#endif
    DMA_CHANNEL_LAST,
} DMA_channel_t;

/*!******************************************************************
 * \enum DMA_direction_t
 * \brief DMA transfer directions list.
 *******************************************************************/
typedef enum {
    DMA_DIRECTION_PERIPHERAL_TO_MEMORY = 0,
    DMA_DIRECTION_MEMORY_TO_PERIPHERAL,
    DMA_DIRECTION_LAST
} DMA_direction_t;

/*!******************************************************************
 * \union DMA_flags_t
 * \brief DMA configuration flags.
 *******************************************************************/
typedef union {
    struct {
        unsigned circular_mode :1;
        unsigned peripheral_increment :1;
        unsigned memory_increment :1;
    };
    uint8_t all;
} DMA_flags_t;

/*!******************************************************************
 * \enum DMA_data_size_t
 * \brief DMA data sizes list.
 *******************************************************************/
typedef enum {
    DMA_DATA_SIZE_8_BITS = 0,
    DMA_DATA_SIZE_16_BITS,
    DMA_DATA_SIZE_32_BITS,
    DMA_DATA_SIZE_LAST
} DMA_data_size_t;

/*!******************************************************************
 * \enum DMA_priority_t
 * \brief DMA transfer priorities list.
 *******************************************************************/
typedef enum {
    DMA_PRIORITY_LOW = 0,
    DMA_PRIORITY_MEDIUM,
    DMA_PRIORITY_HIGH,
    DMA_PRIORITY_VERY_HIGH,
    DMA_PRIORITY_LAST
} DMA_priority_t;

/*!******************************************************************
 * \fn DMA_transfer_complete_irq_cb
 * \brief DMA transfer complete interrupt callback.
 *******************************************************************/
typedef void (*DMA_transfer_complete_irq_cb_t)(void);

/*!******************************************************************
 * \struct DMA_configuration_t
 * \brief DMA configuration structure.
 *******************************************************************/
typedef struct {
    DMA_direction_t direction;
    DMA_flags_t flags;
    uint32_t memory_address;
    DMA_data_size_t memory_data_size;
    uint32_t peripheral_address;
    DMA_data_size_t peripheral_data_size;
    uint16_t number_of_data;
    DMA_priority_t priority;
    DMAMUX_peripheral_request_t request_id;
    DMA_transfer_complete_irq_cb_t tc_irq_callback;
    uint8_t nvic_priority;
} DMA_configuration_t;

/*** DMA functions ***/

/*!******************************************************************
 * \fn DMA_status_t DMA_init(DMA_instance_t instance, DMA_channel_t_t channel, DMA_configuration_t* configuration)
 * \brief Init DMA channel.
 * \param[in]   instance: DMA instance to init.
 * \param[in]:  channel: DMA channel to initialize.
 * \param[in]:  configuration: Pointer to the DMA channel configuration structure.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
DMA_status_t DMA_init(DMA_instance_t instance, DMA_channel_t channel, DMA_configuration_t* configuration);

/*!******************************************************************
 * \fn DMA_status_t DMA_de_init(DMA_instance_t instance, DMA_channel_t_t channel)
 * \brief Release a DMA channel.
 * \param[in]   instance: DMA instance to release.
 * \param[in]:  channel: DMA channel to release.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
DMA_status_t DMA_de_init(DMA_instance_t instance, DMA_channel_t channel);

/*!******************************************************************
 * \fn DMA_status_t DMA_start(DMA_instance_t instance, DMA_channel_t_t channel)
 * \brief Start a DMA channel.
 * \param[in]   instance: DMA instance to use.
 * \param[in]:  channel: DMA channel to start.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
DMA_status_t DMA_start(DMA_instance_t instance, DMA_channel_t channel);

/*!******************************************************************
 * \fn DMA_status_t DMA_stop(DMA_instance_t instance, DMA_channel_t_t channel)
 * \brief Stop a DMA channel.
 * \param[in]   instance: DMA instance to use.
 * \param[in]:  channel: DMA channel to stop.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
DMA_status_t DMA_stop(DMA_instance_t instance, DMA_channel_t channel);

/*!******************************************************************
 * \fn DMA_status_t DMA_set_memory_address(DMA_instance_t instance, DMA_channel_t_t channel, uint32_t memory_addr, uint16_t number_of_data)
 * \brief Set DMA channel memory address.
 * \param[in]   instance: DMA instance to use.
 * \param[in]:  channel: DMA channel to configure.
 * \param[in]   memory_addr: Memory address.
 * \param[in]   number_of_data: DMA transfer size.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
DMA_status_t DMA_set_memory_address(DMA_instance_t instance, DMA_channel_t channel, uint32_t memory_addr, uint16_t number_of_data);

/*!******************************************************************
 * \fn DMA_status_t DMA_set_peripheral_address(DMA_instance_t instance, DMA_channel_t_t channel, uint32_t peripheral_addr, uint16_t number_of_data)
 * \brief Set DMA channel peripheral address.
 * \param[in]   instance: DMA instance to use.
 * \param[in]:  channel: DMA channel to configure.
 * \param[in]   peripheral_addr: Peripheral address.
 * \param[in]   number_of_data: DMA transfer size.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
DMA_status_t DMA_set_peripheral_address(DMA_instance_t instance, DMA_channel_t channel, uint32_t peripheral_addr, uint16_t number_of_data);

/*!******************************************************************
 * \fn DMA_status_t DMA_get_number_of_transfered_data(DMA_instance_t instance, DMA_channel_t channel, uint16_t* number_of_transfered_data)
 * \brief Get current number of data transfered by DMA.
 * \param[in]   instance: DMA instance to read.
 * \param[in]:  channel: DMA channel to read.
 * \param[out]  number_of_transfered_data: Pointer to the effective number of transfered data.
 * \retval      Function execution status.
 *******************************************************************/
DMA_status_t DMA_get_number_of_transfered_data(DMA_instance_t instance, DMA_channel_t channel, uint16_t* number_of_transfered_data);

/*******************************************************************/
#define DMA_exit_error(base) { ERROR_check_exit(dma_status, DMA_SUCCESS, base) }

/*******************************************************************/
#define DMA_stack_error(base) { ERROR_check_stack(dma_status, DMA_SUCCESS, base) }

/*******************************************************************/
#define DMA_stack_exit_error(base, code) { ERROR_check_stack_exit(dma_status, DMA_SUCCESS, base, code)

#endif /* STM32G4XX_DRIVERS_DMA_CHANNEL_MASK */

#endif /* __DMA_H__ */

#endif /* STM32G4XX_DRIVERS_DISABLE */
