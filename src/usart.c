/*
 * usart.c
 *
 *  Created on: 19 feb. 2025
 *      Author: Ludo
 */

#ifndef STM32G4XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32g4xx_drivers_flags.h"
#endif

#ifndef STM32G4XX_DRIVERS_DISABLE

#include "usart.h"

#ifndef STM32G4XX_REGISTERS_DISABLE_FLAGS_FILE
#include "stm32g4xx_registers_flags.h"
#endif
#include "exti.h"
#include "gpio.h"
#include "nvic.h"
#include "rcc.h"
#include "rcc_registers.h"
#include "types.h"
#include "usart_registers.h"

/*** USART local macros ***/

#define USART_TIMEOUT_COUNT         100000

#define USART_REGISTER_MASK_BRR     0x0000FFFF
#define USART_REGISTER_MASK_TDR     0x000000FF

#define USART_BRR_VALUE_MIN         0x0010
#define USART_BRR_VALUE_MAX         USART_REGISTER_MASK_BRR

/*** USART local structures ***/

/*******************************************************************/
typedef struct {
    USART_registers_t* peripheral;
    volatile uint32_t* rcc_enr;
    volatile uint32_t* rcc_smenr;
    uint32_t rcc_mask;
    uint32_t rcc_ccipr_shift;
    NVIC_interrupt_t nvic_interrupt;
} USART_descriptor_t;

/*******************************************************************/
typedef struct {
    uint32_t clock_hz;
    uint8_t init_flag;
    USART_rx_irq_cb_t rxne_irq_callback;
    USART_character_match_irq_cb_t cm_irq_callback;
#ifdef STM32G4XX_DRIVERS_USART_RS485
    USART_rs485_mode_t rs485_mode;
#endif
} USART_context_t;

/*** USART local global variables ***/

static const USART_descriptor_t USART_DESCRIPTOR[USART_INSTANCE_LAST] = {
    { USART1, &(RCC->APB2ENR), &(RCC->APB2SMENR), (0b1 << 14), 0, NVIC_INTERRUPT_USART1 },
    { USART2, &(RCC->APB1ENR1), &(RCC->APB1SMENR1), (0b1 << 17), 2, NVIC_INTERRUPT_USART2 },
    { USART3, &(RCC->APB1ENR1), &(RCC->APB1SMENR1), (0b1 << 18), 4, NVIC_INTERRUPT_USART3 },
    { UART4, &(RCC->APB1ENR1), &(RCC->APB1SMENR1), (0b1 << 19), 6, NVIC_INTERRUPT_UART4 },
#if (STM32G4XX_REGISTERS_MCU_CATEGORY == 3) || (STM32G4XX_REGISTERS_MCU_CATEGORY == 4)
    { UART5, &(RCC->APB1ENR1), &(RCC->APB1SMENR1), (0b1 << 20), 8, NVIC_INTERRUPT_UART5 },
#endif
};

static USART_context_t usart_ctx[USART_INSTANCE_LAST] = {
    [0 ... (USART_INSTANCE_LAST - 1)] = {
        .init_flag = 0,
        .rxne_irq_callback = NULL,
        .cm_irq_callback = NULL,
#ifdef STM32G4XX_DRIVERS_USART_RS485
        .rs485_mode = USART_RS485_MODE_DISABLED
#endif
    }
};

/*** USART local functions ***/

/*******************************************************************/
static void __attribute__((optimize("-O0"))) _USART_irq_handler(USART_instance_t instance) {
    // Local variables.
    uint8_t rx_byte = 0;
    // RXNE interrupt.
    if (((USART_DESCRIPTOR[instance].peripheral->ISR) & (0b1 << 5)) != 0) {
        // Read incoming byte.
        rx_byte = (uint8_t) (USART_DESCRIPTOR[instance].peripheral->RDR);
        // Transmit byte to upper layer.
        if ((((USART_DESCRIPTOR[instance].peripheral->CR1) & (0b1 << 5)) != 0) && (usart_ctx[instance].rxne_irq_callback != NULL)) {
            usart_ctx[instance].rxne_irq_callback(rx_byte);
        }
    }
    // CM interrupt.
    if (((USART_DESCRIPTOR[instance].peripheral->ISR) & (0b1 << 17)) != 0) {
        // Call applicative callback.
        if ((((USART_DESCRIPTOR[instance].peripheral->CR1) & (0b1 << 14)) != 0) && (usart_ctx[instance].cm_irq_callback != NULL)) {
            usart_ctx[instance].cm_irq_callback();
        }
        // Clear CMF flag.
        USART_DESCRIPTOR[instance].peripheral->ICR = (0b1 << 17);
    }
    // PE interrupt.
    if (((USART_DESCRIPTOR[instance].peripheral->ISR) & (0b1 << 0)) != 0) {
        // Clear PE flag.
        USART_DESCRIPTOR[instance].peripheral->ICR = (0b1 << 0);
    }
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) USART1_IRQHandler(void) {
    // Execute internal callback.
    _USART_irq_handler(USART_INSTANCE_USART1);
    // Clear EXTI line.
    EXTI_clear_line_flag(EXTI_LINE_USART1);
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) USART2_IRQHandler(void) {
    // Execute internal callback.
    _USART_irq_handler(USART_INSTANCE_USART2);
    // Clear EXTI line.
    EXTI_clear_line_flag(EXTI_LINE_USART2);
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) USART3_IRQHandler(void) {
    // Execute internal callback.
    _USART_irq_handler(USART_INSTANCE_USART3);
    // Clear EXTI line.
    EXTI_clear_line_flag(EXTI_LINE_USART3);
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) UART4_IRQHandler(void) {
    // Execute internal callback.
    _USART_irq_handler(USART_INSTANCE_UART4);
    // Clear EXTI line.
    EXTI_clear_line_flag(EXTI_LINE_UART4);
}

#if (STM32G4XX_REGISTERS_MCU_CATEGORY == 3) || (STM32G4XX_REGISTERS_MCU_CATEGORY == 4)
/*******************************************************************/
void __attribute__((optimize("-O0"))) UART5_IRQHandler(void) {
    // Execute internal callback.
    _USART_irq_handler(USART_INSTANCE_UART5);
    // Clear EXTI line.
    EXTI_clear_line_flag(EXTI_LINE_UART5);
}
#endif

#ifdef STM32G4XX_DRIVERS_USART_RS485
/*******************************************************************/
static USART_status_t _USART_set_rs485_mode(USART_rs485_mode_t rs485_mode) {
    // Local variables.
    USART_status_t status = USART_SUCCESS;
    // Set RX mode.
    switch (rs485_mode) {
    case USART_RS485_MODE_ADDRESSED:
        // Enable mute mode, address detection and wake up on address match.
        USART1->CR1 |= 0x00002800; // MME='1' and WAKE='1'.
        USART1->CR3 &= 0xFFCFFFFF; // WUS='00'.
        break;
    case USART_RS485_MODE_DIRECT:
        // Disable mute mode, address detection and wake-up on RXNE.
        USART1->CR1 &= 0xFFFFD7FF; // MME='0' and WAKE='0'.
        USART1->CR3 |= 0x00030000; // WUS='11'.
        break;
    default:
        status = USART_ERROR_RS485_MODE;
        goto errors;
    }
errors:
    return status;
}
#endif

/*** USART functions ***/

/*******************************************************************/
USART_status_t USART_init(USART_instance_t instance, const USART_gpio_t* pins, USART_configuration_t* configuration) {
    // Local variables.
    USART_status_t status = USART_SUCCESS;
    uint32_t brr = 0;
    uint32_t reg_value = 0;
    // Check instance.
    if (instance >= USART_INSTANCE_LAST) {
        status = USART_ERROR_INSTANCE;
        goto errors;
    }
    // Check parameters.
    if ((pins == NULL) || (configuration == NULL)) {
        status = USART_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Check state.
    if (usart_ctx[instance].init_flag != 0) {
        status = USART_ERROR_ALREADY_INITIALIZED;
        goto errors;
    }
    // Select peripheral clock.
    RCC->CCIPR &= ~(0b11 << USART_DESCRIPTOR[instance].rcc_ccipr_shift);
    switch (configuration->clock) {
    case RCC_CLOCK_SYSTEM:
        // Nothing to do.
        break;
    case RCC_CLOCK_HSI:
        RCC->CCIPR |= (0b10 << USART_DESCRIPTOR[instance].rcc_ccipr_shift);
        break;
    case RCC_CLOCK_LSE:
        RCC->CCIPR |= (0b11 << USART_DESCRIPTOR[instance].rcc_ccipr_shift);
        break;
    default:
        status = USART_ERROR_CLOCK;
        goto errors;
    }
    // Get clock source frequency.
    RCC_get_frequency_hz((configuration->clock), &(usart_ctx[instance].clock_hz));
    // Enable peripheral clock.
    (*USART_DESCRIPTOR[instance].rcc_enr) |= USART_DESCRIPTOR[instance].rcc_mask;
    (*USART_DESCRIPTOR[instance].rcc_smenr) |= USART_DESCRIPTOR[instance].rcc_mask;
    // Disable overrun detection (OVRDIS='1').
    USART_DESCRIPTOR[instance].peripheral->CR3 |= (0b1 << 12);
    // Baud rate.
    brr = ((usart_ctx[instance].clock_hz) / (configuration->baud_rate));
    // Check value.
    if ((brr < USART_BRR_VALUE_MIN) || (brr > USART_BRR_VALUE_MAX)) {
        status = USART_ERROR_BAUD_RATE;
        goto errors;
    }
    reg_value = ((USART_DESCRIPTOR[instance].peripheral->BRR) & (~USART_REGISTER_MASK_BRR));
    reg_value |= (brr & USART_REGISTER_MASK_BRR);
    USART_DESCRIPTOR[instance].peripheral->BRR = reg_value;
    // Auto baud rate default state.
    USART_DESCRIPTOR[instance].peripheral->CR2 &= ~(0b11 << 21);
    USART_DESCRIPTOR[instance].peripheral->CR2 &= ~(0b1 << 20);
    // Auto baud rate settings.
    switch (configuration->auto_baud_rate_mode) {
    case USART_AUTO_BAUD_RATE_MODE_DISABLED:
        break;
    case USART_AUTO_BAUD_RATE_MODE_LSB_1:
        USART_DESCRIPTOR[instance].peripheral->CR2 |= (0b1 << 20);
        break;
    case USART_AUTO_BAUD_RATE_MODE_PATTERN_10:
        USART_DESCRIPTOR[instance].peripheral->CR2 |= (0b01 << 21);
        USART_DESCRIPTOR[instance].peripheral->CR2 |= (0b1 << 20);
        break;
    case USART_AUTO_BAUD_RATE_MODE_FRAME_7F:
        USART_DESCRIPTOR[instance].peripheral->CR2 |= (0b10 << 21);
        USART_DESCRIPTOR[instance].peripheral->CR2 |= (0b1 << 20);
        break;
    case USART_AUTO_BAUD_RATE_MODE_FRAME_55:
        USART_DESCRIPTOR[instance].peripheral->CR2 |= (0b11 << 21);
        USART_DESCRIPTOR[instance].peripheral->CR2 |= (0b1 << 20);
        break;
    default:
        status = USART_ERROR_AUTO_BAUD_RATE_MODE;
        goto errors;
    }
    // Configure peripheral.
    if (configuration->rxne_irq_callback != NULL) {
        USART_DESCRIPTOR[instance].peripheral->CR1 |= (0b1 << 5); // RXNEIE='1'.
    }
    else {
        USART_DESCRIPTOR[instance].peripheral->CR1 &= ~(0b1 << 5); // RXNEIE='0'.
    }
    if (configuration->cm_irq_callback != NULL) {
        USART_DESCRIPTOR[instance].peripheral->CR1 |= (0b1 << 14); // CMIE='1'.
        USART_DESCRIPTOR[instance].peripheral->CR2 &= 0x00FFFFFF;
        USART_DESCRIPTOR[instance].peripheral->CR2 |= ((configuration->match_character) << 24);
    }
    switch (configuration->parity) {
    case USART_PARITY_NONE:
        USART_DESCRIPTOR[instance].peripheral->CR1 &= ~(0b101 << 8);
        break;
    case USART_PARITY_EVEN:
        USART_DESCRIPTOR[instance].peripheral->CR1 |= (0b101 << 8);
        USART_DESCRIPTOR[instance].peripheral->CR1 &= ~(0b1 << 9);
        break;
    case USART_PARITY_ODD:
        USART_DESCRIPTOR[instance].peripheral->CR1 |= (0b111 << 8);
        break;
    default:
        status = USART_ERROR_PARITY;
        goto errors;
    }
    USART_DESCRIPTOR[instance].peripheral->CR3 |= (0b1 << 6); // DMAR='1'.
#ifdef STM32G4XX_DRIVERS_USART_RS485
    if (configuration->rs485_mode != USART_RS485_MODE_DISABLED) {
        // Configure address.
        USART_DESCRIPTOR[instance].peripheral->CR2 |= ((configuration->self_address) << 24) | (0b1 << 4);
        USART_DESCRIPTOR[instance].peripheral->CR3 |= 0x00004000;
        // Set mode.
        status = _USART_set_rs485_mode(configuration->rs485_mode);
        if (status != USART_SUCCESS) goto errors;
        // Update instance context.
        usart_ctx[instance].rs485_mode = (configuration->rs485_mode);
    }
#endif
    // Set interrupt priority.
    NVIC_set_priority(USART_DESCRIPTOR[instance].nvic_interrupt, (configuration->nvic_priority));
    // Configure GPIOs.
    GPIO_configure((pins->tx), GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure((pins->rx), GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#ifdef STM32G4XX_DRIVERS_USART_RS485
    if (usart_ctx[instance].rs485_mode != USART_RS485_MODE_DISABLED) {
        // Put NRE pin in high impedance since it is directly connected to the DE pin.
        GPIO_configure((pins->de), GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
        GPIO_configure((pins->nre), GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    }
#endif
    // Enable transmitter and receiver.
    USART_DESCRIPTOR[instance].peripheral->CR1 |= (0b11 << 2); // TE='1' and RE='1'.
    // Enable peripheral.
    USART_DESCRIPTOR[instance].peripheral->CR1 |= (0b11 << 0); // UE='1' and UESM='1'.
    // Register callbacks.
    usart_ctx[instance].rxne_irq_callback = (configuration->rxne_irq_callback);
    usart_ctx[instance].cm_irq_callback = (configuration->cm_irq_callback);
    // Update initialization flag.
    usart_ctx[instance].init_flag = 1;
errors:
    return status;
}

/*******************************************************************/
USART_status_t USART_de_init(USART_instance_t instance, const USART_gpio_t* pins) {
    // Local variables.
    USART_status_t status = USART_SUCCESS;
    // Check instance.
    if (instance >= USART_INSTANCE_LAST) {
        status = USART_ERROR_INSTANCE;
        goto errors;
    }
    // Check parameters.
    if (pins == NULL) {
        status = USART_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Check state.
    if (usart_ctx[instance].init_flag == 0) {
        status = USART_ERROR_UNINITIALIZED;
        goto errors;
    }
    // Disable USART alternate function.
    GPIO_configure((pins->tx), GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure((pins->rx), GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#ifdef STM32G4XX_DRIVERS_USART_RS485
    if (usart_ctx[instance].rs485_mode != USART_RS485_MODE_DISABLED) {
        // Put NRE pin in high impedance since it is directly connected to the DE pin.
        GPIO_configure((pins->de), GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
        GPIO_configure((pins->nre), GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    }
#endif
    // Disable peripheral.
    USART_DESCRIPTOR[instance].peripheral->CR1 &= ~(0b1 << 0); // UE='0'.
    // Disable peripheral clock.
    (*USART_DESCRIPTOR[instance].rcc_enr) &= ~(USART_DESCRIPTOR[instance].rcc_mask);
    // Update initialization flag.
    usart_ctx[instance].init_flag = 0;
#ifdef STM32G4XX_DRIVERS_USART_RS485
    usart_ctx[instance].rs485_mode = USART_RS485_MODE_DISABLED;
#endif
errors:
    return status;
}

/*******************************************************************/
USART_status_t USART_enable_rx(USART_instance_t instance) {
    // Local variables.
    USART_status_t status = USART_SUCCESS;
    // Check instance.
    if (instance >= USART_INSTANCE_LAST) {
        status = USART_ERROR_INSTANCE;
        goto errors;
    }
    // Check state.
    if (usart_ctx[instance].init_flag == 0) {
        status = USART_ERROR_UNINITIALIZED;
        goto errors;
    }
    // Clear RXNE flag if needed.
    if (((USART_DESCRIPTOR[instance].peripheral->ISR) & (0b1 << 5)) != 0) {
        USART_DESCRIPTOR[instance].peripheral->RQR |= (0b1 << 3);
    }
    // Enable interrupt.
    NVIC_enable_interrupt(USART_DESCRIPTOR[instance].nvic_interrupt);
    // Enable receiver.
    USART_DESCRIPTOR[instance].peripheral->CR1 |= (0b1 << 2); // RE='1'.
#ifdef STM32G4XX_DRIVERS_USART_RS485
    // Check mode.
    if ((((USART_DESCRIPTOR[instance].peripheral->CR1) & (0b1 << 13)) != 0) && (usart_ctx[instance].rs485_mode != USART_RS485_MODE_DISABLED)) {
        // Mute mode request.
        USART_DESCRIPTOR[instance].peripheral->RQR |= (0b1 << 2); // MMRQ='1'.
    }
#endif
errors:
    return status;
}

/*******************************************************************/
USART_status_t USART_disable_rx(USART_instance_t instance) {
    // Local variables.
    USART_status_t status = USART_SUCCESS;
    // Check instance.
    if (instance >= USART_INSTANCE_LAST) {
        status = USART_ERROR_INSTANCE;
        goto errors;
    }
    // Check state.
    if (usart_ctx[instance].init_flag == 0) {
        status = USART_ERROR_UNINITIALIZED;
        goto errors;
    }
    // Disable receiver.
    USART_DESCRIPTOR[instance].peripheral->CR1 &= ~(0b1 << 2); // RE='0'.
    // Disable interrupt.
    NVIC_disable_interrupt(USART_DESCRIPTOR[instance].nvic_interrupt);
errors:
    return status;
}

/*******************************************************************/
USART_status_t USART_write(USART_instance_t instance, uint8_t* data, uint32_t data_size_bytes) {
    // Local variables.
    USART_status_t status = USART_SUCCESS;
    uint32_t reg_value = 0;
    uint8_t idx = 0;
    uint32_t loop_count = 0;
    // Check instance.
    if (instance >= USART_INSTANCE_LAST) {
        status = USART_ERROR_INSTANCE;
        goto errors;
    }
    // Check parameters.
    if ((data == NULL) || (data_size_bytes == 0)) {
        status = USART_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Check state.
    if (usart_ctx[instance].init_flag == 0) {
        status = USART_ERROR_UNINITIALIZED;
        goto errors;
    }
    // Byte loop.
    for (idx = 0; idx < data_size_bytes; idx++) {
#ifdef STM32G4XX_DRIVERS_USART_DISABLE_TX_0
        // Do not transmit null byte.
        if (data[idx] == 0) continue;
#endif
        // Fill transmit register.
        reg_value = ((USART_DESCRIPTOR[instance].peripheral->TDR) & (~USART_REGISTER_MASK_TDR));
        reg_value |= (uint32_t) (data[idx] & USART_REGISTER_MASK_TDR);
        USART_DESCRIPTOR[instance].peripheral->TDR = reg_value;
        // Wait for transmission to complete.
        loop_count = 0;
        while (((USART_DESCRIPTOR[instance].peripheral->ISR) & (0b1 << 7)) == 0) {
            // Wait for TXE='1' or timeout.
            loop_count++;
            if (loop_count > USART_TIMEOUT_COUNT) {
                status = USART_ERROR_TX_TIMEOUT;
                goto errors;
            }
        }
    }
#ifdef STM32G4XX_DRIVERS_USART_RS485
    if (usart_ctx[instance].rs485_mode != USART_RS485_MODE_DISABLED) {
        // Wait for TC flag.
        while (((USART_DESCRIPTOR[instance].peripheral->ISR) & (0b1 << 6)) == 0) {
            // Exit if timeout.
            loop_count++;
            if (loop_count > USART_TIMEOUT_COUNT) {
                status = USART_ERROR_TC_TIMEOUT;
                goto errors;
            }
        }
    }
#endif
errors:
    return status;
}

/*******************************************************************/
USART_status_t USART_get_baud_rate(USART_instance_t instance, uint32_t* baud_rate) {
    // Local variables.
    USART_status_t status = USART_SUCCESS;
    // Check instance.
    if (instance >= USART_INSTANCE_LAST) {
        status = USART_ERROR_INSTANCE;
        goto errors;
    }
    if (baud_rate == NULL) {
        status = USART_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Compute baud rate.
    (*baud_rate) = ((usart_ctx[instance].clock_hz) / (USART_DESCRIPTOR[instance].peripheral->BRR));
errors:
    return status;
}

/*******************************************************************/
USART_status_t USART_auto_baud_rate_request(USART_instance_t instance) {
    // Local variables.
    USART_status_t status = USART_SUCCESS;
    // Check instance.
    if (instance >= USART_INSTANCE_LAST) {
        status = USART_ERROR_INSTANCE;
        goto errors;
    }
    USART_DESCRIPTOR[instance].peripheral->RQR |= (0b1 << 0); // ABRRQ='1'.
errors:
    return status;
}

/*******************************************************************/
USART_status_t USART_get_rdr_register_address(USART_instance_t instance, uint32_t* rdr_register_address) {
    // Local variables.
    USART_status_t status = USART_SUCCESS;
    // Check parameters.
    if (instance >= USART_INSTANCE_LAST) {
        status = USART_ERROR_INSTANCE;
        goto errors;
    }
    if (rdr_register_address == NULL) {
        status = USART_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Update address.
    (*rdr_register_address) = ((uint32_t) &(USART_DESCRIPTOR[instance].peripheral->RDR));
errors:
    return status;
}

#endif /* STM32G4XX_DRIVERS_DISABLE */
