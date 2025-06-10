/*
 * exti.c
 *
 *  Created on: 15 feb. 2025
 *      Author: Ludo
 */

#ifndef STM32G4XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32g4xx_drivers_flags.h"
#endif

#ifndef STM32G4XX_DRIVERS_DISABLE

#include "exti.h"

#include "exti_registers.h"
#include "gpio.h"
#include "nvic.h"
#include "rcc_registers.h"
#include "syscfg_registers.h"
#include "types.h"

/*** EXTI local macros ***/

#define EXTI_NVIC_SHARED_GPIO_MASK_PIN5_PIN9    0x03E0
#define EXTI_NVIC_SHARED_GPIO_MASK_PIN10_PIN15  0xFC00

#define EXTI_REGISTER_MASK_IMR1_EMR1            0xFFFFFFFF
#define EXTI_REGISTER_MASK_IMR2_EMR2            0x00000F3F
#define EXTI_REGISTER_MASK_PR1                  0xE07BFFFF
#define EXTI_REGISTER_MASK_PR2                  0x00000303

#if ((STM32G4XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_ALL) != 0)

/*** EXTI local structures ***/

/*******************************************************************/
typedef struct {
    NVIC_interrupt_t nvic_interrupt;
    uint16_t nvic_shared_mask;
} EXTI_gpio_descriptor_t;

/*******************************************************************/
typedef struct {
    EXTI_gpio_irq_cb_t edge_irq_callback;
} EXTI_gpio_context_t;

/*******************************************************************/
typedef struct {
    uint16_t enabled_gpio_mask;
    EXTI_gpio_context_t gpio_ctx[GPIO_PINS_PER_PORT];
} EXTI_context_t;

/*** EXTI local global variables ***/

static const EXTI_gpio_descriptor_t EXTI_GPIO_DESCRIPTOR[GPIO_PINS_PER_PORT] = {
    { NVIC_INTERRUPT_EXTI_0, EXTI_GPIO_MASK_PIN0 },
    { NVIC_INTERRUPT_EXTI_1, EXTI_GPIO_MASK_PIN1 },
    { NVIC_INTERRUPT_EXTI_2, EXTI_GPIO_MASK_PIN2 },
    { NVIC_INTERRUPT_EXTI_3, EXTI_GPIO_MASK_PIN3 },
    { NVIC_INTERRUPT_EXTI_4, EXTI_GPIO_MASK_PIN4 },
    { NVIC_INTERRUPT_EXTI_5_9, EXTI_NVIC_SHARED_GPIO_MASK_PIN5_PIN9 },
    { NVIC_INTERRUPT_EXTI_5_9, EXTI_NVIC_SHARED_GPIO_MASK_PIN5_PIN9 },
    { NVIC_INTERRUPT_EXTI_5_9, EXTI_NVIC_SHARED_GPIO_MASK_PIN5_PIN9 },
    { NVIC_INTERRUPT_EXTI_5_9, EXTI_NVIC_SHARED_GPIO_MASK_PIN5_PIN9 },
    { NVIC_INTERRUPT_EXTI_5_9, EXTI_NVIC_SHARED_GPIO_MASK_PIN5_PIN9 },
    { NVIC_INTERRUPT_EXTI_10_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN10_PIN15 },
    { NVIC_INTERRUPT_EXTI_10_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN10_PIN15 },
    { NVIC_INTERRUPT_EXTI_10_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN10_PIN15 },
    { NVIC_INTERRUPT_EXTI_10_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN10_PIN15 },
    { NVIC_INTERRUPT_EXTI_10_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN10_PIN15 },
    { NVIC_INTERRUPT_EXTI_10_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN10_PIN15 }
};

static EXTI_context_t exti_ctx = {
    .enabled_gpio_mask = 0,
    .gpio_ctx = {
        [0 ... (GPIO_PINS_PER_PORT - 1)] {
            .edge_irq_callback = NULL
        }
    }
};

/*** EXTI local functions ***/

/*******************************************************************/
#define _EXTI_irq_handler(pin) { \
    /* Check flag */ \
    if ((((EXTI->EXTIx[0]).PR) & (0b1 << pin)) != 0) { \
        /* Check mask and callback */ \
        if (((((EXTI->EXTIx[0]).IMR) & (0b1 << pin)) != 0) && (exti_ctx.gpio_ctx[pin].edge_irq_callback != NULL)) { \
            /* Execute callback */ \
            exti_ctx.gpio_ctx[pin].edge_irq_callback(); \
        } \
        /* Clear flag */ \
        (EXTI->EXTIx[0]).PR |= (0b1 << pin); \
    } \
}

#if ((STM32G4XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN0) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) EXTI0_IRQHandler(void) {
    _EXTI_irq_handler(0);
}
#endif

#if ((STM32G4XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN1) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) EXTI1_IRQHandler(void) {
    _EXTI_irq_handler(1);
}
#endif

#if ((STM32G4XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN2) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) EXTI2_IRQHandler(void) {
    _EXTI_irq_handler(2);
}
#endif

#if ((STM32G4XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN3) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) EXTI3_IRQHandler(void) {
    _EXTI_irq_handler(3);
}
#endif

#if ((STM32G4XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN4) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) EXTI4_IRQHandler(void) {
    _EXTI_irq_handler(4);
}
#endif

#if ((STM32G4XX_DRIVERS_EXTI_GPIO_MASK & EXTI_NVIC_SHARED_GPIO_MASK_PIN5_PIN9) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) EXTI5_9_IRQHandler(void) {
#if ((STM32G4XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN5) != 0)
    _EXTI_irq_handler(5);
#endif
#if ((STM32G4XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN6) != 0)
    _EXTI_irq_handler(6);
#endif
#if ((STM32G4XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN7) != 0)
    _EXTI_irq_handler(7);
#endif
#if ((STM32G4XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN8) != 0)
    _EXTI_irq_handler(8);
#endif
#if ((STM32G4XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN9) != 0)
    _EXTI_irq_handler(9);
#endif
}
#endif

#if ((STM32G4XX_DRIVERS_EXTI_GPIO_MASK & EXTI_NVIC_SHARED_GPIO_MASK_PIN10_PIN15) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) EXTI10_15_IRQHandler(void) {
#if ((STM32G4XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN10) != 0)
    _EXTI_irq_handler(10);
#endif
#if ((STM32G4XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN11) != 0)
    _EXTI_irq_handler(11);
#endif
#if ((STM32G4XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN12) != 0)
    _EXTI_irq_handler(12);
#endif
#if ((STM32G4XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN13) != 0)
    _EXTI_irq_handler(13);
#endif
#if ((STM32G4XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN14) != 0)
    _EXTI_irq_handler(14);
#endif
#if ((STM32G4XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN15) != 0)
    _EXTI_irq_handler(15);
#endif
}
#endif

#endif /* STM32G4XX_DRIVERS_EXTI_GPIO_MASK */

/*******************************************************************/
static void _EXTI_set_trigger(uint8_t register_index, uint8_t line_index, EXTI_trigger_t trigger) {
    // Select triggers.
    switch (trigger) {
    // Rising edge only.
    case EXTI_TRIGGER_RISING_EDGE:
        (EXTI->EXTIx[register_index]).RTSR |= (0b1 << line_index);
        (EXTI->EXTIx[register_index]).FTSR &= ~(0b1 << line_index);
        break;
    // Falling edge only.
    case EXTI_TRIGGER_FALLING_EDGE:
        (EXTI->EXTIx[register_index]).RTSR &= ~(0b1 << line_index);
        (EXTI->EXTIx[register_index]).FTSR |= (0b1 << line_index);
        break;
    // Both edges.
    case EXTI_TRIGGER_ANY_EDGE:
        (EXTI->EXTIx[register_index]).RTSR |= (0b1 << line_index);
        (EXTI->EXTIx[register_index]).FTSR |= (0b1 << line_index);
        break;
    // Unknown configuration.
    default:
        goto errors;
    }
    // Clear flag.
    (EXTI->EXTIx[register_index]).PR |= (0b1 << line_index);
errors:
    return;
}

/*** EXTI functions ***/

/*******************************************************************/
void EXTI_init(void) {
    // Enable peripheral clock.
    RCC->APB2ENR |= (0b1 << 0);
    // Mask all sources by default.
    (EXTI->EXTIx[0]).IMR &= (~EXTI_REGISTER_MASK_IMR1_EMR1);
    (EXTI->EXTIx[0]).EMR &= (~EXTI_REGISTER_MASK_IMR1_EMR1);
    (EXTI->EXTIx[1]).IMR &= (~EXTI_REGISTER_MASK_IMR2_EMR2);
    (EXTI->EXTIx[1]).EMR &= (~EXTI_REGISTER_MASK_IMR2_EMR2);
    // Clear all flags.
    (EXTI->EXTIx[0]).PR |= EXTI_REGISTER_MASK_PR1;
    (EXTI->EXTIx[1]).PR |= EXTI_REGISTER_MASK_PR2;
}

/*******************************************************************/
void EXTI_de_init(void) {
    // Clear all flags.
    (EXTI->EXTIx[0]).PR |= EXTI_REGISTER_MASK_PR1;
    (EXTI->EXTIx[1]).PR |= EXTI_REGISTER_MASK_PR2;
    // Mask all sources.
    (EXTI->EXTIx[0]).IMR &= (~EXTI_REGISTER_MASK_IMR1_EMR1);
    (EXTI->EXTIx[0]).EMR &= (~EXTI_REGISTER_MASK_IMR1_EMR1);
    (EXTI->EXTIx[1]).IMR &= (~EXTI_REGISTER_MASK_IMR2_EMR2);
    (EXTI->EXTIx[1]).EMR &= (~EXTI_REGISTER_MASK_IMR2_EMR2);
    // Disable peripheral clock.
    RCC->APB2ENR &= ~(0b1 << 0);
}

/*******************************************************************/
void EXTI_enable_line(EXTI_line_t line, EXTI_trigger_t trigger) {
    // Local variables.
    uint8_t register_idx = (line >> 5);
    uint8_t line_idx = (line % 32);
    // Select triggers.
    _EXTI_set_trigger(register_idx, line_idx, trigger);
    // Set mask.
    (EXTI->EXTIx[register_idx]).IMR |= (0b1 << line_idx);
}

/*******************************************************************/
void EXTI_disable_line(EXTI_line_t line) {
    // Local variables.
    uint8_t register_idx = (line >> 5);
    uint8_t line_idx = (line % 32);
    // Set mask.
    (EXTI->EXTIx[register_idx]).IMR &= ~(0b1 << line_idx);
}

/*******************************************************************/
void EXTI_clear_line_flag(EXTI_line_t line) {
    // Local variables.
    uint8_t register_idx = (line >> 5);
    uint8_t line_idx = (line % 32);
    // Clear flag.
    (EXTI->EXTIx[register_idx]).PR |= (0b1 << line_idx);
}

#if ((STM32G4XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_ALL) != 0)
/*******************************************************************/
void EXTI_configure_gpio(const GPIO_pin_t* gpio, GPIO_pull_resistor_t pull_resistor, EXTI_trigger_t trigger, EXTI_gpio_irq_cb_t irq_callback, uint8_t nvic_priority) {
    // Local variables.
    uint8_t line_idx = (gpio->pin);
    // Select GPIO port.
    SYSCFG->EXTICR[(line_idx >> 2)] &= ~(0b1111 << ((line_idx % 4) << 2));
    SYSCFG->EXTICR[(line_idx >> 2)] |= ((gpio->port_index) << ((line_idx % 4) << 2));
    // Select triggers.
    _EXTI_set_trigger(0, line_idx, trigger);
    // Set interrupt priority.
    NVIC_set_priority(EXTI_GPIO_DESCRIPTOR[line_idx].nvic_interrupt, nvic_priority);
    // Register callback.
    exti_ctx.gpio_ctx[line_idx].edge_irq_callback = irq_callback;
    // Configure GPIO.
    GPIO_configure(gpio, GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, pull_resistor);
}
#endif

#if ((STM32G4XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_ALL) != 0)
/*******************************************************************/
void EXTI_release_gpio(const GPIO_pin_t* gpio, GPIO_mode_t released_mode) {
    // Set state.
    EXTI_disable_gpio_interrupt(gpio);
    // Release GPIO.
    GPIO_configure(gpio, released_mode, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    // Reset callback.
    exti_ctx.gpio_ctx[gpio->pin].edge_irq_callback = NULL;
}
#endif

#if ((STM32G4XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_ALL) != 0)
/*******************************************************************/
void EXTI_enable_gpio_interrupt(const GPIO_pin_t* gpio) {
    // Local variables.
    uint8_t line_idx = (gpio->pin);
    // Update mask.
    exti_ctx.enabled_gpio_mask |= (0b1 << line_idx);
    // Set mask.
    (EXTI->EXTIx[0]).IMR |= (0b1 << line_idx);
    // Enable interrupt.
    NVIC_enable_interrupt(EXTI_GPIO_DESCRIPTOR[line_idx].nvic_interrupt);
}
#endif

#if ((STM32G4XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_ALL) != 0)
/*******************************************************************/
void EXTI_disable_gpio_interrupt(const GPIO_pin_t* gpio) {
    // Local variables.
    uint8_t line_idx = (gpio->pin);
    // Update mask.
    exti_ctx.enabled_gpio_mask &= ~(0b1 << line_idx);
    // Set mask.
     (EXTI->EXTIx[0]).IMR &= ~(0b1 << line_idx);
    // Disable interrupt.
    if ((exti_ctx.enabled_gpio_mask & EXTI_GPIO_DESCRIPTOR[line_idx].nvic_shared_mask) == 0) {
        NVIC_disable_interrupt(EXTI_GPIO_DESCRIPTOR[line_idx].nvic_interrupt);
    }
}
#endif

#if ((STM32G4XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_ALL) != 0)
/*******************************************************************/
void EXTI_clear_gpio_flag(const GPIO_pin_t* gpio) {
    // Local variables.
    uint8_t line_idx = (gpio->pin);
    // Clear flag.
    (EXTI->EXTIx[0]).PR |= (0b1 << line_idx);
}
#endif

#endif /* STM32G4XX_DRIVERS_DISABLE */
