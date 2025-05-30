/*
 * dmamux.h
 *
 *  Created on: 26 feb. 2025
 *      Author: Ludo
 */

#ifndef STM32G4XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32g4xx_drivers_flags.h"
#endif

#ifndef STM32G4XX_DRIVERS_DISABLE

#ifndef __DMAMUX_H__
#define __DMAMUX_H__

/*** DMAMUX structures ***/

/*!******************************************************************
 * \enum DMAMUX_request_t
 * \brief DMA requester multiplexer.
 *******************************************************************/
typedef enum {
    DMAMUX_PERIPHERAL_REQUEST_NONE = 0,
    DMAMUX_PERIPHERAL_REQUEST_G0,
    DMAMUX_PERIPHERAL_REQUEST_G1,
    DMAMUX_PERIPHERAL_REQUEST_G2,
    DMAMUX_PERIPHERAL_REQUEST_G3,
    DMAMUX_PERIPHERAL_REQUEST_ADC1,
    DMAMUX_PERIPHERAL_REQUEST_DAC1_CH1,
    DMAMUX_PERIPHERAL_REQUEST_DAC1_CH2,
    DMAMUX_PERIPHERAL_REQUEST_TIM6_UP,
    DMAMUX_PERIPHERAL_REQUEST_TIM7_UP,
    DMAMUX_PERIPHERAL_REQUEST_SPI1_RX,
    DMAMUX_PERIPHERAL_REQUEST_SPI1_TX,
    DMAMUX_PERIPHERAL_REQUEST_SPI2_RX,
    DMAMUX_PERIPHERAL_REQUEST_SPI2_TX,
    DMAMUX_PERIPHERAL_REQUEST_SPI3_RX,
    DMAMUX_PERIPHERAL_REQUEST_SPI3_TX,
    DMAMUX_PERIPHERAL_REQUEST_I2C1_RX,
    DMAMUX_PERIPHERAL_REQUEST_I2C1_TX,
    DMAMUX_PERIPHERAL_REQUEST_I2C2_RX,
    DMAMUX_PERIPHERAL_REQUEST_I2C2_TX,
    DMAMUX_PERIPHERAL_REQUEST_I2C3_RX,
    DMAMUX_PERIPHERAL_REQUEST_I2C3_TX,
    DMAMUX_PERIPHERAL_REQUEST_I2C4_RX,
    DMAMUX_PERIPHERAL_REQUEST_I2C4_TX,
    DMAMUX_PERIPHERAL_REQUEST_USART1_RX,
    DMAMUX_PERIPHERAL_REQUEST_USART1_TX,
    DMAMUX_PERIPHERAL_REQUEST_USART2_RX,
    DMAMUX_PERIPHERAL_REQUEST_USART2_TX,
    DMAMUX_PERIPHERAL_REQUEST_USART3_RX,
    DMAMUX_PERIPHERAL_REQUEST_USART3_TX,
    DMAMUX_PERIPHERAL_REQUEST_UART4_RX,
    DMAMUX_PERIPHERAL_REQUEST_UART4_TX,
    DMAMUX_PERIPHERAL_REQUEST_UART5_RX,
    DMAMUX_PERIPHERAL_REQUEST_UART5_TX,
    DMAMUX_PERIPHERAL_REQUEST_LPUART1_RX,
    DMAMUX_PERIPHERAL_REQUEST_LPUART1_TX,
    DMAMUX_PERIPHERAL_REQUEST_ADC2,
    DMAMUX_PERIPHERAL_REQUEST_ADC3,
    DMAMUX_PERIPHERAL_REQUEST_ADC4,
    DMAMUX_PERIPHERAL_REQUEST_ADC5,
    DMAMUX_PERIPHERAL_REQUEST_QUADSPI,
    DMAMUX_PERIPHERAL_REQUEST_DAC2_CH1,
    DMAMUX_PERIPHERAL_REQUEST_TIM1_CH1,
    DMAMUX_PERIPHERAL_REQUEST_TIM1_CH2,
    DMAMUX_PERIPHERAL_REQUEST_TIM1_CH3,
    DMAMUX_PERIPHERAL_REQUEST_TIM1_CH4,
    DMAMUX_PERIPHERAL_REQUEST_TIM1_UP,
    DMAMUX_PERIPHERAL_REQUEST_TIM1_TRIG,
    DMAMUX_PERIPHERAL_REQUEST_TIM1_COM,
    DMAMUX_PERIPHERAL_REQUEST_TIM8_CH1,
    DMAMUX_PERIPHERAL_REQUEST_TIM8_CH2,
    DMAMUX_PERIPHERAL_REQUEST_TIM8_CH3,
    DMAMUX_PERIPHERAL_REQUEST_TIM8_CH4,
    DMAMUX_PERIPHERAL_REQUEST_TIM8_UP,
    DMAMUX_PERIPHERAL_REQUEST_TIM8_TRIG,
    DMAMUX_PERIPHERAL_REQUEST_TIM8_COM,
    DMAMUX_PERIPHERAL_REQUEST_TIM2_CH1,
    DMAMUX_PERIPHERAL_REQUEST_TIM2_CH2,
    DMAMUX_PERIPHERAL_REQUEST_TIM2_CH3,
    DMAMUX_PERIPHERAL_REQUEST_TIM2_CH4,
    DMAMUX_PERIPHERAL_REQUEST_TIM2_UP,
    DMAMUX_PERIPHERAL_REQUEST_TIM3_CH1,
    DMAMUX_PERIPHERAL_REQUEST_TIM3_CH2,
    DMAMUX_PERIPHERAL_REQUEST_TIM3_CH3,
    DMAMUX_PERIPHERAL_REQUEST_TIM3_CH4,
    DMAMUX_PERIPHERAL_REQUEST_TIM3_UP,
    DMAMUX_PERIPHERAL_REQUEST_TIM3_TRIG,
    DMAMUX_PERIPHERAL_REQUEST_TIM4_CH1,
    DMAMUX_PERIPHERAL_REQUEST_TIM4_CH2,
    DMAMUX_PERIPHERAL_REQUEST_TIM4_CH3,
    DMAMUX_PERIPHERAL_REQUEST_TIM4_CH4,
    DMAMUX_PERIPHERAL_REQUEST_TIM4_UP,
    DMAMUX_PERIPHERAL_REQUEST_TIM5_CH1,
    DMAMUX_PERIPHERAL_REQUEST_TIM5_CH2,
    DMAMUX_PERIPHERAL_REQUEST_TIM5_CH3,
    DMAMUX_PERIPHERAL_REQUEST_TIM5_CH4,
    DMAMUX_PERIPHERAL_REQUEST_TIM5_UP,
    DMAMUX_PERIPHERAL_REQUEST_TIM5_TRIG,
    DMAMUX_PERIPHERAL_REQUEST_TIM15_CH1,
    DMAMUX_PERIPHERAL_REQUEST_TIM15_UP,
    DMAMUX_PERIPHERAL_REQUEST_TIM15_TRIG,
    DMAMUX_PERIPHERAL_REQUEST_TIM15_COM,
    DMAMUX_PERIPHERAL_REQUEST_TIM16_CH1,
    DMAMUX_PERIPHERAL_REQUEST_TIM16_UP,
    DMAMUX_PERIPHERAL_REQUEST_TIM17_CH1,
    DMAMUX_PERIPHERAL_REQUEST_TIM17_UP,
    DMAMUX_PERIPHERAL_REQUEST_TIM20_CH1,
    DMAMUX_PERIPHERAL_REQUEST_TIM20_CH2,
    DMAMUX_PERIPHERAL_REQUEST_TIM20_CH3,
    DMAMUX_PERIPHERAL_REQUEST_TIM20_CH4,
    DMAMUX_PERIPHERAL_REQUEST_TIM20_UP,
    DMAMUX_PERIPHERAL_REQUEST_AES_IN,
    DMAMUX_PERIPHERAL_REQUEST_AES_OUT,
    DMAMUX_PERIPHERAL_REQUEST_TIM20_TRIG,
    DMAMUX_PERIPHERAL_REQUEST_TIM20_COM,
    DMAMUX_PERIPHERAL_REQUEST_HRTIM_MASTER,
    DMAMUX_PERIPHERAL_REQUEST_HRTIM_TIMA,
    DMAMUX_PERIPHERAL_REQUEST_HRTIM_TIMB,
    DMAMUX_PERIPHERAL_REQUEST_HRTIM_TIMC,
    DMAMUX_PERIPHERAL_REQUEST_HRTIM_TIMD,
    DMAMUX_PERIPHERAL_REQUEST_HRTIM_TIME,
    DMAMUX_PERIPHERAL_REQUEST_HRTIM_TIMF,
    DMAMUX_PERIPHERAL_REQUEST_DAC3_CH1,
    DMAMUX_PERIPHERAL_REQUEST_DAC3_CH2,
    DMAMUX_PERIPHERAL_REQUEST_DAC4_CH1,
    DMAMUX_PERIPHERAL_REQUEST_DAC4_CH2,
    DMAMUX_PERIPHERAL_REQUEST_SPI4_RX,
    DMAMUX_PERIPHERAL_REQUEST_SPI4_TX,
    DMAMUX_PERIPHERAL_REQUEST_SAI1_A,
    DMAMUX_PERIPHERAL_REQUEST_SAI1_B,
    DMAMUX_PERIPHERAL_REQUEST_FMAC_READ,
    DMAMUX_PERIPHERAL_REQUEST_FMAC_WRITE,
    DMAMUX_PERIPHERAL_REQUEST_CORDIC_READ,
    DMAMUX_PERIPHERAL_REQUEST_CORDIC_WRITE,
    DMAMUX_PERIPHERAL_REQUEST_UCPD1_RX,
    DMAMUX_PERIPHERAL_REQUEST_UCPD1_TX,
    DMAMUX_PERIPHERAL_REQUEST_LAST
} DMAMUX_peripheral_request_t;

#endif /* __DMAMUX_H__ */

#endif /* STM32G4XX_DRIVERS_DISABLE */
