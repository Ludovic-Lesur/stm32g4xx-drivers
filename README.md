# Description

This repository contains the **peripherals drivers** of the STM32G4xx MCUs.

# Dependencies

The drivers rely on:

* The **STM32G4xx linker scripts** defined in the [stm32g4xx-device](https://github.com/Ludovic-Lesur/stm32g4xx-device) repository.
* The **STM32G4xx registers** defined in the [stm32g4xx-registers](https://github.com/Ludovic-Lesur/stm32g4xx-registers) repository.
* The **embedded utility functions** defined in the [embedded-utils](https://github.com/Ludovic-Lesur/embedded-utils) repository.

Here is the versions compatibility table:

| **stm32g4xx-drivers** | **stm32g4xx-device** | **stm32g4xx-registers** | **embedded-utils** |
|:---:|:---:|:---:|:---:|
| [sw2.1](https://github.com/Ludovic-Lesur/stm32g4xx-drivers/releases/tag/sw2.1) | >= [sw2.0](https://github.com/Ludovic-Lesur/stm32g4xx-device/releases/tag/sw2.0) | >= [sw1.1](https://github.com/Ludovic-Lesur/stm32g4xx-registers/releases/tag/sw1.1) | >= [sw7.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw7.0) |
| [sw2.0](https://github.com/Ludovic-Lesur/stm32g4xx-drivers/releases/tag/sw2.0) | >= [sw2.0](https://github.com/Ludovic-Lesur/stm32g4xx-device/releases/tag/sw2.0) | >= [sw1.1](https://github.com/Ludovic-Lesur/stm32g4xx-registers/releases/tag/sw1.1) | >= [sw7.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw7.0) |
| [sw1.0](https://github.com/Ludovic-Lesur/stm32g4xx-drivers/releases/tag/sw1.0) | >= [sw2.0](https://github.com/Ludovic-Lesur/stm32g4xx-device/releases/tag/sw2.0) | >= [sw1.1](https://github.com/Ludovic-Lesur/stm32g4xx-registers/releases/tag/sw1.1) | >= [sw7.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw7.0) |

# Compilation flags

| **Flag name** | **Value** | **Description** |
|:---:|:---:|:---:|
| `STM32G4XX_DRIVERS_DISABLE_FLAGS_FILE` | `defined` / `undefined` | Disable the `stm32g4xx_drivers_flags.h` header file inclusion when compilation flags are given in the project settings or by command line. |
| `STM32G4XX_DRIVERS_ADC_MODE_MASK` | `0x00` to `0x03` | 2-bits field which defines the ADC operation modes: `0` = single `1` = sequence |
| `STM32G4XX_DRIVERS_ADC_VREF_MV` | `<value>` | If defined, gives the external reference voltage provided to the VREF+ pin. |
| `STM32G4XX_DRIVERS_DMA_CHANNEL_MASK` | `0x0000` to `0xFFFF` | 16-bits field which defines the enabled DMA channels. |
| `STM32G4XX_DRIVERS_EXTI_GPIO_MASK` | `0x0000` to `0xFFFF` | 16-bits field which defines the enabled EXTI GPIO lines. |
| `STM32G4XX_DRIVERS_LPUART_RS485` | `defined` / `undefined` | Enable or disable RS485 operation. |
| `STM32G4XX_DRIVERS_LPUART_DISABLE_TX_0` | `defined` / `undefined` | Disable the transmission of byte 0x00 if defined. |
| `STM32G4XX_DRIVERS_RCC_HSE_ENABLE` | `defined` / `undefined` | Enable or disable external oscillator functions. |
| `STM32G4XX_DRIVERS_RCC_HSE_FREQUENCY_HZ` | `<value>` | Defines the external high speed crystal frequency in Hz (if used). |
| `STM32G4XX_DRIVERS_RCC_LSE_MODE` | `0` / `1` / `2` | LSE crystal mode: `0` = disabled `1` = enabled with LSI/HSI fallback `2` = enabled and mandatory. |
| `STM32G4XX_DRIVERS_RCC_LSE_FREQUENCY_HZ` | `<value>` | Defines the external low speed crystal frequency in Hz (if used). |
| `STM32G4XX_DRIVERS_RTC_WAKEUP_PERIOD_SECONDS` | `<value>` | RTC wakeup period in seconds. |
| `STM32G4XX_DRIVERS_RTC_ALARM_MASK` | `0x00` to `0x03`| 2-bits field which defines the enabled RTC alarms. |
| `STM32G4XX_DRIVERS_TIM_MODE_MASK` | `0x00` to `0x3F`| 6-bits field which defines the enabled timer operation modes: `0` = standard `1` = multi-channel `2` = calibration `3` = PWM `4` = one pulse `5` = capture. |
| `STM32G4XX_DRIVERS_USART_RS485` | `defined` / `undefined` | Enable or disable RS485 operation. |
| `STM32G4XX_DRIVERS_USART_DISABLE_TX_0` | `defined` / `undefined` | Disable the transmission of byte 0x00 if defined. |
