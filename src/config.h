/*

Copyright 2011-2018 Stratify Labs, Inc

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

*/

#ifndef CONFIG_H_
#define CONFIG_H_

#include <mcu/arch.h>

#include "board_config.h"

//openocd -f interface/stlink-v2-1.cfg -f target/stm32f4x_stlink.cfg


#define SOS_BOARD_SYSTEM_CLOCK 216000000
#define SOS_BOARD_SYSTEM_MEMORY_SIZE (8192*3)
#define SOS_BOARD_ID "-LABUlXF_3U2nEosAKsD"
#define SOS_BOARD_VERSION "0.12"
#define SOS_BOARD_NAME "STM32F723E-DISCO"

#define SOS_BOARD_USB_RX_BUFFER_SIZE 512
#define SOS_BOARD_STDIO_BUFFER_SIZE 512
#define SOS_BOARD_TMR 1

//Total number of tasks (threads) for the entire system
#define SOS_BOARD_TASK_TOTAL 10
#define SOS_BOARD_EVENT_HANDLER board_event_handler
#define SOS_BOARD_TRACE_EVENT board_trace_event

#define STM32_ARCH_O_FLAGS STM32_CONFIG_FLAG_IS_HSE_ON
#define STM32_ARCH_CLOCK_PLLM 25
#define STM32_ARCH_CLOCK_PLLN 432
#define STM32_ARCH_CLOCK_PLLP 2
#define STM32_ARCH_CLOCK_PLLQ 9
#define STM32_ARCH_CLOCK_PLLR 0
#define STM32_ARCH_CLOCK_AHB_CLOCK_DIVIDER 1
#define STM32_ARCH_CLOCK_APB1_CLOCK_DIVIDER 4
#define STM32_ARCH_CLOCK_APB2_CLOCK_DIVIDER 2
#define STM32_ARCH_CLOCK_VOLTAGE_SCALE 1
#define STM32_ARCH_CLOCK_FLASH_LATENCY 7

//Testing with high speed USB
#if 0
#define SOS_BOARD_USB_PORT 1
#define SOS_BOARD_USB_DM_PIN mcu_pin(1,14)
#define SOS_BOARD_USB_DP_PIN mcu_pin(1,15)
#define SOS_BOARD_RX_FIFO_WORDS 0x200
#define SOS_BOARD_TX0_FIFO_WORDS 0x80
#define SOS_BOARD_TX1_FIFO_WORDS 0x80
#define SOS_BOARD_USB_ATTR_FLAGS (USB_FLAG_SET_DEVICE | USB_FLAG_IS_HIGH_SPEED)
#endif

//--------------------------------------------Disco Definitions-------------------------------------------------

#define DISCO_DEBUG_UART_PORT 5
#define DISCO_DEBUG_UART_RX_PORT 2
#define DISCO_DEBUG_UART_RX_PIN 7
#define DISCO_DEBUG_UART_TX_PORT 2
#define DISCO_DEBUG_UART_TX_PIN 6
#define DISCO_LED_PORT 0
#define DISCO_LED_PIN 5
#define DISCO_BOOT_HARDWARE_REQUEST_PORT 0
#define DISCO_BOOT_HARDWARE_REQUEST_PIN 0

//--------------------------------------------Symbols-------------------------------------------------

/* By defining Ignore switches, functions can be omitted from the kernel
 * This means any applications that use these functions will fail
 * to install on the BSP.
 * 
 * If you are building a custom board, ignoring symbols is a good
 * way to squeeze down the kernel to only what is necessary. However,
 * if you plan on allowing your users to install applications, they
 * might find it challenging when some functions are missing (the
 * applications will compile but fail to install).
 * 
 * See [sos/symbols/defines.h](https://github.com/StratifyLabs/StratifyOS/blob/master/include/sos/symbols/defines.h) 
 * for all available switches.
 * 
 */ 

#define SYMBOLS_IGNORE_DCOMPLEX 1
#define SYMBOLS_IGNORE_POSIX_TRACE 1 //Always ignore -- deprecated
#define SYMBOLS_IGNORE_SG 1 //Stratify Graphics -- ignore if board will not support displays
#define SYMBOLS_IGNORE_SOCKET 1
#define SYMBOLS_IGNORE_LWIP 1

//other ignore switches
#if 0
#define SYMBOLS_IGNORE_MATH_F 1
#define SYMBOLS_IGNORE_DOUBLE 1
#define SYMBOLS_IGNORE_STDIO_FILE 1
#define SYMBOLS_IGNORE_SIGNAL 1
#define SYMBOLS_IGNORE_PTHREAD_MUTEX 1
#define SYMBOLS_IGNORE_PTHREAD_COND 1
#define SYMBOLS_IGNORE_AIO 1
#define SYMBOLS_IGNORE_WCTYPE 1
#define SYMBOLS_IGNORE_STR 1
#define SYMBOLS_IGNORE_SEM 1
#define SYMBOLS_IGNORE_MQ 1
#endif

/* Uncomment to add ARM CMSIS DSP libraries to the BSP
 * 
 * See [ARM CMSIS Declaration](https://github.com/StratifyLabs/StratifyOS-CMSIS/blob/master/arm_dsp_api_declaration.h)
 * for more detailed link configuration switches.
 * 
 * 
 */
#define SOS_BOARD_ARM_DSP_API_Q7 0
#define SOS_BOARD_ARM_DSP_API_Q15 0
#define SOS_BOARD_ARM_DSP_API_Q31 0
#define SOS_BOARD_ARM_DSP_API_F32 0
#define SOS_BOARD_ARM_DSP_CONVERSION_API 0

//--------------------------------------------Hardware Pins-------------------------------------------------

#define SOS_BOARD_USART2_TX_PORT 3 //PD5
#define SOS_BOARD_USART2_TX_PIN 5
#define SOS_BOARD_USART2_RX_PORT 3 //PD6
#define SOS_BOARD_USART2_RX_PIN 6

#define SOS_BOARD_USART3_TX_PORT 3 //PD8
#define SOS_BOARD_USART3_TX_PIN 8
#define SOS_BOARD_USART3_RX_PORT 3 //PD9
#define SOS_BOARD_USART3_RX_PIN 9

#define SOS_BOARD_USART6_TX_PORT 6 //PG5
#define SOS_BOARD_USART6_TX_PIN 9
#define SOS_BOARD_USART6_RX_PORT 6 //PG14
#define SOS_BOARD_USART6_RX_PIN 14

#define SOS_BOARD_I2C1_SDA_PORT 1 //PB9
#define SOS_BOARD_I2C1_SDA_PIN 9
#define SOS_BOARD_I2C1_SCL_PORT 1 //PB8
#define SOS_BOARD_I2C1_SCL_PIN 8

#define SOS_BOARD_I2C2_SDA_PORT 5 //PF0
#define SOS_BOARD_I2C2_SDA_PIN 0
#define SOS_BOARD_I2C2_SCL_PORT 5 //PF1
#define SOS_BOARD_I2C2_SCL_PIN 1

#define SOS_BOARD_SPI1_MISO_PORT 0 //PA6
#define SOS_BOARD_SPI1_MISO_PIN 6
#define SOS_BOARD_SPI1_MOSI_PORT 0 //PA7
#define SOS_BOARD_SPI1_MOSI_PIN 7
#define SOS_BOARD_SPI1_SCK_PORT 0 //PA5
#define SOS_BOARD_SPI1_SCK_PIN 5
#define SOS_BOARD_SPI1_CS_PORT 0xff //Not used
#define SOS_BOARD_SPI1_CS_PIN 0xff

#define SOS_BOARD_SPI3_MISO_PORT 1 //PB4
#define SOS_BOARD_SPI3_MISO_PIN 4
#define SOS_BOARD_SPI3_MOSI_PORT 1 //PB5
#define SOS_BOARD_SPI3_MOSI_PIN 5
#define SOS_BOARD_SPI3_SCK_PORT 1  //PB3
#define SOS_BOARD_SPI3_SCK_PIN 3
#define SOS_BOARD_SPI3_CS_PORT 0xff //Not used
#define SOS_BOARD_SPI3_CS_PIN 0xff

#define SOS_BOARD_QSPI_CLK_PORT 1 //PB2
#define SOS_BOARD_QSPI_CLK_PIN 2
#define SOS_BOARD_QSPI_NCS_PORT 1 //PB6
#define SOS_BOARD_QSPI_NCS_PIN 6
#define SOS_BOARD_QSPI_IO0_PORT 2 //PC9
#define SOS_BOARD_QSPI_IO0_PIN 9
#define SOS_BOARD_QSPI_IO1_PORT 2 //PC10
#define SOS_BOARD_QSPI_IO1_PIN 10
#define SOS_BOARD_QSPI_IO2_PORT 3 //PD13
#define SOS_BOARD_QSPI_IO2_PIN 13
#define SOS_BOARD_QSPI_IO3_PORT 4 //PE2
#define SOS_BOARD_QSPI_IO3_PIN 2

#define SOS_BOARD_FMC_RAM_N_CE_PORT 3 //PD7
#define SOS_BOARD_FMC_RAM_N_CE_PIN 7
#define SOS_BOARD_FMC_RAM_N_WE_PORT 3 //PD5
#define SOS_BOARD_FMC_RAM_N_WE_PIN 5
#define SOS_BOARD_FMC_RAM_N_OE_PORT 3 //PD4
#define SOS_BOARD_FMC_RAM_N_OE_PIN 4
#define SOS_BOARD_FMC_RAM_N_BL0_PORT 4 //PE0
#define SOS_BOARD_FMC_RAM_N_BL0_PIN 0
#define SOS_BOARD_FMC_RAM_N_BL1_PORT 4 //PE1
#define SOS_BOARD_FMC_RAM_N_BL1_PIN 1
#define SOS_BOARD_FMC_RAM_A0_PORT 5 //PF0
#define SOS_BOARD_FMC_RAM_A0_PIN 0
#define SOS_BOARD_FMC_RAM_A1_PORT 5 //PF1
#define SOS_BOARD_FMC_RAM_A1_PIN 1
#define SOS_BOARD_FMC_RAM_A2_PORT 5 //PF2
#define SOS_BOARD_FMC_RAM_A2_PIN 2
#define SOS_BOARD_FMC_RAM_A3_PORT 5 //PF3
#define SOS_BOARD_FMC_RAM_A3_PIN 3
#define SOS_BOARD_FMC_RAM_A4_PORT 5 //PF4
#define SOS_BOARD_FMC_RAM_A4_PIN 4
#define SOS_BOARD_FMC_RAM_A5_PORT 5 //PF5
#define SOS_BOARD_FMC_RAM_A5_PIN 5
#define SOS_BOARD_FMC_RAM_A6_PORT 5 //PF12
#define SOS_BOARD_FMC_RAM_A6_PIN 12
#define SOS_BOARD_FMC_RAM_A7_PORT 5 //PF13
#define SOS_BOARD_FMC_RAM_A7_PIN 13
#define SOS_BOARD_FMC_RAM_A8_PORT 5 //PF14
#define SOS_BOARD_FMC_RAM_A8_PIN 14
#define SOS_BOARD_FMC_RAM_A9_PORT 5 //PF15
#define SOS_BOARD_FMC_RAM_A9_PIN 15
#define SOS_BOARD_FMC_RAM_A10_PORT 6 //PG0
#define SOS_BOARD_FMC_RAM_A10_PIN 0
#define SOS_BOARD_FMC_RAM_A11_PORT 6 //PG1
#define SOS_BOARD_FMC_RAM_A11_PIN 1
#define SOS_BOARD_FMC_RAM_A12_PORT 6 //PG2
#define SOS_BOARD_FMC_RAM_A12_PIN 2
#define SOS_BOARD_FMC_RAM_A13_PORT 6 //PG3
#define SOS_BOARD_FMC_RAM_A13_PIN 3
#define SOS_BOARD_FMC_RAM_A14_PORT 6 //PG4
#define SOS_BOARD_FMC_RAM_A14_PIN 4
#define SOS_BOARD_FMC_RAM_A15_PORT 6 //PG5
#define SOS_BOARD_FMC_RAM_A15_PIN 5
#define SOS_BOARD_FMC_RAM_A16_PORT 3 //PD11
#define SOS_BOARD_FMC_RAM_A16_PIN 11
#define SOS_BOARD_FMC_RAM_A17_PORT 3 //PD12
#define SOS_BOARD_FMC_RAM_A17_PIN 12
#define SOS_BOARD_FMC_RAM_D0_PORT 3 //PD14
#define SOS_BOARD_FMC_RAM_D0_PIN 14
#define SOS_BOARD_FMC_RAM_D1_PORT 3 //PD15
#define SOS_BOARD_FMC_RAM_D1_PIN 15
#define SOS_BOARD_FMC_RAM_D2_PORT 3 //PD0
#define SOS_BOARD_FMC_RAM_D2_PIN 0
#define SOS_BOARD_FMC_RAM_D3_PORT 3 //PD1
#define SOS_BOARD_FMC_RAM_D3_PIN 1
#define SOS_BOARD_FMC_RAM_D4_PORT 4 //PE7
#define SOS_BOARD_FMC_RAM_D4_PIN 7
#define SOS_BOARD_FMC_RAM_D5_PORT 4 //PE8
#define SOS_BOARD_FMC_RAM_D5_PIN 8
#define SOS_BOARD_FMC_RAM_D6_PORT 4 //PE9
#define SOS_BOARD_FMC_RAM_D6_PIN 9
#define SOS_BOARD_FMC_RAM_D7_PORT 4 //PE10
#define SOS_BOARD_FMC_RAM_D7_PIN 10
#define SOS_BOARD_FMC_RAM_D8_PORT 4 //PE11
#define SOS_BOARD_FMC_RAM_D8_PIN 11
#define SOS_BOARD_FMC_RAM_D9_PORT 4 //PE12
#define SOS_BOARD_FMC_RAM_D9_PIN 12
#define SOS_BOARD_FMC_RAM_D10_PORT 4 //PE13
#define SOS_BOARD_FMC_RAM_D10_PIN 13
#define SOS_BOARD_FMC_RAM_D11_PORT 4 //PE14
#define SOS_BOARD_FMC_RAM_D11_PIN 14
#define SOS_BOARD_FMC_RAM_D12_PORT 4 //PE15
#define SOS_BOARD_FMC_RAM_D12_PIN 15
#define SOS_BOARD_FMC_RAM_D13_PORT 3 //PD8
#define SOS_BOARD_FMC_RAM_D13_PIN 8
#define SOS_BOARD_FMC_RAM_D14_PORT 3 //PD9
#define SOS_BOARD_FMC_RAM_D14_PIN 9
#define SOS_BOARD_FMC_RAM_D15_PORT 3 //PD10
#define SOS_BOARD_FMC_RAM_D15_PIN 10

#define SOS_BOARD_FMC_LCD_N_CE_PORT 6 //PG9
#define SOS_BOARD_FMC_LCD_N_CE_PIN 9
#define SOS_BOARD_LCD_BL_PORT 7 //PH11
#define SOS_BOARD_LCD_BL_PIN 11
#define SOS_BOARD_LCD_TE_INT_PORT 2 //PC8
#define SOS_BOARD_LCD_TE_INT_PIN 8
#define SOS_BOARD_LCD_RST_PORT 7 //PH7
#define SOS_BOARD_LCD_RST_PIN 7
#define SOS_BOARD_LCD_RS_A0_PORT 5 //PF0
#define SOS_BOARD_LCD_RS_A0_PIN 0
/*LCD touch pin configuration*/
#define  SOS_BOARD_LCD_TOUCH_I2C3_SDA_PORT 7 //PH8
#define  SOS_BOARD_LCD_TOUCH_I2C3_SDA_PIN 8
#define  SOS_BOARD_LCD_TOUCH_I2C3_SCL_PORT 0 //PA8
#define  SOS_BOARD_LCD_TOUCH_I2C3_SCL_PIN 8
#define  SOS_BOARD_LCD_TOUCH_INT_PORT 8 //PI9
#define  SOS_BOARD_LCD_TOUCH_INT_PIN 9
#define  SOS_BOARD_LCD_TOUCH_RST_PORT 7 //PH9
#define  SOS_BOARD_LCD_TOUCH_RST_PIN 9

/*SAI A interface*/
#define SOS_BOARD_SAI2_MCLK_A_PORT 8 //PI4
#define SOS_BOARD_SAI2_MCLK_A_PIN 4
#define SOS_BOARD_SAI2_SCK_A_PORT 8 //PI5
#define SOS_BOARD_SAI2_SCK_A_PIN 5
#define SOS_BOARD_SAI2_SD_A_PORT 8 //PI6
#define SOS_BOARD_SAI2_SD_A_PIN 6
#define SOS_BOARD_SAI2_FS_A_PORT 8 //PI7
#define SOS_BOARD_SAI2_FS_A_PIN 7
/*SAI B interface*/
#define SOS_BOARD_SAI2_SD_B_PORT 6 //PG10
#define SOS_BOARD_SAI2_SD_B_PIN 10


#define SOS_BOARD_SPI1_RX_DMA STM32_DMA2
#define SOS_BOARD_SPI1_RX_DMA_STREAM 0
#define SOS_BOARD_SPI1_RX_DMA_CHANNEL 3
#define SOS_BOARD_SPI1_TX_DMA STM32_DMA2
#define SOS_BOARD_SPI1_TX_DMA_STREAM 3
#define SOS_BOARD_SPI1_TX_DMA_CHANNEL 3

#define SOS_BOARD_SPI3_RX_DMA STM32_DMA1
#define SOS_BOARD_SPI3_RX_DMA_STREAM 0
#define SOS_BOARD_SPI3_RX_DMA_CHANNEL 0
#define SOS_BOARD_SPI3_TX_DMA STM32_DMA1
#define SOS_BOARD_SPI3_TX_DMA_STREAM 5
#define SOS_BOARD_SPI3_TX_DMA_CHANNEL 0

#define SOS_BOARD_SD_DATA0_PORT 2 //PC8
#define SOS_BOARD_SD_DATA0_PIN 8
#define SOS_BOARD_SD_DATA1_PORT 2 //PC9
#define SOS_BOARD_SD_DATA1_PIN 9
#define SOS_BOARD_SD_DATA2_PORT 2 //PC10
#define SOS_BOARD_SD_DATA2_PIN 10
#define SOS_BOARD_SD_DATA3_PORT 2 //PC11
#define SOS_BOARD_SD_DATA3_PIN 11
#define SOS_BOARD_SD_CLK_PORT 2 //PC12
#define SOS_BOARD_SD_CLK_PIN 12
#define SOS_BOARD_SD_CMD_PORT 3 //PD2
#define SOS_BOARD_SD_CMD_PIN 2

#endif /* CONFIG_H_ */
