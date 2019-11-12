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


#include <sys/lock.h>
#include <fcntl.h>
#include <errno.h>
#include <mcu/mcu.h>
#include <mcu/debug.h>
#include <mcu/periph.h>
#include <device/sys.h>
#include <device/uartfifo.h>
#include <device/usbfifo.h>
#include <device/fifo.h>
#include <device/cfifo.h>
#include <device/sys.h>
#include <sos/link.h>
#include <sos/fs/sysfs.h>
#include <sos/fs/appfs.h>
#include <sos/fs/devfs.h>
#include <sos/fs/sffs.h>
#include <mcu/appfs.h>
#include <sos/sos.h>
#include <device/drive_cfi.h>

#include "config.h"
#include "sl_config.h"
#include "link_config.h"
#include "stm32f723e_discovery_psram.h"
#include "display_device.h"

#if !defined SOS_BOARD_FLAGS
#define SOS_BOARD_FLAGS 0
#endif

//--------------------------------------------Stratify OS Configuration-------------------------------------------------
const sos_board_config_t sos_board_config = {
	.clk_usecond_tmr = SOS_BOARD_TMR, //TIM2 -- 32 bit timer
	.task_total = SOS_BOARD_TASK_TOTAL,
	.stdin_dev = "/dev/stdio-in" ,
	.stdout_dev = "/dev/stdio-out",
	.stderr_dev = "/dev/stdio-out",
	.o_sys_flags = SYS_FLAG_IS_STDIO_FIFO | SYS_FLAG_IS_TRACE | SOS_BOARD_FLAGS,
	.sys_name = SL_CONFIG_NAME,
	.sys_version = SL_CONFIG_VERSION_STRING,
	.sys_id = SL_CONFIG_DOCUMENT_ID,
	.sys_memory_size = SOS_BOARD_SYSTEM_MEMORY_SIZE,
	.start = sos_default_thread,
	.start_args = &link_transport,
	.start_stack_size = SOS_DEFAULT_START_STACK_SIZE,
	.socket_api = 0,
	.request = 0,
	.trace_dev = "/dev/trace",
	.trace_event = SOS_BOARD_TRACE_EVENT,
	.git_hash = SOS_GIT_HASH
};

//This declares the task tables required by Stratify OS for applications and threads
SOS_DECLARE_TASK_TABLE(SOS_BOARD_TASK_TOTAL);

//--------------------------------------------Device Filesystem-------------------------------------------------


/*
 * Defaults configurations
 *
 * This provides the default pin assignments and settings for peripherals. If
 * the defaults are not provided, the application must specify them.
 *
 * Defaults should be added for peripherals that are dedicated for use on the
 * board. For example, if a UART has an external connection and label on the
 * board, the BSP should provide the default configuration.
 *
 *
 *
 */
//USART2
UARTFIFO_DECLARE_CONFIG_STATE(uart1_fifo, 1024, 1,
										UART_FLAG_SET_LINE_CODING_DEFAULT, 8, 115200,
										SOS_BOARD_USART2_RX_PORT, SOS_BOARD_USART2_RX_PIN, //RX
										SOS_BOARD_USART2_TX_PORT, SOS_BOARD_USART2_TX_PIN, //TX
										0xff, 0xff,
										0xff, 0xff);
#if !defined __debug
//USART3
UARTFIFO_DECLARE_CONFIG_STATE(uart2_fifo, 1024, 1,
										UART_FLAG_SET_LINE_CODING_DEFAULT, 8, 115200,
										SOS_BOARD_USART3_RX_PORT, SOS_BOARD_USART3_RX_PIN, //RX
										SOS_BOARD_USART3_TX_PORT, SOS_BOARD_USART3_TX_PIN, //TX
										0xff, 0xff,
										0xff, 0xff);
#endif

//USART6
UARTFIFO_DECLARE_CONFIG_STATE(uart5_fifo, 1024, 1,
										UART_FLAG_SET_LINE_CODING_DEFAULT, 8, 115200,
										SOS_BOARD_USART6_RX_PORT, SOS_BOARD_USART6_RX_PIN, //RX
										SOS_BOARD_USART6_TX_PORT, SOS_BOARD_USART6_TX_PIN, //TX
										0xff, 0xff,
										0xff, 0xff);

//I2C1 audio wm8994 0011 0100
I2C_DECLARE_CONFIG_MASTER(i2c0,
								  I2C_FLAG_SET_MASTER,
								  100000,
								  SOS_BOARD_I2C1_SDA_PORT, SOS_BOARD_I2C1_SDA_PIN, //SDA
								  SOS_BOARD_I2C1_SCL_PORT, SOS_BOARD_I2C1_SCL_PIN); //SCL

//I2C2
I2C_DECLARE_CONFIG_MASTER(i2c1,
								  I2C_FLAG_SET_MASTER,
								  100000,
								  SOS_BOARD_I2C2_SDA_PORT, SOS_BOARD_I2C2_SDA_PIN, //SDA
								  SOS_BOARD_I2C2_SCL_PORT, SOS_BOARD_I2C2_SCL_PIN); //SCL
//i2c3 for lcd touch
I2C_DECLARE_CONFIG_MASTER(i2c2,
								  I2C_FLAG_SET_MASTER,
								  100000,
								  SOS_BOARD_LCD_TOUCH_I2C3_SDA_PORT, SOS_BOARD_LCD_TOUCH_I2C3_SDA_PIN, //SDA
								  SOS_BOARD_LCD_TOUCH_I2C3_SCL_PORT, SOS_BOARD_LCD_TOUCH_I2C3_SDA_PIN); //SCL

//qspi
#define QPI_READ_4_BYTE_ADDR_CMD             0xEC
#define QUAD_OUT_FAST_READ_CMD               0x6B
#define QPI_PAGE_PROG_4_BYTE_ADDR_CMD        0x12
#define QSPI_DUMMY_CYCLES_READ_QUAD_IO       10



#define SPI_DMA_FLAGS STM32_DMA_FLAG_IS_NORMAL | \
	STM32_DMA_FLAG_IS_MEMORY_SINGLE | \
	STM32_DMA_FLAG_IS_PERIPH_SINGLE | \
	STM32_DMA_FLAG_IS_PERIPH_BYTE | \
	STM32_DMA_FLAG_IS_MEMORY_BYTE

const stm32_spi_dma_config_t spi0_dma_config = {
	.spi_config = {
		.attr = {
			.o_flags = SPI_FLAG_SET_MASTER | SPI_FLAG_IS_FORMAT_SPI | SPI_FLAG_IS_MODE0 | SPI_FLAG_IS_FULL_DUPLEX,
			.width = 8,
			.freq = 1000000UL,
			.pin_assignment = {
				.miso = {SOS_BOARD_SPI1_MISO_PORT, SOS_BOARD_SPI1_MISO_PIN},
				.mosi = {SOS_BOARD_SPI1_MOSI_PORT, SOS_BOARD_SPI1_MOSI_PIN},
				.sck = {SOS_BOARD_SPI1_SCK_PORT, SOS_BOARD_SPI1_SCK_PIN},
				.cs = {SOS_BOARD_SPI1_CS_PORT, SOS_BOARD_SPI1_CS_PIN}
			}
		}
	},
	.dma_config = {
		.rx = {
			.dma_number = SOS_BOARD_SPI1_RX_DMA,
			.stream_number = SOS_BOARD_SPI1_RX_DMA_STREAM,
			.channel_number = SOS_BOARD_SPI1_RX_DMA_CHANNEL,
			.priority = STM32_DMA_PRIORITY_LOW,
			.o_flags = STM32_DMA_FLAG_IS_PERIPH_TO_MEMORY |
			SPI_DMA_FLAGS,
		},
		.tx = {
			.dma_number = SOS_BOARD_SPI1_TX_DMA,
			.stream_number = SOS_BOARD_SPI1_TX_DMA_STREAM,
			.channel_number = SOS_BOARD_SPI1_TX_DMA_CHANNEL,
			.priority = STM32_DMA_PRIORITY_LOW,
			.o_flags = STM32_DMA_FLAG_IS_MEMORY_TO_PERIPH |
			SPI_DMA_FLAGS
		}
	}
};

const stm32_spi_dma_config_t spi2_dma_config = {
	.spi_config = {
		.attr = {
			.o_flags = SPI_FLAG_SET_MASTER | SPI_FLAG_IS_FORMAT_SPI | SPI_FLAG_IS_MODE0 | SPI_FLAG_IS_FULL_DUPLEX,
			.width = 8,
			.freq = 1000000UL,
			.pin_assignment = {
				.miso = {SOS_BOARD_SPI3_MISO_PORT, SOS_BOARD_SPI3_MISO_PIN},
				.mosi = {SOS_BOARD_SPI3_MOSI_PORT, SOS_BOARD_SPI3_MOSI_PIN},
				.sck = {SOS_BOARD_SPI3_SCK_PORT, SOS_BOARD_SPI3_SCK_PIN},
				.cs = {SOS_BOARD_SPI3_CS_PORT, SOS_BOARD_SPI3_CS_PIN}
			}
		}
	},
	.dma_config = {
		.rx = {
			.dma_number = SOS_BOARD_SPI3_RX_DMA,
			.stream_number = SOS_BOARD_SPI3_RX_DMA_STREAM,
			.channel_number = SOS_BOARD_SPI3_RX_DMA_CHANNEL,
			.priority = STM32_DMA_PRIORITY_LOW,
			.o_flags = STM32_DMA_FLAG_IS_PERIPH_TO_MEMORY |
			SPI_DMA_FLAGS,
		},
		.tx = {
			.dma_number = SOS_BOARD_SPI3_TX_DMA,
			.stream_number = SOS_BOARD_SPI3_TX_DMA_STREAM,
			.channel_number = SOS_BOARD_SPI3_TX_DMA_CHANNEL,
			.priority = STM32_DMA_PRIORITY_LOW,
			.o_flags = STM32_DMA_FLAG_IS_MEMORY_TO_PERIPH |
			SPI_DMA_FLAGS
		}
	}
};

/*fmc psram config*/
const emc_config_t fmc_psram0_config = {
	.attr = {
		.o_flags = EMC_FLAG_IS_PSRAM | EMC_FLAG_ENABLE | EMC_FLAG_IS_PSRAM_BANK1,
		.base_address = 0x60000000,
		.size = 0x100000,
		.freq = 0,
		.data_bus_width = 16,
		.pin_assignment = {
			.we = {SOS_BOARD_FMC_RAM_N_WE_PORT,SOS_BOARD_FMC_RAM_N_WE_PIN},
			.oe = {SOS_BOARD_FMC_RAM_N_OE_PORT,SOS_BOARD_FMC_RAM_N_OE_PIN},
			.bl = {{SOS_BOARD_FMC_RAM_N_BL0_PORT,SOS_BOARD_FMC_RAM_N_BL0_PIN},{SOS_BOARD_FMC_RAM_N_BL1_PORT,SOS_BOARD_FMC_RAM_N_BL1_PIN},{0xFF,0xFF},{0xFF,0xFF}},
			.nadv = {0xFF,0xFF},
			.nwait = {0xFF,0xFF},
			.ncs = {{SOS_BOARD_FMC_RAM_N_CE_PORT,SOS_BOARD_FMC_RAM_N_CE_PIN},{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF}},
			.address = {
				{SOS_BOARD_FMC_RAM_A0_PORT,SOS_BOARD_FMC_RAM_A0_PIN},
				{SOS_BOARD_FMC_RAM_A1_PORT,SOS_BOARD_FMC_RAM_A1_PIN},
				{SOS_BOARD_FMC_RAM_A2_PORT,SOS_BOARD_FMC_RAM_A2_PIN},
				{SOS_BOARD_FMC_RAM_A3_PORT,SOS_BOARD_FMC_RAM_A3_PIN},
				{SOS_BOARD_FMC_RAM_A4_PORT,SOS_BOARD_FMC_RAM_A4_PIN},
				{SOS_BOARD_FMC_RAM_A5_PORT,SOS_BOARD_FMC_RAM_A5_PIN},
				{SOS_BOARD_FMC_RAM_A6_PORT,SOS_BOARD_FMC_RAM_A6_PIN},
				{SOS_BOARD_FMC_RAM_A7_PORT,SOS_BOARD_FMC_RAM_A7_PIN},
				{SOS_BOARD_FMC_RAM_A8_PORT,SOS_BOARD_FMC_RAM_A8_PIN},
				{SOS_BOARD_FMC_RAM_A9_PORT,SOS_BOARD_FMC_RAM_A9_PIN},
				{SOS_BOARD_FMC_RAM_A10_PORT,SOS_BOARD_FMC_RAM_A10_PIN},
				{SOS_BOARD_FMC_RAM_A11_PORT,SOS_BOARD_FMC_RAM_A11_PIN},
				{SOS_BOARD_FMC_RAM_A12_PORT,SOS_BOARD_FMC_RAM_A12_PIN},
				{SOS_BOARD_FMC_RAM_A13_PORT,SOS_BOARD_FMC_RAM_A13_PIN},
				{SOS_BOARD_FMC_RAM_A14_PORT,SOS_BOARD_FMC_RAM_A14_PIN},
				{SOS_BOARD_FMC_RAM_A15_PORT,SOS_BOARD_FMC_RAM_A15_PIN},
				{SOS_BOARD_FMC_RAM_A16_PORT,SOS_BOARD_FMC_RAM_A16_PIN},
				{SOS_BOARD_FMC_RAM_A17_PORT,SOS_BOARD_FMC_RAM_A17_PIN},
				{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},
				{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF}
			},
			.data = {
				{SOS_BOARD_FMC_RAM_D0_PORT,SOS_BOARD_FMC_RAM_D0_PIN},
				{SOS_BOARD_FMC_RAM_D1_PORT,SOS_BOARD_FMC_RAM_D1_PIN},
				{SOS_BOARD_FMC_RAM_D2_PORT,SOS_BOARD_FMC_RAM_D2_PIN},
				{SOS_BOARD_FMC_RAM_D3_PORT,SOS_BOARD_FMC_RAM_D3_PIN},
				{SOS_BOARD_FMC_RAM_D4_PORT,SOS_BOARD_FMC_RAM_D4_PIN},
				{SOS_BOARD_FMC_RAM_D5_PORT,SOS_BOARD_FMC_RAM_D5_PIN},
				{SOS_BOARD_FMC_RAM_D6_PORT,SOS_BOARD_FMC_RAM_D6_PIN},
				{SOS_BOARD_FMC_RAM_D7_PORT,SOS_BOARD_FMC_RAM_D7_PIN},
				{SOS_BOARD_FMC_RAM_D8_PORT,SOS_BOARD_FMC_RAM_D8_PIN},
				{SOS_BOARD_FMC_RAM_D9_PORT,SOS_BOARD_FMC_RAM_D9_PIN},
				{SOS_BOARD_FMC_RAM_D10_PORT,SOS_BOARD_FMC_RAM_D10_PIN},
				{SOS_BOARD_FMC_RAM_D11_PORT,SOS_BOARD_FMC_RAM_D11_PIN},
				{SOS_BOARD_FMC_RAM_D12_PORT,SOS_BOARD_FMC_RAM_D12_PIN},
				{SOS_BOARD_FMC_RAM_D13_PORT,SOS_BOARD_FMC_RAM_D13_PIN},
				{SOS_BOARD_FMC_RAM_D14_PORT,SOS_BOARD_FMC_RAM_D14_PIN},
				{SOS_BOARD_FMC_RAM_D15_PORT,SOS_BOARD_FMC_RAM_D15_PIN},
				{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},
				{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},
				{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},
				{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF}
			},
		}
	}
};
/*fmc psram config*/
const emc_config_t lcd0_config = {
	.attr = {
		.o_flags = EMC_FLAG_IS_AHB | EMC_FLAG_ENABLE | EMC_FLAG_IS_PSRAM_BANK2,
		.base_address = 0x64000000,
		.size = 0x100000,
		.freq = 0,
		.data_bus_width = 16,
		.pin_assignment = {
			.we = {SOS_BOARD_FMC_RAM_N_WE_PORT,SOS_BOARD_FMC_RAM_N_WE_PIN},
			.oe = {SOS_BOARD_FMC_RAM_N_OE_PORT,SOS_BOARD_FMC_RAM_N_OE_PIN},
			.bl = {{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF}},
			.nadv = {0xFF,0xFF},
			.nwait = {0xFF,0xFF},
			.ncs = {{SOS_BOARD_FMC_LCD_N_CE_PORT,SOS_BOARD_FMC_LCD_N_CE_PIN},{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF}},
			.address = {{SOS_BOARD_LCD_RS_A0_PORT,SOS_BOARD_LCD_RS_A0_PIN},{0xFF,0xFF},
							{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},
							{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},
							{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},
							{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},
							{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},
							{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},
			},
			.data = {
				{SOS_BOARD_FMC_RAM_D0_PORT,SOS_BOARD_FMC_RAM_D0_PIN},
				{SOS_BOARD_FMC_RAM_D1_PORT,SOS_BOARD_FMC_RAM_D1_PIN},
				{SOS_BOARD_FMC_RAM_D2_PORT,SOS_BOARD_FMC_RAM_D2_PIN},
				{SOS_BOARD_FMC_RAM_D3_PORT,SOS_BOARD_FMC_RAM_D3_PIN},
				{SOS_BOARD_FMC_RAM_D4_PORT,SOS_BOARD_FMC_RAM_D4_PIN},
				{SOS_BOARD_FMC_RAM_D5_PORT,SOS_BOARD_FMC_RAM_D5_PIN},
				{SOS_BOARD_FMC_RAM_D6_PORT,SOS_BOARD_FMC_RAM_D6_PIN},
				{SOS_BOARD_FMC_RAM_D7_PORT,SOS_BOARD_FMC_RAM_D7_PIN},
				{SOS_BOARD_FMC_RAM_D8_PORT,SOS_BOARD_FMC_RAM_D8_PIN},
				{SOS_BOARD_FMC_RAM_D9_PORT,SOS_BOARD_FMC_RAM_D9_PIN},
				{SOS_BOARD_FMC_RAM_D10_PORT,SOS_BOARD_FMC_RAM_D10_PIN},
				{SOS_BOARD_FMC_RAM_D11_PORT,SOS_BOARD_FMC_RAM_D11_PIN},
				{SOS_BOARD_FMC_RAM_D12_PORT,SOS_BOARD_FMC_RAM_D12_PIN},
				{SOS_BOARD_FMC_RAM_D13_PORT,SOS_BOARD_FMC_RAM_D13_PIN},
				{SOS_BOARD_FMC_RAM_D14_PORT,SOS_BOARD_FMC_RAM_D14_PIN},
				{SOS_BOARD_FMC_RAM_D15_PORT,SOS_BOARD_FMC_RAM_D15_PIN},
				{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},
				{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},
				{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},
				{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF},{0xFF,0xFF}
			},
		}
	}
};

SAI_DMA_DECLARE_CONFIG(sai2 ,
							  I2S_FLAG_SET_MASTER|I2S_FLAG_IS_TRANSMITTER|SAI_FLAG_IS_OUTPUTDRIVE_DISABLE|\
							  SAI_FLAG_IS_FIFOTHRESHOLD_1QF|SAI_FLAG_ENABLE|SAI_DMA_ENABLE,
							  16000,
							  1,0x0f,  //mclk mult, slot
							  SOS_BOARD_SAI2_SCK_A_PORT, SOS_BOARD_SAI2_SCK_A_PIN,
							  SOS_BOARD_SAI2_SD_A_PORT, SOS_BOARD_SAI2_SD_A_PIN,
							  SOS_BOARD_SAI2_MCLK_A_PORT , SOS_BOARD_SAI2_MCLK_A_PIN,
							  SOS_BOARD_SAI2_FS_A_PORT, SOS_BOARD_SAI2_FS_A_PIN,
							  1,4,
							  3,STM32_DMA_PRIORITY_HIGH,
							  STM32_DMA_FLAG_IS_FIFO_THRESHOLD_FULL|STM32_DMA_FLAG_IS_FIFO|\
							  STM32_DMA_FLAG_IS_MEMORY_HALFWORD|STM32_DMA_FLAG_IS_PERIPH_HALFWORD|
							  STM32_DMA_FLAG_IS_CIRCULAR|STM32_DMA_FLAG_IS_MEMORY_TO_PERIPH) ;
SAI_DMA_DECLARE_CONFIG(sai3 ,
							  I2S_FLAG_SET_SLAVE|I2S_FLAG_IS_RECEIVER|SAI_FLAG_IS_OUTPUTDRIVE_DISABLE|\
							  SAI_FLAG_IS_FIFOTHRESHOLD_1QF|SAI_FLAG_ENABLE|SAI_DMA_ENABLE,
							  16000,
							  1,0x0f,//mclk mult, slot
							  0xff, 0xff,
							  SOS_BOARD_SAI2_SD_B_PORT, SOS_BOARD_SAI2_SD_B_PIN,
							  0xff, 0xff,
							  0xff, 0xff,
							  1,6,
							  3,STM32_DMA_PRIORITY_HIGH,
							  STM32_DMA_FLAG_IS_FIFO_THRESHOLD_FULL|\
							  STM32_DMA_FLAG_IS_MEMORY_HALFWORD|STM32_DMA_FLAG_IS_PERIPH_HALFWORD|
							  STM32_DMA_FLAG_IS_CIRCULAR|STM32_DMA_FLAG_IS_PERIPH_TO_MEMORY) ;

//Coming Soon
//SD Card as DMA
//I2S2
//I2S3
//SAI1A/B

const stm32_qspi_dma_config_t qspi_dma_config = {
	.qspi_config = {
		.attr = {
			.o_flags = QSPI_FLAG_SET_MASTER,
			.freq = 60000000UL,
			.pin_assignment = {
				.data[0] = {SOS_BOARD_QSPI_IO0_PORT,SOS_BOARD_QSPI_IO0_PIN},
				.data[1] = {SOS_BOARD_QSPI_IO1_PORT,SOS_BOARD_QSPI_IO1_PIN},
				.data[2] = {SOS_BOARD_QSPI_IO2_PORT,SOS_BOARD_QSPI_IO2_PIN},
				.data[3] = {SOS_BOARD_QSPI_IO3_PORT,SOS_BOARD_QSPI_IO3_PIN},
				.sck = {SOS_BOARD_QSPI_CLK_PORT,SOS_BOARD_QSPI_CLK_PIN},
				.cs = {SOS_BOARD_QSPI_NCS_PORT,SOS_BOARD_QSPI_NCS_PIN}
			}
		}
	},
	.dma_config = {
		.tx = {
			.dma_number = STM32_DMA2,
			.stream_number = 7,
			.channel_number = 3,
			.priority = STM32_DMA_PRIORITY_LOW,
			.o_flags = STM32_DMA_FLAG_IS_NORMAL |
			STM32_DMA_FLAG_IS_FIFO |
			STM32_DMA_FLAG_IS_MEMORY_TO_PERIPH |
			STM32_DMA_FLAG_IS_MEMORY_BYTE |
			STM32_DMA_FLAG_IS_PERIPH_BYTE
		},
		.rx = {
			.dma_number = STM32_DMA2,
			.stream_number = 7,
			.channel_number = 3,
			.priority = STM32_DMA_PRIORITY_LOW,
			.o_flags = STM32_DMA_FLAG_IS_NORMAL |
			STM32_DMA_FLAG_IS_FIFO |
			STM32_DMA_FLAG_IS_PERIPH_TO_MEMORY |
			STM32_DMA_FLAG_IS_MEMORY_BYTE |
			STM32_DMA_FLAG_IS_PERIPH_BYTE
		}
	}
};

const devfs_device_t qspi_drive_device = DEVFS_DEVICE("qspi", mcu_qspi_dma, 0, &qspi_dma_config, 0, 0666, SOS_USER_ROOT, S_IFCHR);

const drive_cfi_config_t drive_cfi_config = {
	.serial_device = &qspi_drive_device,
	.info = {
		.addressable_size = 1,
		.write_block_size = 1, //smallest available write size
		.num_write_blocks = 64*1024*1024UL,  //64MB (512Mbit)
		.erase_block_size = 4096, //smallest eraseable block
		.erase_block_time = 30000UL, //45ms typical
		.erase_device_time = 140000000UL, //140s typical
		.bitrate = 66000000UL
	},
	.opcode = {
		.write_enable = 0x06,
		.page_program = 0x02,
		.block_erase = 0x20,
		.device_erase = 0xC7,
		.fast_read = 0xEB,
		.power_up = 0xAB,
		.power_down = 0xB9,
		.enable_reset = 0x66,
		.reset = 0x99,
		.protect = 0x7E,
		.unprotect = 0x7A,
		.read_busy_status = 0x05, //busy bit is bit 0 of status register 1
		.busy_status_mask = 0x01,
		.enter_qpi_mode = 0x35,
		.enter_4byte_address_mode = 0xb7,
		.page_program_size = 256,
		.read_dummy_cycles = 6,
		.write_dummy_cycles = 0
	},
	.cs = { 0xff, 0xff },
	.qspi_flags =
	QSPI_FLAG_IS_OPCODE_QUAD |
	QSPI_FLAG_IS_DATA_QUAD |
	QSPI_FLAG_IS_ADDRESS_QUAD |
	QSPI_FLAG_IS_ADDRESS_32_BITS
};

drive_cfi_state_t drive_cfi_state MCU_SYS_MEM;


FIFO_DECLARE_CONFIG_STATE(stdio_in, SOS_BOARD_STDIO_BUFFER_SIZE);
FIFO_DECLARE_CONFIG_STATE(stdio_out, SOS_BOARD_STDIO_BUFFER_SIZE);
CFIFO_DECLARE_CONFIG_STATE_4(board_fifo, 256);

#if !defined SOS_BOARD_USB_PORT
#define SOS_BOARD_USB_PORT 0
#endif
/* This is the list of devices that will show up in the /dev folder.
 */
const devfs_device_t devfs_list[] = {
	//System devices
	DEVFS_DEVICE("trace", ffifo, 0, &board_trace_config, &board_trace_state, 0666, SOS_USER_ROOT, S_IFCHR),
	DEVFS_DEVICE("fifo", cfifo, 0, &board_fifo_config, &board_fifo_state, 0666, SOS_USER_ROOT, S_IFCHR),
	DEVFS_DEVICE("stdio-out", fifo, 0, &stdio_out_config, &stdio_out_state, 0666, SOS_USER_ROOT, S_IFCHR),
	DEVFS_DEVICE("stdio-in", fifo, 0, &stdio_in_config, &stdio_in_state, 0666, SOS_USER_ROOT, S_IFCHR),
	DEVFS_DEVICE("link-phy-usb", usbfifo, SOS_BOARD_USB_PORT, &sos_link_transport_usb_fifo_cfg, &sos_link_transport_usb_fifo_state, 0666, SOS_USER_ROOT, S_IFCHR),
	DEVFS_DEVICE("sys", sys, 0, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),
	//DEVFS_DEVICE("rtc", mcu_rtc, 0, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),

	DEVFS_DEVICE("drive0", drive_cfi_qspi, 0, &drive_cfi_config, &drive_cfi_state, 0666, SOS_USER_ROOT, S_IFBLK),
	DEVFS_DEVICE("display0", display_device, 0, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),

	//MCU peripherals
	DEVFS_DEVICE("core", mcu_core, 0, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),
	DEVFS_DEVICE("core0", mcu_core, 0, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),

	DEVFS_DEVICE("i2c0", mcu_i2c, 0, &i2c0_config, 0, 0666, SOS_USER_ROOT, S_IFCHR),    //WM8994ECS wolfson audio
	DEVFS_DEVICE("i2c1", mcu_i2c, 1, &i2c1_config, 0, 0666, SOS_USER_ROOT, S_IFCHR),
	DEVFS_DEVICE("i2c2", mcu_i2c, 2, &i2c2_config, 0, 0666, SOS_USER_ROOT, S_IFCHR),    // touch screen
	DEVFS_DEVICE("i2c3", mcu_i2c, 3, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),

	DEVFS_DEVICE("pio0", mcu_pio, 0, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR), //GPIOA
	DEVFS_DEVICE("pio1", mcu_pio, 1, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR), //GPIOB
	DEVFS_DEVICE("pio2", mcu_pio, 2, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR), //GPIOC
	DEVFS_DEVICE("pio3", mcu_pio, 3, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR), //GPIOD
	DEVFS_DEVICE("pio4", mcu_pio, 4, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR), //GPIOE
	DEVFS_DEVICE("pio5", mcu_pio, 5, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR), //GPIOF
	DEVFS_DEVICE("pio6", mcu_pio, 6, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR), //GPIOG
	DEVFS_DEVICE("pio7", mcu_pio, 7, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR), //GPIOH
	DEVFS_DEVICE("pio8", mcu_pio, 8, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR), //GPIOI

	DEVFS_DEVICE("spi0", mcu_spi, 0, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),
	DEVFS_DEVICE("spi1", mcu_spi, 1, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),
	DEVFS_DEVICE("spi2", mcu_spi, 2, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),
	DEVFS_DEVICE("spi3", mcu_spi, 3, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),
	//DEVFS_DEVICE("qspi0", mcu_qspi, 0, &qspi0_config, 0, 0666, SOS_USER_ROOT, S_IFCHR),
	//                device_name  periph_name    handle_port, handle_config,      handle_state, mode_value, uid_value,     device_type) {
	//DEVFS_DEVICE("fmc_psram0", mcu_emc_psram, 0,           &fmc_psram0_config, 0,            0666,       SOS_USER_ROOT, S_IFCHR),
	//DEVFS_DEVICE("lcd0", mcu_emc_fmc_ahb, 0,                 &lcd0_config      , 0,            0666,       SOS_USER_ROOT, S_IFCHR),
	DEVFS_DEVICE("i2s2", mcu_sai_dma, 2,                 &sai2_config      , 0,            0666,       SOS_USER_ROOT, S_IFCHR),
	DEVFS_DEVICE("i2s3", mcu_sai_dma, 3,                 &sai3_config      , 0,            0666,       SOS_USER_ROOT, S_IFCHR),
	DEVFS_DEVICE("tmr0", mcu_tmr, 0, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR), //TIM1
	DEVFS_DEVICE("tmr1", mcu_tmr, 1, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR), //TIM2
	DEVFS_DEVICE("tmr2", mcu_tmr, 2, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),
	DEVFS_DEVICE("tmr3", mcu_tmr, 3, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),
	DEVFS_DEVICE("tmr4", mcu_tmr, 4, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),
	DEVFS_DEVICE("tmr5", mcu_tmr, 5, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),
	DEVFS_DEVICE("tmr6", mcu_tmr, 6, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR),
	DEVFS_DEVICE("tmr7", mcu_tmr, 7, 0, 0, 0666, SOS_USER_ROOT, S_IFCHR), //TIM8
	//Does this chip have more timers?

	DEVFS_DEVICE("uart1", uartfifo, 0, &uart1_fifo_config, &uart1_fifo_state, 0666, SOS_USER_ROOT, S_IFCHR),
	#if !defined __debug
	DEVFS_DEVICE("uart2", uartfifo, 0, &uart2_fifo_config, &uart2_fifo_state, 0666, SOS_USER_ROOT, S_IFCHR),
	#endif
	DEVFS_DEVICE("uart5", uartfifo, 0, &uart5_fifo_config, &uart5_fifo_state, 0666, SOS_USER_ROOT, S_IFCHR),
	DEVFS_TERMINATOR
};


//--------------------------------------------Root Filesystem---------------------------------------------------


/*
 * This is the root filesystem that determines what is mounted at /.
 *
 * The default is /app (for installing and running applciations in RAM and flash) and /dev which
 * provides the device tree defined above.
 *
 * Additional filesystems (such as FatFs) can be added if the hardware and drivers
 * are provided by the board.
 *
 */

#define TCM_RAM_PAGE_COUNT 32 //the other 32 pages are used for sys_mem
#define INTERNAL_RAM_PAGE_COUNT 176
#define EXTERNAL_RAM_PAGE_COUNT (PSRAM_DEVICE_SIZE/MCU_RAM_PAGE_SIZE)
#define APPFS_RAM_PAGES (TCM_RAM_PAGE_COUNT + INTERNAL_RAM_PAGE_COUNT + EXTERNAL_RAM_PAGE_COUNT)

u32 ram_usage_table[APPFS_RAM_USAGE_WORDS(APPFS_RAM_PAGES)] MCU_SYS_MEM;
const devfs_device_t flash0 = DEVFS_DEVICE("flash0", mcu_flash, 0, 0, 0, 0666, SOS_USER_ROOT, S_IFBLK);


const appfs_mem_config_t appfs_mem_config = {
	.usage_size = sizeof(ram_usage_table),
	.usage = ram_usage_table,
	.system_ram_page = (u32)0, //system RAM is not located in the APPFS memory sections
	.flash_driver = &flash0,
	.section_count = 6,
	.sections = {
		{ .o_flags = MEM_FLAG_IS_FLASH, .page_count = 4, .page_size = 16*1024, .address = 0x08000000 },
		{ .o_flags = MEM_FLAG_IS_FLASH, .page_count = 1, .page_size = 64*1024, .address = 0x08000000 + 16*1024*4 },
		{ .o_flags = MEM_FLAG_IS_FLASH, .page_count = 3, .page_size = 128*1024, .address = 0x08000000 + 16*1024*4 + 64*1024*1 },
		{ .o_flags = MEM_FLAG_IS_RAM, .page_count = INTERNAL_RAM_PAGE_COUNT, .page_size = MCU_RAM_PAGE_SIZE, .address = 0x20010000 },
		{ .o_flags = MEM_FLAG_IS_RAM | MEM_FLAG_IS_TIGHTLY_COUPLED, .page_count = TCM_RAM_PAGE_COUNT, .page_size = MCU_RAM_PAGE_SIZE, .address = 0x20008000 },
		{ .o_flags = MEM_FLAG_IS_RAM | MEM_FLAG_IS_EXTERNAL, .page_count = EXTERNAL_RAM_PAGE_COUNT, .page_size = MCU_RAM_PAGE_SIZE, .address = PSRAM_DEVICE_ADDR }
	}
};

const devfs_device_t mem0 = DEVFS_DEVICE("mem0", appfs_mem, 0, &appfs_mem_config, 0, 0666, SOS_USER_ROOT, S_IFBLK);

sffs_state_t sffs_state;
const sffs_config_t sffs_configuration = {
	.drive = {
		.devfs = &(sysfs_list[1]),
		.name = "drive0",
		.state = (sysfs_shared_state_t*)&sffs_state
	}
};

const sysfs_t sysfs_list[] = {
	APPFS_MOUNT("/app", &mem0, 0777, SYSFS_ROOT), //the folder for ram/flash applications
	DEVFS_MOUNT("/dev", devfs_list, 0777, SYSFS_ROOT), //the list of devices
	SFFS_MOUNT("/home", &sffs_configuration, 0777, SYSFS_ROOT), //the stratify file system on external flash
	SYSFS_MOUNT("/", sysfs_list, 0777, SYSFS_ROOT), //the root filesystem (must be last)
	SYSFS_TERMINATOR
};


