
#include <mcu/arch.h>
#include <mcu/arch/stm32/stm32f7xx/stm32f7xx_hal.h>


void SystemClock_Config(){

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
	HAL_RCCEx_GetPeriphCLKConfig(&PeriphClkInitStruct);
	/**Configure the main internal regulator output voltage
		 */
	__HAL_RCC_PWR_CLK_ENABLE();
	
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	
	/**Initializes the CPU, AHB and APB busses clocks
		 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 432;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 9;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);
	
	/**Activate the Over-Drive mode
		 */
	HAL_PWREx_EnableOverDrive();
	
	/**Initializes the CPU, AHB and APB busses clocks
		 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
	
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART6|RCC_PERIPHCLK_CLK48|RCC_PERIPHCLK_SAI2;
	PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
	PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S;
	PeriphClkInitStruct.PLLI2S.PLLI2SN = 344;
	PeriphClkInitStruct.PLLI2S.PLLI2SQ = 7;
	PeriphClkInitStruct.PLLI2SDivQ = 1;

	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
}
