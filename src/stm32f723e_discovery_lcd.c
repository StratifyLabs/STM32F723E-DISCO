/**
  ******************************************************************************
  * @file    stm32f723e_discovery_lcd.c
  * @author  MCD Application Team
  * @brief   This file includes the driver for Liquid Crystal Display (LCD) module
  *          mounted on STM32F723E-DISCOVERY board.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 


/* File Info : -----------------------------------------------------------------
                                   User NOTES
1. How To use this driver:
--------------------------
   - This driver is used to drive indirectly an LCD TFT.
   - This driver supports the ST7789H2 LCD.
   - The ST7789H2 component driver MUST be included with this driver.

2. Driver description:
---------------------
  + Initialization steps:
     o Initialize the LCD using the BSP_LCD_Init() function.
  
  + Display on LCD
     o Clear the hole LCD using BSP_LCD_Clear() function or only one specified string
       line using the BSP_LCD_ClearStringLine() function.
     o Display a character on the specified line and column using the BSP_LCD_DisplayChar()
       function or a complete string line using the BSP_LCD_DisplayStringAtLine() function.
     o Display a string line on the specified position (x,y in pixel) and align mode
       using the BSP_LCD_DisplayStringAtLine() function.          
     o Draw and fill a basic shapes (dot, line, rectangle, circle, ellipse, .. bitmap) 
       on LCD using the available set of functions.     
 
------------------------------------------------------------------------------*/

/* Dependencies
- stm32f723e_discovery.c
- stm32f7xx_hal_gpio.c
- stm32f7xx_hal_sram.c
- stm32f7xx_hal_rcc_ex.h
- st7789h2.c
- fonts.h
- font24.c
- font20.c
- font16.c
- font12.c
- font8.c"
EndDependencies */

/* Includes ------------------------------------------------------------------*/
#include "stm32f723e_discovery_lcd.h"  

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32F723E_DISCOVERY
  * @{
  */
    
/** @defgroup STM32F723E_DISCOVERY_LCD STM32F723E-DISCOVERY LCD
  * @{
  */ 

/** @defgroup STM32F723E_DISCOVERY_LCD_Private_TypesDefinitions STM32F723E Discovery Lcd Private TypesDef
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup STM32F723E_DISCOVERY_LCD_Private_Defines STM32F723E Discovery Lcd Private Defines
  * @{
  */
/**
  * @}
  */ 

/** @defgroup STM32F723E_DISCOVERY_LCD_Private_Macros STM32F723E Discovery Lcd Private Macros
  * @{
  */
#define ABS(X)  ((X) > 0 ? (X) : -(X))      
/**
  * @}
  */ 
    
/** @defgroup STM32F723E_DISCOVERY_LCD_Private_Variables STM32F723E Discovery Lcd Private Variables
  * @{
  */ 
static LCD_DrvTypeDef  *LcdDrv MCU_SYS_MEM;

/**
  * @}
  */ 

/** @defgroup STM32F723E_DISCOVERY_LCD_Private_FunctionPrototypes STM32F723E Discovery Lcd Private Prototypes
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup STM32F723E_DISCOVERY_LCD_Private_Functions STM32F723E Discovery Lcd Private Functions
  * @{
  */
/**
  * @brief  Initializes the LCD.
  * @retval LCD state
  */
uint8_t BSP_LCD_Init(void)
{
 return (BSP_LCD_InitEx(LCD_ORIENTATION_LANDSCAPE));
}
/**
  * @brief  Initializes the LCD with a given orientation.
  * @param  orientation: LCD_ORIENTATION_PORTRAIT or LCD_ORIENTATION_LANDSCAPE
  * @retval LCD state
  */
uint8_t BSP_LCD_InitEx(uint32_t orientation)
{ 
  uint8_t ret = LCD_ERROR;
  
  /* Initialize LCD special pins GPIOs */
  BSP_LCD_MspInit();
  
  /* Backlight control signal assertion */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_PORT, LCD_BL_CTRL_PIN, GPIO_PIN_SET);
  
  /* Apply hardware reset according to procedure indicated in FRD154BP2901 documentation */
  HAL_GPIO_WritePin(LCD_RESET_GPIO_PORT, LCD_RESET_PIN, GPIO_PIN_RESET);
  HAL_Delay(5);   /* Reset signal asserted during 5ms  */
  HAL_GPIO_WritePin(LCD_RESET_GPIO_PORT, LCD_RESET_PIN, GPIO_PIN_SET);
  HAL_Delay(10);  /* Reset signal released during 10ms */
  HAL_GPIO_WritePin(LCD_RESET_GPIO_PORT, LCD_RESET_PIN, GPIO_PIN_RESET);
  HAL_Delay(20);  /* Reset signal asserted during 20ms */
  HAL_GPIO_WritePin(LCD_RESET_GPIO_PORT, LCD_RESET_PIN, GPIO_PIN_SET);
  HAL_Delay(10);  /* Reset signal released during 10ms */

  if(ST7789H2_drv.ReadID() == ST7789H2_ID)
  {    
    LcdDrv = &ST7789H2_drv;
    
    /* LCD Init */   
    LcdDrv->Init();
    
    if(orientation == LCD_ORIENTATION_PORTRAIT)
    {
      ST7789H2_SetOrientation(LCD_ORIENTATION_PORTRAIT); 
    }
    else if(orientation == LCD_ORIENTATION_LANDSCAPE_ROT180)
    {
      ST7789H2_SetOrientation(LCD_ORIENTATION_LANDSCAPE_ROT180);
    }
    else
    {
      /* Default landscape orientation is selected */
    }
    /* Initialize the font */
    
    ret = LCD_OK;   
  }
  
  return ret;
}

/**
  * @brief  DeInitializes the LCD.
  * @retval LCD state
  */
uint8_t BSP_LCD_DeInit(void)
{ 
  /* Actually LcdDrv does not provide a DeInit function */
  return LCD_OK;
}

/**
  * @brief  Gets the LCD X size.    
  * @retval Used LCD X size
  */
uint32_t BSP_LCD_GetXSize(void)
{
  return(LcdDrv->GetLcdPixelWidth());
}

/**
  * @brief  Gets the LCD Y size.   
  * @retval Used LCD Y size
  */
uint32_t BSP_LCD_GetYSize(void)
{
  return(LcdDrv->GetLcdPixelHeight());
}


/**
  * @brief  Enables the display.
  * @retval None
  */
void BSP_LCD_DisplayOn(void)
{
  LcdDrv->DisplayOn();
}

/**
  * @brief  Disables the display.
  * @retval None
  */
void BSP_LCD_DisplayOff(void)
{
  LcdDrv->DisplayOff();
}


/**
  * @brief  Initializes the LCD GPIO special pins MSP.
  * @retval None
  */
__weak void BSP_LCD_MspInit(void)
{
  GPIO_InitTypeDef gpio_init_structure;

  /* Enable GPIOs clock */
  LCD_RESET_GPIO_CLK_ENABLE();
  LCD_TE_GPIO_CLK_ENABLE();
  LCD_BL_CTRL_GPIO_CLK_ENABLE();
  
  /* LCD_RESET GPIO configuration */
  gpio_init_structure.Pin       = LCD_RESET_PIN;     /* LCD_RESET pin has to be manually controlled */
  gpio_init_structure.Pull      = GPIO_NOPULL;
  gpio_init_structure.Speed     = GPIO_SPEED_FAST;
  gpio_init_structure.Mode      = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(LCD_RESET_GPIO_PORT, &gpio_init_structure);
  HAL_GPIO_WritePin( LCD_RESET_GPIO_PORT, LCD_RESET_PIN, GPIO_PIN_RESET);
  
  /* LCD_TE GPIO configuration */
  gpio_init_structure.Pin       = LCD_TE_PIN;        /* LCD_TE pin has to be manually managed */
  gpio_init_structure.Mode      = GPIO_MODE_INPUT;
  HAL_GPIO_Init(LCD_TE_GPIO_PORT, &gpio_init_structure);

  /* LCD_BL_CTRL GPIO configuration */
  gpio_init_structure.Pin       = LCD_BL_CTRL_PIN;   /* LCD_BL_CTRL pin has to be manually controlled */
  gpio_init_structure.Mode      = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Speed     = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_PORT, &gpio_init_structure);
}

/**
  * @brief  DeInitializes LCD GPIO special pins MSP.
  * @retval None
  */
__weak void BSP_LCD_MspDeInit(void)
{
  GPIO_InitTypeDef  gpio_init_structure;

  /* LCD_RESET GPIO deactivation */
  gpio_init_structure.Pin       = LCD_RESET_PIN;
  HAL_GPIO_DeInit(LCD_RESET_GPIO_PORT, gpio_init_structure.Pin);

  /* LCD_TE GPIO deactivation */
  gpio_init_structure.Pin       = LCD_TE_PIN;
  HAL_GPIO_DeInit(LCD_TE_GPIO_PORT, gpio_init_structure.Pin);

  /* LCD_BL_CTRL GPIO deactivation */
  gpio_init_structure.Pin       = LCD_BL_CTRL_PIN;
  HAL_GPIO_DeInit(LCD_BL_CTRL_GPIO_PORT, gpio_init_structure.Pin);

  /* GPIO pins clock can be shut down in the application
     by surcharging this __weak function */
}



/**
  * @}
  */  
  
/**
  * @}
  */ 
  
/**
  * @}
  */     

/**
  * @}
  */  

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
