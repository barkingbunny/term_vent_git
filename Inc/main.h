/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define D_SW1_Pin GPIO_PIN_13
#define D_SW1_GPIO_Port GPIOC
#define A_TEMP5_Pin GPIO_PIN_0
#define A_TEMP5_GPIO_Port GPIOC
#define A_TEMP4_Pin GPIO_PIN_1
#define A_TEMP4_GPIO_Port GPIOC
#define A_TEMP3_Pin GPIO_PIN_2
#define A_TEMP3_GPIO_Port GPIOC
#define A_V_IN_MEAS_Pin GPIO_PIN_3
#define A_V_IN_MEAS_GPIO_Port GPIOC
#define D_IO1_Pin GPIO_PIN_0
#define D_IO1_GPIO_Port GPIOA
#define A_TEMP2_Pin GPIO_PIN_1
#define A_TEMP2_GPIO_Port GPIOA
#define A_TEMP1_Pin GPIO_PIN_2
#define A_TEMP1_GPIO_Port GPIOA
#define D_LCD_TFT_LED_Pin GPIO_PIN_3
#define D_LCD_TFT_LED_GPIO_Port GPIOA
#define A_IN_50HZ_Pin GPIO_PIN_4
#define A_IN_50HZ_GPIO_Port GPIOA
#define D_LCD_D_C_Pin GPIO_PIN_4
#define D_LCD_D_C_GPIO_Port GPIOC
#define D_LCD_RST_Pin GPIO_PIN_5
#define D_LCD_RST_GPIO_Port GPIOC
#define _D_LCD_CS_Pin GPIO_PIN_0
#define _D_LCD_CS_GPIO_Port GPIOB
#define D_OUT2_Pin GPIO_PIN_1
#define D_OUT2_GPIO_Port GPIOB
#define _D_MEM_CS_Pin GPIO_PIN_2
#define _D_MEM_CS_GPIO_Port GPIOB
#define D_LCD_LIGHT_Pin GPIO_PIN_10
#define D_LCD_LIGHT_GPIO_Port GPIOB
#define D_OUT3_Pin GPIO_PIN_11
#define D_OUT3_GPIO_Port GPIOB
#define D_RLY2_Pin GPIO_PIN_12
#define D_RLY2_GPIO_Port GPIOB
#define D_CHAN_A_Pin GPIO_PIN_6
#define D_CHAN_A_GPIO_Port GPIOC
#define D_CHAN_B_Pin GPIO_PIN_7
#define D_CHAN_B_GPIO_Port GPIOC
#define _D_EXTRA_CS_Pin GPIO_PIN_8
#define _D_EXTRA_CS_GPIO_Port GPIOC
#define D_EXTRA_PSU_EN_Pin GPIO_PIN_9
#define D_EXTRA_PSU_EN_GPIO_Port GPIOC
#define D_EXTRA_IO_Pin GPIO_PIN_8
#define D_EXTRA_IO_GPIO_Port GPIOA
#define D_LED1_Pin GPIO_PIN_15
#define D_LED1_GPIO_Port GPIOA
#define D_WIRE1_Pin GPIO_PIN_12
#define D_WIRE1_GPIO_Port GPIOC
#define D_WIRE2_Pin GPIO_PIN_2
#define D_WIRE2_GPIO_Port GPIOD
#define D_BUZER_Pin GPIO_PIN_3
#define D_BUZER_GPIO_Port GPIOB
#define D_OUT1_Pin GPIO_PIN_4
#define D_OUT1_GPIO_Port GPIOB
#define D_LED2_Pin GPIO_PIN_5
#define D_LED2_GPIO_Port GPIOB
#define D_RLY1_Pin GPIO_PIN_6
#define D_RLY1_GPIO_Port GPIOB
#define D_IO2_Pin GPIO_PIN_7
#define D_IO2_GPIO_Port GPIOB
#define D_SW_ENC_Pin GPIO_PIN_8
#define D_SW_ENC_GPIO_Port GPIOB
#define D_SW2_Pin GPIO_PIN_9
#define D_SW2_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
