/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
/**
 * Program pro ventilacni system
 */
#include "pinmap.h"
#include "peripherals.h"
#include "global.h"
#include "lcd_12864.h"
#include "BME280.h"
#include "ds18b20.h"
#include <stdio.h>
#include "rtc_api.h"
#include "Time.h"
//#include "menu.h"
#include "log.h"
#include "sleep.h"
#include "pwm.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

static States_loop current_state;
static Screen show;
Flags_main flags;
int8_t en_count=0;

/* Definitions of data related to this example */
  /* Definition of ADCx conversions data table size */
  #define ADC_CONVERTED_DATA_BUFFER_SIZE   5   /* Size of array aADCxConvertedData[] */
/* Variable containing ADC conversions data */
static uint16_t   aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];

Buttons pushed_button; //cleared each main cycle

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  char buffer_s [32];

  uint32_t InputVoltage=0;
  uint8_t en_count_last=0;
  char aShowTime[50] = {0};
  Bool show_time=TRUE;

  //timeouts
  Compare_t backlite_compare, measure_compare, led_compare, time_compare, button_compare, heating_compare, logging_compare, show_timeout, heating_instant_timeout;
  backlite_compare.overflow=FALSE , measure_compare.overflow=FALSE, led_compare.overflow=FALSE, time_compare.overflow=FALSE, button_compare.overflow=FALSE, heating_compare.overflow=FALSE, logging_compare.overflow=FALSE, show_timeout.overflow=FALSE, heating_instant_timeout.overflow=FALSE;
  actual_HALtick.overflow = FALSE;
  past_HALtick.overflow = FALSE;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM22_Init();
  MX_USART1_UART_Init();
  MX_USART4_UART_Init();
  MX_USB_PCD_Init();
  MX_TIM21_Init();

  /* USER CODE BEGIN 2 */
  backliteOn();
  fill_comparer(BACKLITE_TIMEOUT, &backlite_compare);

  lcd12864_init(&hspi1);
  line(0,60,110,60,1);
  lcd_setCharPos(0,0);
  lcd_printString("Initialization unit\r");
  lcd_printString("term_vent_git\r");
  snprintf(buffer_s, 11, "SW v 0.%03d", SW_VERSION);
  lcd_printString(buffer_s);
  //ENCODER initialization
  HAL_TIM_Encoder_Start(&htim22,TIM_CHANNEL_1);
  htim22.Instance->EGR = 1;           // Generate an update event
  htim22.Instance->CR1 = 1;           // Enable the counter

  Log_Init(); // initialization of the logging, each LOG_PERIODE second would be logged the data

  HAL_Delay(1700);
  lcd_clear();

  current_state = MEASURING;
  show=desktop;



HAL_ADC_Start(&hadc);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  for (;;)
  {
	  switch (current_state){

	  case MEASURING:
	  {

		  HAL_ADC_PollForConversion(&hadc,100);

		  /* Check if the continous conversion of regular channel is finished */
		      if ((HAL_ADC_GetState(&hadc) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC)
		      {
		        /*##-6- Get the converted value of regular channel  ########################*/
		    	  aADCxConvertedData[2] = HAL_ADC_GetValue(&hadc);
		      }
		  //START ADC CONVERSION:
/*
		    if (HAL_ADC_Start_DMA(&hadc,(uint32_t *)aADCxConvertedData,  ADC_CONVERTED_DATA_BUFFER_SIZE) != HAL_OK)
		    {
		  	  Error_Handler();
		    }
*/	    // end of the ADC start rutine conversion

//DEBUG

		  PWM_togle(21);
		  HAL_GPIO_TogglePin(D_RLY1_GPIO_Port,D_RLY1_Pin);

 //debug



			  InputVoltage = aADCxConvertedData[2];
		  flags.new_data_to_show=TRUE;
		  current_state = IDLE;
		  fill_comparer(MEASURE_PERIODE, &measure_compare);

		  break;
	  }
	  case IDLE:
		  {
			  break;
		  }


	  default:
	  {
		  break;
	  }

	  }// end of switch CURRENT STATE

	  /* **** SCREEN **** */
	  switch (show){

	  case blind:
	  {
		  lcd_clear();
		  show = idle;
		  break;
	  }
	  case desktop:
	  {// showing main screen - temperatures and Hum
		  if (flags.new_data_to_show==TRUE){

//			  lcd_setCharPos(1,3);
//			  char_magnitude(2);
//			  snprintf(buffer_s, 12, "%3ld.%02d C",temperature/100,abs(temperature%100));
//			  lcd_printString(buffer_s);


			  char_magnitude(1);

//			  lcd_setCharPos(3,2);
//			  snprintf(buffer_s, 12, "set %3ld.%02d C",temperature_set/100,abs(temperature_set%100));
//			  lcd_printString(buffer_s);

			  /*	lcd_setCharPos(6,4);
	    					snprintf(buffer_s, 18, "Pres %d.%02d hp",presure/100,presure%100);
	    					lcd_printString(buffer_s);
			   */

			  // resistance divider is 10k to 2k2
			  lcd_setCharPos(6,0);
			  snprintf(buffer_s, 13, "%lu-> %lu.%02luV",InputVoltage,InputVoltage*671/9350,(InputVoltage*671%9350*100/255)/10 );// get two numbers for voltage
			  lcd_printString(buffer_s);

			  flags.new_data_to_show=FALSE; // the data was showed.


		  }// end if - new data to show
		  if (show_time){

			  RTC_DateShow(&hrtc,aShowTime);
			  lcd_setCharPos(0,11);
			  lcd_printString(aShowTime);
			  RTC_TimeShow(&hrtc,aShowTime);
			  //lcd_setCharPos(0,12);
			  lcd_setCharPos(0,1);
			  lcd_printString(aShowTime);

			  // end of the time part - new timer set.
			  fill_comparer(TIME_PERIODE, &time_compare);
			  show_time = FALSE;
		  }

		  show = debug;
		  break;
	  }
	  case idle:
	  {
		  break;
	  }

//	  case menu:
//	  {
//		  display_menu(ActualMenu);
//		  break;
//	  }
	  case debug:
	  {
		  // debug

		  /**Popis kanalu
		   * chanel 1		TEMP2
		   * chanel 2		TEMP3
		   * chanel 4		V_IN_MEAS
		   * chanel 10	TEMP1
		   * chanel 11	50HZ
		   * chanel 12	TEMP5
		   * chanel 13	TEMP4
		   */

		  		  lcd_setCharPos(1,3);
		  		  snprintf(buffer_s, 12, "%d",aADCxConvertedData[0]);
		  		  lcd_printString(buffer_s);

		  		  lcd_setCharPos(2,3);
		  		  snprintf(buffer_s, 12, "%d",aADCxConvertedData[1]);
		  		  lcd_printString(buffer_s);

	  				  lcd_setCharPos(3,3);
	  				  snprintf(buffer_s, 12, "%d",aADCxConvertedData[2]);
	  				  lcd_printString(buffer_s);


	  				  lcd_setCharPos(4,3);
	  					 snprintf(buffer_s, 12, "%d",aADCxConvertedData[3]);
	  					  lcd_printString(buffer_s);
		  break;
	  }

	  default:{
		  lcd_setCharPos(1,1);
		  lcd_printString("ERROR display -default" );
	  }
	  }// switch show

	  /* *------ TIME ELAPSING CHECK -------* */
	  		get_actual_HAL_tick(); // put the actual number of ms counter to variable actual_HALtic.tick

	  		if(comparer_timeout(&logging_compare)) //log data after defined period.
	  		{
	  			current_state = LOG;
	  		}
	  		if(comparer_timeout(&heating_compare)) //measure after defined period.
	  		{
	  			current_state = HEATING;
	  		}
	  		if (comparer_timeout(&measure_compare))  //measure after defined period.
	  		{
	  			current_state = MEASURING;
	  		}
	  		if(comparer_timeout(&time_compare)) //change time after defined period.
	  		{
	  			show_time = TRUE;
	  		}

	  		if(comparer_timeout(&show_timeout)) //change time after defined period.
	  		{
	  			show = desktop;
	  		}
	  // MENU TIMEOUT
	  		if ((TRUE==flags.menu_running)) // je to takhle slozite , protoze jsem neprisel na jiny efektivni zpusob, jak smazat displej, po zkonceni menu
	  			if(!menu_timout()) {
	  				if (!menu_action()){ // exit from menu condition
	  					flags.menu_running=0;
	  					lcd_clear();
	  					show = desktop;
	  				}
	  				else
	  					show = menu;
	  			} // if menu - TIMEOUT
	  			else {
	  				flags.menu_running=0;
	  				lcd_clear();
	  				show = desktop;
	  			}

	  		if (flags.temp_new_set){
	  			flags.temp_new_set = FALSE;
	  			show = desktop;
	  		}

	  		//if (backlite_compare.tick <= actual_HALtick) // shut down the backligh
	  		if (comparer_timeout(&backlite_compare))
	  		{
	  			backliteOff();
	  		}

	  		/* *---- READ KEYBOARD -----* */

	  		pushed_button = BUT_NONE;
	  		if(comparer_timeout(&button_compare)) //every delay time
	  		{
	  			pushed_button = checkButtons();
	  			fill_comparer(BUT_DELAY, &button_compare);
	  			//flags.enc_changed = FALSE;
	  			// reading the actual number from encoder
	  			uint32_t actual_counter_CNT = htim22.Instance->CNT;
	  			if (en_count_last != actual_counter_CNT){
	  				en_count+= actual_counter_CNT-en_count_last;
	  				en_count_last=actual_counter_CNT;
	  				flags.enc_changed = TRUE;
	  				if (pushed_button == BUT_NONE) // enabling light/ increase time constants.
	  					pushed_button = ENCODER;
	  			}
	  		}
	  		if(pushed_button != BUT_NONE) // any button pushed?
	  		{
	  			backliteOn();
	  			fill_comparer(BUT_DELAY*200, &button_compare); // 200x - zpozdeni cteni pri stisknuti
	  			fill_comparer(BACKLITE_TIMEOUT, &backlite_compare);
	  			fill_comparer(SHOW_TIMEOUT, &show_timeout);
	  		}

	  		// -- BUTTON PROCCESS
	  		switch (pushed_button){
	  		case BUT_1:
	  		{// activate heater
	  			if (!flags.regulation_temp){
	  				flags.regulation_temp=TRUE;
	  				flags.heating_up = TRUE;

	  			}
	  			else {
	  				flags.regulation_temp=FALSE;
	  			}
	  			// new data to show - heating icon.
	  			flags.new_data_to_show=TRUE;
	  			//	show = debug;

	  			break;
	  		}
	  		case BUT_2:
	  		{// Immediattely heating for 15 minutes

	  			flags.heating_instant = TRUE;  // predelat na foukej ///

	  			break;
	  		}
	  		case BUT_ENC:
	  		{



	  			break;
	  		}
	  		case ENCODER:
	  		{
	  			break;
	  		}
	  		default:
	  		{

	  			break;
	  		}

	  		} // switch pushed button

	  		HAL_Delay(MAIN_LOOP);

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

// DEBUG LED

	  		PWM_togle(32);	//LED2
	  		PWM_togle(31);	//OUT1
	  		PWM_togle(34);	//OUT2
	  		//PWM_togle(22); //BUZZER
	  		PWM_togle(23); //LCD light

	  		HAL_GPIO_TogglePin(D_RLY2_GPIO_Port,D_RLY2_Pin);
	  		HAL_Delay(400);


// END DEBUG LED

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Configure LSE Drive Capability 
    */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_12;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_RTC
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
