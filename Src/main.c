
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"
#include "rtc-board.h"
/* USER CODE BEGIN Includes */
#include <string.h>
#include "board.h"
#include "gpio.h"
#include "delay.h"
#include "timer.h"
#include "radio.h"


#if defined( REGION_EU868 )

#define RF_FREQUENCY                                868000000 // Hz

#endif
#define TX_OUTPUT_POWER                             14        // dBm

#if defined( USE_MODEM_LORA )

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,

#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#endif
#define RX_TIMEOUT_VALUE                            1000

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

LPTIM_HandleTypeDef hlptim1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int8_t RssiValue = 0;
int8_t SnrValue = 0;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;
static TimerEvent_t SystemWakeupTimeTimer;
static bool eventWakeUpTime = false;
static bool frameSent = false;

/*!
 * LED GPIO pins objects
 */
extern Gpio_t Led1;
extern Gpio_t Led2;

OP_MODE opmode;
static volatile GPIO_PinState pinStateM;
static volatile GPIO_PinState pinStateN;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC_Init(void);
static void MX_LPTIM1_Init(void);

/* USER CODE BEGIN PFP */

extern void initialise_monitor_handles(void);

/* Private function prototypes -----------------------------------------------*/
/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( void );

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/*!
 * Callback indicating the end of the system wake-up time calibration
 */
static void OnSystemWakeupTimeTimerEvent( void )
{
	eventWakeUpTime = true;
	setState(INIT);
}

void TimerInitialisation(void)
{
	TimerInit( &SystemWakeupTimeTimer, OnSystemWakeupTimeTimerEvent );
	TimerSetValue( &SystemWakeupTimeTimer, 1000 );
	TimerStart( &SystemWakeupTimeTimer );
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  initialise_monitor_handles();
  uint8_t buffer[4] = {0U,0U,0U,0U};
  GPIO_TypeDef pinState;
  static HAL_LPTIM_StateTypeDef timerState;
  static HAL_StatusTypeDef timerStatus;
  static  uint32_t  counter = 0u;
  uint8_t pinst;
  // Target board initialization
  BoardInitMcu( );
  BoardInitPeriph( );
  MX_LPTIM1_Init();

  // Radio initialization
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init( &RadioEvents );

  Radio.SetChannel( RF_FREQUENCY );

  #if defined( USE_MODEM_LORA )

  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                 LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                 LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                 true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

  Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                 LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                 LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                 0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
#endif

  Radio.Rx( RX_TIMEOUT_VALUE );

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch(opmode)
	  {
		  case INIT:
		  {
			  printf("initialisation\n");
			  HAL_GPIO_EXTI_IRQHandler(REED_IN0_Pin);
			  setState(RUN);
		  }
		  break;

		  case RUN:
		  {
			buffer[0] = HAL_GPIO_ReadPin(REED_IN0_GPIO_Port, REED_IN0_Pin);
			buffer[1] = (uint8_t)((0x1122 & 0xFF00) >> 8);
			buffer[2] = (uint8_t)(0x1122 & 0x00FF);

            /*printf("%d\n", buffer[0]);
            printf("%d\n", buffer[1]);
            printf("%d\n", buffer[2]);
            */
			printf("run\n");
            Radio.Send(buffer,sizeof(buffer));

			  while(!frameSent)
			  {
			      DelayMs(1);
		      }

			  opmode = SLEEP;
		  }
		  break;

		  case SLEEP:
		  {
			  HAL_GPIO_WritePin(REED_EN_GPIO_Port, REED_EN_Pin, GPIO_PIN_RESET);

			  timerStatus = HAL_LPTIM_TimeOut_Start(&hlptim1, 65535,32767);
			  HAL_LPTIM_Counter_Start(&hlptim1,32757);
			  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
			  //HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
			  HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
			  //HAL_LPTIM_TimeOut_Stop_IT(&hlptim1);
		  }
		  break;
    }
  /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
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
  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_LPTIM1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.LptimClockSelection = RCC_LPTIM1CLKSOURCE_PCLK;

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

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* LPTIM1 init function */
static void MX_LPTIM1_Init(void)
{

  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


/* RTC init function */
static void MX_RTC_Init(void)
{

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, VBAT_DIV_EN_Pin|DIO5_Pin|RST_Pin|SPI1_NSS_Pin 
                          |DIO3_Pin|DIO4_Pin|DIO0_Pin|DIO1_Pin 
                          |DIO2_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(REED_EN_GPIO_Port, REED_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : VBAT_DIV_EN_Pin DIO5_Pin RST_Pin SPI1_NSS_Pin 
                           DIO3_Pin DIO4_Pin DIO0_Pin DIO1_Pin 
                           DIO2_Pin LED_Pin */
  GPIO_InitStruct.Pin = VBAT_DIV_EN_Pin|DIO5_Pin|RST_Pin|SPI1_NSS_Pin 
                          |DIO3_Pin|DIO4_Pin|DIO0_Pin|DIO1_Pin 
                          |DIO2_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : REED_EN_Pin */
  GPIO_InitStruct.Pin = REED_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(REED_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : REED_IN0_Pin REED_IN1_Pin */
  GPIO_InitStruct.Pin = REED_IN0_Pin|REED_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void OnTxDone( void )
{
    Radio.Sleep( );
    printf("TxDone\n");
    frameSent = true;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( );
//    BufferSize = size;
//    memcpy( Buffer, payload, BufferSize );
//    RssiValue = rssi;
//    SnrValue = snr;
//    State = RX;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
//    State = TX_TIMEOUT;
}

void OnRxTimeout( void )
{
    Radio.Sleep( );
//    State = RX_TIMEOUT;
    printf("RxTimeout\n");
}

void OnRxError( void )
{
    Radio.Sleep( );
//    State = RX_ERROR;
}

OP_MODE getState(void)
{
	return opmode;
}

void setState(OP_MODE mystate)
{
	opmode = mystate;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

/**
  * @brief  Compare match callback in non blocking mode
  * @param  hlptim : LPTIM handle
  * @retval None
  */
void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hlptim);
  setState(INIT);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_LPTIM_CompareMatchCallback could be implemented in the user file
   */
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
