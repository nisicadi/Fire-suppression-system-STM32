/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

osThreadId checkGasHandle;
osThreadId checkTemperaturHandle;
osThreadId runActionsHandle;
osSemaphoreId countingSemaphoreHandle;
/* USER CODE BEGIN PV */
volatile float currentTemp = 0.0;
const float maxTemp = 300.0;

volatile bool isGasDetected = false;
volatile bool areSprinklersActivated = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
void StartGasSensor(void const * argument);
void StartTemperatureSensor(void const * argument);
void StartRunActions(void const * argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	UNUSED(GPIO_Pin);
	if(GPIO_Pin == GPIO_PIN_12 && areSprinklersActivated)
	{
		osSemaphoreWait(countingSemaphoreHandle, osWaitForever);
		areSprinklersActivated = false;
			
		HAL_GPIO_WritePin(GPIOA, RLED_Pin, 0);
		HAL_GPIO_WritePin(GPIOA, YLED_Pin, 0);
		HAL_GPIO_WritePin(GPIOA, GLED_Pin, 0);

		osSemaphoreRelease(countingSemaphoreHandle);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of countingSemaphore */
  osSemaphoreDef(countingSemaphore);
  countingSemaphoreHandle = osSemaphoreCreate(osSemaphore(countingSemaphore), 2);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of checkGas */
  osThreadDef(checkGas, StartGasSensor, osPriorityNormal, 0, 128);
  checkGasHandle = osThreadCreate(osThread(checkGas), NULL);

  /* definition and creation of checkTemperatur */
  osThreadDef(checkTemperatur, StartTemperatureSensor, osPriorityNormal, 0, 128);
  checkTemperaturHandle = osThreadCreate(osThread(checkTemperatur), NULL);

  /* definition and creation of runActions */
  osThreadDef(runActions, StartRunActions, osPriorityAboveNormal, 0, 128);
  runActionsHandle = osThreadCreate(osThread(runActions), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RLED_Pin|YLED_Pin|GLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GasSensor_Pin */
  GPIO_InitStruct.Pin = GasSensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GasSensor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RLED_Pin YLED_Pin GLED_Pin */
  GPIO_InitStruct.Pin = RLED_Pin|YLED_Pin|GLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : EXTI_Pin */
  GPIO_InitStruct.Pin = EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EXTI_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartGasSensor */
/**
  * @brief  Function implementing the checkGas thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartGasSensor */
void StartGasSensor(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		if(HAL_GPIO_ReadPin(GPIOC, GasSensor_Pin) == 0)
			isGasDetected = true;
		else
			isGasDetected = false;
		
    osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTemperatureSensor */
/**
* @brief Function implementing the checkTemperatur thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTemperatureSensor */
void StartTemperatureSensor(void const * argument)
{
  /* USER CODE BEGIN StartTemperatureSensor */
  /* Infinite loop */
  for(;;)
  {
		HAL_ADC_Start(&hadc1);
		if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)
				currentTemp = (((HAL_ADC_GetValue(&hadc1) * 500.0)/4095.0) * 2.393)+0.5; 
		HAL_ADC_Stop(&hadc1);

    osDelay(10);
  }
  /* USER CODE END StartTemperatureSensor */
}

/* USER CODE BEGIN Header_StartRunActions */
/**
* @brief Function implementing the runActions thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRunActions */
void StartRunActions(void const * argument)
{
  /* USER CODE BEGIN StartRunActions */
  /* Infinite loop */
  for(;;)
  {				
		if(!areSprinklersActivated)
		{
			osSemaphoreWait(countingSemaphoreHandle, osWaitForever);
			if(currentTemp > maxTemp && isGasDetected)
			{		
				HAL_GPIO_WritePin(GPIOA, RLED_Pin, 1);
				HAL_GPIO_WritePin(GPIOA, YLED_Pin, 0);
				HAL_GPIO_WritePin(GPIOA, GLED_Pin, 0);
			
				areSprinklersActivated = true;
			} 
			else if (currentTemp <= maxTemp && isGasDetected)
			{
				HAL_GPIO_WritePin(GPIOA, RLED_Pin, 0);
				HAL_GPIO_WritePin(GPIOA, YLED_Pin, 1);
				HAL_GPIO_WritePin(GPIOA, GLED_Pin, 0);
			} 
			else 
			{
				HAL_GPIO_WritePin(GPIOA, RLED_Pin, 0);
				HAL_GPIO_WritePin(GPIOA, YLED_Pin, 0);
				HAL_GPIO_WritePin(GPIOA, GLED_Pin, 1);
			}
			osSemaphoreRelease(countingSemaphoreHandle);
		}
    osDelay(10);
  }
  /* USER CODE END StartRunActions */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
