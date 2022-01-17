/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define TIM2PSC  19999  // SysClk 84MHz; 20000 PSC == 4.2KHz
#define STEPPERIOD      2
#define TEMPSPEEDSCALER 6300
#define PHASEDIV        20
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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
 static uint8_t phase;
 static uint8_t step_index;
 
 uint8_t mode_value[8];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void init_mode_values();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  init_mode_values();
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = TIM2PSC;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = STEPPERIOD;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GRN_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, DIR1_MODE4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(REF_GPIO_Port, REF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RST_Pin|ENABLE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STCK_MODE3_Pin|MODE1_Pin|MODE2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GRN_LED_Pin_Pin DIR1_MODE4_Pin */
  GPIO_InitStruct.Pin = GRN_LED_Pin|DIR1_MODE4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : REF_Pin */
  GPIO_InitStruct.Pin = REF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(REF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RST_Pin ENABLE_Pin */
  GPIO_InitStruct.Pin = RST_Pin|ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : STCK_MODE3_Pin MODE1_Pin MODE2_Pin */
  GPIO_InitStruct.Pin = STCK_MODE3_Pin|MODE1_Pin|MODE2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint16_t threeSecondsDivided = ((84000000 / (TIM2PSC + 1)) / (STEPPERIOD + 1)) * (3.0 / PHASEDIV);
uint16_t dynamicScaler = PHASEDIV;
uint8_t phase = 0; // 0 = ramp up; 1 = full speed; 2 = ramp down
uint16_t timerCounter = 0;
uint16_t phaseCounter = 0;
uint32_t halfSteps = 0; //378 steps from 3s ramp-up; 2.1k steps from 3s full-speed; 377 steps from 3s ramp-down
uint32_t inputRcvd = 2600;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if(0 != inputRcvd)
    {
        uint32_t halfStepsPerPhase = inputRcvd * 0.333333;
        if(0 == phase) // Ramp up
        {
            if(++timerCounter >= dynamicScaler)
            {
                HAL_GPIO_TogglePin(GPIOB, STCK_MODE3_Pin);
                halfSteps++;
                
                timerCounter = 0;
            }
            
            if(++phaseCounter >= threeSecondsDivided)
            {
                phaseCounter = 0;
                dynamicScaler--;
            }
            
            if(0 == dynamicScaler)
            {
                phase = 1;
            }
            else if(halfSteps >= halfStepsPerPhase)
            {
                phase = 1;
            }
        }
        else if(1 == phase) // Full Speed
        {
            if(++timerCounter >= dynamicScaler)
            {
                HAL_GPIO_TogglePin(GPIOB, STCK_MODE3_Pin);
                halfSteps++;
                
                timerCounter = 0;
            }
            
            if(halfSteps >= (2 * halfStepsPerPhase))
            {
                phaseCounter = 0;
                dynamicScaler++;
                phase = 2;
            }
        }
        else if(2 == phase) // Ramp Down
        {
            if(++timerCounter >= dynamicScaler)
            {
                HAL_GPIO_TogglePin(GPIOB, STCK_MODE3_Pin);
                halfSteps++;
                
                timerCounter = 0;
            }
            
            if(++phaseCounter >= threeSecondsDivided)
            {
                phaseCounter = 0;
                
                if(dynamicScaler < PHASEDIV)
                {
                    dynamicScaler++;
                }
            }
            
            if(halfSteps >= inputRcvd)
            {
                HAL_GPIO_WritePin(GPIOA, RST_Pin, GPIO_PIN_RESET);
                dynamicScaler = PHASEDIV;
                phase = 3;
            }
        }
        else if(3 == phase) // Idle
        {
            if(++phaseCounter >= threeSecondsDivided * PHASEDIV)
            {
                HAL_GPIO_WritePin(GPIOA, RST_Pin, GPIO_PIN_SET);
                phaseCounter = 0;
                phase = 0;
                halfSteps = 0;
                dynamicScaler = PHASEDIV;
            }
        }
    }
    else
    {
        halfSteps = 0;
    }
}

void init_mode_values()
{
	mode_value[ONE_BY_256] = MODE_VALUE_ONE_BY_256;
	mode_value[ONE_BY_128] = MODE_VALUE_ONE_BY_128;
	mode_value[ONE_BY_64] = MODE_VALUE_ONE_BY_64;
	mode_value[ONE_BY_32] = MODE_VALUE_ONE_BY_32;
	mode_value[ONE_BY_16] = MODE_VALUE_ONE_BY_16;
	mode_value[ONE_BY_8] = MODE_VALUE_ONE_BY_8;
	mode_value[ONE_BY_4] = MODE_VALUE_ONE_BY_4;
	mode_value[HALF_STEP] = MODE_VALUE_HALF_STEP;
	mode_value[FULL_STEP] = MODE_VALUE_FULL_STEP;

	phase = RAMP_UP;
	step_index = ONE_BY_256;
        
        update_mode(MODE_VALUE_FULL_STEP);
}

void clear_modes()
{
	uint8_t value = 0X0;
	HAL_GPIO_WritePin(GPIOB, STCK_MODE3_Pin, (value & MODE3_BIT_MASK));
	HAL_GPIO_WritePin(GPIOA, DIR1_MODE4_Pin, (value & MODE4_BIT_MASK));
	HAL_GPIO_WritePin(GPIOB, MODE1_Pin, (value & MODE1_BIT_MASK));
	HAL_GPIO_WritePin(GPIOB, MODE2_Pin, (value & MODE2_BIT_MASK));
}

uint8_t val1 = 0x0b;
uint8_t val2 = 0x0e;
        
void update_mode(uint8_t value)
{
	//clear_modes();
        /*
        if(value == 0)
        {
          value = val1;
        }
        else if(value = 1)
        {
          value = val2;
        }
        */
        uint16_t i = 0;
  
	HAL_GPIO_WritePin(GPIOA, RST_Pin, GPIO_PIN_RESET);
        while(i++ < 50000) ;
        i = 0;
	HAL_GPIO_WritePin(GPIOB, STCK_MODE3_Pin, (value & MODE3_BIT_MASK));
	HAL_GPIO_WritePin(GPIOA, DIR1_MODE4_Pin, (value & MODE4_BIT_MASK));
	HAL_GPIO_WritePin(GPIOB, MODE1_Pin, (value & MODE1_BIT_MASK));
	HAL_GPIO_WritePin(GPIOB, MODE2_Pin, (value & MODE2_BIT_MASK));
        while(i++ < 50000) ;
	HAL_GPIO_WritePin(GPIOA, RST_Pin, GPIO_PIN_SET);
        
        HAL_GPIO_WritePin(GPIOA, DIR1_MODE4_Pin, GPIO_PIN_SET);
}

void init_step_index()
{
	if(phase == RAMP_UP)
	{
		step_index = ONE_BY_256;
	}
	if(phase == HOLD)
	{
		step_index = FULL_STEP;
	}
	if(phase == RAMP_DOWN)
	{
		step_index = HALF_STEP;
	}
}

void microstep_motor()
{
	static uint8_t hold_counter;

	if(phase == RAMP_UP)
	{
		if((step_index >= ONE_BY_256) && (step_index < FULL_STEP))
		{
			update_mode(mode_value[step_index]);
			++step_index;
		}
		else
		{
			phase = HOLD;
			hold_counter = 0;
		}
	}
	if(phase == HOLD)
	{
		if((step_index == FULL_STEP) && (hold_counter <= step_index))
		{
			update_mode(mode_value[step_index]);
			++hold_counter;
		}
		else
		{
			phase = RAMP_DOWN;
		}
	}
	if(phase == RAMP_DOWN)
	{
		if((step_index <= FULL_STEP) && (step_index > ONE_BY_256))
		{
			update_mode(mode_value[step_index]);
			--step_index;
		}
		else
		{
                  HAL_GPIO_WritePin(GPIOA, RST_Pin, 0);
			phase = RAMP_UP;
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
