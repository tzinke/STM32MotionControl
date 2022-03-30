/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define NUMBER_OF_MODES 	9
#define MODE3_BIT_MASK 		0X8
#define MODE4_BIT_MASK 		0X4
#define MODE1_BIT_MASK 		0X2
#define MODE2_BIT_MASK 		0X1

#define MODE_VALUE_ONE_BY_256	0x2//0x9
#define MODE_VALUE_ONE_BY_128	0X2
#define MODE_VALUE_ONE_BY_64	0x2//0X7
#define MODE_VALUE_ONE_BY_32	0X2
#define MODE_VALUE_ONE_BY_16	0XD
#define MODE_VALUE_ONE_BY_8	0XB
#define MODE_VALUE_ONE_BY_4	0XA
#define MODE_VALUE_HALF_STEP	0X5
#define MODE_VALUE_FULL_STEP	0X0


/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
enum modes_index
{
	ONE_BY_256 = 0,				// 		1/256 STEP
	ONE_BY_128,					// 		1/128 STEP
	ONE_BY_64,					// 		1/164 STEP
	ONE_BY_32,					// 		1/32 STEP
	ONE_BY_16,					// 		1/16 STEP
	ONE_BY_8,					// 		1/8 STEP
	ONE_BY_4,					// 		1/4 STEP
	HALF_STEP,					// 		1/2 STEP
	FULL_STEP,					// 		1 STEP
	MAX_MODES_INDEX				// 		MAX NUMBER OF SUPPORTED MODES
};

enum motor_phase
{
	RAMP_UP = 0,
	HOLD,
	RAMP_DOWN
};

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define GRN_LED_Pin GPIO_PIN_5
#define GRN_LED_GPIO_Port GPIOA
#define REF_Pin GPIO_PIN_7
#define REF_GPIO_Port GPIOC
#define DIR1_MODE4_Pin GPIO_PIN_8
#define DIR1_MODE4_GPIO_Port GPIOA
#define RST_Pin GPIO_PIN_9
#define RST_GPIO_Port GPIOA
#define ENABLE_Pin GPIO_PIN_10
#define ENABLE_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_3
#define PWM1_GPIO_Port GPIOB
#define MODE1_Pin GPIO_PIN_4
#define MODE1_GPIO_Port GPIOB
#define MODE2_Pin GPIO_PIN_6
#define MODE2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

//void init_mode_values();
void clear_modes();
void update_mode(uint8_t value);
void init_step_index();
void microstep_motor();
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
