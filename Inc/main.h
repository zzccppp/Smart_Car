/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR3_IN1_Pin GPIO_PIN_2
#define MOTOR3_IN1_GPIO_Port GPIOF
#define MOTOR3_IN2_Pin GPIO_PIN_3
#define MOTOR3_IN2_GPIO_Port GPIOF
#define MOTOR4_IN1_Pin GPIO_PIN_4
#define MOTOR4_IN1_GPIO_Port GPIOF
#define MOTOR4_IN2_Pin GPIO_PIN_5
#define MOTOR4_IN2_GPIO_Port GPIOF
#define MOTOR1_IN1_Pin GPIO_PIN_6
#define MOTOR1_IN1_GPIO_Port GPIOF
#define MOTOR1_IN2_Pin GPIO_PIN_7
#define MOTOR1_IN2_GPIO_Port GPIOF
#define MOTOR2_IN1_Pin GPIO_PIN_8
#define MOTOR2_IN1_GPIO_Port GPIOF
#define MOTOR2_IN2_Pin GPIO_PIN_9
#define MOTOR2_IN2_GPIO_Port GPIOF
#define SONIC_1_Pin GPIO_PIN_6
#define SONIC_1_GPIO_Port GPIOA
#define SONIC_2_Pin GPIO_PIN_7
#define SONIC_2_GPIO_Port GPIOA
#define SONIC_3_Pin GPIO_PIN_0
#define SONIC_3_GPIO_Port GPIOB
#define SONIC_4_Pin GPIO_PIN_1
#define SONIC_4_GPIO_Port GPIOB
#define BT_LED_Pin GPIO_PIN_0
#define BT_LED_GPIO_Port GPIOG
#define BT_KEY_Pin GPIO_PIN_1
#define BT_KEY_GPIO_Port GPIOG
#define SONIC_PWM1_Pin GPIO_PIN_9
#define SONIC_PWM1_GPIO_Port GPIOE
#define SONIC_PWM2_Pin GPIO_PIN_11
#define SONIC_PWM2_GPIO_Port GPIOE
#define SONIC_PWM3_Pin GPIO_PIN_13
#define SONIC_PWM3_GPIO_Port GPIOE
#define SONIC_PWM4_Pin GPIO_PIN_14
#define SONIC_PWM4_GPIO_Port GPIOE
#define MOTOR1_PWM_Pin GPIO_PIN_12
#define MOTOR1_PWM_GPIO_Port GPIOD
#define MOTOR2_PWM_Pin GPIO_PIN_13
#define MOTOR2_PWM_GPIO_Port GPIOD
#define MOTOR3_PWM_Pin GPIO_PIN_14
#define MOTOR3_PWM_GPIO_Port GPIOD
#define MOTOR4_PWM_Pin GPIO_PIN_15
#define MOTOR4_PWM_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
