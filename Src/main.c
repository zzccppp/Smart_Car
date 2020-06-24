/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
/* USER CODE BEGIN PTD */
uint8_t USART3_RECEIVE_BUFFER[50];
uint8_t USART3_RECEIVE_BUFFER_SIZE = 45;

uint8_t USART2_RECEIVE_BUFFER[50];
uint8_t USART2_RECEIVE_BUFFER_SIZE = 1;

uint8_t capture_state_ch1 = 0;
uint16_t start_val_ch1 = 0;
uint16_t end_val_ch1 = 0;
uint16_t cnt_ch1 = 0;

uint8_t capture_state_ch2 = 0;
uint16_t start_val_ch2 = 0;
uint16_t end_val_ch2 = 0;
uint16_t cnt_ch2 = 0;

uint8_t capture_state_ch3 = 0;
uint16_t start_val_ch3 = 0;
uint16_t end_val_ch3 = 0;
uint16_t cnt_ch3 = 0;

uint8_t capture_state_ch4 = 0;
uint16_t start_val_ch4 = 0;
uint16_t end_val_ch4 = 0;
uint16_t cnt_ch4 = 0;

uint32_t distance_ch1 = 0;
uint32_t distance_ch2 = 0;
uint32_t distance_ch3 = 0;
uint32_t distance_ch4 = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
#define BUFSIZE 512
char myprintf_buf[BUFSIZE];
char ustart2_printf_buf[BUFSIZE];
char ustart3_printf_buf[BUFSIZE];

void Debug_printf(const char *fmt, ...) {
    va_list args;
    int n;
    va_start(args, fmt);
    n = vsnprintf(myprintf_buf, BUFSIZE, fmt, args);
    va_end(args);
    int i = 0;
    for (i = 0; i < n; i++) {
        HAL_UART_Transmit(&huart1, (uint8_t *) &myprintf_buf[i], 1, 0xFFFF);
    }
}

void u2_printf(const char *fmt, ...) {
    va_list args;
    int n;
    va_start(args, fmt);
    n = vsnprintf(ustart2_printf_buf, BUFSIZE, fmt, args);
    va_end(args);
    int i = 0;
    for (i = 0; i < n; i++) {
        HAL_UART_Transmit(&huart2, (uint8_t *) &ustart2_printf_buf[i], 1, 0xFFFF);
    }
}

void u3_printf(const char *fmt, ...) {
    va_list args;
    int n;
    va_start(args, fmt);
    n = vsnprintf(ustart3_printf_buf, BUFSIZE, fmt, args);
    va_end(args);
    int i = 0;
    for (i = 0; i < n; i++) {
        HAL_UART_Transmit(&huart3, (uint8_t *) &ustart3_printf_buf[i], 1, 0xFFFF);
    }
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_TIM3_Init(void);

static void MX_USART1_UART_Init(void);

static void MX_USART2_UART_Init(void);

static void MX_USART3_UART_Init(void);

static void MX_TIM1_Init(void);

static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void initMotor();

void controlMotor();

void initMotor() {
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);//speed = 0
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

    //set the motor init state
    HAL_GPIO_WritePin(GPIOF, MOTOR3_IN1_Pin | MOTOR4_IN1_Pin | MOTOR1_IN1_Pin | MOTOR2_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOF, MOTOR3_IN2_Pin | MOTOR4_IN2_Pin | MOTOR1_IN2_Pin | MOTOR2_IN2_Pin, GPIO_PIN_RESET);
}

void controlMotor() {
    uint32_t avg = (distance_ch2 + distance_ch1) / 2;

    if ((distance_ch3 + 5) < avg) {
        HAL_GPIO_WritePin(GPIOF, MOTOR3_IN2_Pin | MOTOR4_IN1_Pin | MOTOR1_IN2_Pin | MOTOR2_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOF, MOTOR3_IN1_Pin | MOTOR4_IN2_Pin | MOTOR1_IN1_Pin | MOTOR2_IN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 300);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 300);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 300);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 300);
        //turn right
    } else if ((distance_ch4 + 5) < avg) {
        HAL_GPIO_WritePin(GPIOF, MOTOR3_IN1_Pin | MOTOR4_IN2_Pin | MOTOR1_IN1_Pin | MOTOR2_IN2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOF, MOTOR3_IN2_Pin | MOTOR4_IN1_Pin | MOTOR1_IN2_Pin | MOTOR2_IN1_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 300);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 300);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 300);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 300);
        //turn left
    } else {
        HAL_GPIO_WritePin(GPIOF, MOTOR3_IN1_Pin | MOTOR4_IN1_Pin | MOTOR1_IN1_Pin | MOTOR2_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOF, MOTOR3_IN2_Pin | MOTOR4_IN2_Pin | MOTOR1_IN2_Pin | MOTOR2_IN2_Pin, GPIO_PIN_RESET);
        if (avg > 250) {
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
        } else if (avg < 20) {
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
        } else if (avg >= 20 && avg <= 80) {
            uint tmp = (25 * avg / 3 - 500 / 3);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, tmp);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, tmp);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, tmp);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, tmp);
        } else {
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 500);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 500);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 500);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 500);
        }
    }
}

/* USER CODE END 0 */


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
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
    MX_TIM3_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_TIM1_Init();
    MX_TIM4_Init();
    /* USER CODE BEGIN 2 */
    HAL_UART_Receive_IT(&huart2, USART2_RECEIVE_BUFFER, USART2_RECEIVE_BUFFER_SIZE);

    u2_printf("AT+ROLE=0\r\n");
    HAL_Delay(100);
    u2_printf("AT+UART=38400,0,0\r\n");
    HAL_Delay(100);
    u2_printf("AT+PSWD=1234\r\n");
    HAL_Delay(100);
    u2_printf("AT+NAME=SMART_CAR\r\n");
    HAL_Delay(100);
    u2_printf("AT+CMODE=1\r\n");
    HAL_Delay(100);

    u2_printf("AT+ROLE?\r\n");
    HAL_Delay(100);
    u2_printf("AT+UART?\r\n");
    HAL_Delay(100);
    u2_printf("AT+PSWD?\r\n");
    HAL_Delay(100);
    u2_printf("AT+NAME?\r\n");
    HAL_Delay(100);
    u2_printf("AT+CMODE?\r\n");
    HAL_Delay(100);

    HAL_GPIO_WritePin(BT_KEY_GPIO_Port, BT_KEY_Pin, GPIO_PIN_RESET);

    u2_printf("AT+RESET\r\n");
//    HAL_Delay(100);

    HAL_Delay(1000);//wait for the bluetooth device to initialize

    while (1) {
        GPIO_PinState re = HAL_GPIO_ReadPin(BT_LED_GPIO_Port, BT_LED_Pin);
        if (re == GPIO_PIN_RESET) {
            Debug_printf("Not connect yet!\r\n");
            HAL_Delay(1000);
        } else {
            Debug_printf("connect successful\r\n");
            break;
        }
    }

    HAL_UART_Receive_IT(&huart3, USART3_RECEIVE_BUFFER, USART3_RECEIVE_BUFFER_SIZE);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 46000);
    HAL_TIM_Base_Start_IT(&htim3);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 46000);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 46000);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 46000);

    initMotor();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
    while (1) {
//        HAL_Delay(1000);
        if (capture_state_ch1 == 0) {
            capture_state_ch1++; // 1-> start capture
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1); // 上升沿capture__
        } else if (capture_state_ch1 == 3) {
            //计算时间
            uint32_t tmp;
            if (cnt_ch1 == 0) {
                tmp = end_val_ch1 - start_val_ch1;
            } else {
                tmp = 500 - start_val_ch1 + end_val_ch1 + 500 * (cnt_ch1 - 1);
            }
            distance_ch1 = tmp * 17 / 500;
            Debug_printf("CHANNEL1=%d\r\n", distance_ch1);
            capture_state_ch1 = 0;
            start_val_ch1 = 0;
            end_val_ch1 = 0;
            cnt_ch1 = 0;
        }

        if (capture_state_ch2 == 0) {
            capture_state_ch2++; // 1-> start capture
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
            HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2); // 上升沿capture__
        } else if (capture_state_ch2 == 3) {
            //计算时间
            uint32_t tmp;
            if (cnt_ch2 == 0) {
                tmp = end_val_ch2 - start_val_ch2;
            } else {
                tmp = 500 - start_val_ch2 + end_val_ch2 + 500 * (cnt_ch2 - 1);
            }
            distance_ch2 = tmp * 17 / 500;
            Debug_printf("CHANNEL2=%d\r\n", distance_ch2);
            capture_state_ch2 = 0;
            start_val_ch2 = 0;
            end_val_ch2 = 0;
            cnt_ch2 = 0;
        }

        if (capture_state_ch3 == 0) {
            capture_state_ch3++; // 1-> start capture
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
            HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3); // 上升沿capture__
        } else if (capture_state_ch3 == 3) {
            //计算时间
            uint32_t tmp;
            if (cnt_ch3 == 0) {
                tmp = end_val_ch3 - start_val_ch3;
            } else {
                tmp = 500 - start_val_ch3 + end_val_ch3 + 500 * (cnt_ch3 - 1);
            }
            distance_ch3 = tmp * 17 / 500;
            Debug_printf("CHANNEL3=%d\r\n", distance_ch3);
            capture_state_ch3 = 0;
            start_val_ch3 = 0;
            end_val_ch3 = 0;
            cnt_ch3 = 0;
        }

        if (capture_state_ch4 == 0) {
            capture_state_ch4++; // 1-> start capture
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
            HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4); // 上升沿capture__
        } else if (capture_state_ch4 == 3) {
            //计算时间
            uint32_t tmp;
            if (cnt_ch4 == 0) {
                tmp = end_val_ch4 - start_val_ch4;
            } else {
                tmp = 500 - start_val_ch4 + end_val_ch4 + 500 * (cnt_ch4 - 1);
            }
            distance_ch4 = tmp * 17 / 500;
            Debug_printf("CHANNEL4=%d\r\n", distance_ch4);
            capture_state_ch4 = 0;
            start_val_ch4 = 0;
            end_val_ch4 = 0;
            cnt_ch4 = 0;
        }


        controlMotor();


        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
#pragma clang diagnostic pop
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void) {

    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 239;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 49999;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
    HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void) {

    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 71;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 499;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_IC_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void) {

    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 71;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 499;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */
    HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void) {

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void) {

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 38400;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void) {

    /* USER CODE BEGIN USART3_Init 0 */

    /* USER CODE END USART3_Init 0 */

    /* USER CODE BEGIN USART3_Init 1 */

    /* USER CODE END USART3_Init 1 */
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 38400;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART3_Init 2 */

    /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOF, MOTOR3_IN1_Pin | MOTOR3_IN2_Pin | MOTOR4_IN1_Pin | MOTOR4_IN2_Pin
                             | MOTOR1_IN1_Pin | MOTOR1_IN2_Pin | MOTOR2_IN1_Pin | MOTOR2_IN2_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(BT_KEY_GPIO_Port, BT_KEY_Pin, GPIO_PIN_SET);

    /*Configure GPIO pins : MOTOR3_IN1_Pin MOTOR3_IN2_Pin MOTOR4_IN1_Pin MOTOR4_IN2_Pin
                             MOTOR1_IN1_Pin MOTOR1_IN2_Pin MOTOR2_IN1_Pin MOTOR2_IN2_Pin */
    GPIO_InitStruct.Pin = MOTOR3_IN1_Pin | MOTOR3_IN2_Pin | MOTOR4_IN1_Pin | MOTOR4_IN2_Pin
                          | MOTOR1_IN1_Pin | MOTOR1_IN2_Pin | MOTOR2_IN1_Pin | MOTOR2_IN2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /*Configure GPIO pin : BT_LED_Pin */
    GPIO_InitStruct.Pin = BT_LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BT_LED_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : BT_KEY_Pin */
    GPIO_InitStruct.Pin = BT_KEY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BT_KEY_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (&huart2 == huart) {
        HAL_UART_Transmit(&huart1, USART2_RECEIVE_BUFFER, USART2_RECEIVE_BUFFER_SIZE, 1000);
    } else if (&huart3 == huart) {
        HAL_UART_Transmit(&huart2, USART3_RECEIVE_BUFFER, USART3_RECEIVE_BUFFER_SIZE, 1000);
//        HAL_UART_Transmit(&huart1, USART3_RECEIVE_BUFFER, USART3_RECEIVE_BUFFER_SIZE, 1000);
    }
}

//输入捕获的callback
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
            if (capture_state_ch1 == 1) {
                start_val_ch1 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
                capture_state_ch1++;//2->in capture mode
            } else if (capture_state_ch1 == 2) {
                end_val_ch1 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
                HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);
                capture_state_ch1++;// 3->finished
            }
        }
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
            if (capture_state_ch2 == 1) {
                start_val_ch2 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
                capture_state_ch2++;//2->in capture mode
            } else if (capture_state_ch2 == 2) {
                end_val_ch2 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
                HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_2);
                capture_state_ch2++;// 3->finished
            }
        }
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
            if (capture_state_ch3 == 1) {
                start_val_ch3 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
                capture_state_ch3++;//3->in capture mode
            } else if (capture_state_ch3 == 2) {
                end_val_ch3 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);
                HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_3);
                capture_state_ch3++;// 3->finished
            }
        }
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
            if (capture_state_ch4 == 1) {
                start_val_ch4 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4);
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
                capture_state_ch4++;//2->in capture mode
            } else if (capture_state_ch4 == 2) {
                end_val_ch4 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4);
                HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_4);
                capture_state_ch4++;// 3->finished
            }
        }
    }
}

//溢出的callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        if (capture_state_ch1 == 2) {
            cnt_ch1++;
        }
        if (capture_state_ch2 == 2) {
            cnt_ch2++;
        }
        if (capture_state_ch3 == 2) {
            cnt_ch3++;
        }
        if (capture_state_ch4 == 2) {
            cnt_ch4++;
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
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

#pragma clang diagnostic pop