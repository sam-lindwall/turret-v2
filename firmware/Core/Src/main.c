/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "pid.h"
#include "telemetry.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUF_SIZE 16
#define MAX_FIELD 4 // i.e: "p-70" is 4 chars

#define I2C_FAIL_LIMIT 5   // consecutive failures before recovery

#define TELEM_TARGET_PAN 1 // Which axis telemetry streams. 1 = pan, 0 = tilt.

#define TILT_ANGLE_MAX 210.0 // from chassis physical constraints + encoder readings based on assembled magnet orientation
#define TILT_ANGLE_MIN 100

#define PAN_ANGLE_MAX 190.0
#define PAN_ANGLE_MIN 50.0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

DMA_HandleTypeDef hdma_usart2_tx; // for telemetry read from stm to mac to tune PID gains

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void parse (char *);
float read_angle(I2C_HandleTypeDef *hi2c);
void i2c_recover(PID_t *pid, GPIO GPIO_1, GPIO GPIO_2, I2C_HandleTypeDef *hi2c);
static float boot_read_angle(I2C_HandleTypeDef *hi2c);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint32_t pid_tick = 0; // for telemetry


// ------ UART ------
volatile char rx_buff[RX_BUF_SIZE];
volatile char rxByte;
volatile uint8_t rxIndex;
volatile uint8_t cmdReady;
// ------------------

// ------ PID target positions ------
float target_tilt_position;
float target_pan_position;
// ----------------------------------

volatile uint8_t flag; //toggles at pid frequency (160hz form timer 3)

// ------ Motor driver GPIOs to set direction ------
// ------     (B = Pan, A = Tilt)      ------
GPIO AIN1 = {AI_1_GPIO_Port, AI_1_Pin};
GPIO AIN2 = {AI_2_GPIO_Port, AI_2_Pin};

GPIO BIN1 = {BI_1_GPIO_Port, BI_1_Pin};
GPIO BIN2 = {BI_2_GPIO_Port, BI_2_Pin};
// --------------------------------------------


// ------ PID motor gain assignment ------
PID_t pan_pid = { .kp = 30, .ki = 0, .kd = 0.75f, .out_limit = 999.0f, .integral_limit = 300.0f, .slew = 25.0f,
                  .ff_fwd = 86.0f, .ff_rev = 84.0f, .ff_vel_fade = 60.0f, .kv = 1.6f, .band_stop = 3.0f, .band_go = 4.5f,
                  .track_vel = 5.0f, .stall_ticks = 24, .fric_fwd = 0.0f, .fric_rev = 0.0f, .fric_vel = 0.0f};

PID_t tilt_pid = { .kp = 10, .ki = 3.0f, .kd = 0.75f, .out_limit = 999.0f, .integral_limit = 300.0f, .slew = 0.0f,
                   .ff_fwd = 90.0f, .ff_rev = 75.0f, .ff_vel_fade = 60.0f, .kv = 0.0f, .band_stop = 0.5f, .band_go = 1.2f,
                   .track_vel = 0.0f, .stall_ticks = 0, .fric_fwd = 155.0f, .fric_rev = 125.0f, .fric_vel = 2.0f};
// ---------------------------------------


float pan_angle_max;
float pan_angle_min;

float tilt_angle_max;
float tilt_angle_min;


// ------ Reference (Zero) point ------
float boot_pan_pos;
float boot_tilt_pos;
// ------------------------------------


// ------ Used to transmit the current absolute position of the motors relative to the reference point to the pi ------
static char tx_buf[48];
// --------------------------------------------------------------------------------------------------------------------


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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */



  HAL_UART_Receive_IT(&huart3, &rxByte, 1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim3);
  __HAL_I2C_ENABLE(&hi2c1);
  __HAL_I2C_ENABLE(&hi2c3);



  // test to make sure motors spin the right way
//    HAL_GPIO_WritePin(AIN1.port, AIN1.pin, GPIO_PIN_SET); // should  be up
//    HAL_GPIO_WritePin(AIN2.port, AIN2.pin, GPIO_PIN_RESET);
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 200);
//    HAL_Delay(500);
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
//    HAL_GPIO_WritePin(AIN1.port, AIN1.pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(AIN2.port, AIN2.pin, GPIO_PIN_RESET);
//    while(1);

//    HAL_GPIO_WritePin(BIN1.port, BIN1.pin, GPIO_PIN_SET); // should be clockwise
//    HAL_GPIO_WritePin(BIN2.port, BIN2.pin, GPIO_PIN_RESET);
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 200);
//      HAL_Delay(500);
//      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
//      HAL_GPIO_WritePin(BIN1.port, BIN1.pin, GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(BIN2.port, BIN2.pin, GPIO_PIN_RESET);
//
//    while(1);

  boot_pan_pos = boot_read_angle(&hi2c1);
  target_pan_position   = boot_pan_pos;
  pan_pid.prev_measurement = boot_pan_pos;
  pan_angle_max = PAN_ANGLE_MAX;
  pan_angle_min = PAN_ANGLE_MIN;

  boot_tilt_pos = boot_read_angle(&hi2c3);
  target_tilt_position  = boot_tilt_pos;
  tilt_pid.prev_measurement = boot_tilt_pos;
  tilt_angle_max = TILT_ANGLE_MAX;
  tilt_angle_min = TILT_ANGLE_MIN;


  // arm the driver
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (cmdReady) {
	  		 parse(rx_buff); // Populate target_pan_position and target_tilt_position
	  	     cmdReady = 0;
	  	  }

	  if (flag) {

		  float current_tilt_position = read_angle(&hi2c3);
		  float current_pan_position  = read_angle(&hi2c1);

		  if (current_tilt_position == 0xFFFF) {
		      i2c_recover(&tilt_pid, AIN2, AIN1, &hi2c3);
		  } else {
			  if (tilt_pid.reseed) {
			      tilt_pid.prev_measurement = current_tilt_position;
			      tilt_pid.prev_target      = target_tilt_position;
			      tilt_pid.tvel_f           = 0.0f;
			      tilt_pid.speed_f          = 0.0f;
			      tilt_pid.reseed           = 0;
			  }
		      tilt_pid.fail_count = 0;

		      if (target_tilt_position > tilt_angle_max) target_tilt_position = tilt_angle_max;
		      if (target_tilt_position < tilt_angle_min) target_tilt_position = tilt_angle_min;

		      float tilt_ccr = pid(target_tilt_position, current_tilt_position, AIN2, AIN1, &tilt_pid);
		      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, tilt_ccr);
		  }

		  if (current_pan_position == 0xFFFF) {
		      i2c_recover(&pan_pid, BIN1, BIN2, &hi2c1);
		  } else {
			  if (pan_pid.reseed) {
			      	  pan_pid.prev_measurement = current_pan_position;
			      	  pan_pid.prev_target      = target_pan_position;
			      	  pan_pid.tvel_f           = 0.0f;
			      	  pan_pid.speed_f          = 0.0f;
			      	  pan_pid.reseed           = 0;
			  }
		      pan_pid.fail_count = 0;

		      if (target_pan_position > pan_angle_max) target_pan_position = pan_angle_max;
		      if (target_pan_position < pan_angle_min) target_pan_position = pan_angle_min;

		      float pan_ccr = pid(target_pan_position, current_pan_position, BIN1, BIN2, &pan_pid);
		      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pan_ccr);
		  }

			  // ----- Telemetry for tuning -----

		  if (TELEM_TARGET_PAN) {
			  if (current_pan_position != 0xFFFF) {
				  telemetry_emit(pid_tick, (float)target_pan_position, current_pan_position,
			                 (float)target_pan_position - current_pan_position,
			                 pan_pid.last_output, &pan_pid);
			  }
		  }
		  else {
			  if (current_tilt_position != 0xFFFF) {
				  telemetry_emit(pid_tick, (float)target_tilt_position, current_tilt_position,
			                 (float)target_tilt_position - current_tilt_position,
			                 tilt_pid.last_output, &tilt_pid);
			  }
		  }


		  // ----- Send updated current pan and tilt position back to the pi ------
		  int len = snprintf(tx_buf, sizeof(tx_buf), "p%.2ft%.2f\n", current_pan_position  - boot_pan_pos, current_tilt_position - boot_tilt_pos);
		  HAL_UART_Transmit_IT(&huart3, (uint8_t *)tx_buf, len);

	      pid_tick++;
	      flag = 0;
	  }

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 6249;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|AI_2_Pin|AI_1_Pin|BI_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BI_1_GPIO_Port, BI_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin AI_2_Pin AI_1_Pin BI_2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|AI_2_Pin|AI_1_Pin|BI_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BI_1_Pin */
  GPIO_InitStruct.Pin = BI_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BI_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STBY_Pin */
  GPIO_InitStruct.Pin = STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STBY_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


// ------ Takes UART command from PI and parses into target motor positions ------
void parse (char *buffer) {

	char pan_buf[MAX_FIELD + 1]; // + 1 for the null character
	uint8_t i = 0;

	while (buffer[i] != ' ' && i < MAX_FIELD && buffer[i] != '\0') { // command protocol from the pi should be of the form "pxx txx"
		pan_buf[i] = buffer[i];
		i++;
	}
	pan_buf[i] = '\0';
	target_pan_position = atoi(&pan_buf[1]) + boot_pan_pos;


	if(target_pan_position > pan_angle_max) {
		target_pan_position = pan_angle_max;
	}

	if(target_pan_position < pan_angle_min) {
		target_pan_position = pan_angle_min;
	}

	if (buffer[++i] == 't') {
		char *tilt_buf = &(buffer[i + 1]); // i.e. at this point buffer[i] = t15
		target_tilt_position = atoi(tilt_buf) + boot_tilt_pos;
	}

	if(target_tilt_position > tilt_angle_max) {
		target_tilt_position = tilt_angle_max;
	}

	if(target_tilt_position < tilt_angle_min) {
		target_tilt_position = tilt_angle_min;
	}
}



// ------ Takes each byte one at a time over UART from the pi and stores it in the rx_buf ------
// ------           Overwrites weak ISR function (so dont need a prototype at top)          ------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3) {
		if (rxByte == '\r' || rxByte == '\n') {
		    rx_buff[rxIndex] = '\0';
		    rxIndex = 0;
		    cmdReady = 1;
		} else if (rxIndex < RX_BUF_SIZE - 1) {
		    rx_buff[rxIndex] = rxByte;
		    rxIndex++;
		}
	}
	HAL_UART_Receive_IT(&huart3, &rxByte, 1);
}

// ------ Flag sets frequency of PID loop ------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        // fires at 160 Hz
    	flag = 1;
    }
}


// ------ Read current position from encoder ------
#define MT6701_I2C_ADDR   (0x06 << 1)
#define MT6701_ANGLE_REG  0x03

float read_angle(I2C_HandleTypeDef *hi2c)
{
    uint8_t data[2] = {0};

    if (HAL_I2C_Mem_Read(hi2c, MT6701_I2C_ADDR, MT6701_ANGLE_REG,
                         I2C_MEMADD_SIZE_8BIT, data, 2, 100) != HAL_OK)
    {
        return 0xFFFF;
    }
    uint16_t raw = ((uint16_t)data[0] << 6) | (data[1] >> 2); // 14-bit raw encoder output
    return (raw * 360.0) / 16384.0; // returns angle in degrees
}



void i2c_recover(PID_t *pid, GPIO GPIO_1, GPIO GPIO_2, I2C_HandleTypeDef *hi2c)
{
	char err[] = "I2C read failed\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t*)err, sizeof(err) - 1, HAL_MAX_DELAY);

	pid -> fail_count++;

	pid->integral = 0;
	pid -> reseed = 1;
	pid->last_output = 0.0f;
	pid-> stall_count = 0;
	pid -> in_band = 1;
	HAL_GPIO_WritePin(GPIO_1.port, GPIO_1.pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO_2.port, GPIO_2.pin, GPIO_PIN_SET);


	if (pid->fail_count >= I2C_FAIL_LIMIT) {

	    char msg[] = "I2C recovering...\r\n";
	    HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg) - 1, HAL_MAX_DELAY);

	    HAL_I2C_DeInit(hi2c);
	    HAL_Delay(2);

		if (hi2c->Instance == I2C3) {
			MX_I2C3_Init();
		} else if(hi2c->Instance == I2C1){
			MX_I2C1_Init();
		}
		pid->fail_count = 0;
	}
}



// Blocks until a valid reading (0..360). Driver stays disabled meanwhile.
static float boot_read_angle(I2C_HandleTypeDef *hi2c)
{
    float a = read_angle(hi2c);
    uint32_t t0 = HAL_GetTick();
    while (a > 360.0f) {                 // 65535 sentinel or any garbage
        HAL_Delay(5);
        a = read_angle(hi2c);
        if (HAL_GetTick() - t0 > 3000) { // encoder never answered
            char msg[] = "BOOT: encoder dead, motors disabled\r\n";
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg)-1, HAL_MAX_DELAY);
            HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
            while (1) { }                // hang safe rather than slam
        }
    }
    return a;
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
