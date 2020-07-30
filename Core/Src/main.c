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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <bmp280.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ESC_PWM_MIN 1000
#define ESC_PWM_MAX 2000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
BMP280_HandleTypedef bmp280;
uint8_t buffer[200];
uint16_t strSize;
uint8_t IMUBuffer[16];
uint8_t GPSBuffer[300];
static char GPSDataStatus = 0;
static char IMUDataStatus = 0;
HAL_StatusTypeDef huart2Status;
HAL_StatusTypeDef huart3Status;
HAL_StatusTypeDef huart4Status;
int dutyCycle = ESC_PWM_MIN;
float pressureRef = 0;
bool bme280p;
bool RisingEdge = 1, FallingEdge = 0;
float pressure, temperature, humidity;

struct PWM_DATA PWM_CH1, PWM_CH2, PWM_CH3, PWM_CH4;
struct GPS_STRING GPS_String;
struct IMU_DATA IMU_Data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void getIMUData(struct IMU_DATA *IMU_Data);
void CalibrateESC();
void setPWM(TIM_HandleTypeDef htim, uint32_t channel, uint32_t dutyCycle);
void BMPInit();
void IMUInit();
void setPWM_DATA(TIM_HandleTypeDef *htim, uint32_t channel, struct PWM_DATA* PWM_Data);
void initPWM_DATA(struct PWM_DATA* PWM_Data);
uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
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
  initPWM_DATA(&PWM_CH1);
  initPWM_DATA(&PWM_CH2);
  initPWM_DATA(&PWM_CH3);
  initPWM_DATA(&PWM_CH4);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  //Kalibrasi Tilt

  IMUInit();
  huart4Status = HAL_UART_Receive_DMA(&huart4, GPSBuffer, 300);
  BMPInit();
	//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  CalibrateESC();
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
	  if(dutyCycle > ESC_PWM_MAX) dutyCycle = ESC_PWM_MIN;
	  setPWM(htim3, TIM_CHANNEL_3, dutyCycle);
	  dutyCycle = dutyCycle + 100;
	  if(IMUDataStatus == 1){
		  getIMUData(&IMU_Data);
		  strSize = sprintf(buffer, "YAW: %f, PITCH: %f, ROLL: %f\r\n", IMU_Data.YAW, IMU_Data.PITCH, IMU_Data.ROLL);
		  HAL_UART_Transmit(&huart3, buffer, strSize, 100);
		  IMUDataStatus = 0;
	  }

	  if(GPSDataStatus == 1){
		  int GPSBuffer_len = strlen(GPSBuffer);
		  for(int i = 0; i < GPSBuffer_len; i++){
			  if(GPSBuffer[i] == '$') {
				  char str[4] = {GPSBuffer[i+3], GPSBuffer[i+4], GPSBuffer[i+5], '\0'};
				  if(strcmp("GGA", str) == 0){
					  	int length = strchr(&GPSBuffer[i], '\n') - (unsigned)&GPSBuffer[i] + 1;
					  	if(length <= 0) break;
					  	GPS_String.GNGGA = (char*)malloc(length);
						memcpy(GPS_String.GNGGA, &GPSBuffer[i], length);
						GPS_String.GNGGA[length] = '\0';
						i = i + length;
						HAL_UART_Transmit(&huart3, GPS_String.GNGGA, length, 100);
				  }

				  else if(strcmp("GLL", str) == 0){
						int length = strchr(&GPSBuffer[i], '\n') - (unsigned)&GPSBuffer[i] + 1;
					  	if(length <= 0) break;
						GPS_String.GNGLL = (char*)malloc(length);
						memcpy(GPS_String.GNGLL, &GPSBuffer[i], length);
						GPS_String.GNGLL[length] = '\0';
						i = i + length;
						HAL_UART_Transmit(&huart3, GPS_String.GNGLL, length, 100);
				  }

				  else if(strcmp("RMC", str) == 0){
						int length = strchr(&GPSBuffer[i], '\n') - (unsigned)&GPSBuffer[i] + 1;
					  	if(length <= 0) break;
						GPS_String.GNRMC = (char*)malloc(length);
						memcpy(GPS_String.GNRMC, &GPSBuffer[i], length);
						GPS_String.GNRMC[length] = '\0';
						i = i + length;
						HAL_UART_Transmit(&huart3, GPS_String.GNRMC, length, 100);
				  } else continue;
			  }  else continue;
		  }
		  //strSize = sprintf(buffer, "%s\r\n", GPSBuffer);
		  GPSDataStatus = 0;
	  }

	  strSize = sprintf((char*)buffer, "TIM2 CH1: %lu us\r\n", PWM_CH1.DutyCycleVal); //ROLL
	  HAL_UART_Transmit(&huart3, buffer, strSize, 100);
	  strSize = sprintf((char*)buffer, "TIM2 CH2: %lu us\r\n", PWM_CH2.DutyCycleVal); //YAW
	  HAL_UART_Transmit(&huart3, buffer, strSize, 100);
	  strSize = sprintf((char*)buffer, "TIM2 CH3: %lu us\r\n", PWM_CH3.DutyCycleVal); //THROTTLE
	  HAL_UART_Transmit(&huart3, buffer, strSize, 100);
	  strSize = sprintf((char*)buffer, "TIM2 CH4: %lu us\r\n", PWM_CH4.DutyCycleVal); //PITCH
	  HAL_UART_Transmit(&huart3, buffer, strSize, 100);
	  HAL_Delay(500);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF - 1;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000 - 1;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2000 - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
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
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void initPWM_DATA(struct PWM_DATA* PWM_Data){
	PWM_Data->onFallingEdge = false;
	PWM_Data->onRisingEdge = true;
}

void getIMUData(struct IMU_DATA *IMU_Data){
	  uint8_t YPR[8];
	  IMU_Data->YAW = 1000.0f, IMU_Data->PITCH = 1000.0f, IMU_Data->ROLL = 1000.0f;
	  char* buf;
	  buf = strchr(IMUBuffer, 0xAA);
	  memcpy(YPR, buf, 8);
	  if(YPR[0] == 0xAA && YPR[7] == 0x55){
		  IMU_Data->YAW = (YPR[1] << 8 | YPR[2]) * 0.01f;
		  if(IMU_Data->YAW > 179) IMU_Data->YAW = IMU_Data->YAW - 655;

		  IMU_Data->PITCH = (YPR[3] << 8 | YPR[4]) * 0.01f;
		  if(IMU_Data->PITCH > 179) IMU_Data->PITCH = IMU_Data->PITCH - 655;

		  IMU_Data->ROLL = (YPR[5] << 8 | YPR[6]) * 0.01f;
		  if(IMU_Data->ROLL > 179) IMU_Data->ROLL = IMU_Data->ROLL - 655;
	  }
	  //strSize = sprintf(buffer, "YAW: %f, PITCH: %f, ROLL: %f\r\n", IMU_Data.YAW, IMU_Data.PITCH, IMU_Data.ROLL);
	 //HAL_UART_Transmit(&huart3, buffer, strSize, 100);
}

void setPWM(TIM_HandleTypeDef htim, uint32_t channel, uint32_t dutyCycle){
	HAL_TIM_PWM_Stop(&htim, channel);
	TIM_OC_InitTypeDef sConfigOC;
	htim.Init.Period = 20000 - 1;
	HAL_TIM_PWM_Init(&htim);
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = dutyCycle;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, channel);

	HAL_TIM_PWM_Start(&htim, channel);
}

void CalibrateESC(){
	setPWM(htim3, TIM_CHANNEL_3, ESC_PWM_MAX);
	HAL_Delay(2000);
	setPWM(htim3, TIM_CHANNEL_3, ESC_PWM_MIN);
	HAL_Delay(2000);
}

void BMPInit(){
	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c1;

	while(!bmp280_init(&bmp280, &bmp280.params)){
		strSize = sprintf((char*)buffer, "BMP280 initialization failed\r\n");
		HAL_UART_Transmit(&huart3, buffer, strSize, 1000);
	}
	HAL_Delay(1000);
	bme280p = bmp280.id == BME280_CHIP_ID;
	strSize = sprintf((char*)buffer, "BMP280: found %s\r\n", bme280p ? "BME280" : "BMP280");
	HAL_UART_Transmit(&huart3, buffer, strSize, 1000);

	strSize = sprintf((char*)buffer, "Calibrating.");
	HAL_UART_Transmit(&huart3, buffer, strSize, 10);


	bmp280_read_float(&bmp280, &temperature, &pressure, &humidity);
	pressureRef = pressure;


}

void IMUInit(){
	huart2Status = HAL_UART_Transmit(&huart2, (u_char*)0xA5, 1, 10);
	huart2Status = HAL_UART_Transmit(&huart2, (u_char*)0x54, 1, 10);

	  HAL_Delay(3000);
	  strSize = sprintf((char*)buffer,"Kalibrasi tilt done\r\n");
	  huart3Status = HAL_UART_Transmit(&huart3, buffer, strSize, 100);

	  //Kalibrasi heading
	  HAL_Delay(1000);
	  huart2Status = HAL_UART_Transmit(&huart2, (u_char*)0xA5, 1, 10);
	  huart2Status = HAL_UART_Transmit(&huart2, (u_char*)0x55, 1, 10);

	  strSize = sprintf((char*)buffer,"Kalibrasi heading done\r\n");
	  huart3Status = HAL_UART_Transmit(&huart3, buffer, strSize, 100);

	  //Konfigurasi Output ASCII
	  HAL_Delay(100);
	  huart2Status = HAL_UART_Transmit(&huart2, (u_char*)0xA5, 1, 10);
	  huart2Status = HAL_UART_Transmit(&huart2, (u_char*)0x52, 1, 10);

	  huart2Status = HAL_UART_Receive_DMA(&huart2, IMUBuffer, 16);
}

void setPWM_DATA(TIM_HandleTypeDef *htim, uint32_t channel, struct PWM_DATA* PWM_Data){
	if(PWM_Data->onRisingEdge && !PWM_Data->onFallingEdge){
		PWM_Data->onRisingEdge = false;
		PWM_Data->onFallingEdge = true;
		PWM_Data->RisingEdgeVal = HAL_TIM_ReadCapturedValue(htim, channel);
		__HAL_TIM_SET_CAPTUREPOLARITY(htim, channel, TIM_INPUTCHANNELPOLARITY_FALLING);

	} else if(PWM_Data->onFallingEdge && !PWM_Data->onRisingEdge) {
		PWM_Data->onFallingEdge = false;
		PWM_Data->onRisingEdge =  true;
		PWM_Data->FallingEdgeVal = HAL_TIM_ReadCapturedValue(htim, channel);
		__HAL_TIM_SET_CAPTUREPOLARITY(htim, channel, TIM_INPUTCHANNELPOLARITY_RISING);
	}

	if(PWM_Data->FallingEdgeVal > PWM_Data->RisingEdgeVal){
		PWM_Data->DutyCycleVal = PWM_Data->FallingEdgeVal - PWM_Data->RisingEdgeVal;
		PWM_Data->FallingEdgeVal = 0;
		PWM_Data->RisingEdgeVal = 0;
	}
}

uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
  //HAL_UART_Transmit(&huart3, RxBuffer, 8, 100);
  if(huart->Instance == USART2 && IMUDataStatus == 0) IMUDataStatus = 1;
  else if(huart->Instance == UART4 && GPSDataStatus == 0){
	  GPSDataStatus = 1;
  }

}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			setPWM_DATA(htim, TIM_CHANNEL_1, &PWM_CH1);
		} else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
			setPWM_DATA(htim, TIM_CHANNEL_2, &PWM_CH2);

		} else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
			setPWM_DATA(htim, TIM_CHANNEL_3, &PWM_CH3);
			//uint32_t input = map(PWM_CH3.DutyCycleVal, 980, 1960, 1000, 2000);
			//setPWM(htim3, TIM_CHANNEL_3, PWM_CH3.DutyCycleVal + 20);

		} else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
			setPWM_DATA(htim, TIM_CHANNEL_4, &PWM_CH4);
		}
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM4){
		  while(!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)){
				  strSize = sprintf((char*)buffer, "Temperature/pressure reading failed\r\n");
				  HAL_UART_Transmit(&huart3, buffer, strSize, 100);
			  }

			  float altitude = bmp280_read_altitude(pressure / 100, pressureRef / 100);
			  strSize = sprintf((char*)buffer, "Pressure: %.2f Pa, Temperature: %.2f C, Altitude: %.2f m, ", pressure, temperature, altitude);
			  HAL_UART_Transmit(&huart3, buffer, strSize, 100);

			  if(bme280p){
				  strSize = sprintf((char *)buffer,", Humidity: %.2f\r\n", humidity);
				  HAL_UART_Transmit(&huart3, buffer, strSize, 100);
			  } else {
				  strSize = sprintf((char *)buffer, "\r\n");
				  HAL_UART_Transmit(&huart3, buffer, strSize, 100);
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
