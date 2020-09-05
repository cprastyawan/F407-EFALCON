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
#include <stdbool.h>

#include "bmp280.h"
#include "Kalman.h"
#include "HMC5883L.h"
#include "GlobalVariables.h"
#include "pid.h"
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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim9;

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
uint8_t GPSBuffer[800];
int GPSBufferLength;
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
int x_heading, y_heading, z_heading;
const UART_HandleTypeDef *UART_Telemetry = &huart1;
const UART_HandleTypeDef *UART_IMU = &huart2;
const UART_HandleTypeDef *UART_GPS = &huart4;
uint32_t w_output[4];

HMC5883L_t hmc5883l;

Kalman_t kalman_altitude;

PIDType_t PIDYaw, PIDRoll, PIDPitch;

double inputYAW, inputPITCH, inputROLL;

PWM_DATA RC_CH1, RC_CH2, RC_CH3, RC_CH4, RC_CH5, RC_CH6;
GPS_STRING GPS_String;
IMU_DATA IMU_Data;

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
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void getIMUData(struct IMU_DATA *IMU_Data);
void CalibrateESC();
void setPWM(TIM_HandleTypeDef htim, uint32_t channel, uint32_t dutyCycle);
void BMPInit();
void IMUInit();
void HMC5883LInit();
void setPWM_DATA(struct PWM_DATA* pwm_data);
void initPWM_DATA(struct PWM_DATA* pwm_data, TIM_HandleTypeDef *htim, uint32_t channel);
void kinematic(uint32_t *output, uint32_t input1, uint32_t input2, uint32_t input3, uint32_t input4);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  //Kalibrasi Tilt

  initPWM_DATA(&RC_CH1, &htim3, TIM_CHANNEL_2);
  initPWM_DATA(&RC_CH2, &htim9, TIM_CHANNEL_2);
  initPWM_DATA(&RC_CH3, &htim5, TIM_CHANNEL_1);
  initPWM_DATA(&RC_CH4, &htim3, TIM_CHANNEL_1);
  initPWM_DATA(&RC_CH5, &htim4, TIM_CHANNEL_1);

  //Inisialisasi PID
  //ROLL
  PIDInit(&PIDRoll, 1, 1, 1, 1); //kp = 1, kd = 1, ki = 1, timesampling = 1

  //PITCH
  PIDInit(&PIDPitch, 1, 1, 1, 1); //kp = 1, kd = 1, ki = 1, timesampling = 1

  //YAW
  PIDInit(&PIDYaw,1,1,1,1); //kp = 1, kd = 1, ki = 1, timesampling =1

  PIDControl(&PIDRoll, (float)IMU_Data->ROLL, RC_CH2.DutyCycleVal);
  PIDControl(&PIDPitch, (float)IMU_Data->PITCH, RC_CH1.DutyCycleVal);
  PIDControl(&PIDYaw, (float)IMU_Data->YAW, RC_CH4.DutyCycleVal);

  IMUInit();
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
  huart4Status = HAL_UART_Receive_DMA(&huart4, GPSBuffer, 800);

  kalman_init(&kalman_altitude, 0.1, 0.1, 0.03);
  BMPInit();
  //HMC5883LInit();
  //HAL_UART_Receive_IT(&huart4, GPSBuffer, 10);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  CalibrateESC();
  //HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_IC_Start_IT(RC_CH1.htim, RC_CH1.channel);
  HAL_TIM_IC_Start_IT(RC_CH2.htim, RC_CH2.channel);
  HAL_TIM_IC_Start_IT(RC_CH3.htim, RC_CH3.channel);
  HAL_TIM_IC_Start_IT(RC_CH4.htim, RC_CH4.channel);
  HAL_TIM_IC_Start_IT(RC_CH5.htim, RC_CH5.channel);

  float altitude;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
	  kinematic(w_output, RC_CH1.DutyCycleVal, RC_CH2.DutyCycleVal, RC_CH3.DutyCycleVal, RC_CH4.DutyCycleVal);

	  strSize = sprintf((char*)buffer, "ESC1: %lu\r\n", w_output[0]);
	  HAL_UART_Transmit(&huart1, buffer, strSize, 100);

	  strSize = sprintf((char*)buffer, "ESC2: %lu\r\n", w_output[1]);
	  HAL_UART_Transmit(&huart1, buffer, strSize, 100);

	  strSize = sprintf((char*)buffer, "ESC3: %lu\r\n", w_output[2]);
	  HAL_UART_Transmit(&huart1, buffer, strSize, 100);

	  strSize = sprintf((char*)buffer, "ESC4: %lu\r\n", w_output[3]);
	  HAL_UART_Transmit(&huart1, buffer, strSize, 100);

	  HAL_UART_Transmit(&huart1, "\r\n", 2, 5);

	  HAL_Delay(200);

	  if(!bmp280_is_measuring(&bmp280)){
		  bmp280_read_float(&bmp280, &temperature, &pressure, &humidity);
		  altitude = bmp280_read_altitude(pressure/100, pressureRef/100);
		  float estimated_altitude = kalman_updateEstimate(&kalman_altitude, altitude);
		  strSize = sprintf((char*)buffer, "%f\r\n", estimated_altitude);
		  //HAL_UART_Transmit(&huart1, buffer, strSize, 1);
	  }
	  /*if(HMC5883L_GetReadyStatus(&hmc5883l)){
		  HMC5883L_GetHeading(&hmc5883l, &x_heading, &y_heading, &z_heading);
		  strSize = sprintf((char*)buffer, "x: %d\r\n y: %d\r\n z: %d\r\n", x_heading, y_heading, z_heading);
		  HAL_UART_Transmit(&huart1, buffer, strSize, 30);
	  }*/
	  /*
	  if(dutyCycle > ESC_PWM_MAX) dutyCycle = ESC_PWM_MIN;
	  setPWM(htim3, TIM_CHANNEL_3, dutyCycle);
	  dutyCycle = dutyCycle + 100;*/
	  /*if(IMUDataStatus == 1){
		  getIMUData(&IMU_Data);
		  //strSize = sprintf(buffer, "YAW: %f, PITCH: %f, ROLL: %f\r\n", IMU_Data.YAW, IMU_Data.PITCH, IMU_Data.ROLL);
		  strSize = sprintf(buffer, "%f\r\n", IMU_Data.YAW);
		  HAL_UART_Transmit(&huart1, buffer, strSize, 100);
		  IMUDataStatus = 0;
	  }*/
	  /*if(GPSDataStatus == 1){
		  //int GPSBuffer_len = strlen(GPSBuffer);
		  for(int i = 0; i < GPSBufferLength; i++){
			  if(GPSBuffer[i] == '$') {
				  char str[4] = {GPSBuffer[i+3], GPSBuffer[i+4], GPSBuffer[i+5], '\0'};
				  if(strcmp("GGA", str) == 0){
					  	int length = strchr(&GPSBuffer[i], '\n') - (unsigned)&GPSBuffer[i] + 1;
					  	if(length <= 0) break;
					  	GPS_String.GNGGA = (char*)malloc(length + 1);
						memcpy(GPS_String.GNGGA, &GPSBuffer[i], length);
						GPS_String.GNGGA[length] = '\0';
						i = i + length;
						HAL_UART_Transmit(&huart1, GPS_String.GNGGA, length, 100);
				  }

				  else if(strcmp("GLL", str) == 0){
						int length = strchr(&GPSBuffer[i], '\n') - (unsigned)&GPSBuffer[i] + 1;
					  	if(length <= 0) break;
						GPS_String.GNGLL = (char*)malloc(length + 1);
						memcpy(GPS_String.GNGLL, &GPSBuffer[i], length);
						GPS_String.GNGLL[length] = '\0';
						i = i + length;
						HAL_UART_Transmit(&huart1, GPS_String.GNGLL, length, 100);
				  }

				  else if(strcmp("RMC", str) == 0){
						int length = strchr(&GPSBuffer[i], '\n') - (unsigned)&GPSBuffer[i] + 1;
					  	if(length <= 0) break;
						GPS_String.GNRMC = (char*)malloc(length + 1);
						memcpy(GPS_String.GNRMC, &GPSBuffer[i], length);
						GPS_String.GNRMC[length] = '\0';
						i = i + length;
						HAL_UART_Transmit(&huart1, GPS_String.GNRMC, length, 100);
				  } else continue;
			  }  else continue;
		  }
		  //strSize = sprintf(buffer, "%s\r\n", GPSBuffer);
		  GPSDataStatus = 0;
		  memset(GPSBuffer, 0, GPSBufferLength);
		  GPSBufferLength = 0;
		  HAL_UART_Receive_DMA(&huart4, GPSBuffer, 800);
	  }*/
/*
	  strSize = sprintf((char*)buffer, "RC CH1: %lu us\r\n", RC_CH1.DutyCycleVal);
	  HAL_UART_Transmit(&huart1, buffer, strSize, 100);
	  strSize = sprintf((char*)buffer, "RC CH2: %lu us\r\n", RC_CH2.DutyCycleVal);
	  HAL_UART_Transmit(&huart1, buffer, strSize, 100);
	  strSize = sprintf((char*)buffer, "RC CH3: %lu us\r\n", RC_CH3.DutyCycleVal);
	  HAL_UART_Transmit(&huart1, buffer, strSize, 100);
	  strSize = sprintf((char*)buffer, "RC CH4: %lu us\r\n", RC_CH4.DutyCycleVal);
	  HAL_UART_Transmit(&huart1, buffer, strSize, 100);
	  strSize = sprintf((char*)buffer, "RC CH5: %lu us\r\n", RC_CH5.DutyCycleVal);
	  HAL_UART_Transmit(&huart1, buffer, strSize, 100);
*/
	  //setPWM(htim2, TIM_CHANNEL_4, RC_CH3.DutyCycleVal);

	  //HAL_Delay(500);

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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 16 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000 - 1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF - 1;
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
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
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
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF - 1;
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
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 16 - 1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFF - 1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 16 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000 - 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 16 - 1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 0xFFFF;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim9, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

void kinematic(uint32_t *output, uint32_t input1, uint32_t input2, uint32_t input3, uint32_t input4){
	static double Tr, Tp, Tz, Ty;
	const double KT=1, KQ=1, L=0.23;

	double w1, w2, w3, w4;

	Tr = map((double)input1, 1000, 2000, -10, 10);
	Tp = map((double)input2, 1000, 2000, 10, -10);
	Tz = map((double)input3, 1000, 2000, 1000, 2000);
	Ty = map((double)input4, 1000, 2000, -5, 5);


	  if (Tz == 1000.0f && Tr == 0.0f && Tp == 0.0f && Ty == 0.0f) {
			output[0] = 1000;
			output[1] = 1000;
			output[2] = 1000;
			output[3] = 1000;
	  }
	  else {
			//QUADCOPTER
			w1 = Tz/(4*KT) + Tr/(2*KT*L) + Ty/(2*KQ);
			w2 = Tz/(4*KT) + Tp/(2*KT*L) - Ty/(2*KQ);
			w3 = Tz/(4*KT) - Tp/(2*KT*L) - Ty/(2*KQ);
			w4 = Tz/(4*KT) - Tr/(2*KT*L) + Ty/(2*KQ);

			output[0] = (uint32_t)map(w1, 250, 500, 1000, 2000);

			output[1] = (uint32_t)map(w2, 250, 500, 1000, 2000);
			output[2] = (uint32_t)map(w3, 250, 500, 1000, 2000);
			output[3] = (uint32_t)map(w4, 250, 500, 1000, 2000);

			output[0] = constrain(output[0], 1000, 2000);
			output[1] = constrain(output[1], 1000, 2000);
			output[2] = constrain(output[2], 1000, 2000);
			output[3] = constrain(output[3], 1000, 2000);
	  }
}

void initPWM_DATA(struct PWM_DATA* pwm_data, TIM_HandleTypeDef *htim, uint32_t channel){
	pwm_data->onFallingEdge = false;
	pwm_data->onRisingEdge = true;
	pwm_data->channel = channel;
	pwm_data->htim = htim;
}

void getIMUData(struct IMU_DATA *IMU_Data){
	  uint8_t YPR[8];
	  IMU_Data->YAW = 1000.0f, IMU_Data->PITCH = 1000.0f, IMU_Data->ROLL = 1000.0f;
	  char* buf;
	  buf = memchr(IMUBuffer, 0xAA, 16);
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
	setPWM(htim2, TIM_CHANNEL_4, ESC_PWM_MAX);
	HAL_Delay(2000);
	setPWM(htim3, TIM_CHANNEL_4, ESC_PWM_MIN);
	HAL_Delay(2000);
}

void HMC5883LInit(){
	HMC5883L_set_default_params(&hmc5883l);
	hmc5883l.hi2c = &hi2c3;

	HMC5883L_Initialize(&hmc5883l);
	HAL_Delay(1000);
	while(!HMC5883L_TestConnection(&hmc5883l)){
		strSize = sprintf((char*)buffer, "HMC5883L Gagal!\r\n");
		HAL_UART_Transmit(&huart1, buffer, strSize, 20);
		HMC5883L_Initialize(&hmc5883l);
		HAL_Delay(500);
	}

	strSize = sprintf((char*)buffer, "Koneksi HMC5883L Sukses!\r\n");
	HAL_UART_Transmit(&huart1, buffer, strSize, 20);
}

void BMPInit(){
	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c1;

	while(!bmp280_init(&bmp280, &bmp280.params)){
		strSize = sprintf((char*)buffer, "BMP280 initialization failed\r\n");
		HAL_UART_Transmit(&huart1, buffer, strSize, 1000);
	}
	HAL_Delay(1000);
	bme280p = bmp280.id == BME280_CHIP_ID;
	strSize = sprintf((char*)buffer, "BMP280: found %s\r\n", bme280p ? "BME280" : "BMP280");
	HAL_UART_Transmit(&huart1, buffer, strSize, 1000);

	strSize = sprintf((char*)buffer, "Calibrating.\r\n");
	HAL_UART_Transmit(&huart1, buffer, strSize, 10);

	float pres_total = 0;

	for(int i = 0; i < 100; ++i){
		while(bmp280_is_measuring(&bmp280)){
			continue;
		}
		bmp280_read_float(&bmp280, &temperature, &pressure, &humidity);
		HAL_UART_Transmit(&huart1, ".", 1, 10);
		pres_total = pres_total + pressure;
	}

	pressureRef = pres_total / 100;
	strSize = sprintf((char*)buffer,"Done!\r\n");
	HAL_UART_Transmit(&huart1, buffer, strSize, 10);
	HAL_Delay(2000);
}

void IMUInit(){
	  HAL_Delay(1000);
	  huart2Status = HAL_UART_Transmit(&huart2, (u_char*)0xA5, 1, 10);
	  huart2Status = HAL_UART_Transmit(&huart2, (u_char*)0x54, 1, 10);

	  HAL_Delay(3000);
	  strSize = sprintf((char*)buffer,"Kalibrasi tilt done\r\n");
	  huart3Status = HAL_UART_Transmit(&huart1, buffer, strSize, 100);

	  //Kalibrasi heading
	  HAL_Delay(1000);
	  huart2Status = HAL_UART_Transmit(&huart2, (u_char*)0xA5, 1, 10);
	  huart2Status = HAL_UART_Transmit(&huart2, (u_char*)0x55, 1, 10);

	  strSize = sprintf((char*)buffer,"Kalibrasi heading done\r\n");
	  huart3Status = HAL_UART_Transmit(&huart1, buffer, strSize, 100);

	  //Konfigurasi Output ASCII
	  HAL_Delay(100);
	  huart2Status = HAL_UART_Transmit(&huart2, (u_char*)0xA5, 1, 10);
	  huart2Status = HAL_UART_Transmit(&huart2, (u_char*)0x52, 1, 10);

	  huart2Status = HAL_UART_Receive_DMA(&huart2, IMUBuffer, 16);
}

void setPWM_DATA(struct PWM_DATA* pwm_data){
	if(pwm_data->onRisingEdge && !pwm_data->onFallingEdge){
		pwm_data->onRisingEdge = false;
		pwm_data->onFallingEdge = true;
		pwm_data->RisingEdgeVal = HAL_TIM_ReadCapturedValue(pwm_data->htim, pwm_data->channel);
		__HAL_TIM_SET_CAPTUREPOLARITY(pwm_data->htim, pwm_data->channel, TIM_INPUTCHANNELPOLARITY_FALLING);

	} else if(pwm_data->onFallingEdge && !pwm_data->onRisingEdge) {
		pwm_data->onFallingEdge = false;
		pwm_data->onRisingEdge =  true;
		pwm_data->FallingEdgeVal = HAL_TIM_ReadCapturedValue(pwm_data->htim, pwm_data->channel);
		__HAL_TIM_SET_CAPTUREPOLARITY(pwm_data->htim, pwm_data->channel, TIM_INPUTCHANNELPOLARITY_RISING);
	}
	if(pwm_data->FallingEdgeVal > pwm_data->RisingEdgeVal){
		pwm_data->DutyCycleVal = pwm_data->FallingEdgeVal - pwm_data->RisingEdgeVal;
		pwm_data->FallingEdgeVal = 0;
		pwm_data->RisingEdgeVal = 0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
  //HAL_UART_Transmit(&huart3, RxBuffer, 8, 100);
  if(huart->Instance == USART2 && IMUDataStatus == 0)
	  IMUDataStatus = 1;

  else if(huart->Instance == UART4 && GPSDataStatus == 0){
	  if(__HAL_UART_GET_FLAG (&huart4, UART_FLAG_IDLE)){
			  GPSBufferLength = 1000 - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);
		  	  GPSDataStatus = 1;
			  HAL_UART_DMAStop(&huart4);
	  }
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if(htim == RC_CH1.htim){
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
			setPWM_DATA(&RC_CH1);
		} else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			setPWM_DATA(&RC_CH4);
		}
	}
	else if(htim == RC_CH2.htim){
		setPWM_DATA(&RC_CH2);
	}
	else if(htim == RC_CH3.htim) {
		setPWM_DATA(&RC_CH3);
	}
	else if(htim == RC_CH4.htim) {
		setPWM_DATA(&RC_CH4);
	}
	else if(htim == RC_CH5.htim) {
		setPWM_DATA(&RC_CH5);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM6){
/*		  while(!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)){
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
			  }*/
		bmp280_read_float(&bmp280, &temperature, &pressure, &humidity);
		strSize = sprintf((char*)buffer, "%f\r\n", pressure);
		//HAL_UART_Transmit(&huart3, buffer, strSize, 1);
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
