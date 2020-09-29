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
#include "pid.h"
#include "I2Cdev.h"
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
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
BMP280_HandleTypedef bmp280;
uint8_t buffer[200];
uint16_t strSize;
uint8_t IMUBuffer[16];

bool GPSDataStatus;
bool IMUDataStatus;
int dutyCycle = ESC_PWM_MIN;
float pressureRef = 0;
bool bme280p;
bool RisingEdge = 1, FallingEdge = 0;
int16_t x_heading, y_heading, z_heading;
float altitude;
char latitude[20];
char longitude[20];

uint8_t gps_rx_buf[GPS_BUF_SIZE];
uint8_t GPSBuffer[GPS_BUF_SIZE];

uint32_t w_output[4];

Kalman_t kalman_altitude;

char lat[20];
char lat_a;
char lon[20];
char lon_a;

PWM_DATA RC_CH1, RC_CH2, RC_CH3, RC_CH4, RC_CH5, RC_CH6;

IMU_DATA IMU_Data;
float pitchRef = 0, yawRef = 0, rollRef = 0;
FLY_MODE fly_mode;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
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
static void MX_TIM7_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

void ESCInit();
void setPWM(TIM_HandleTypeDef htim, uint32_t channel, uint32_t dutyCycle);
void BMPInit();
void IMUInit();
void GPSInit();
void RemoteInit();
void CompassInit();
void setPWM_DATA(PWM_DATA* pwm_data);
void initPWM_DATA(PWM_DATA* pwm_data, TIM_HandleTypeDef *htim, uint32_t channel);
void getIMUData(IMU_DATA *IMU_Data);
void getGPSLatLon();
void getCompass();
void getBMPAltitude();
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
  MX_TIM7_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  /*PIDControl(&PIDRoll, (float)IMU_Data->ROLL, RC_CH2.DutyCycleVal);
  PIDControl(&PIDPitch, (float)IMU_Data->PITCH, RC_CH1.DutyCycleVal);
  PIDControl(&PIDYaw, (float)IMU_Data->YAW, RC_CH4.DutyCycleVal);*/

  //init sensor

  CompassInit();
  BMPInit();
  GPSInit();
  IMUInit();

  //Remote init
  RemoteInit();

  //Inisialisasi PID
  //ROLL
  PIDInit(&PIDRoll, 0.0f, 0.0f, 0.0f, 0.01); //kp = 1, kd = 1, ki = 1, timesampling = 0.01

  //PITCH
  PIDInit(&PIDPitch, 0.0f, 0.0f, 0.0f, 0.01); //kp = 1, kd = 1, ki = 1, timesampling = 0.01

  //YAW
  PIDInit(&PIDYaw, 0.0f, 0.0f, 0.0f, 0.01); //kp = 1, kd = 1, ki = 1, timesampling = 0.01

  strSize = sprintf((char*)buffer, "Mulai\r\n");
  HAL_UART_Transmit(&huart1, buffer, strSize, 10);
  ESCInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
	  if(inputFlyMode >= 1000 && inputFlyMode <= 1050 && fly_mode != FLY_MODE_OFF){
		  HAL_TIM_Base_Stop_IT(&htim7);
		  fly_mode = FLY_MODE_OFF;
		  PIDReset(&PIDRoll);
		  PIDReset(&PIDPitch);
		  PIDReset(&PIDYaw);
		  pulseESC1 = pulseESC2 = pulseESC3 = pulseESC4 = 1000;
		  strSize = sprintf((char*)buffer, "Wahana Mode Off\r\n");
		  HAL_UART_Transmit(&huart1, buffer, strSize, 10);
	  } else if(inputFlyMode >= 1450 && inputFlyMode <= 1550 && fly_mode != FLY_MODE_ON){
		  fly_mode = FLY_MODE_ON;
		  strSize = sprintf((char*)buffer, "Wahana Mode On\r\n");
		  HAL_UART_Transmit(&huart1, buffer, strSize, 10);
		  HAL_TIM_Base_Start_IT(&htim7);
	  } else if(inputFlyMode >= 1900 && inputFlyMode <= 2000 && fly_mode != FLY_MODE_HOLD){
		  fly_mode = FLY_MODE_HOLD;
		  strSize = sprintf((char*)buffer, "Wahana Mode Hold\r\n");
		  HAL_UART_Transmit(&huart1, buffer, strSize, 10);
	  }

	  //strSize = sprintf((char*)buffer, "ESC1: %d\tESC2: %d\tESC3: %d\tESC4: %d\r\n", pulseESC1, pulseESC2, pulseESC3, pulseESC4);
	  //HAL_UART_Transmit(&huart1, buffer, strSize, 100);

	  //strSize = sprintf((char*)buffer, "YAW: %.2f\tPitch: %.2f\tRoll: %.2f\tThrottle: %d\r\n", inputYaw, inputPitch, inputRoll, inputThrottle);
	  //HAL_UART_Transmit(&huart1, buffer, strSize, 100);

	  //strSize = sprintf((char*)buffer, "YAW: %f\tPITCH: %f\tROLL: %f\r\n", sensorYaw, sensorPitch, sensorRoll);
	  //HAL_UART_Transmit(&huart1, buffer, strSize, 10);

	  strSize = sprintf((char*)buffer, "CH1: %lu, CH2: %lu, CH3: %lu, CH4: %lu, CH5: %lu, CH6: %lu\r\n",
			  RC_CH1.DutyCycleVal, RC_CH2.DutyCycleVal, RC_CH3.DutyCycleVal, RC_CH4.DutyCycleVal, RC_CH5.DutyCycleVal, RC_CH6.DutyCycleVal);
	  HAL_UART_Transmit(&huart1, buffer, strSize, 100);

	  getIMUData(&IMU_Data);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  htim1.Init.Prescaler = 42 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF;
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
  htim2.Init.Prescaler = 42 - 1;
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
  sConfigOC.Pulse = 2000 - 1;
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
  htim3.Init.Prescaler = 42 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
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
  htim4.Init.Prescaler = 42 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF;
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
  htim5.Init.Prescaler = 42 - 1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFF;
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
  htim6.Init.Prescaler = 42 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xFFFF;
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 42 - 1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10000 - 1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  htim9.Init.Prescaler = 42 - 1;
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
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : RC_CH6_Pin */
  GPIO_InitStruct.Pin = RC_CH6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(RC_CH6_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */



void initPWM_DATA(PWM_DATA* pwm_data, TIM_HandleTypeDef *htim, uint32_t channel){
	pwm_data->onFallingEdge = false;
	pwm_data->onRisingEdge = true;
	pwm_data->channel = channel;
	pwm_data->htim = htim;
}

void setPWM(TIM_HandleTypeDef htim, uint32_t channel, uint32_t dutyCycle){
	__HAL_TIM_SET_COMPARE(&htim, channel, dutyCycle);
}

void ESCInit(){
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	setPWM(htim2, TIM_CHANNEL_1, ESC_PWM_MAX);
	setPWM(htim2, TIM_CHANNEL_2, ESC_PWM_MAX);
	setPWM(htim2, TIM_CHANNEL_3, ESC_PWM_MAX);
	setPWM(htim2, TIM_CHANNEL_4, ESC_PWM_MAX);
	HAL_Delay(2000);
	setPWM(htim2, TIM_CHANNEL_1, ESC_PWM_MIN);
	setPWM(htim2, TIM_CHANNEL_2, ESC_PWM_MIN);
	setPWM(htim2, TIM_CHANNEL_3, ESC_PWM_MIN);
	setPWM(htim2, TIM_CHANNEL_4, ESC_PWM_MIN);
	HAL_Delay(2000);
}

void BMPInit(){
	kalman_init(&kalman_altitude, 0.1, 0.1, 0.03);
	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c1;

	while(!bmp280_init(&bmp280, &bmp280.params)){
		strSize = sprintf((char*)buffer, "BMP280 initialization failed\r\n");
		HAL_UART_Transmit(&huart1, buffer, strSize, 1000);
		HAL_Delay(50);
	}
	HAL_Delay(1000);
	bme280p = bmp280.id == BME280_CHIP_ID;
	strSize = sprintf((char*)buffer, "BMP280: found %s\r\n", bme280p ? "BME280" : "BMP280");
	HAL_UART_Transmit(&huart1, buffer, strSize, 1000);

	strSize = sprintf((char*)buffer, "Calibrating.\r\n");
	HAL_UART_Transmit(&huart1, buffer, strSize, 10);

	float pres_total = 0;
	float pressure, temperature, humidity;

	for(int i = 0; i < 100; ++i){
		while(bmp280_is_measuring(&bmp280)) continue;
		bmp280_read_float(&bmp280, &temperature, &pressure, &humidity);
		HAL_UART_Transmit(&huart1, (uint8_t*)".", 1, 10);
		pres_total = pres_total + pressure;
	}

	pressureRef = pres_total / 100;
	strSize = sprintf((char*)buffer,"Done!\r\n");
	HAL_UART_Transmit(&huart1, buffer, strSize, 10);
	HAL_Delay(1000);
}

void GPSInit(){
	  //__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

	  if(HAL_UART_Receive_DMA(&huart3, gps_rx_buf, GPS_BUF_SIZE) != HAL_OK){
		  Error_Handler();
	  }

	  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);

	  strSize = sprintf((char*)buffer, "GPS Done!\r\n");
	  HAL_UART_Transmit(&huart1, buffer, strSize, 10);
}

void CompassInit(){
	I2Cdev_init(&hi2c3);

	HMC5883L_initialize();
	while(!HMC5883L_testConnection()){
		strSize = sprintf((char*)buffer, "Inisialisasi HMC5883L gagal!\r\n");
		HAL_UART_Transmit(&huart1, buffer, strSize, 10);
		HAL_Delay(10);
	}
	strSize = sprintf((char*)buffer, "HMC5883L Sukses!\r\n");
	HAL_UART_Transmit(&huart1, buffer, strSize, 10);
}

void RemoteInit(){
	  fly_mode = FLY_MODE_OFF;
	  strSize = sprintf((char*)buffer, "Wahana Mode Off\r\n");
	  HAL_UART_Transmit(&huart1, buffer, strSize, 10);

	  initPWM_DATA(&RC_CH1, &htim3, TIM_CHANNEL_2);
	  initPWM_DATA(&RC_CH2, &htim9, TIM_CHANNEL_2);
	  initPWM_DATA(&RC_CH3, &htim5, TIM_CHANNEL_1);
	  initPWM_DATA(&RC_CH4, &htim3, TIM_CHANNEL_1);
	  initPWM_DATA(&RC_CH5, &htim4, TIM_CHANNEL_1);
	  initPWM_DATA(&RC_CH6, &htim6, 0);

	  HAL_TIM_IC_Start_IT(RC_CH1.htim, RC_CH1.channel);
	  HAL_TIM_IC_Start_IT(RC_CH2.htim, RC_CH2.channel);
	  HAL_TIM_IC_Start_IT(RC_CH3.htim, RC_CH3.channel);
	  HAL_TIM_IC_Start_IT(RC_CH4.htim, RC_CH4.channel);
	  HAL_TIM_IC_Start_IT(RC_CH5.htim, RC_CH5.channel);
	  HAL_TIM_Base_Start(RC_CH6.htim);
}

void IMUInit(){
	  HAL_UART_Transmit(&huart2, (u_char*)0xA5, 1, 10);
	  HAL_UART_Transmit(&huart2, (u_char*)0x54, 1, 10);

	  HAL_Delay(3000);
	  strSize = sprintf((char*)buffer,"Kalibrasi tilt done\r\n");
	  HAL_UART_Transmit(&huart1, buffer, strSize, 100);

	  //Kalibrasi heading
	  HAL_UART_Transmit(&huart2, (u_char*)0xA5, 1, 10);
	  HAL_UART_Transmit(&huart2, (u_char*)0x55, 1, 10);
	  HAL_Delay(3000);

	  strSize = sprintf((char*)buffer,"Kalibrasi heading done\r\n");
	  HAL_UART_Transmit(&huart1, buffer, strSize, 100);

	  //Konfigurasi Output ASCII
	  HAL_UART_Transmit(&huart2, (u_char*)0xA5, 1, 10);
	  HAL_UART_Transmit(&huart2, (u_char*)0x52, 1, 10);
	  HAL_Delay(1000);

	  HAL_UART_Receive_DMA(&huart2, IMUBuffer, 16);

	  float pitchTotal = 0, yawTotal = 0, rollTotal = 0;

	  int i = 0;
	  while(i < 100){
		  if(IMUDataStatus){
			  i += 1;
			  getIMUData(&IMU_Data);
			  pitchTotal += IMU_Data.PITCH;
			  yawTotal += IMU_Data.YAW;
			  rollTotal += IMU_Data.ROLL;
		  } else continue;
	  }
	  pitchRef = pitchTotal / 100;
	  yawRef = yawTotal / 100;
	  rollRef = rollTotal / 100;
}

void getBMPAltitude(){
	if(!bmp280_is_measuring(&bmp280)){
		  float pressure, temperature, humidity;
		  bmp280_read_float(&bmp280, &temperature, &pressure, &humidity);
		  float altitude_reading = bmp280_read_altitude(pressure/100, pressureRef/100);
		  float estimated_altitude = kalman_updateEstimate(&kalman_altitude, altitude_reading);
		  altitude = estimated_altitude;
		  //strSize = sprintf((char*)buffer, "%f\r\n", estimated_altitude);
		  //HAL_UART_Transmit(&huart1, buffer, strSize, 1);
	}
}

void getCompass(){
	  if(HMC5883L_getReadyStatus()){
		  HMC5883L_getHeading(&x_heading, &y_heading, &z_heading);
		  //strSize = sprintf((char*)buffer, "X: %d\tY: %d\tZ: %d\r\n", x, y, z);
		  //HAL_UART_Transmit(&huart1, buffer, strSize, 100);
	  }
}

void getGPSLatLon(){
	  if(GPSDataStatus){
		  char *pointer;
		  int length = sizeof(GPSBuffer);

		  memset(lat, '\0', 20);
		  memset(lon, '\0', 20);
		  pointer = strchr((char*)GPSBuffer, '$');

		  do{
			  char *ptrstart;
			  char *ptrend;

			  if(strncmp(pointer, "$GNGGA" , 6) == 0){ //$GNGGA
				  ptrstart = (char*)memchr(pointer + 1, ',', length);
				  ptrstart = (char*)memchr(ptrstart + 1, ',', length);
				  ptrend = (char*)memchr(ptrstart + 1, ',', length);

			  } else if(strncmp(pointer, "$GNGLL", 6) == 0){ //$GNGLL
				  ptrstart = (char*)memchr(pointer + 1, ',', length);
				  ptrend = (char*)memchr(ptrstart + 1, ',', length);

			  } else if(strncmp(pointer, "$GNRMC", 6) == 0){ //$GNRMC
				  ptrstart = (char*)memchr(pointer + 1, ',', length);
				  ptrstart = (char*)memchr(ptrstart + 1, ',', length);
				  ptrstart = (char*)memchr(ptrstart + 1, ',', length);
				  ptrend = (char*)memchr(ptrstart + 1, ',', length);

			  } else {
				  pointer = strchr(pointer + 6, '$');
				  continue;
			  }

			  for(int i = 1; i < (ptrend - ptrstart); i++) lat[i - 1] = ptrstart[i];
			  lat_a = *(ptrend + 1);

			  ptrstart = (char*)memchr(ptrend + 1, ',', length);
			  ptrend = (char*)memchr(ptrstart + 1, ',', length);

			  for(int i = 1; i < (ptrend - ptrstart); i++) lon[i - 1] = ptrstart[i];
			  lon_a = *(ptrend + 1);
			  if(lon[0] != '\0' && lat[0] != '\0'){
				  //strSize = sprintf((char*)buffer, "Lat: %s | %c\tLon: %s | %c\r\n", lat, lat_a, lon, lon_a);
				  //HAL_UART_Transmit(&huart1, buffer , strSize, 100);
				  break;
			  }
			  pointer = strchr(pointer + 4, '$');
		  }
		  while(pointer != NULL);
		  GPSDataStatus = false;
	  }
}

void getIMUData(IMU_DATA *IMU_Data){
	if(IMUDataStatus){
		  uint8_t YPR[8];
		  IMU_Data->YAW = 1000.0f, IMU_Data->PITCH = 1000.0f, IMU_Data->ROLL = 1000.0f;
		  char* buf;
		  buf = memchr(IMUBuffer, 0xAA, 16);
		  memcpy(YPR, buf, 8);
		  if(YPR[0] == 0xAA && YPR[7] == 0x55){
			  IMU_Data->YAW = (float)((YPR[1] << 8 | YPR[2]) * 0.01f);
			  if(IMU_Data->YAW > 179) IMU_Data->YAW = IMU_Data->YAW - 655;

			  sensorYaw = IMU_Data->YAW - yawRef;

			  IMU_Data->PITCH = (float)((YPR[3] << 8 | YPR[4]) * 0.01f);
			  if(IMU_Data->PITCH > 179) IMU_Data->PITCH = IMU_Data->PITCH - 655;

			  sensorPitch = IMU_Data->PITCH - pitchRef;

			  IMU_Data->ROLL = (float)((YPR[5] << 8 | YPR[6]) * 0.01f);
			  if(IMU_Data->ROLL > 179) IMU_Data->ROLL = IMU_Data->ROLL - 655;

			  sensorRoll = IMU_Data->ROLL - rollRef;

			  //strSize = sprintf((char*)buffer, "Y: %f, P: %f, R: %f\r\n", sensorYaw, sensorPitch, sensorRoll);
			  //HAL_UART_Transmit(&huart1, buffer, strSize, 100);
		  }
		  IMUDataStatus = false;
	}

}

void setPWM_DATA(PWM_DATA* pwm_data){
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
	if(pwm_data->FallingEdgeVal >= pwm_data->RisingEdgeVal){
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
  if(huart->Instance == USART2 && !IMUDataStatus)
	  IMUDataStatus = true;

  if(huart->Instance == USART3){
	  memcpy(GPSBuffer, gps_rx_buf, GPS_BUF_SIZE);
	  GPSDataStatus = true;
  }
 }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM7 && (fly_mode == FLY_MODE_ON || fly_mode == FLY_MODE_HOLD)){
		PIDRoll.timesampling = PIDYaw.timesampling = PIDPitch.timesampling = 0.01;
		PIDControl(&PIDRoll, sensorRoll, inputRoll);
		PIDControl(&PIDPitch, sensorPitch, inputPitch);
		PIDControl(&PIDYaw, sensorYaw, inputYaw);
		trustControl();
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if(htim == RC_CH1.htim){
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
			setPWM_DATA(&RC_CH1);
			inputRoll = map((float)RC_CH1.DutyCycleVal, 1000, 2000, -30, 30);
			inputRoll = constrain(inputRoll, -30, 30);
		} else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			setPWM_DATA(&RC_CH4);
			inputYaw = map((float)RC_CH4.DutyCycleVal, 1000, 2000, -30, 30);
			inputYaw = constrain(inputYaw, -30, 30);
		}
	}
	else if(htim == RC_CH2.htim){
		setPWM_DATA(&RC_CH2);
		inputPitch = map((float)RC_CH2.DutyCycleVal, 1000, 2000, -30, 30);
		inputPitch = constrain(inputPitch, -30, 30);
	}
	else if(htim == RC_CH3.htim) {
		setPWM_DATA(&RC_CH3);
		if(RC_CH3.DutyCycleVal >= 1150){
			inputThrottle = constrain(RC_CH3.DutyCycleVal, 1000, 2000);
		} else {
			RC_CH3.DutyCycleVal = 1000;
			inputThrottle = 1000;
		}
	}
	else if(htim == RC_CH5.htim) {
		setPWM_DATA(&RC_CH5);
		inputFlyMode = constrain(RC_CH5.DutyCycleVal, 1000, 2000);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(RC_CH6.onRisingEdge && !RC_CH6.onFallingEdge){
		RC_CH6.onRisingEdge = false;
		RC_CH6.onFallingEdge = true;
		RC_CH6.RisingEdgeVal = __HAL_TIM_GET_COUNTER(RC_CH6.htim);

	} else if(RC_CH6.onFallingEdge && !RC_CH6.onRisingEdge) {
		RC_CH6.onFallingEdge = false;
		RC_CH6.onRisingEdge =  true;
		RC_CH6.FallingEdgeVal = __HAL_TIM_GET_COUNTER(RC_CH6.htim);
	}
	if(RC_CH6.FallingEdgeVal >= RC_CH6.RisingEdgeVal){
		RC_CH6.DutyCycleVal = RC_CH6.FallingEdgeVal - RC_CH6.RisingEdgeVal;
		RC_CH6.FallingEdgeVal = 0;
		RC_CH6.RisingEdgeVal = 0;
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
