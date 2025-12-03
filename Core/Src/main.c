/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "control.h"
#include "mt6835.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RX_SIZE 16

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for state_machine */
osThreadId_t state_machineHandle;
const osThreadAttr_t state_machine_attributes = {
  .name = "state_machine",
  .stack_size = 96 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for cli */
osThreadId_t cliHandle;
const osThreadAttr_t cli_attributes = {
  .name = "cli",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for lc_posicion */
osThreadId_t lc_posicionHandle;
const osThreadAttr_t lc_posicion_attributes = {
  .name = "lc_posicion",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for lc_corriente */
osThreadId_t lc_corrienteHandle;
const osThreadAttr_t lc_corriente_attributes = {
  .name = "lc_corriente",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* USER CODE BEGIN PV */

xSemaphoreHandle semaforo_consola;
xSemaphoreHandle semaforo_adc;
xSemaphoreHandle semaforo_posicion;
xSemaphoreHandle semaforo_corriente;
xQueueHandle cola_estados;
volatile uint16_t tim1OF = 0;
volatile uint16_t tim3OF = 0;
encoder_t hencoder = { 0 };
estados_e estado_sistema;
uint8_t buffer_rx[RX_SIZE] = {0};
int16_t adc_offsets[2] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
void sm(void *argument);
void consola(void *argument);
void control_posicion(void *argument);
void control_corriente(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  semaforo_consola = xSemaphoreCreateBinary();
  semaforo_adc = xSemaphoreCreateBinary();
  semaforo_posicion = xSemaphoreCreateBinary();
  semaforo_corriente = xSemaphoreCreateBinary();
  cola_estados = xQueueCreate(3, sizeof(mensaje_t));

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* creation of state_machine */
  state_machineHandle = osThreadNew(sm, NULL, &state_machine_attributes);

  /* creation of cli */
  cliHandle = osThreadNew(consola, NULL, &cli_attributes);

  /* creation of lc_posicion */
  lc_posicionHandle = osThreadNew(control_posicion, NULL, &lc_posicion_attributes);

  /* creation of lc_corriente */
  lc_corrienteHandle = osThreadNew(control_corriente, NULL, &lc_corriente_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT_INJECSIMULT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  // Agregado manualmente por mi
//  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 35999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  htim3.Init.Prescaler = 2;
  htim3.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim3.Init.Period = 799;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 150;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TEST_GPIO_Port, TEST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_MT6835_GPIO_Port, CS_MT6835_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|STEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TEST_Pin */
  GPIO_InitStruct.Pin = TEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TEST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_MT6835_Pin */
  GPIO_InitStruct.Pin = CS_MT6835_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_MT6835_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : STEP_Pin */
  GPIO_InitStruct.Pin = STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STEP_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void init_sistema() {
	mensaje_t mensaje = { 0 };
	HAL_StatusTypeDef error;

	// INIT ENCODER
	error = init_encoder(&hspi1, CS_MT6835_GPIO_Port, CS_MT6835_Pin, &hencoder, 600,
						 MT6835_ABZ_NO_SWAP, MT6835_ZRE, MT6835_Z_WIDTH_1LSB, MT6835_Z_ARE,
						 MT6835_CCW_AB);

	if (error != HAL_OK) {
		mensaje.estado = FAULT;
		mensaje.origen = ENCODER;
		mensaje.error = error;
		xQueueSend(cola_estados, &mensaje, 1000);
		return;
	}

	// INIT ADCs
	error = HAL_ADCEx_Calibration_Start(&hadc1);

	if (error != HAL_OK) {
		mensaje.estado = FAULT;
		mensaje.origen = ADC1_CAL;
		mensaje.error = error;
		xQueueSend(cola_estados, &mensaje, 1000);
		return;
	}

	error = HAL_ADCEx_Calibration_Start(&hadc2);

	if (error != HAL_OK) {
		mensaje.estado = FAULT;
		mensaje.origen = ADC2_CAL;
		mensaje.error = error;
		xQueueSend(cola_estados, &mensaje, 1000);
		return;
	}

	error = HAL_ADC_Start(&hadc2);

	if (error != HAL_OK) {
		mensaje.estado = FAULT;
		mensaje.origen = ADC_2;
		mensaje.error = error;
		xQueueSend(cola_estados, &mensaje, 1000);
		return;
	}

	error = HAL_ADCEx_MultiModeStart_DMA(&hadc1, &lecturas_adcs, 1);

	if (error != HAL_OK) {
		mensaje.estado = FAULT;
		mensaje.origen = ADC_MM;
		mensaje.error = error;
		xQueueSend(cola_estados, &mensaje, 1000);
		return;
	}

	// Conversiones injected para ver zero offsets de ADCs
	error = HAL_ADCEx_InjectedStart_IT(&hadc1);

	if (error != HAL_OK) {
		mensaje.estado = FAULT;
		mensaje.origen = ADC1_INJ;
		mensaje.error = error;
		xQueueSend(cola_estados, &mensaje, 1000);
		return;
	}

	error = HAL_ADCEx_InjectedStart_IT(&hadc2);

	if (error != HAL_OK) {
		mensaje.estado = FAULT;
		mensaje.origen = ADC2_INJ;
		mensaje.error = error;
		xQueueSend(cola_estados, &mensaje, 1000);
		return;
	}

	// Timer 1 para lazo de posicion (sin IT todavia)
	error = HAL_TIM_Base_Start(&htim1);

	if (error != HAL_OK) {
		mensaje.estado = FAULT;
		mensaje.origen = TIMER1;
		mensaje.error = error;
		xQueueSend(cola_estados, &mensaje, 1000);
		return;
	}

	// Timer 3 para PWM de las fases
	error = HAL_TIM_Base_Start(&htim3);

	if (error != HAL_OK) {
		mensaje.estado = FAULT;
		mensaje.origen = TIMER3;
		mensaje.error = error;
		xQueueSend(cola_estados, &mensaje, 1000);
		return;
	}

	error = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

	if (error != HAL_OK) {
		mensaje.estado = FAULT;
		mensaje.origen = PWM1;
		mensaje.error = error;
		xQueueSend(cola_estados, &mensaje, 1000);
		return;
	}

	error = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	if (error != HAL_OK) {
		mensaje.estado = FAULT;
		mensaje.origen = PWM2;
		mensaje.error = error;
		xQueueSend(cola_estados, &mensaje, 1000);
		return;
	}

	error = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	if (error != HAL_OK) {
		mensaje.estado = FAULT;
		mensaje.origen = PWM3;
		mensaje.error = error;
		xQueueSend(cola_estados, &mensaje, 1000);
		return;
	}

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);

	// UART con DMA para recibir comandos desde PC
	// Deshabilito interrupcion por half-transmit
	error = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, buffer_rx, RX_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

	if (error != HAL_OK) {
		mensaje.estado = FAULT;
		mensaje.origen = UART_RX;
		mensaje.error = error;
		xQueueSend(cola_estados, &mensaje, 1000);
		return;
	}

	init_controlador(&controlador);
	init_interpolador(&interpolador);

	// Alineo eje d con fase A para hacer 0
	error = alinear_rotor();

	if (error != HAL_OK) {
		mensaje.estado = FAULT;
		// Encoder no devuelve 0x55 luego de enviar comando para hacer zero
		mensaje.origen = ENCODER;
		mensaje.error = error;
		xQueueSend(cola_estados, &mensaje, 1000);
	}

	// Timer 2 para encoder
	error = HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	if (error != HAL_OK) {
		mensaje.estado = FAULT;
		mensaje.origen = TIMER2;
		mensaje.error = error;
		xQueueSend(cola_estados, &mensaje, 1000);
		return;
	}
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
	__HAL_TIM_SET_COUNTER(&htim2, 0);

	// Luego de hacer 0, hago las corrientes 0 para calibrar ADCs
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

	mensaje.estado = IDLE;
	xQueueSend(cola_estados, &mensaje, 1000);
}

void fault_handler(mensaje_t *mensaje) {
	char cadena[64];
	int len;

	switch (mensaje->origen) {
		case CORRIENTES:
			// Deshabilito PWMs e interrupciones para lazos de control
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
			__HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);

			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);

			len = snprintf(cadena, sizeof(cadena), "E: SOBRECORRIENTE\n");
			HAL_UART_Transmit(&huart1, (uint8_t*) cadena, len, 1000);
			break;

		case ENCODER:
			len = snprintf(cadena, sizeof(cadena), "E: Encoder:\n%u\n%u\n%u\n%u\n%u\n%u\n", hencoder.abz_res, hencoder.abz_swap, hencoder.z_edge,
																							hencoder.z_width, hencoder.z_phase, hencoder.rot_dir);
			HAL_UART_Transmit(&huart1, (uint8_t*) cadena, len, 1000);
			break;

		case ADC1_CAL:
			len = snprintf(cadena, sizeof(cadena), "E: ADC1 CAL (%d)\n", mensaje->error);
			HAL_UART_Transmit(&huart1, (uint8_t*) cadena, len, 1000);
			break;

		case ADC2_CAL:
			len = snprintf(cadena, sizeof(cadena), "E: ADC2 CAL (%d)\n", mensaje->error);
			HAL_UART_Transmit(&huart1, (uint8_t*) cadena, len, 1000);
			break;

		case ADC_2:
			len = snprintf(cadena, sizeof(cadena), "E: ADC2 Start (%d)\n", mensaje->error);
			HAL_UART_Transmit(&huart1, (uint8_t*) cadena, len, 1000);
			break;

		case ADC_MM:
			len = snprintf(cadena, sizeof(cadena), "E: ADC MultiMode (%d)\n", mensaje->error);
			HAL_UART_Transmit(&huart1, (uint8_t*) cadena, len, 1000);
			break;

		case ADC1_INJ:
			len = snprintf(cadena, sizeof(cadena), "E: ADC1 injected (%d)\n", mensaje->error);
			HAL_UART_Transmit(&huart1, (uint8_t*) cadena, len, 1000);
			break;

		case ADC2_INJ:
			len = snprintf(cadena, sizeof(cadena), "E: ADC2 injected (%d)\n", mensaje->error);
			HAL_UART_Transmit(&huart1, (uint8_t*) cadena, len, 1000);
			break;

		case TIMER1:
			len = snprintf(cadena, sizeof(cadena), "E: Timer lazo de posicion (%d)\n", mensaje->error);
			HAL_UART_Transmit(&huart1, (uint8_t*) cadena, len, 1000);
			break;

		case TIMER2:
			len = snprintf(cadena, sizeof(cadena), "E: Timer encoder (%d)\n", mensaje->error);
			HAL_UART_Transmit(&huart1, (uint8_t*) cadena, len, 1000);
			break;

		case PWM1:
			len = snprintf(cadena, sizeof(cadena), "E: PWM 1 (%d)\n", mensaje->error);
			HAL_UART_Transmit(&huart1, (uint8_t*) cadena, len, 1000);
			break;

		case PWM2:
			len = snprintf(cadena, sizeof(cadena), "E: PWM 2 (%d)\n", mensaje->error);
			HAL_UART_Transmit(&huart1, (uint8_t*) cadena, len, 1000);
			break;

		case PWM3:
			len = snprintf(cadena, sizeof(cadena), "E: PWM 3 (%d)\n", mensaje->error);
			HAL_UART_Transmit(&huart1, (uint8_t*) cadena, len, 1000);
			break;

		case UART_RX:
			len = snprintf(cadena, sizeof(cadena), "E: UART RX (%d)\n", mensaje->error);
			HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);

		default:
			HAL_UART_Transmit(&huart1, (uint8_t *)"Error desconocido.\n", 19, 1000);
			break;
	}

	mensaje->estado = NOT_INIT;
	xQueueSend(cola_estados, &mensaje, 1000);
}

void get_adc_offsets() {
	if (estado_sistema != IDLE) {
		HAL_UART_Transmit(&huart1, (uint8_t *)"Pasar a estado IDLE\n", 20, 1000);
		return;
	}
	HAL_TIM_Base_Start(&htim3);
	// Filtro de media movil para eliminar componente de 50Hz
	uint32_t sum_adc1 = 0, sum_adc2 = 0;
	uint16_t avg_adc1, avg_adc2;
	for (int j = 0; j < 512; j++) {
		sum_adc1 += (uint16_t)((lecturas_adcs & 0xFFFF));
		sum_adc2 += (uint16_t)(((lecturas_adcs >> 16) & 0xFFFF));
	}

	// >> 9 es igual a / 512
	avg_adc1 = sum_adc1 >> 9;
	avg_adc2 = sum_adc2 >> 9;

	adc_offsets[0] = 2048 - avg_adc1;
	adc_offsets[1] = 2048 - avg_adc2;

	HAL_TIM_Base_Stop(&htim3);
	__HAL_TIM_SET_COUNTER(&htim3, 0);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	buffer_rx[Size-1] = '\0';
	BaseType_t tarea_mayor_prioridad = pdFALSE;
	xSemaphoreGiveFromISR(semaforo_consola, &tarea_mayor_prioridad);
	portYIELD_FROM_ISR(tarea_mayor_prioridad);

	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, buffer_rx, RX_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
	uint16_t inj_conv1 = (uint16_t)(hadc->Instance->DR & 0xFFFF);
	uint16_t inj_conv2 = (uint16_t)((hadc->Instance->DR >> 16) & 0xFFFF);

//	adc_offsets[0] = 2048 - inj_conv1;
//	adc_offsets[1] = 2048 - inj_conv2;

	// Mismo filtro IIR que para lecturas de corriente
//	adc_offsets[0] = adc_offsets[0] + (((int64_t)FX_ALPHA * ((2048 - inj_conv1) - adc_offsets[0])) >> 16);
//	adc_offsets[1] = adc_offsets[1] + (((int64_t)FX_ALPHA * ((2048 - inj_conv2) - adc_offsets[1])) >> 16);
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_sm */
/**
 * @brief  Function implementing the maquina_de_esta thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_sm */
void sm(void *argument)
{
  /* USER CODE BEGIN 5 */
	mensaje_t mensaje = {
		.estado = INIT,
		.origen = 0,
		.error = 0
	};

	/* Infinite loop */
	while (1) {
		switch (mensaje.estado) {
			case INIT:
				// Inicio encoder, ADCs y timers
				// El parametro es un puntero al buffer de rx para uart
				init_sistema();
				break;

			case NOT_INIT:
				HAL_UART_Transmit(&huart1, (uint8_t *)"Error, reiniciar\n", 17, 1000);
				break;

			case IDLE:
				if (mensaje.origen == 0) {
					HAL_UART_Transmit(&huart1, (uint8_t *)"Sistema iniciado\n", 17, 1000);
					xSemaphoreGive(semaforo_consola);
				}
				else {
					HAL_UART_Transmit(&huart1, (uint8_t *)"Lazos de control parados\n", 25, 1000);
				}
				break;

			case READY:
				HAL_UART_Transmit(&huart1, (uint8_t *)"Lazos de control iniciados\n", 27, 1000);
				break;

			case CLOSED_LOOP:
				break;

			case FAULT:
				fault_handler(&mensaje);
				break;

			default:
				HAL_UART_Transmit(&huart1, (uint8_t *)"Estado desconocido\n", 19, 1000);
				break;
		}
		estado_sistema = mensaje.estado;

		xQueueReceive(cola_estados, &mensaje, portMAX_DELAY);
//		osMessageQueueGet(cola_estadosHandle, &mensaje, NULL, osWaitForever);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_consola */
/**
 * @brief Function implementing the cli thread.
 * @param argument: Not used
 * @retval None
 */

/* USER CODE END Header_consola */
void consola(void *argument)
{
  /* USER CODE BEGIN consola */
	// No ejecuto la tarea hasta que termina el inicio del sistema
	xSemaphoreTake(semaforo_consola, osWaitForever);

	const char menu[] = "\n1. Ver estado del sistema\n"
						"2. Iniciar lazos de control\n"
						"3. Parar lazos de control\n"
						"4. Consigna nueva\n"
						"5. Posicion del rotor (Q15.16)\n"
						"6. Calibrar ADCs\n"
						"7. Obtener Ld\n"
						"8. Obtener Lq\n"
						">> ";
	int len_menu = strlen(menu);

	uint8_t comando;

	char cadena[32];
	int len;

	/* Infinite loop */
	while (1) {
		HAL_UART_Transmit(&huart1, (uint8_t *)menu, len_menu, 1000);

		xSemaphoreTake(semaforo_consola, osWaitForever);

		comando = atoi((char *)buffer_rx);

		switch (comando) {
			case 1:
				len = snprintf(cadena, sizeof(cadena), "Estado: %u\n", estado_sistema);
				HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);
				break;

			case 2:
				init_lazos_control();
				break;

			case 3:
				deinit_lazos_control();
				break;

			case 4:
				len = snprintf(cadena, sizeof(cadena), "Ej: P123.45\n");
				HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);
				len = snprintf(cadena, sizeof(cadena), "ENTER para cancelar\n>>");
				HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);
				xSemaphoreTake(semaforo_consola, osWaitForever);

				if ((char)buffer_rx[0] == '\0') {
					len = snprintf(cadena, sizeof(cadena), "Cancelado\n");
					HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);
				}
				else if ((char)buffer_rx[0] != 'P'){
					len = snprintf(cadena, sizeof(cadena), "Comando desconocido\n");
					HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);
				}
				else {
					int32_t fx_nueva_consigna = fp2fx(strtod((char *)&buffer_rx[1], NULL));
					len = snprintf(cadena, sizeof(cadena), "%ld\n", fx_nueva_consigna);
					HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);
					set_posicion_final_nueva(fx_nueva_consigna);
				}
				// VERIFICAR TASKENTERCRITICAL() Y TASKEXITCRITICAL()
				break;

			case 5:
				len = snprintf(cadena, sizeof(cadena), "Posicion: %ld (Q15.16)\n", get_fx_position());
				HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);
				break;

			case 6:
				get_adc_offsets();
				len = snprintf(cadena, sizeof(cadena), "Offsets: %d, %d\n", adc_offsets[0], adc_offsets[1]);
				HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);
				break;

			case 7:
				get_Ld();
				break;

			case 8:
				get_Lq();
				break;

			case 9:
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1) > 0 ? 0 : 200);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);

			default:
				len = snprintf(cadena, sizeof(cadena), "RX: %s\n", (char *)buffer_rx);
				HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);
				break;
		}

	}
  /* USER CODE END consola */
}

/* USER CODE BEGIN Header_control_posicion */
/**
* @brief Function implementing the lc_posicion thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_control_posicion */
void control_posicion(void *argument)
{
  /* USER CODE BEGIN control_posicion */
  /* Infinite loop */
	while (1) {
		xSemaphoreTake(semaforo_posicion, portMAX_DELAY);
		lazo_posicion();
	}
  /* USER CODE END control_posicion */
}

/* USER CODE BEGIN Header_control_corriente */
/**
* @brief Function implementing the lc_corriente thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_control_corriente */
void control_corriente(void *argument)
{
  /* USER CODE BEGIN control_corriente */
  /* Infinite loop */
	while (1) {
		xSemaphoreTake(semaforo_corriente, portMAX_DELAY);
		lazo_corriente();
	}
  /* USER CODE END control_corriente */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	else if (htim->Instance == TIM3) {
		BaseType_t tarea_mayor_prioridad = pdFALSE;
		xSemaphoreGiveFromISR(semaforo_corriente, &tarea_mayor_prioridad);
		portYIELD_FROM_ISR(tarea_mayor_prioridad);
	}

	else if (htim->Instance == TIM1) {
		BaseType_t tarea_mayor_prioridad = pdFALSE;
		xSemaphoreGiveFromISR(semaforo_posicion, &tarea_mayor_prioridad);
		portYIELD_FROM_ISR(tarea_mayor_prioridad);
	}

	else if (htim->Instance == TIM2) {
		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) {
			overflow_encoder--;
		}
		else {
			overflow_encoder++;
		}
	}

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
	while (1) {
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
