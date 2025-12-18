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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "control.h"
#include "mt6835.h"

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

/* USER CODE BEGIN PV */

volatile uint16_t tim1OF = 0;
volatile uint16_t tim3OF = 0;
encoder_t hencoder = { 0 };
estados_e estado_sistema;
int16_t adc_offsets[2] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
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

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /* Infinite loop */
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

/* USER CODE BEGIN 4 */

void init_sistema() {
	mensaje_t mensaje = { 0 };
	HAL_StatusTypeDef error;

	// INIT ENCODER
	error = init_encoder(&hspi1, CS_MT6835_GPIO_Port, CS_MT6835_Pin, &hencoder, ABZ_RES,
						 MT6835_ABZ_NO_SWAP, MT6835_ZRE, MT6835_Z_WIDTH_1LSB, MT6835_Z_ARE,
						 MT6835_CCW_BA);

	if (error != HAL_OK) {
		mensaje.estado = FAULT;
		mensaje.origen = ENC_INIT;
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

	// Timer 3 para PWM de las fases
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
		mensaje.origen = ENC_INIT;
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
	// Luego de hacer 0, hago las corrientes 0 para calibrar ADCs
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

	// Pongo el 0 de la posicion en la mitad del timer para tener valores positivos y negativos
	__HAL_TIM_SET_COUNTER(&htim2, 32768);
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);

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

			HAL_TIM_Base_Stop_IT(&htim3);

			HAL_TIM_Base_Stop_IT(&htim1);

			len = snprintf(cadena, sizeof(cadena), "\nE: SOBRECORRIENTE\n");
			HAL_UART_Transmit(&huart1, (uint8_t*) cadena, len, 1000);
			break;

		case ENC_INIT:
			len = snprintf(cadena, sizeof(cadena), "\nE: Encoder:\n%u\n%u\n%u\n%u\n%u\n%u\n", hencoder.abz_res, hencoder.abz_swap, hencoder.z_edge,
																							hencoder.z_width, hencoder.z_phase, hencoder.rot_dir);
			HAL_UART_Transmit(&huart1, (uint8_t*) cadena, len, 1000);
			break;

		case ADC1_CAL:
			len = snprintf(cadena, sizeof(cadena), "\nE: ADC1 CAL (%d)\n", mensaje->error);
			HAL_UART_Transmit(&huart1, (uint8_t*) cadena, len, 1000);
			break;

		case ADC2_CAL:
			len = snprintf(cadena, sizeof(cadena), "\nE: ADC2 CAL (%d)\n", mensaje->error);
			HAL_UART_Transmit(&huart1, (uint8_t*) cadena, len, 1000);
			break;

		case ADC_2:
			len = snprintf(cadena, sizeof(cadena), "\nE: ADC2 Start (%d)\n", mensaje->error);
			HAL_UART_Transmit(&huart1, (uint8_t*) cadena, len, 1000);
			break;

		case ADC_MM:
			len = snprintf(cadena, sizeof(cadena), "\nE: ADC MultiMode (%d)\n", mensaje->error);
			HAL_UART_Transmit(&huart1, (uint8_t*) cadena, len, 1000);
			break;

		case TIMER1:
			len = snprintf(cadena, sizeof(cadena), "\nE: Timer lazo de posicion (%d)\n", mensaje->error);
			HAL_UART_Transmit(&huart1, (uint8_t*) cadena, len, 1000);
			break;

		case TIMER2:
			len = snprintf(cadena, sizeof(cadena), "\nE: Timer encoder (%d)\n", mensaje->error);
			HAL_UART_Transmit(&huart1, (uint8_t*) cadena, len, 1000);
			break;

		case PWM1:
			len = snprintf(cadena, sizeof(cadena), "\nE: PWM 1 (%d)\n", mensaje->error);
			HAL_UART_Transmit(&huart1, (uint8_t*) cadena, len, 1000);
			break;

		case PWM2:
			len = snprintf(cadena, sizeof(cadena), "\nE: PWM 2 (%d)\n", mensaje->error);
			HAL_UART_Transmit(&huart1, (uint8_t*) cadena, len, 1000);
			break;

		case PWM3:
			len = snprintf(cadena, sizeof(cadena), "\nE: PWM 3 (%d)\n", mensaje->error);
			HAL_UART_Transmit(&huart1, (uint8_t*) cadena, len, 1000);
			break;

		case UART_RX:
			len = snprintf(cadena, sizeof(cadena), "\nE: UART RX (%d)\n", mensaje->error);
			HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);
			break;

		case CLI:
			HAL_UART_Transmit(&huart1, (uint8_t *)"\nE: CLI\n", 8, 1000);
			break;

		case ENC_OVERFLOW:
			HAL_TIM_Base_Stop_IT(&htim3);
			HAL_TIM_Base_Stop_IT(&htim1);
			HAL_UART_Transmit(&huart1, (uint8_t *)"\nE: Overflow encoder\n", 21, 1000);
			break;

		default:
			HAL_UART_Transmit(&huart1, (uint8_t *)"\nError desconocido\n", 19, 1000);
			break;
	}
}

/* USER CODE END 4 */

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
		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) {
			lazo_corriente();
		}
	}

	else if (htim->Instance == TIM1) {
		lazo_posicion();
		tim1OF++;
	}

	else if (htim->Instance == TIM2) {
		if (estado_sistema == IDLE || estado_sistema == CLOSED_LOOP) {
			mensaje_t mensaje = {
				.estado = FAULT,
				.origen = ENC_OVERFLOW
			};

			BaseType_t tarea_mayor_prioridad = pdFALSE;
			xQueueSendFromISR(cola_estados, &mensaje, &tarea_mayor_prioridad);
			portYIELD_FROM_ISR(tarea_mayor_prioridad);
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
