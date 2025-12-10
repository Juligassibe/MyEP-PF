/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

#include "cmsis_os.h"
#include "task.h"
#include "semphr.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef enum estados_e {
	INIT,
	NOT_INIT,
	IDLE,
	CLOSED_LOOP,
	FAULT
} estados_e;

typedef enum origen_e {
	CORRIENTES,
	ENC_INIT,
	ADC1_CAL,
	ADC2_CAL,
	ADC_MM,
	ADC_2,
	ADC1_INJ,
	ADC2_INJ,
	TIMER1,
	TIMER2,
	TIMER3,
	PWM1,
	PWM2,
	PWM3,
	UART_RX,
	CLI,
	ENC_OVERFLOW
} origen_e;

typedef struct mensaje_t {
	estados_e estado;
	origen_e origen;
	HAL_StatusTypeDef error;
} mensaje_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern SPI_HandleTypeDef hspi1;
extern osMessageQId cola_estadosHandle;
extern xQueueHandle cola_estados;
extern xSemaphoreHandle semaforo_consola;
extern xSemaphoreHandle semaforo_adc;
extern int16_t adc_offsets[2];
extern volatile uint16_t tim1OF;
extern volatile uint16_t tim3OF;
extern estados_e estado_sistema;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void fault_handler(mensaje_t *mensaje);
void init_sistema();
void init_posicion();
void get_adc_offsets();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TEST_Pin GPIO_PIN_13
#define TEST_GPIO_Port GPIOC
#define CS_MT6835_Pin GPIO_PIN_4
#define CS_MT6835_GPIO_Port GPIOA
#define STEP_Pin GPIO_PIN_11
#define STEP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define RX_SIZE 16
extern uint8_t buffer_rx[RX_SIZE];

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
