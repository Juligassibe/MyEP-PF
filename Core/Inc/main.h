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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef enum estados_e {
	INIT,
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
	TIMER1,
	TIMER2,
	TIMER3,
	PWM1,
	PWM2,
	PWM3,
	UART_RX,
	CLI,
	ENC_OVERFLOW,
	PARADA
} origen_e;

typedef struct mensaje_t {
	estados_e estado;
	origen_e origen;
	HAL_StatusTypeDef error;
} mensaje_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

extern xQueueHandle cola_estados;
extern xSemaphoreHandle semaforo_consola;
extern int16_t adc_offsets[2];
extern volatile uint16_t tim1OF;
extern volatile uint16_t tim3OF;
extern estados_e estado_sistema;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void fault_handler(mensaje_t *mensaje);
void init_sistema();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TEST_Pin GPIO_PIN_13
#define TEST_GPIO_Port GPIOC
#define CS_MT6835_Pin GPIO_PIN_4
#define CS_MT6835_GPIO_Port GPIOA
#define STOP_Pin GPIO_PIN_12
#define STOP_GPIO_Port GPIOB
#define STOP_EXTI_IRQn EXTI15_10_IRQn
#define LED_ROJO_Pin GPIO_PIN_13
#define LED_ROJO_GPIO_Port GPIOB
#define LED_VERDE_Pin GPIO_PIN_14
#define LED_VERDE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
