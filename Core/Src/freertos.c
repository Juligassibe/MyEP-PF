/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#include "control.h"

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
/* USER CODE BEGIN Variables */

xSemaphoreHandle semaforo_consola;
xQueueHandle cola_estados;

/* USER CODE END Variables */
/* Definitions for state_machine */
osThreadId_t state_machineHandle;
const osThreadAttr_t state_machine_attributes = {
  .name = "state_machine",
  .stack_size = 96 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for cli */
osThreadId_t cliHandle;
const osThreadAttr_t cli_attributes = {
  .name = "cli",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for test */
osThreadId_t testHandle;
const osThreadAttr_t test_attributes = {
  .name = "test",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void sm(void *argument);
void consola(void *argument);
void test_task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
	char cadena[64];
	int len;

	len = snprintf(cadena, sizeof(cadena), "STACK OVERFLOW en tarea: %s\n", pcTaskName);
	HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);
	taskDISABLE_INTERRUPTS();
	while(1);
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */

	semaforo_consola = xSemaphoreCreateBinary();

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */

	cola_estados = xQueueCreate(3, sizeof(mensaje_t));

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of state_machine */
  state_machineHandle = osThreadNew(sm, NULL, &state_machine_attributes);

  /* creation of cli */
  cliHandle = osThreadNew(consola, NULL, &cli_attributes);

  /* creation of test */
  testHandle = osThreadNew(test_task, NULL, &test_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_sm */
/**
  * @brief  Function implementing the state_machine thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_sm */
void sm(void *argument)
{
  /* USER CODE BEGIN sm */
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

			case IDLE:
				if (mensaje.origen == 0) {
					HAL_UART_Transmit(&huart1, (uint8_t *)"Inicio listo\n", 13, 1000);
					xSemaphoreGive(semaforo_consola);
				}
				else {
					HAL_UART_Transmit(&huart1, (uint8_t *)"Lazos de control detenidos\n", 27, 1000);
				}
				break;

			case CLOSED_LOOP:
				HAL_UART_Transmit(&huart1, (uint8_t *)"Lazos de control iniciados\n", 27, 1000);
				break;

			case FAULT:
				fault_handler(&mensaje);
				HAL_UART_Transmit(&huart1, (uint8_t *)"Reiniciar\n", 10, 1000);
				break;

			default:
				HAL_UART_Transmit(&huart1, (uint8_t *)"Estado desconocido\n", 19, 1000);
				break;
		}

		estado_sistema = mensaje.estado;

		xQueueReceive(cola_estados, &mensaje, portMAX_DELAY);
	}
  /* USER CODE END sm */
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

	const char menu[] = "\n1. Estado del sistema\n"
					"2. Iniciar lazos de control\n"
					"3. Parar lazos de control\n"
					"4. Nueva consigna\n"
					"5. Obtener posicion\n"
					"6. Offsets ADCs\n"
					"7. Cambiar constantes de control\n"
					">> ";

	uint16_t len_menu = strlen(menu);

	char cadena[64];
	int len;

  /* Infinite loop */
	while (1) {
		HAL_UART_Transmit(&huart1, (uint8_t *)menu, len_menu, 1000);

		xSemaphoreTake(semaforo_consola, osWaitForever);

		switch (buffer_rx[0]) {
			case '1':
				len = snprintf(cadena, sizeof(cadena), "Estado: %u", estado_sistema);
				HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);
				break;

			case '2':
				init_lazos_control();
				break;

			case '3':
				deinit_lazos_control();
				break;

			case '4':
				set_posicion_final_nueva();
				len = snprintf(cadena, sizeof(cadena), "\n%ld\n", interpolador.fx_posicion_final);
				HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);
				break;

			case '5':
				len = snprintf(cadena, sizeof(cadena), "\nPosicion: %ld (Q15.16)\n", get_fx_position());
				HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);
				break;

			case '6':
				get_adc_offsets();
				len = snprintf(cadena, sizeof(cadena), "\nOffsets: %d, %d\n", adc_offsets[0], adc_offsets[1]);
				HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);
				break;

			case '7':
				modificar_constantes();
				len = snprintf(cadena, sizeof(cadena), "\n%ld\n%ld\n%ld\n%ld\n%ld\n", controlador.fx_P,
																					controlador.fx_I,
																					controlador.fx_D,
																					controlador.fx_Pq,
																					controlador.fx_Pd);

				HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);
				break;

			case '8':
				get_Lq();
				break;

			case '9':
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1) > 0 ? 0 : 200);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
				break;

			default:
				len = snprintf(cadena, sizeof(cadena), "\nRX: %s\n", (char *)buffer_rx);
				HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);
				break;
		}

	}
  /* USER CODE END consola */
}

/* USER CODE BEGIN Header_test_task */
/**
* @brief Function implementing the test thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_test_task */
void test_task(void *argument)
{
  /* USER CODE BEGIN test_task */
	osDelay(osWaitForever);
	char cadena[32];
	int len;

	int32_t corrientes[2];
	HAL_TIM_Base_Start(&htim3);
  /* Infinite loop */
	while (1) {
		get_corrientes_qd0(corrientes);

		len = snprintf(cadena, sizeof(cadena), "%ld, %ld\n", corrientes[0], corrientes[1]);
		HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);

		osDelay(pdMS_TO_TICKS(10));
	}
  /* USER CODE END test_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

