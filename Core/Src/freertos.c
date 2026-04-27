/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "freertos_handle.h"
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

/* USER CODE END Variables */
/* Definitions for UART_SOLVE */
osThreadId_t UART_SOLVEHandle;
const osThreadAttr_t UART_SOLVE_attributes = {
    .name = "UART_SOLVE",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for Control_solve */
osThreadId_t Control_solveHandle;
const osThreadAttr_t Control_solve_attributes = {
    .name = "Control_solve",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for Arm_time */
osTimerId_t Arm_timeHandle;
const osTimerAttr_t Arm_time_attributes = {.name = "Arm_time"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void UART_Task(void *argument);
void Control_Task(void *argument);
void A_T_Notify(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of Arm_time */
  Arm_timeHandle =
      osTimerNew(A_T_Notify, osTimerPeriodic, NULL, &Arm_time_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */

  osTimerStart(Arm_timeHandle, pdMS_TO_TICKS(1));
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UART_SOLVE */
  UART_SOLVEHandle = osThreadNew(UART_Task, NULL, &UART_SOLVE_attributes);

  /* creation of Control_solve */
  Control_solveHandle =
      osThreadNew(Control_Task, NULL, &Control_solve_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_UART_Task */
/**
 * @brief  Function implementing the UART_SOLVE thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_UART_Task */
__weak void UART_Task(void *argument) {
  /* USER CODE BEGIN UART_Task */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END UART_Task */
}

/* USER CODE BEGIN Header_Control_Task */
/**
 * @brief Function implementing the Control_solve thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Control_Task */
__weak void Control_Task(void *argument) {
  /* USER CODE BEGIN Control_Task */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END Control_Task */
}

/* A_T_Notify function */
__weak void A_T_Notify(void *argument) {
  /* USER CODE BEGIN A_T_Notify */

  /* USER CODE END A_T_Notify */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
