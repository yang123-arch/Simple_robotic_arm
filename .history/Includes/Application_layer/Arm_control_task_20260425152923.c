#include "Arm_control_task.h"

#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"

#include "freertos_handle.h"

void A_T_Notify(void *argument) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGive(Control_solveHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
