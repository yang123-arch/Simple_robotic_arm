#include "Arm_control_task.h"
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"

#include "freertos_handle.h"

void A_T_Notify(void *argument) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyGive(Control_solveHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
