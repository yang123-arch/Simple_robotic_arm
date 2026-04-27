#include "Arm_control_task.h"
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"

#include "freertos_handle.h"

void A_T_Notify(void *argument) { xTaskNotifyGive(Control_solveHandle); }

void Control_Task(void *argument) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // 在这里添加控制算法的代码
  }
}
