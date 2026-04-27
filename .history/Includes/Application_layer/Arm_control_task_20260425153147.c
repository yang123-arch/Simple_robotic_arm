#include "Arm_control_task.h"
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"

#include "freertos_handle.h"

void A_T_Notify(void *argument) { xTaskNotifyGive(Control_solveHandle); }
