#include "Arm_control_task.h"
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"

#include "freertos_handle.h"
#include "robotic_arm.h"

void A_T_Notify(void *argument) { xTaskNotifyGive(Control_solveHandle); }

void Control_Task(void *argument) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // 在这里添加控制算法的代码
    Arm_Control_Single(
        robotic_arm_state.joint1_target, &(robotic_arm_state.pid_joint1_PD),
        &(robotic_arm_state.pid_joint1_PI), &robotic_arm_state.joint1);
  }
}
