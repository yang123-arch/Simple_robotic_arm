#include "Arm_control_task.h"
#include "cmsis_os2.h"
#include "define.h"
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
        &(robotic_arm_state.pid_joint1_PI),
        (Motor *)(&robotic_arm_state.joint1
                       .buffer[robotic_arm_state.joint1.busy_buffer_index]));

    Arm_Control_Single(
        robotic_arm_state.joint2_target, &(robotic_arm_state.pid_joint2_PD),
        &(robotic_arm_state.pid_joint2_PI),
        (Motor *)(&robotic_arm_state.joint2
                       .buffer[robotic_arm_state.joint2.busy_buffer_index]));

    Arm_Control_Single(
        robotic_arm_state.joint3_target, &(robotic_arm_state.pid_joint3_PD),
        &(robotic_arm_state.pid_joint3_PI),
        (Motor *)(&robotic_arm_state.joint3
                       .buffer[robotic_arm_state.joint3.busy_buffer_index]));

    Arm_Control_Single(
        robotic_arm_state.joint4_target, &(robotic_arm_state.pid_joint4_PD),
        &(robotic_arm_state.pid_joint4_PI),
        (Motor *)(&robotic_arm_state.joint4
                       .buffer[robotic_arm_state.joint4.busy_buffer_index]));

    Arm_Control_Single(
        robotic_arm_state.gripper_target, &(robotic_arm_state.pid_gripper_PD),
        &(robotic_arm_state.pid_gripper_PI),
        (Motor *)(&robotic_arm_state.gripper
                       .buffer[robotic_arm_state.gripper.busy_buffer_index]));
  }
}
