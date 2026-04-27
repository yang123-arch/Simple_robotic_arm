#include "robotic_arm.h"
#include "Double_buffer.h"
#include "Motor_driver.h"
#include "define.h"
#include "pid.h"

#include "stm32f4xx_hal.h"
#include <string.h>

RoboticArmState robotic_arm_state;

Motor joint_1[2];
Motor joint_2[2];
Motor joint_3[2];
Motor joint_4[2];
Motor gripper[2];

void Robotic_Arm_Init(void) {
  // 在这里添加机械臂初始化的代码
  DoubleBuffer_Template.init_Buffer(&robotic_arm_state.joint1, &joint_1[0],
                                    &joint_1[1], sizeof(Motor));
  DoubleBuffer_Template.init_Buffer(&robotic_arm_state.joint2, &joint_2[0],
                                    &joint_2[1], sizeof(Motor));
  DoubleBuffer_Template.init_Buffer(&robotic_arm_state.joint3, &joint_3[0],
                                    &joint_3[1], sizeof(Motor));
  DoubleBuffer_Template.init_Buffer(&robotic_arm_state.joint4, &joint_4[0],
                                    &joint_4[1], sizeof(Motor));
  DoubleBuffer_Template.init_Buffer(&robotic_arm_state.gripper, &gripper[0],
                                    &gripper[1], sizeof(Motor));

  PID_Init(&robotic_arm_state.pid_joint1, 1.0f, 0.0f, 0.0f);
  PID_Init(&robotic_arm_state.pid_joint2, 1.0f, 0.0f, 0.0f);
  PID_Init(&robotic_arm_state.pid_joint3, 1.0f, 0.0f, 0.0f);
  PID_Init(&robotic_arm_state.pid_joint4, 1.0f, 0.0f, 0.0f);
  PID_Init(&robotic_arm_state.pid_gripper, 1.0f, 0.0f, 0.0f);

  robotic_arm_state.joint1_target = 0.0f;
  robotic_arm_state.joint2_target = 0.0f;
  robotic_arm_state.joint3_target = 0.0f;
  robotic_arm_state.joint4_target = 0.0f;
  robotic_arm_state.gripper_target = 0.0f;
}

void Robotic_Arm_SendCommand(void) {
  // 在这里添加机械臂控制算法的代码
}

/**
 * @brief 单关节控制
 *
 * @param angle_target
 * @param pid_PD
 * @param pid_PI
 * @param status
 */
void Arm_Control_Single(float angle_target, PID_CONTROL *pid_PD,
                        PID_CONTROL *pid_PI, Motor *status) {

  float angle_error = angle_target - status->angle_feedback;

  while (angle_error > 180.0f)
    angle_error -= 360.0f;
  while (angle_error < -180.0f)
    angle_error += 360.0f;

  if (fabs(angle_error) < 5.0f) {
    pid_PD->error_total = 0; // 清积分防饱和
    status->current_TX = 0;
    Robotic_Arm_SendCommand();
  }

  float equivalent_target = status->angle_feedback + angle_error;

  float yaw_output_PD =
      PID_control(status->angle_feedback, equivalent_target, pid_PD);
  float speed_target = yaw_output_PD;

  float yaw_output_PI =
      PID_control(status->speed_feedback, speed_target, pid_PI);

  float boost = (fabs(speed_target) < 5.0f) ? 1900.0f : 700.0f;
  yaw_output_PI += (speed_target > 0) ? boost : -boost;

  if (fabs(yaw_output_PI) > 2000.0f) {
    yaw_output_PI = (yaw_output_PI > 0.0f) ? 2000.0f : -2000.0f;
  }

  status->current_TX = (int16_t)yaw_output_PI;
  Robotic_Arm_SendCommand();
}