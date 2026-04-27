#include "robotic_arm.h"
#include "Double_buffer.h"
#include "Motor_driver.h"
#include "define.h"
#include "pid.h"
#include "protocol.h"

#include "stm32f4xx_hal.h"
#include <math.h>
#include <string.h>

RoboticArmState robotic_arm_state;

Motor joint_1[2];
Motor joint_2[2];
Motor joint_3[2];
Motor joint_4[2];
Motor gripper[2];

void Robotic_Arm_Init(void) {
  // 在这里添加机械臂初始化的代码
  // 初始化PID参数
  establish_pid(&robotic_arm_state.pid_joint1_PI, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  establish_pid(&robotic_arm_state.pid_joint1_PD, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  establish_pid(&robotic_arm_state.pid_joint2_PI, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  establish_pid(&robotic_arm_state.pid_joint2_PD, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  establish_pid(&robotic_arm_state.pid_joint4_PI, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  establish_pid(&robotic_arm_state.pid_joint4_PD, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  establish_pid(&robotic_arm_state.pid_gripper_PI, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  establish_pid(&robotic_arm_state.pid_gripper_PD, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  // 初始化双缓冲区
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

  // 初始化目标角度

  robotic_arm_state.joint1_target = 0.0f;
  robotic_arm_state.joint2_target = 0.0f;
  robotic_arm_state.joint3_target = 0.0f;
  robotic_arm_state.joint4_target = 0.0f;
  robotic_arm_state.gripper_target = 0.0f;
}

void Robotic_Arm_SendCommand(void) {
  /* 收集各关节当前力矩值（从 busy 缓冲区读取） */
  int16_t torques[PROTOCOL_MOTOR_COUNT];

  torques[0] =
      ((Motor *)(robotic_arm_state.joint1
                     .buffer[robotic_arm_state.joint1.busy_buffer_index]))
          ->current_TX;
  torques[1] =
      ((Motor *)(robotic_arm_state.joint2
                     .buffer[robotic_arm_state.joint2.busy_buffer_index]))
          ->current_TX;
  torques[2] =
      ((Motor *)(robotic_arm_state.joint3
                     .buffer[robotic_arm_state.joint3.busy_buffer_index]))
          ->current_TX;
  torques[3] =
      ((Motor *)(robotic_arm_state.joint4
                     .buffer[robotic_arm_state.joint4.busy_buffer_index]))
          ->current_TX;
  torques[4] =
      ((Motor *)(robotic_arm_state.gripper
                     .buffer[robotic_arm_state.gripper.busy_buffer_index]))
          ->current_TX;

  /* 打包命令帧并发送 */
  static uint8_t seq = 0U;
  uint8_t frame[PROTOCOL_TORQUE_CMD_FRAME_SIZE];
  protocol_pack_torque_cmd(frame, seq++, torques);

  HAL_UART_Transmit(Electronic_SPEED_CONTROL_SIMULATION_HUART, frame,
                    PROTOCOL_TORQUE_CMD_FRAME_SIZE, HAL_MAX_DELAY);
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
}