#ifndef ROBOTIC_ARM_H
#define ROBOTIC_ARM_H

#include "Double_buffer.h"
#include "define.h"
#include "pid.h"
#include "stm32f4xx_hal.h"

typedef struct {
  DoubleBuffer_t joint1;  // 关节1电机
  PID_CONTROL pid_joint1; // 关节1 PID控制器
  float joint1_target;    // 关节1设定角度

  DoubleBuffer_t joint2;  // 关节2电机
  PID_CONTROL pid_joint2; // 关节2 PID控制器
  float joint2_target;    // 关节2设定角度

  DoubleBuffer_t joint3;  // 关节3电机
  PID_CONTROL pid_joint3; // 关节3 PID控制器
  float joint3_target;    // 关节3设定角度

  DoubleBuffer_t joint4;  // 关节4电机
  PID_CONTROL pid_joint4; // 关节4 PID控制器
  float joint4_target;    // 关节4设定角度

  DoubleBuffer_t gripper;  // 夹爪电机
  PID_CONTROL pid_gripper; // 夹爪 PID控制器
  float gripper_target;    // 夹爪设定位置

} RoboticArmState;

extern RoboticArmState robotic_arm_state;

void Robotic_Arm_Init(void);
void Robotic_Arm_SendCommand(void);
void Arm_Control_Single(float angle_target, PID_CONTROL *pid_PD,
                        PID_CONTROL *pid_PI, Motor *status);

#endif