#ifndef ROBOTIC_ARM_H
#define ROBOTIC_ARM_H

#include "Double_buffer.h"
#include "pid.h"
#include "stm32f4xx_hal.h"

typedef struct {
  int16_t current_TX;

  float angle_feedback;
  float speed_feedback;

  PID_CONTROL pid_motor;
} Motor; // 电机最基础结构体（后面看看电机再做细致更改，也后放在电机驱动代码里）

typedef struct {
  DoubleBuffer_t joint1; // 关节1电机
  float joint1_target;   // 关节1设定角度

  DoubleBuffer_t joint2; // 关节2电机
  float joint2_target;   // 关节2设定角度

  DoubleBuffer_t joint3; // 关节3电机
  float joint3_target;   // 关节3设定角度

  DoubleBuffer_t joint4; // 关节4电机
  float joint4_target;   // 关节4设定角度

  DoubleBuffer_t gripper; // 夹爪电机
  float gripper_target;   // 夹爪设定位置

} RoboticArmState;

#endif