#ifndef ROBOTIC_ARM_H
#define ROBOTIC_ARM_H

#include "pid.h"
#include "stm32f4xx_hal.h"

typedef struct {
  float joint1_angle;        // 关节1角度
  float joint1_angle_actual; // 关节1实际角度

  float joint2_angle;        // 关节2角度
  float joint2_angle_actual; // 关节2实际角度

  float joint3_angle;        // 关节3角度
  float joint3_angle_actual; // 关节3实际角度

  float joint4_angle;        // 关节4角度
  float joint4_angle_actual; // 关节4实际角度

  float gripper_position;        // 夹爪位置
  float gripper_position_actual; // 夹爪实际位置

} RoboticArmState;

#endif