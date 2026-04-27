#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "stm32f4xx_hal.h"

typedef struct {
  int16_t current_TX;

  float angle_feedback;
  float speed_feedback;

} Motor; // 电机最基础结构体（后面看看电机再做细致更改，也后放在电机驱动代码里）

#endif /* MOTOR_DRIVER_H */