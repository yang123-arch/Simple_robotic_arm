#include "Motor_driver.h"

void Motor_Init(Motor *motor) {
  motor->current_TX = 0;
  motor->angle_feedback = 0.0f;
  motor->speed_feedback = 0.0f;
}