#include "Motor_driver.h"

void Motor_Init(Motor *motor) {
  motor->current_TX = 0;
  motor->angle_feedback = 0.0f;
  motor->speed_feedback = 0.0f;
}

void Motor_UpdateFeedback(Motor *motor, uint8_t *data) {
  uint16_t raw_angle = (data[0] << 8) | data[1]; // 角度反馈
  uint16_t speed = (data[2] << 8) | data[3];     // 速度反馈

  motor->angle_feedback = (raw_angle - 4096.0f) * 180.0f / 4096.0f;
  ;
  motor->speed_feedback = speed;
}
