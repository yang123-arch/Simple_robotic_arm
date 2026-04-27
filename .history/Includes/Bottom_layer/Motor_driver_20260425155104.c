#include "Motor_driver.h"
#include "define.h"

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

void Motor_SendCommand(Motor *motor) {
  uint8_t command[2];
  command[0] = (motor->current_TX >> 8) & 0xFF; // 目标角度高字节
  command[1] = motor->current_TX & 0xFF;        // 目标角度低字节

  HAL_UART_Transmit(Electronic_SPEED_CONTROL_SIMULATION_HUART, command,
                    sizeof(command), HAL_MAX_DELAY);
}
