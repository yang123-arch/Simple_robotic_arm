#ifndef BOTTOM_LAYER_DEFINE_H
#define BOTTOM_LAYER_DEFINE_H

#include "stm32f4xx_hal.h"
#include "usart.h"

#define UART1_DMA_RX_BUFFER_SIZE 128U // UART1 DMA接收缓冲区大小，定义为128字节

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

#define Electronic_SPEED_CONTROL_SIMULATION_HUART (&huart1)
// 模拟电子速度控制器的UART句柄，指向huart1
#define ROBOTIC_ARM_CONTROL_HUART (&huart6)
// 机械臂控制的UART句柄，指向huart6

typedef struct {
  int16_t current_TX;

  float angle_feedback;
  float speed_feedback;

} Motor; // 电机最基础结构体（后面看看电机再做细致更改，也后放在电机驱动代码里）
typedef struct {
  void (*init_Motor)(Motor *motor);
  void (*update_feedback)(Motor *motor, uint8_t *data);
  void (*send_command)(Motor *motor);
} Motor_Control_Fuctions; // UART DMA空闲接收上下文结构体定义

#endif /* BOTTOM_LAYER_DEFINE_H */
