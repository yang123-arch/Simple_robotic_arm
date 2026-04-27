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
  void (*init_Motor)(
      Motor *motor); // 电机初始化函数指针，接受一个Motor结构体指针参数
  void (*update_feedback)(
      Motor *motor,
      uint8_t *
          data); // 电机反馈更新函数指针，接受一个Motor结构体指针和一个uint8_t类型的数据指针参数
  void (*send_command)(
      Motor *motor); // 电机命令发送函数指针，接受一个Motor结构体指针参数
} Motor_Control_Fuctions; // UART DMA空闲接收上下文结构体定义

#endif /* BOTTOM_LAYER_DEFINE_H */
