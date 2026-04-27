#ifndef BOTTOM_LAYER_DEFINE_H
#define BOTTOM_LAYER_DEFINE_H

#include "stm32f4xx_hal.h"

#define UART1_DMA_RX_BUFFER_SIZE 128U // UART1 DMA接收缓冲区大小，定义为128字节

extern UART_HandleTypeDef huart1;

#define Electronic_SPEED_CONTROL_SIMULATION_HUART (&huart1)
// 模拟电子速度控制器的UART句柄，指向huart1

#endif /* BOTTOM_LAYER_DEFINE_H */
