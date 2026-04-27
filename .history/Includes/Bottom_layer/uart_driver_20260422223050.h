#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include "Double_buffer.h"
#include "define.h"
#include "stm32f4xx_hal.h"
#include "usart.h"

#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

/* UART1 DMA Idle Receive Context */
/* 双缓冲接收，用于UART1的DMA空闲接收，开启任务通知写入环形缓冲区 */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  UART_HandleTypeDef *huart; // UART句柄指针，指向对应的UART外设

  DoubleBuffer_t
      uart_data; // UART双缓冲存储数据结构体，包含两个接收缓冲区指针、缓冲区大小、活动缓冲区索引和事件就绪标志
  volatile uint16_t event_length; // 接收事件数据长度

  TaskHandle_t
      task_to_notify; // 需要通知的任务句柄，在接收事件就绪时通知对应的任务进行处理

} UartDmaIdleRx_t; // UART
                   // DMA空闲接收上下文结构体，包含UART句柄、缓冲区大小、活动缓冲区索引、两个DMA接收缓冲区、接收事件相关的索引、长度、类型和就绪标志

extern UartDmaIdleRx_t uart1_dma_idle_rx;

#ifdef __cplusplus
}
#endif

#endif
