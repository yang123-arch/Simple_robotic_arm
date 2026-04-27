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

  TaskHandle_t
      task_to_notify; // 需要通知的任务句柄，在接收事件就绪时通知对应的任务进行处理

} UartDmaIdleRx_t; // UART
                   // DMA空闲接收上下文结构体，包含UART句柄、缓冲区大小、活动缓冲区索引、两个DMA接收缓冲区、接收事件相关的索引、长度、类型和就绪标志

typedef struct {
  HAL_StatusTypeDef (*establish_UartDmaIdleRx_t)(
      UartDmaIdleRx_t *ctx, UART_HandleTypeDef *huart, uint16_t buffer_size,
      TaskHandle_t
          task_to_notify); // 初始化函数指针，指向Uart_DmaIdleRx_Start函数，用于初始化UART，DMA空闲接收上下文，并开启接收
  uint8_t (*get_data_from_uart)(
      UartDmaIdleRx_t *ctx, uint8_t **buffer, uint16_t *length,
      HAL_UART_RxEventTypeTypeDef *
          event_type); // 获取数据函数指针，指向UartDmaIdleRx_GetEvent函数，用于获取接收事件，如果有事件就绪，则返回对应缓冲区指针、数据长度和事件类型，并清除事件就绪状态
  void (*handle_event_from_isr)(
      UartDmaIdleRx_t *ctx, uint16_t size,
      BaseType_t *
          pxHigherPriorityTaskWoken); // 接收数据函数指针，指向UartDmaIdleRx_RxEvent函数，用于处理UART接收事件回调，根据接收事件类型和数据长度更新上下文状态，并重新启动DMA空闲接收,如果设置了需要通知的任务句柄，则进行任务通知
} UartDmaIdle_fuctions;

extern const UartDmaIdle_fuctions UartDmaIdle_Template;

extern UartDmaIdleRx_t uart1_dma_idle_rx;

#ifdef __cplusplus
}
#endif

#endif
