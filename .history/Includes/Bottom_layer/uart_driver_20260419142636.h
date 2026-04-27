#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include "define.h"
#include "stm32f4xx_hal.h"
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  UART_HandleTypeDef *huart; // UART句柄指针，指向对应的UART外设
  uint16_t buffer_size; // DMA接收缓冲区大小，定义为宏UART1_DMA_RX_BUFFER_SIZE
  uint8_t
      active_buffer_index; // 当前活动缓冲区索引，0或1，表示当前正在使用的DMA接收缓冲区

  uint8_t rx_buffer0[UART1_DMA_RX_BUFFER_SIZE];
  uint8_t rx_buffer1[UART1_DMA_RX_BUFFER_SIZE];

  volatile uint8_t
      event_buffer_index; // 接收事件缓冲区索引，0或1，表示当前接收事件对应的缓冲区索引
  volatile uint16_t
      event_length; // 接收事件数据长度，表示当前接收事件中有效数据的长度
  volatile HAL_UART_RxEventTypeTypeDef
      event_type; // 接收事件类型，表示接收完成的事件类型，如空闲、传输完成等
  volatile uint8_t
      event_ready; // 接收事件就绪标志，1表示有新的接收事件就绪，0表示没有新的接收事件
} UartDmaIdleRx_t;

extern UartDmaIdleRx_t g_uart1_dma_idle_rx;

HAL_StatusTypeDef Uart_DmaIdleRx_Start(UartDmaIdleRx_t *ctx,
                                       UART_HandleTypeDef *huart,
                                       uint16_t buffer_size);
void UartDmaIdleRx_RxEvent(UartDmaIdleRx_t *ctx, UART_HandleTypeDef *huart,
                           uint16_t size);
uint8_t UartDmaIdleRx_GetEvent(UartDmaIdleRx_t *ctx, uint8_t **buffer,
                               uint16_t *length,
                               HAL_UART_RxEventTypeTypeDef *event_type);

void Uart1_DmaIdleRxCpltEvent(uint8_t *buffer, uint16_t length,
                              HAL_UART_RxEventTypeTypeDef event_type);

#ifdef __cplusplus
}
#endif

#endif
