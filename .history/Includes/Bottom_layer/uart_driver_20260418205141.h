#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include "stm32f4xx_hal.h"
#include "define.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  UART_HandleTypeDef *huart;
  uint16_t buffer_size;
  uint8_t active_buffer_index;
  uint8_t rx_buffer0[UART1_DMA_RX_BUFFER_SIZE];
  uint8_t rx_buffer1[UART1_DMA_RX_BUFFER_SIZE];
  volatile uint8_t event_buffer_index;
  volatile uint16_t event_length;
  volatile HAL_UART_RxEventTypeTypeDef event_type;
  volatile uint8_t event_ready;
} UartDmaIdleRx_t;

extern UartDmaIdleRx_t g_uart1_dma_idle_rx;

HAL_StatusTypeDef Uart1_DmaIdleRx_Start(void);
void UartDmaIdleRx_RxEvent(UartDmaIdleRx_t *ctx, UART_HandleTypeDef *huart, uint16_t size);
uint8_t UartDmaIdleRx_GetEvent(UartDmaIdleRx_t *ctx, uint8_t **buffer, uint16_t *length,
                               HAL_UART_RxEventTypeTypeDef *event_type);

void Uart1_DmaIdleRxCpltEvent(uint8_t *buffer, uint16_t length,
                              HAL_UART_RxEventTypeTypeDef event_type);

#ifdef __cplusplus
}
#endif

#endif
