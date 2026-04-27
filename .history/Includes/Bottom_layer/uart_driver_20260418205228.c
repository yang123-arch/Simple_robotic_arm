#include "uart_driver.h"

#include "main.h"
#include "usart.h"

UartDmaIdleRx_t g_uart1_dma_idle_rx = {0};

static uint8_t *UartDmaIdleRx_GetBufferByIndex(UartDmaIdleRx_t *ctx, uint8_t index)
{
  return (index == 0U) ? ctx->rx_buffer0 : ctx->rx_buffer1;
}

static HAL_StatusTypeDef UartDmaIdleRx_StartDma(UartDmaIdleRx_t *ctx)
{
  HAL_StatusTypeDef status;

  status = HAL_UARTEx_ReceiveToIdle_DMA(
      ctx->huart,
      UartDmaIdleRx_GetBufferByIndex(ctx, ctx->active_buffer_index),
      ctx->buffer_size);

  if ((status == HAL_OK) && (ctx->huart->hdmarx != NULL))
  {
    __HAL_DMA_DISABLE_IT(ctx->huart->hdmarx, DMA_IT_HT);
  }

  return status;
}

HAL_StatusTypeDef Uart1_DmaIdleRx_Start(void)
{
  g_uart1_dma_idle_rx.huart = &huart1;
  g_uart1_dma_idle_rx.buffer_size = UART1_DMA_RX_BUFFER_SIZE;
  g_uart1_dma_idle_rx.active_buffer_index = 0U;
  g_uart1_dma_idle_rx.event_buffer_index = 0U;
  g_uart1_dma_idle_rx.event_length = 0U;
  g_uart1_dma_idle_rx.event_type = HAL_UART_RXEVENT_IDLE;
  g_uart1_dma_idle_rx.event_ready = 0U;

  return UartDmaIdleRx_StartDma(&g_uart1_dma_idle_rx);
}

void UartDmaIdleRx_RxEvent(UartDmaIdleRx_t *ctx, UART_HandleTypeDef *huart, uint16_t size)
{
  uint8_t finished_buffer_index;
  HAL_UART_RxEventTypeTypeDef event_type;

  if ((ctx == NULL) || (huart == NULL) || (ctx->huart != huart))
  {
    return;
  }

  finished_buffer_index = ctx->active_buffer_index;
  event_type = HAL_UARTEx_GetRxEventType(huart);

  ctx->event_buffer_index = finished_buffer_index;
  ctx->event_length = size;
  ctx->event_type = event_type;
  ctx->event_ready = 1U;

  if ((event_type == HAL_UART_RXEVENT_TC) || (size >= ctx->buffer_size))
  {
    ctx->active_buffer_index ^= 1U;
  }

  if (UartDmaIdleRx_StartDma(ctx) != HAL_OK)
  {
    Error_Handler();
  }

  Uart1_DmaIdleRxCpltEvent(
      UartDmaIdleRx_GetBufferByIndex(ctx, finished_buffer_index),
      size,
      event_type);
}

uint8_t UartDmaIdleRx_GetEvent(
    UartDmaIdleRx_t *ctx,
    uint8_t **buffer,
    uint16_t *length,
    HAL_UART_RxEventTypeTypeDef *event_type)
{
  if ((ctx == NULL) || (buffer == NULL) || (length == NULL))
  {
    return 0U;
  }

  if (ctx->event_ready == 0U)
  {
    return 0U;
  }

  *buffer = UartDmaIdleRx_GetBufferByIndex(ctx, ctx->event_buffer_index);
  *length = ctx->event_length;

  if (event_type != NULL)
  {
    *event_type = ctx->event_type;
  }

  ctx->event_ready = 0U;
  return 1U;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  UartDmaIdleRx_RxEvent(&g_uart1_dma_idle_rx, huart, Size);
}

__weak void Uart1_DmaIdleRxCpltEvent(
    uint8_t *buffer,
    uint16_t length,
    HAL_UART_RxEventTypeTypeDef event_type)
{
  (void)buffer;
  (void)length;
  (void)event_type;
}
