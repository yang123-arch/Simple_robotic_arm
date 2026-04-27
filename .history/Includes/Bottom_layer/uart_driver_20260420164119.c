#include "uart_driver.h"

#include "main.h"
#include "stm32f4xx_hal_def.h"
#include "usart.h"

/* UART1 DMA Idle Receive Context */
/* 双缓冲接收，用于UART1的DMA空闲接收，开启任务通知写入环形缓冲区 */

UartDmaIdle_fuctions uart1_dma_idle_rx = {
    0}; // UART1 DMA空闲接收上下文结构体实例，初始化为0

/**
 * @brief
 * 根据索引获取对应的DMA接收缓冲区指针,返回对应缓冲区的指针，索引0对应rx_buffer0，索引1对应rx_buffer1
 *
 * @param ctx
 * @param index
 * @return uint8_t*
 */
static uint8_t *UartDmaIdleRx_GetBufferByIndex(UartDmaIdleRx_t *ctx,
                                               uint8_t index) {
  return (index == 0U) ? ctx->rx_buffer0 : ctx->rx_buffer1;
}

/**
 * @brief
 * 启动DMA空闲接收，调用HAL库函数HAL_UARTEx_ReceiveToIdle_DMA启动DMA空闲接收，并传入当前活动缓冲区的指针和缓冲区大小，如果启动成功且DMA句柄不为NULL，则禁用DMA半传输中断，以避免在半传输完成时触发中断，返回启动状态
 *
 * @param ctx
 * @return HAL_StatusTypeDef
 */
static HAL_StatusTypeDef UartDmaIdleRx_StartDma(UartDmaIdleRx_t *ctx) {
  HAL_StatusTypeDef status;

  status = HAL_UARTEx_ReceiveToIdle_DMA(
      ctx->huart, UartDmaIdleRx_GetBufferByIndex(ctx, ctx->active_buffer_index),
      ctx->buffer_size);

  if ((status == HAL_OK) && (ctx->huart->hdmarx != NULL)) {
    __HAL_DMA_DISABLE_IT(ctx->huart->hdmarx, DMA_IT_HT);
  }

  return status;
}

/**
 * @brief
 * 初始化结构体参数，并调用UartDmaIdleRx_StartDma函数启动DMA空闲接收，传入当前活动缓冲区的指针和缓冲区大小，返回启动状态
 *
 * @param ctx
 * @param huart
 * @param buffer_size
 * @return HAL_StatusTypeDef
 */
static HAL_StatusTypeDef UartDmaIdleRx_Init(UartDmaIdleRx_t *ctx,
                                            UART_HandleTypeDef *huart,
                                            uint16_t buffer_size,
                                            TaskHandle_t task_to_notify) {
  // 初始化底层硬件相关
  ctx->huart = huart;
  ctx->buffer_size = buffer_size;
  ctx->active_buffer_index = 0U;
  ctx->event_buffer_index = 0U;
  ctx->event_length = 0U;
  ctx->event_type = HAL_UART_RXEVENT_IDLE;
  ctx->event_ready = 0U;

  ctx->task_to_notify = task_to_notify;

  // 启动 DMA
  return UartDmaIdleRx_StartDma(ctx);
}

/**
 * @brief
 * uart接收事件处理函数，更新接收事件相关参数，并重新启动DMA空闲接收，调用Uart1_DmaIdleRxCpltEvent函数通知上层应用接收完成事件，传入接收数据缓冲区指针、数据长度和事件类型
 *
 *
 * @param ctx
 * @param size
 */
static void
UartDmaIdleRx_HandleEventFromISR(UartDmaIdleRx_t *ctx, uint16_t size,
                                 BaseType_t *pxHigherPriorityTaskWoken) {
  if (ctx == NULL)
    return;

  uint8_t finished_buffer_index = ctx->active_buffer_index;
  HAL_UART_RxEventTypeTypeDef event_type =
      HAL_UARTEx_GetRxEventType(ctx->huart);

  // 更新事件状态
  ctx->event_buffer_index = finished_buffer_index;
  ctx->event_length = size;
  ctx->event_type = event_type;
  ctx->event_ready = 1U;

  // 切换双缓冲
  if ((event_type == HAL_UART_RXEVENT_TC) || (size >= ctx->buffer_size)) {
    ctx->active_buffer_index ^= 1U;
  }

  // 重新启动 DMA
  if (UartDmaIdleRx_StartDma(ctx) != HAL_OK) {
    Error_Handler();
  }

  // 发 FreeRTOS 任务通知，唤醒应用层任务
  if (ctx->task_to_notify != NULL) {
    vTaskNotifyGiveFromISR(ctx->task_to_notify, pxHigherPriorityTaskWoken);
  }
}

/**
 * @brief
 * 获取接收事件函数，检查事件准备状态，如果有事件准备好，则返回对应缓冲区指针、数据长度和事件类型，并清除事件准备状态，返回1表示成功获取事件，0表示没有事件或参数错误
 *
 * @param ctx
 * @param buffer （多级指针，指向缓冲区）
 * @param length
 * @param event_type
 * @return uint8_t
 */
static uint8_t UartDmaIdleRx_GetEvent(UartDmaIdleRx_t *ctx, uint8_t **buffer,
                                      uint16_t *length,
                                      HAL_UART_RxEventTypeTypeDef *event_type) {
  if ((ctx == NULL) || (buffer == NULL) || (length == NULL) ||
      (ctx->event_ready == 0U)) {
    return 0U;
  }

  // 拿数据
  *buffer = UartDmaIdleRx_GetBufferByIndex(ctx, ctx->event_buffer_index);
  *length = ctx->event_length;
  if (event_type != NULL) {
    *event_type = ctx->event_type;
  }

  // 清标志
  ctx->event_ready = 0U;
  return 1U;
}

const UartDmaIdle_fuctions UartDmaIdle_Template = {
    .establish_UartDmaIdleRx_t = UartDmaIdleRx_Init,
    .get_data_from_uart = UartDmaIdleRx_GetEvent,
    .handle_event_from_isr = UartDmaIdleRx_HandleEventFromISR,
}; // fuctions结构体模板，包含UART
   // DMA空闲接收上下文结构体实例和相关函数指针，初始化为0或对应函数地址

void init_uart_dma(void) {}

/**
 * @brief UART接收事件回调函数
 *
 * @param huart
 * @param Size
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {}