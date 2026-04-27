#include "uart_driver.h"

#include "cmsis_os2.h"
#include "freertos_handle.h"
#include "main.h"
#include "stm32f4xx_hal_def.h"
#include "usart.h"

extern const DoubleBuffer_Control DoubleBuffer_Template;

/* UART1 DMA Idle Receive Context */
/* 双缓冲接收，用于UART1的DMA空闲接收，开启任务通知写入环形缓冲区 */

UartDmaIdleRx_t uart1_dma_idle_rx = {
    0}; // UART1 DMA空闲接收上下文结构体实例，初始化为0

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
      ctx->huart, ctx->uart_data.buffer[ctx->active_buffer_index],
      ctx->uart_data.buffer_size);

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
 * 中断处理函数，处理UART接收事件回调，根据接收事件类型和数据长度更新上下文状态，并重新启动DMA空闲接收,如果设置了需要通知的任务句柄，则进行任务通知
 *
 * @param ctx
 * @param size
 * @param pxHigherPriorityTaskWoken
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
 * (可选参数，指向事件类型的指针，如果不为NULL，则返回事件类型)
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

/**
 * @brief
 * 底层函数操作表，定义函数指针指向对应的实现函数，提供统一的接口供应用层调用
 * @note establish_UartDmaIdleRx_t 初始化
 * @note get_data_from_uart 获取数据
 * @note handle_event_from_isr 处理接收事件回调
 */
const UartDmaIdle_fuctions UartDmaIdle_Template = {
    .establish_UartDmaIdleRx_t = UartDmaIdleRx_Init,
    .get_data_from_uart = UartDmaIdleRx_GetEvent,
    .handle_event_from_isr = UartDmaIdleRx_HandleEventFromISR,
};

static inline void UART_DMA_IDLE_IRQ_HANDLER(UartDmaIdleRx_t *ctx,
                                             UART_HandleTypeDef *huart,
                                             uint16_t size) {
  if ((ctx) != NULL && (huart) == (ctx)->huart) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    UartDmaIdle_Template.handle_event_from_isr((ctx), (size),
                                               &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void init_uart_dma(void) {

  UartDmaIdle_Template.establish_UartDmaIdleRx_t(
      &uart1_dma_idle_rx, Electronic_SPEED_CONTROL_SIMULATION_HUART,
      UART1_DMA_RX_BUFFER_SIZE, (TaskHandle_t)UART_SOLVEHandle);
}

/**
 * @brief UART接收事件回调函数
 *
 * @param huart
 * @param Size
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {

  UART_DMA_IDLE_IRQ_HANDLER(&uart1_dma_idle_rx, huart, Size);
}