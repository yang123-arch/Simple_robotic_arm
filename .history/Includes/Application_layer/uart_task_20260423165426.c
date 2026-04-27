#include "uart_task.h"
#include "CRC_check.h"
#include "projdefs.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

extern UartDmaIdleRx_t uart1_dma_idle_rx;

RingBuffer uart_rx_ring_buffer;
uint8_t uart_rx_buffer[UART_TASK_RING_BUFFER_SIZE];

void UartTaskParser_Init(UartTaskParser_t *parser, uint8_t *buffer) {
  if (parser == NULL) {
    return;
  }
  (void)RingBuffer_Init(&uart_rx_ring_buffer, UART_TASK_RING_BUFFER_SIZE,
                        sizeof(UartFeedbackFrame_t), buffer);
  parser->overflow_bytes = 0;
  parser->dropped_bytes = 0;
  parser->checksum_error_count = 0;
}
/**
 * @brief 从环形缓冲区中解析一帧数据
 * @param parser 指向UART解析器结构体的指针
 * @param out_frame 输出参数，用于存储解析到的帧
 * @return 解析状态（详见UartTaskParseStatus_t定义）
 * @note
 */
UartTaskParseStatus_t
UartTaskParser_ParseFrame(UartTaskParser_t *parser,
                          UartFeedbackFrame_t *out_frame) {

  if ((parser == NULL) || (out_frame == NULL))
    return UART_TASK_PARSE_NONE;
  if (RingBuffer_GetUnreadCount(&parser->ring_buffer) == 0)
    return UART_TASK_PARSE_NONE;
  if (RingBuffer_GetUnreadCount(&parser->ring_buffer) < UART_FEEDBACK_FRAME)
    return UART_TASK_PARSE_NEED_MORE_DATA;

  /* 1. 查找SOF */
  while (RingBuffer_GetUnreadCount(&parser->ring_buffer) >= 32) {
    uint8_t sof0, sof1;
    (void)UartTaskRing_Peek(parser, 0, &sof0);
    (void)UartTaskRing_Peek(parser, 1, &sof1);
    if ((sof0 == UART_FEEDBACK_FRAME_SOF0) &&
        (sof1 == UART_FEEDBACK_FRAME_SOF1)) {
      break;
    }
    UartTaskRing_Discard(parser, 1);
    parser->dropped_bytes++;
  }
  if (parser->count < UART_FEEDBACK_FRAME)
    return UART_TASK_PARSE_NEED_MORE_DATA;

  /* 2. 环形拷贝一帧数据 */
  for (i = 0; i < UART_FEEDBACK_FRAME; i++) {
    idx = (parser->tail + i) % UART_TASK_RING_BUFFER_SIZE;
    frame_ptr[i] = parser->ring[idx];
  }

  /* 3. 移除已解析的帧数据 */
  UartTaskRing_Discard(parser, UART_FEEDBACK_FRAME);

  return UART_TASK_PARSE_OK;
}

void UART_Task(void *argument) {
  UartTaskParser_t parser;
  UartFeedbackFrame_t frame;

  UartTaskParser_Init(&parser);
  /* 假设 uartDmaCtx 已在驱动层初始化，并与硬件绑定 */

  for (;;) {
    /* 等待 DMA 空闲中断发来的通知 */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    /* 服务解析器，直到无更多数据可获取 */
    UartTaskParseStatus_t status;
    do {
      status = UartTaskParser_Service(&parser, &uart1_dma_idle_rx, &frame);
      if (status == UART_TASK_PARSE_OK) {
        /* 处理解析成功的帧 */
        ProcessFeedbackFrame(&frame);
        /* 若未在 ParseFrame 中丢弃数据，则需手动丢弃 */
        // UartTaskRing_Discard(&parser, UART_FEEDBACK_FRAME);
      }
      /* 其他状态（如 NEED_MORE_DATA）可忽略，继续循环 */
    } while (status != UART_TASK_PARSE_NONE); /* NONE 表示无数据可取 */
  }
}