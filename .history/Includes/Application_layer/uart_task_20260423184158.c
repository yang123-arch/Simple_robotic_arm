#include "uart_task.h"
#include "CRC_check.h"
#include "RingBuffer.h"
#include "projdefs.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

extern UartDmaIdleRx_t uart1_dma_idle_rx;

UartTaskParser_t parser_uart1;
RingBuffer uart_rx_ring_buffer;
uint8_t uart_rx_buffer[UART_TASK_RING_BUFFER_SIZE];

DoubleBuffer_t uart1_double_buffer;
UartFeedbackFrame_t uart1_frame_buffer[2]; // 双缓冲区，每个缓冲区可存储一帧数据

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

  uint8_t byte;
  uint16_t index = 0;

  while (RingBuffer_GetFreeCount(&parser->ring_buffer) >= UART_FEEDBACK_FRAME) {
    RingBuffer_PeekByte(&parser->ring_buffer, &byte, index);
    if (byte != UART_FEEDBACK_FRAME_SOF0) {
      index++;
      continue; // 找到帧起始符，准备解析
    }
    RingBuffer_PeekByte(&parser->ring_buffer, &byte, index + 1);
    if (byte != UART_FEEDBACK_FRAME_SOF1) {
      index++;
      continue; // 第二个起始符不匹配，继续寻找
    }
    // 已找到完整帧头，尝试读取整帧
    if (RingBuffer_GetUnreadCount(&parser->ring_buffer) <
        index + UART_FEEDBACK_FRAME) {
      return UART_TASK_PARSE_NEED_MORE_DATA; // 数据不足，等待更多数据到来
    }
    // 读取完整帧数据到临时缓冲区
    uint8_t temp_frame[UART_FEEDBACK_FRAME];
    for (uint16_t i = 0; i < UART_FEEDBACK_FRAME; i++) {
      RingBuffer_PeekByte(&parser->ring_buffer, &temp_frame[i], index + i);
    }
    memcpy(out_frame, temp_frame, sizeof(UartFeedbackFrame_t));
    return UART_TASK_PARSE_OK;
  }
  return UART_TASK_PARSE_NONE; // 没有找到有效帧
}

void UART_Task(void *argument) {

  UartTaskParser_Init(&parser_uart1, uart_rx_buffer);
  DoubleBuffer_Template.init_Buffer(
      &uart1_double_buffer, (uint8_t *)&uart1_frame_buffer[0],
      (uint8_t *)&uart1_frame_buffer[1], sizeof(UartFeedbackFrame_t));

  for (;;) {
    /* 等待 DMA 空闲中断发来的通知 */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    /* 服务解析器，直到无更多数据可获取 */
    UartTaskParseStatus_t status;
    do {
      status = UartTaskParser_ParseFrame(
          &parser_uart1,
          uart1_double_buffer.buffer[uart1_double_buffer.free_buffer_index]);

      /* 其他状态（如 NEED_MORE_DATA）可忽略，继续循环 */
    } while (status != UART_TASK_PARSE_NONE); /* NONE 表示无数据可取 */
  }
}