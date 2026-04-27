#include "uart_task.h"
#include "CRC_check.h"
#include "Double_buffer.h"
#include "RingBuffer.h"
#include "freertos_handle.h"
#include "projdefs.h"
#include "protocol.h"
#include "robotic_arm.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

extern UartDmaIdleRx_t uart1_dma_idle_rx;
extern RoboticArmState robotic_arm_state;

UartTaskParser_t parser_uart1;
uint8_t uart_rx_buffer[UART_TASK_RING_BUFFER_SIZE];

void UartTaskParser_Init(UartTaskParser_t *parser, uint8_t *buffer) {
  if (parser == NULL) {
    return;
  }
  (void)RingBuffer_Init(&parser->ring_buffer, UART_TASK_RING_BUFFER_SIZE,
                        1U, buffer);
  parser->state = UART_TASK_PARSE_STATE_SEARCH_SOF0;
  parser->frame_index = 0U;
  parser->expected_frame_size = 0U;
  parser->overflow_bytes = 0U;
  parser->dropped_bytes = 0U;
  parser->checksum_error_count = 0U;
}

/**
 * @brief 字节级有限状态机解析器
 *
 * 状态转换:
 *   SEARCH_SOF0 → SEARCH_SOF1 → READ_HEADER → READ_REMAINING → OK
 *        ↑            ↑              ↓(CMD未知)    ↓(CRC失败)
 *        └────────────┴──────────────┴─────────────┘
 *
 * @return UART_TASK_PARSE_OK            — 成功解析一帧
 *         UART_TASK_PARSE_NEED_MORE_DATA — 数据不足
 *         UART_TASK_PARSE_NONE           — 缓冲区空或参数无效
 */
UartTaskParseStatus_t UartTaskParser_ParseFrame(UartTaskParser_t *parser) {
  if (parser == NULL)
    return UART_TASK_PARSE_NONE;

  uint8_t byte;

  while (RingBuffer_GetUnreadCount(&parser->ring_buffer) > 0U) {

    switch (parser->state) {

    /* ---- 搜索 SOF0 (0xA5) ---- */
    case UART_TASK_PARSE_STATE_SEARCH_SOF0:
      RingBuffer_Read(&parser->ring_buffer, 1U, &byte);
      if (byte == PROTOCOL_SOF0) {
        parser->frame_buffer[0] = byte;
        parser->frame_index = 1U;
        parser->state = UART_TASK_PARSE_STATE_SEARCH_SOF1;
      } else {
        parser->dropped_bytes++;
      }
      break;

    /* ---- 搜索 SOF1 (0x5A) ---- */
    case UART_TASK_PARSE_STATE_SEARCH_SOF1:
      /* 确保至少能窥视 1 字节 */
      if (RingBuffer_GetUnreadCount(&parser->ring_buffer) == 0U) {
        return UART_TASK_PARSE_NEED_MORE_DATA;
      }
      RingBuffer_PeekByte(&parser->ring_buffer, &byte, 0U);
      if (byte == PROTOCOL_SOF1) {
        /* SOF1 匹配，消耗该字节 */
        RingBuffer_Read(&parser->ring_buffer, 1U, &byte);
        parser->frame_buffer[1] = byte;
        parser->frame_index = 2U;
        parser->state = UART_TASK_PARSE_STATE_READ_HEADER;
      } else if (byte == PROTOCOL_SOF0) {
        /* 可能是新的 SOF0，丢弃旧的 SOF0 保留新字节 */
        RingBuffer_Read(&parser->ring_buffer, 1U, &byte);
        parser->dropped_bytes++;
        /* frame_buffer[0] 已被覆盖为新的 0xA5，frame_index 仍为 1 */
        /* 状态保持 SEARCH_SOF1 */
      } else {
        /* 两个字节都不匹配，丢弃 SOF0 和当前字节 */
        RingBuffer_Read(&parser->ring_buffer, 1U, &byte);
        parser->dropped_bytes += 2U;
        parser->state = UART_TASK_PARSE_STATE_SEARCH_SOF0;
      }
      break;

    /* ---- 读取 SEQ + CMD (2 字节) ---- */
    case UART_TASK_PARSE_STATE_READ_HEADER:
      if (RingBuffer_GetUnreadCount(&parser->ring_buffer) < 2U) {
        return UART_TASK_PARSE_NEED_MORE_DATA;
      }
      RingBuffer_Read(&parser->ring_buffer, 2U,
                      &parser->frame_buffer[parser->frame_index]);
      parser->frame_index += 2U;

      /* 根据 CMD 确定帧长 */
      parser->expected_frame_size =
          protocol_get_frame_size(parser->frame_buffer[3]);
      if (parser->expected_frame_size == 0U) {
        /* 未知 CMD，帧可能未对齐，丢弃已缓存的 SOF0 并重新搜索 */
        parser->dropped_bytes++;
        parser->state = UART_TASK_PARSE_STATE_SEARCH_SOF0;
        break;
      }
      parser->state = UART_TASK_PARSE_STATE_READ_REMAINING;
      /* 继续执行 READ_REMAINING */
      /* fallthrough */

    /* ---- 读取 Payload + CRC16 ---- */
    case UART_TASK_PARSE_STATE_READ_REMAINING: {
      uint8_t remaining = parser->expected_frame_size - parser->frame_index;
      if (RingBuffer_GetUnreadCount(&parser->ring_buffer) < remaining) {
        return UART_TASK_PARSE_NEED_MORE_DATA;
      }
      RingBuffer_Read(&parser->ring_buffer, remaining,
                      &parser->frame_buffer[parser->frame_index]);

      /* CRC16 校验 */
      if (crc16_verify_checksum(parser->frame_buffer,
                                 parser->expected_frame_size)) {
        /* 校验通过 */
        parser->state = UART_TASK_PARSE_STATE_SEARCH_SOF0;
        parser->frame_index = 0U;
        return UART_TASK_PARSE_OK;
      }

      /* 校验失败，丢弃整个帧并重新同步 */
      parser->checksum_error_count++;
      parser->state = UART_TASK_PARSE_STATE_SEARCH_SOF0;
      parser->frame_index = 0U;
      return UART_TASK_PARSE_NONE;
    }

    default:
      parser->state = UART_TASK_PARSE_STATE_SEARCH_SOF0;
      parser->frame_index = 0U;
      break;
    }
  }

  /* 环形缓冲区已空 */
  return (parser->state == UART_TASK_PARSE_STATE_SEARCH_SOF0 ||
          parser->frame_index > 0U)
             ? UART_TASK_PARSE_NEED_MORE_DATA
             : UART_TASK_PARSE_NONE;
}

void UART_Task(void *argument) {

  UartTaskParser_Init(&parser_uart1, uart_rx_buffer);

  for (;;) {
    /* 等待 DMA 空闲中断发来的通知 */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    /* 服务解析器，直到无更多完整帧可获取 */
    UartTaskParseStatus_t status;
    do {
      status = UartTaskParser_ParseFrame(&parser_uart1);
      if (status == UART_TASK_PARSE_OK) {
        /* 解包电机反馈帧，更新各关节 Motor 状态 */
        float angles[PROTOCOL_MOTOR_COUNT];
        uint8_t seq;
        if (protocol_unpack_motor_fb(parser_uart1.frame_buffer,
                                      angles, &seq)) {
          /* 写入各关节空闲缓冲区（仅角度，速度无反馈源故置零） */
          Motor *motor;

          motor = (Motor *)robotic_arm_state.joint1
                       .buffer[robotic_arm_state.joint1.free_buffer_index];
          motor->angle_feedback = angles[0];
          motor->speed_feedback = 0.0f;

          motor = (Motor *)robotic_arm_state.joint2
                       .buffer[robotic_arm_state.joint2.free_buffer_index];
          motor->angle_feedback = angles[1];
          motor->speed_feedback = 0.0f;

          motor = (Motor *)robotic_arm_state.joint3
                       .buffer[robotic_arm_state.joint3.free_buffer_index];
          motor->angle_feedback = angles[2];
          motor->speed_feedback = 0.0f;

          motor = (Motor *)robotic_arm_state.joint4
                       .buffer[robotic_arm_state.joint4.free_buffer_index];
          motor->angle_feedback = angles[3];
          motor->speed_feedback = 0.0f;

          motor = (Motor *)robotic_arm_state.gripper
                       .buffer[robotic_arm_state.gripper.free_buffer_index];
          motor->angle_feedback = angles[4];
          motor->speed_feedback = 0.0f;

          /* 切换所有关节双缓冲 */
          DoubleBuffer_Template.Buffer_event(&robotic_arm_state.joint1);
          DoubleBuffer_Template.Buffer_event(&robotic_arm_state.joint2);
          DoubleBuffer_Template.Buffer_event(&robotic_arm_state.joint3);
          DoubleBuffer_Template.Buffer_event(&robotic_arm_state.joint4);
          DoubleBuffer_Template.Buffer_event(&robotic_arm_state.gripper);

          /* 通知控制任务运行 PID */
          xTaskNotifyGive(Control_solveHandle);
        }
      }
    } while (status == UART_TASK_PARSE_OK);
  }
}
