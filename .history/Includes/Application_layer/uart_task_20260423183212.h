#ifndef UART_TASK_H
#define UART_TASK_H

#include <stdint.h>

#include "RingBuffer.h"
#include "uart_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 反馈帧格式:
 * Byte0  : SOF0 (0x5A)       - 帧起始符0
 * Byte1  : 字节数  32         - 帧起始符1
 * Byte2  : LEN                - 有效载荷字节数
 * (0~UART_FEEDBACK_FRAME_MAX_PAYLOAD_LEN) Byte3  : CMD                -
 * 反馈命令ID Byte4~ : Payload            - 有效载荷数据 Last   : CHECKSUM -
 * 校验和 = XOR(Byte2 到 Payload最后一个字节)
 */

/* 帧起始符定义 */
#define UART_FEEDBACK_FRAME_SOF0 0x5AU // 帧起始符第一个字节
#define UART_FEEDBACK_FRAME_SOF1 0x1AU // 帧起始符第二个字节（字节数，32）

/* 帧长度相关定义 */
#define UART_FEEDBACK_FRAME 32U // 帧固定开销32

/* 环形缓冲区大小 */
#define UART_TASK_RING_BUFFER_SIZE 512U // 环形缓冲区总字节数

/**
 * @brief 解析状态枚举
 */
typedef enum {
  UART_TASK_PARSE_NONE = 0, // 无有效帧（参数错误或缓冲区空）
  UART_TASK_PARSE_OK,       // 解析成功，得到一帧有效数据
  UART_TASK_PARSE_NEED_MORE_DATA // 数据不足，需要更多字节才能解析
} UartTaskParseStatus_t;

/**
 * @brief 解析后的反馈帧结构体
 */
#pragma pack(1)
typedef struct {
  uint8_t header; /* 0xA6 */
  uint8_t length; /* 32 */
  uint8_t seq;
  float j1;
  float j2;
  float j3;
  float j4;
  float gripper;
  uint8_t online;     /* Online 枚举值 */
  uint8_t status;     /* Status 枚举值 */
  uint8_t error_code; /* ErrorCode 枚举值 */
  uint8_t reserved[4];
  uint16_t checksum;
} UartFeedbackFrame_t;
#pragma pack()
/**
 * @brief UART解析器结构体（包含环形缓冲区与统计信息）
 */
typedef struct {
  RingBuffer ring_buffer;
  uint32_t overflow_bytes; // 缓冲区溢出时被覆盖的字节数统计
  uint32_t dropped_bytes;  // 解析过程中丢弃的无效字节数统计
  uint32_t checksum_error_count; // 校验和错误次数统计
} UartTaskParser_t;

extern UartTaskParser_t parser_uart1;

#ifdef __cplusplus
}
#endif

#endif /* UART_TASK_H */