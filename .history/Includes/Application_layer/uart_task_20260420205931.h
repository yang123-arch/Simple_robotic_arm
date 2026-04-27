#ifndef UART_TASK_H
#define UART_TASK_H

#include <stdint.h>

#include "uart_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 反馈帧格式:
 * Byte0  : SOF0 (0x5A)
 * Byte1  : SOF1 (0xA5)
 * Byte2  : LEN   (Payload字节数, 0~UART_FEEDBACK_FRAME_MAX_PAYLOAD_LEN)
 * Byte3  : CMD   (反馈命令ID)
 * Byte4~ : Payload
 * Last   : CHECKSUM = XOR(Byte2...Payload最后一个字节)
 */
#define UART_FEEDBACK_FRAME_SOF0 0x5AU
#define UART_FEEDBACK_FRAME_SOF1 0xA5U
#define UART_FEEDBACK_FRAME_FIXED_OVERHEAD 5U
#define UART_FEEDBACK_FRAME_MIN_SIZE UART_FEEDBACK_FRAME_FIXED_OVERHEAD
#define UART_FEEDBACK_FRAME_MAX_PAYLOAD_LEN 64U
#define UART_FEEDBACK_FRAME_MAX_SIZE                                            \
  (UART_FEEDBACK_FRAME_FIXED_OVERHEAD + UART_FEEDBACK_FRAME_MAX_PAYLOAD_LEN)

#define UART_TASK_RING_BUFFER_SIZE 512U

typedef enum {
  UART_TASK_PARSE_NONE = 0,
  UART_TASK_PARSE_OK,
  UART_TASK_PARSE_NEED_MORE_DATA
} UartTaskParseStatus_t;

typedef struct {
  uint8_t command_id;
  uint8_t payload_length;
  uint8_t payload[UART_FEEDBACK_FRAME_MAX_PAYLOAD_LEN];
} UartFeedbackFrame_t;

typedef struct {
  uint8_t ring[UART_TASK_RING_BUFFER_SIZE];
  uint16_t head;
  uint16_t tail;
  uint16_t count;
  uint32_t overflow_bytes;
  uint32_t dropped_bytes;
  uint32_t checksum_error_count;
} UartTaskParser_t;

void UartTaskParser_Init(UartTaskParser_t *parser);
uint16_t UartTaskParser_GetBufferedBytes(const UartTaskParser_t *parser);

uint16_t UartTaskParser_Write(UartTaskParser_t *parser, const uint8_t *data,
                              uint16_t length);

UartTaskParseStatus_t UartTaskParser_ParseFrame(UartTaskParser_t *parser,
                                                UartFeedbackFrame_t *out_frame);

uint8_t UartTaskParser_FetchFromDriver(UartTaskParser_t *parser,
                                       UartDmaIdleRx_t *ctx);

UartTaskParseStatus_t UartTaskParser_Service(UartTaskParser_t *parser,
                                             UartDmaIdleRx_t *ctx,
                                             UartFeedbackFrame_t *out_frame);

#ifdef __cplusplus
}
#endif

#endif /* UART_TASK_H */
