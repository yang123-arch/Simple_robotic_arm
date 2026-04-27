#include "uart_task.h"

#include <stddef.h>
#include <string.h>

static uint16_t UartTaskRing_NextIndex(uint16_t index) {
  uint16_t next = (uint16_t)(index + 1U);
  if (next >= UART_TASK_RING_BUFFER_SIZE) {
    next = 0U;
  }
  return next;
}

static void UartTaskRing_Discard(UartTaskParser_t *parser, uint16_t length) {
  while ((length > 0U) && (parser->count > 0U)) {
    parser->tail = UartTaskRing_NextIndex(parser->tail);
    parser->count--;
    length--;
  }
}

static uint8_t UartTaskRing_Peek(const UartTaskParser_t *parser,
                                 uint16_t offset, uint8_t *byte) {
  uint32_t index;

  if ((parser == NULL) || (byte == NULL) || (offset >= parser->count)) {
    return 0U;
  }

  index = ((uint32_t)parser->tail + offset) % UART_TASK_RING_BUFFER_SIZE;
  *byte = parser->ring[index];
  return 1U;
}

static uint8_t UartTaskFrame_CalcChecksum(const UartTaskParser_t *parser,
                                          uint8_t payload_length) {
  uint8_t checksum = 0U;
  uint8_t value = 0U;
  uint16_t i;
  uint16_t bytes_to_xor =
      (uint16_t)payload_length + 2U; /* LEN + CMD + PAYLOAD */

  for (i = 0U; i < bytes_to_xor; i++) {
    (void)UartTaskRing_Peek(parser, (uint16_t)(2U + i), &value);
    checksum ^= value;
  }

  return checksum;
}

void UartTaskParser_Init(UartTaskParser_t *parser) {
  if (parser == NULL) {
    return;
  }
  (void)memset(parser, 0, sizeof(*parser));
}

uint16_t UartTaskParser_GetBufferedBytes(const UartTaskParser_t *parser) {
  if (parser == NULL) {
    return 0U;
  }
  return parser->count;
}

uint16_t UartTaskParser_Write(UartTaskParser_t *parser, const uint8_t *data,
                              uint16_t length) {
  uint16_t i;

  if ((parser == NULL) || (data == NULL) || (length == 0U)) {
    return 0U;
  }

  for (i = 0U; i < length; i++) {
    if (parser->count >= UART_TASK_RING_BUFFER_SIZE) {
      parser->tail = UartTaskRing_NextIndex(parser->tail);
      parser->count--;
      parser->overflow_bytes++;
    }

    parser->ring[parser->head] = data[i];
    parser->head = UartTaskRing_NextIndex(parser->head);
    parser->count++;
  }

  return length;
}

UartTaskParseStatus_t
UartTaskParser_ParseFrame(UartTaskParser_t *parser,
                          UartFeedbackFrame_t *out_frame) {
  uint8_t sof0;
  uint8_t sof1;
  uint8_t payload_length;
  uint8_t command_id;
  uint8_t expected_checksum;
  uint8_t calculated_checksum;
  uint16_t frame_size;
  uint16_t i;

  if ((parser == NULL) || (out_frame == NULL)) {
    return UART_TASK_PARSE_NONE;
  }

  while (parser->count >= 2U) {
    (void)UartTaskRing_Peek(parser, 0U, &sof0);
    (void)UartTaskRing_Peek(parser, 1U, &sof1);

    if ((sof0 == UART_FEEDBACK_FRAME_SOF0) &&
        (sof1 == UART_FEEDBACK_FRAME_SOF1)) {
      break;
      for (;;) {
        while (parser->count >= 2U) {
          (void)UartTaskRing_Peek(parser, 0U, &sof0);
          (void)UartTaskRing_Peek(parser, 1U, &sof1);
          if ((sof0 == UART_FEEDBACK_FRAME_SOF0) &&
              (sof1 == UART_FEEDBACK_FRAME_SOF1)) {
            break;
          }
          UartTaskRing_Discard(parser, 1U);
          parser->dropped_bytes++;
        }

        UartTaskRing_Discard(parser, 1U);
        parser->dropped_bytes++;
      }
      if (parser->count == 0U) {
        return UART_TASK_PARSE_NONE;
      }
      if (parser->count < UART_FEEDBACK_FRAME_MIN_SIZE) {
        return UART_TASK_PARSE_NEED_MORE_DATA;
      }

      while (parser->count >= UART_FEEDBACK_FRAME_MIN_SIZE) {
        (void)UartTaskRing_Peek(parser, 2U, &payload_length);
        parser->checksum_error_count++;

        while (parser->count >= 2U) {
          (void)UartTaskRing_Peek(parser, 0U, &sof0);
          (void)UartTaskRing_Peek(parser, 1U, &sof1);
          if ((sof0 == UART_FEEDBACK_FRAME_SOF0) &&
              (sof1 == UART_FEEDBACK_FRAME_SOF1)) {
            break;
          }
          UartTaskRing_Discard(parser, 1U);
          parser->dropped_bytes++;
        }
        continue;
      }

      if (parser->count == 0U) {
        return UART_TASK_PARSE_NONE;
      }
    }
    return UART_TASK_PARSE_NEED_MORE_DATA;
  }
}

uint8_t UartTaskParser_FetchFromDriver(UartTaskParser_t *parser,
                                       UartDmaIdleRx_t *ctx) {
  uint8_t *rx_buffer = NULL;
  uint16_t rx_length = 0U;

  if ((parser == NULL) || (ctx == NULL)) {
    return 0U;
  }

  if (UartDmaIdle_Template.get_data_from_uart(ctx, &rx_buffer, &rx_length,
                                              NULL) == 0U) {
    return 0U;
  }

  if ((rx_buffer == NULL) || (rx_length == 0U)) {
    return 0U;
  }

  (void)UartTaskParser_Write(parser, rx_buffer, rx_length);
  return 1U;
}

UartTaskParseStatus_t UartTaskParser_Service(UartTaskParser_t *parser,
                                             UartDmaIdleRx_t *ctx,
                                             UartFeedbackFrame_t *out_frame) {
  UartTaskParseStatus_t status;

  if ((parser == NULL) || (ctx == NULL) || (out_frame == NULL)) {
    return UART_TASK_PARSE_NONE;
  }

  status = UartTaskParser_ParseFrame(parser, out_frame);
  if (status == UART_TASK_PARSE_OK) {
    return status;
  }

  while (UartTaskParser_FetchFromDriver(parser, ctx) != 0U) {
    status = UartTaskParser_ParseFrame(parser, out_frame);
    if (status == UART_TASK_PARSE_OK) {
      return status;
    }
  }

  return status;
}
