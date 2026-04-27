#include "uart_task.h"
#include "CRC_check.h"

#include <stddef.h>
#include <string.h>

/**
 * @brief 计算环形缓冲区的下一个索引
 * @param index 当前索引
 * @return 下一个索引，超过缓冲区大小时自动回绕到0
 */
static uint16_t UartTaskRing_NextIndex(uint16_t index) {
  uint16_t next = (uint16_t)(index + 1U);
  if (next >= UART_TASK_RING_BUFFER_SIZE) {
    next = 0U;
  }
  return next;
}

/**
 * @brief 从环形缓冲区尾部丢弃指定长度的数据（直接移动尾部读指针）
 * @param parser 指向UART解析器结构体的指针
 * @param length 要丢弃的字节数
 */
static void UartTaskRing_Discard(UartTaskParser_t *parser, uint16_t length) {
  while ((length > 0U) && (parser->count > 0U)) {
    parser->tail = UartTaskRing_NextIndex(parser->tail);
    parser->count--;
    length--;
  }
}

/**
 * @brief 查看环形缓冲区中指定偏移量的数据（不修改缓冲区）
 * @param parser 指向UART解析器结构体的指针
 * @param offset 指定偏移量
 * @param byte 输出参数，用于存储读取到的字节（只能查看单个uint8_t）
 * @return 1表示成功，0表示参数错误或偏移量超出范围
 */
static uint8_t UartTaskRing_Peek(const UartTaskParser_t *parser,
                                 uint16_t offset, uint8_t *byte) {
  uint32_t index;

  if ((parser == NULL) || (byte == NULL) || (offset >= parser->count)) {
    return 0U;
  }

  // 计算实际索引：尾部 + 偏移量，然后对缓冲区大小取模
  index = ((uint32_t)parser->tail + offset) % UART_TASK_RING_BUFFER_SIZE;
  *byte = parser->ring[index];
  return 1U;
}

/**
 * @brief 计算数据帧的校验和（XOR校验）
 * @param parser 指向UART解析器结构体的指针
 * @param payload_length 有效载荷长度
 * @return 计算得到的校验和
 * @note 校验范围：LEN(1字节) + CMD(1字节) + PAYLOAD(N字节)
 */
static uint8_t UartTaskFrame_CalcChecksum(const UartTaskParser_t *parser,
                                          uint8_t payload_length) {
  uint8_t checksum = 0U;
  uint8_t value = 0U;
  uint16_t i;
  // 总校验字节数：LEN(1) + CMD(1) + PAYLOAD(N)
  uint16_t bytes_to_xor = (uint16_t)payload_length + 2U;

  // 从偏移2开始读取（跳过SOF0和SOF1）
  for (i = 0U; i < bytes_to_xor; i++) {
    (void)UartTaskRing_Peek(parser, (uint16_t)(2U + i), &value);
    checksum ^= value;
  }

  return checksum;
}

/**
 * @brief 初始化UART解析器结构体
 * @param parser 指向UART解析器结构体的指针
 */
void UartTaskParser_Init(UartTaskParser_t *parser) {
  if (parser == NULL) {
    return;
  }
  // 清空整个结构体
  (void)memset(parser, 0, sizeof(*parser));
}

/**
 * @brief 获取环形缓冲区中已缓存的字节数
 * @param parser 指向UART解析器结构体的指针
 * @return 已缓存的字节数，参数错误时返回0
 */
uint16_t UartTaskParser_GetBufferedBytes(const UartTaskParser_t *parser) {
  if (parser == NULL) {
    return 0U;
  }
  return parser->count;
}

/**
 * @brief 获取帧数量（根据指定帧长度计算当前缓冲区中完整帧的数量）
 *
 * @param parser
 * @param frame_length
 * @return uint16_t
 */
uint16_t UartTaskParser_GetBufferedFrameCount(const UartTaskParser_t *parser,
                                              uint16_t frame_length) {
  if (parser == NULL || frame_length == 0) {
    return 0U;
  }
  uint16_t bytes_avail =
      (parser->head + UART_TASK_RING_BUFFER_SIZE - parser->tail) %
      UART_TASK_RING_BUFFER_SIZE;
  return bytes_avail / frame_length; // 整除得到完整帧数
}

/**
 * @brief 向环形缓冲区写入数据
 * @param parser 指向UART解析器结构体的指针
 * @param data 要写入的数据指针
 * @param length 要写入的字节数
 * @return 实际写入的字节数
 * @note 如果缓冲区满，会覆盖旧数据，并记录溢出字节数
 */
uint16_t UartTaskParser_Write(UartTaskParser_t *parser, const uint8_t *data,
                              uint16_t length) {
  uint16_t i;

  if ((parser == NULL) || (data == NULL) || (length == 0U)) {
    return 0U;
  }

  for (i = 0U; i < length; i++) {
    // 如果缓冲区已满，丢弃最旧的数据（移动尾部）
    if (parser->count >= UART_TASK_RING_BUFFER_SIZE) {
      parser->tail = UartTaskRing_NextIndex(parser->tail);
      parser->count--;
      parser->overflow_bytes++;
    }

    // 写入新数据到头部，然后移动头部
    parser->ring[parser->head] = data[i];
    parser->head = UartTaskRing_NextIndex(parser->head);
    parser->count++;
  }

  return length;
}

/**
 * @brief 从环形缓冲区中解析一帧数据
 * @param parser 指向UART解析器结构体的指针
 * @param out_frame 输出参数，用于存储解析到的帧
 * @return 解析状态（详见UartTaskParseStatus_t定义）
 * @note 此函数存在逻辑错误（break后有不可达代码），仅按原代码添加注释
 */
UartTaskParseStatus_t
UartTaskParser_ParseFrame(UartTaskParser_t *parser,
                          UartFeedbackFrame_t *out_frame) {
  uint8_t sof0;
  uint8_t sof1;

  if ((parser == NULL) || (out_frame == NULL)) {
    return UART_TASK_PARSE_NONE;
  }

  if (parser->count == 0U) {
    return UART_TASK_PARSE_NONE;
  }
  if (parser->count < UART_FEEDBACK_FRAME) {
    return UART_TASK_PARSE_NEED_MORE_DATA;
  }
  while (parser->count >= 2U) {
    // 确保至少有2字节（SOF0 + SOF1）
    // 读取前两个字节，检查是否为帧起始符
    (void)UartTaskRing_Peek(parser, 0U, &sof0);
    (void)UartTaskRing_Peek(parser, 1U, &sof1);

    if ((sof0 == UART_FEEDBACK_FRAME_SOF0) &&
        (sof1 == UART_FEEDBACK_FRAME_SOF1)) {
      break;
    }
    // 丢弃1字节，继续查找SOF
    UartTaskRing_Discard(parser, 1U);
    parser->dropped_bytes++;
  }
  if (parser->count < UART_FEEDBACK_FRAME) {
    return UART_TASK_PARSE_NEED_MORE_DATA;
  }

  return UART_TASK_PARSE_OK;
}

/**
 * @brief 从UART DMA驱动获取数据并写入解析器缓冲区
 * @param parser 指向UART解析器结构体的指针
 * @param ctx 指向UART DMA空闲接收上下文的指针
 * @return 1表示成功获取并写入数据，0表示失败
 */
uint8_t UartTaskParser_FetchFromDriver(UartTaskParser_t *parser,
                                       UartDmaIdleRx_t *ctx) {
  uint8_t *rx_buffer = NULL;
  uint16_t rx_length = 0U;

  if ((parser == NULL) || (ctx == NULL)) {
    return 0U;
  }

  // 从DMA驱动获取接收到的数据
  if (UartDmaIdle_Template.get_data_from_uart(ctx, &rx_buffer, &rx_length,
                                              NULL) == 0U) {
    return 0U;
  }

  if ((rx_buffer == NULL) || (rx_length == 0U)) {
    return 0U;
  }

  // 将获取到的数据写入解析器的环形缓冲区
  (void)UartTaskParser_Write(parser, rx_buffer, rx_length);
  return 1U;
}

/**
 * @brief UART解析器服务函数（整合获取数据和解析帧）
 * @param parser 指向UART解析器结构体的指针
 * @param ctx 指向UART DMA空闲接收上下文的指针
 * @param out_frame 输出参数，用于存储解析到的帧
 * @return 解析状态（详见UartTaskParseStatus_t定义）
 */
UartTaskParseStatus_t UartTaskParser_Service(UartTaskParser_t *parser,
                                             UartDmaIdleRx_t *ctx,
                                             UartFeedbackFrame_t *out_frame) {
  UartTaskParseStatus_t status;

  if ((parser == NULL) || (ctx == NULL) || (out_frame == NULL)) {
    return UART_TASK_PARSE_NONE;
  }

  // 先尝试解析已有数据
  status = UartTaskParser_ParseFrame(parser, out_frame);
  if (status == UART_TASK_PARSE_OK) {
    return status;
  }

  // 如果解析不成功，持续从驱动获取新数据并尝试解析
  while (UartTaskParser_FetchFromDriver(parser, ctx) != 0U) {
    status = UartTaskParser_ParseFrame(parser, out_frame);
    if (status == UART_TASK_PARSE_OK) {
      return status;
    }
  }

  return status;
}
