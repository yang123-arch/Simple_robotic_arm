#include "RingBuffer.h"
#include <string.h>

/**
 * @brief 返回两个 uint32_t 值中的较小值
 */
static inline uint32_t ringBuffer_MinU32(uint32_t a, uint32_t b) {
  return (a < b) ? a : b;
}

/**
 * @brief 判断是否为 2 的幂
 */
static inline uint8_t ringBuffer_IsPowerOf2(uint32_t x) {
  return ((x != 0U) && ((x & (x - 1U)) == 0U)) ? 1U : 0U;
}

/**
 * @brief 将索引回卷到缓冲区范围内
 * @note 索引单调递增，实际访问时再回卷
 */
static inline uint32_t ringBuffer_WrapIndex(uint32_t index,
                                            uint32_t max_elements) {
  if (ringBuffer_IsPowerOf2(max_elements)) {
    return index & (max_elements - 1U); // 2的幂时用位运算优化
  }
  return index % max_elements;
}

/**
 * @brief 检查配置是否有效
 */
static inline uint8_t ringBuffer_IsConfigValid(const RingBuffer *control) {
  return (control != NULL) && (control->buffer != NULL) &&
         (control->max_elements > 0U) && (control->element_size > 0U);
}

uint8_t RingBuffer_Init(RingBuffer *control, uint32_t max_elements,
                        size_t element_size, uint8_t *buffer) {
  if ((control == NULL) || (buffer == NULL) || (max_elements == 0U) ||
      (element_size == 0U)) {
    return RING_BUFFER_ERROR;
  }

  control->buffer = buffer;
  control->read_index = 0U;
  control->write_index = 0U;
  control->max_elements = max_elements;
  control->element_size = element_size;
  return RING_BUFFER_SUCCESS;
}

uint32_t RingBuffer_GetUnreadCount(const RingBuffer *control) {
  if (!ringBuffer_IsConfigValid(control)) {
    return 0U;
  }
  // 利用无符号数减法天然模2^32的特性，兼容索引回绕
  uint32_t unread = control->write_index - control->read_index;
  // 状态保护：限制在容量范围内
  return (unread <= control->max_elements) ? unread : control->max_elements;
}

uint32_t RingBuffer_GetFreeCount(const RingBuffer *control) {
  if (!ringBuffer_IsConfigValid(control)) {
    return 0U;
  }
  uint32_t unread = RingBuffer_GetUnreadCount(control);
  return control->max_elements - unread;
}

uint8_t RingBuffer_Write(RingBuffer *control, const uint8_t *src_data,
                         uint32_t elem_count) {
  if (!ringBuffer_IsConfigValid(control) || (src_data == NULL)) {
    return RING_BUFFER_ERROR;
  }
  if (elem_count == 0U) {
    return RING_BUFFER_SUCCESS;
  }

  // 检查空间是否足够
  uint32_t free_count = RingBuffer_GetFreeCount(control);
  if (free_count < elem_count) {
    return RING_BUFFER_ERROR;
  }

  uint32_t write_pos =
      ringBuffer_WrapIndex(control->write_index, control->max_elements);
  uint32_t first_chunk =
      ringBuffer_MinU32(elem_count, control->max_elements - write_pos);
  uint32_t second_chunk = elem_count - first_chunk;

  size_t item_size = control->element_size;
  size_t first_bytes = first_chunk * item_size;
  size_t second_bytes = second_chunk * item_size;

  // 写入第一部分（不回卷）
  (void)memcpy(control->buffer + (write_pos * item_size), src_data,
               first_bytes);
  // 写入第二部分（回卷到缓冲区头部）
  if (second_chunk > 0U) {
    (void)memcpy(control->buffer, src_data + first_bytes, second_bytes);
  }

  control->write_index += elem_count; // 索引单调递增，不回卷
  return (second_chunk > 0U) ? RING_BUFFER_SUCCESS_COIL : RING_BUFFER_SUCCESS;
}

uint8_t RingBuffer_Read(RingBuffer *control, uint32_t elem_count,
                        uint8_t *output_data) {
  if (!ringBuffer_IsConfigValid(control) || (output_data == NULL)) {
    return RING_BUFFER_ERROR;
  }
  if (elem_count == 0U) {
    return RING_BUFFER_SUCCESS;
  }

  // 检查数据是否足够
  uint32_t unread = RingBuffer_GetUnreadCount(control);
  if (unread < elem_count) {
    return RING_BUFFER_ERROR;
  }

  uint32_t read_pos =
      ringBuffer_WrapIndex(control->read_index, control->max_elements);
  uint32_t first_chunk =
      ringBuffer_MinU32(elem_count, control->max_elements - read_pos);
  uint32_t second_chunk = elem_count - first_chunk;

  size_t item_size = control->element_size;
  size_t first_bytes = first_chunk * item_size;
  size_t second_bytes = second_chunk * item_size;

  // 读取第一部分（不回卷）
  (void)memcpy(output_data, control->buffer + (read_pos * item_size),
               first_bytes);
  // 读取第二部分（回卷到缓冲区头部）
  if (second_chunk > 0U) {
    (void)memcpy(output_data + first_bytes, control->buffer, second_bytes);
  }

  control->read_index += elem_count; // 索引单调递增，不回卷
  return (second_chunk > 0U) ? RING_BUFFER_SUCCESS_COIL : RING_BUFFER_SUCCESS;
}

uint32_t RingBuffer_ReadAll(RingBuffer *control, uint8_t *output_data) {
  if (!ringBuffer_IsConfigValid(control) || (output_data == NULL)) {
    return 0U;
  }

  uint32_t elem_count = RingBuffer_GetUnreadCount(control);
  if (elem_count == 0U) {
    return 0U;
  }

  uint8_t status = RingBuffer_Read(control, elem_count, output_data);
  return (status != RING_BUFFER_ERROR) ? elem_count : 0U;
}