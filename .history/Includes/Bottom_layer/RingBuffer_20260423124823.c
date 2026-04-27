#include "RingBuffer.h"

#include <stddef.h>
#include <string.h>

/**
 * @brief 返回两个 uint32_t 值中的较小值
 *
 * @param a
 * @param b
 * @return uint32_t
 */
static inline uint32_t ringBuffer_MinU32(uint32_t a, uint32_t b) {
  return (a < b) ? a : b;
}

/**
 * @brief 判断是否为 2 的幂
 *
 * @param x
 * @return uint8_t
 */
static inline uint8_t RING_BUFFER_IS_POWER_OF_2(uint32_t x) {
  return ((x != 0U) && ((x & (x - 1U)) == 0U)) ? 1U : 0U;
}

/**
 * @brief 将索引回卷到缓冲区范围内
 *
 * @param index
 * @param max_elements
 * @return uint32_t
 * @note 索引单调递增，实际访问时再回卷
 */
static inline uint32_t ringBuffer_WrapIndex(uint32_t index,
                                            uint32_t max_elements) {
  if (RING_BUFFER_IS_POWER_OF_2(max_elements)) {
    return index & (max_elements - 1U);
  }
  return index % max_elements;
}

static uint8_t ringBuffer_IsConfigValid(uint64_t element_size,
                                        uint32_t max_elements) {
  if ((element_size == 0U) || (max_elements == 0U)) {
    return 0U;
  }
  return 1U;
}

uint32_t ringBuffer_GetUnreadCount(uint32_t read_index, uint32_t write_index,
                                   uint32_t max_elements) {
  uint32_t unread;

  if (max_elements == 0U) {
    return 0U;
  }

  unread = write_index - read_index;

  /* 状态保护：如果索引关系异常，限制到容量上限 */
  if (unread > max_elements) {
    unread = max_elements;
  }

  return unread;
}

uint32_t ringBuffer_GetFreeCount(uint32_t read_index, uint32_t write_index,
                                 uint32_t max_elements) {
  uint32_t unread;

  if (max_elements == 0U) {
    return 0U;
  }

  unread = ringBuffer_GetUnreadCount(read_index, write_index, max_elements);
  return max_elements - unread;
}

uint8_t ringBuffer_Write(uint32_t *write_index, uint32_t read_index,
                         const uint8_t *src_data, uint32_t elem_count,
                         uint64_t element_size, uint32_t max_elements,
                         uint8_t *buffer) {
  uint32_t write_pos;
  uint32_t first_chunk;
  uint32_t second_chunk;
  uint32_t unread;
  size_t item_size;
  size_t first_bytes;
  size_t second_bytes;

  if ((write_index == NULL) || (src_data == NULL) || (buffer == NULL) ||
      (ringBuffer_IsConfigValid(element_size, max_elements) == 0U)) {
    return RING_BUFFER_ERROR;
  }

  if (elem_count == 0U) {
    return RING_BUFFER_SUCCESS;
  }

  unread = *write_index - read_index;
  if ((unread > max_elements) ||
      (ringBuffer_GetFreeCount(read_index, *write_index, max_elements) <
       elem_count)) {
    return RING_BUFFER_ERROR;
  }

  item_size = (size_t)element_size;
  write_pos = ringBuffer_WrapIndex(*write_index, max_elements);
  first_chunk = ringBuffer_MinU32(elem_count, max_elements - write_pos);
  second_chunk = elem_count - first_chunk;

  first_bytes = (size_t)first_chunk * item_size;
  (void)memcpy(buffer + ((size_t)write_pos * item_size), src_data, first_bytes);

  if (second_chunk > 0U) {
    second_bytes = (size_t)second_chunk * item_size;
    (void)memcpy(buffer, src_data + first_bytes, second_bytes);
  }

  *write_index += elem_count;
  return (second_chunk > 0U) ? RING_BUFFER_SUCCESS_COIL : RING_BUFFER_SUCCESS;
}

uint8_t ringBuffer_Read(uint32_t *read_index, uint32_t write_index,
                        uint32_t elem_count, uint64_t element_size,
                        uint32_t max_elements, const uint8_t *buffer,
                        uint8_t *output_data) {
  uint32_t read_pos;
  uint32_t first_chunk;
  uint32_t second_chunk;
  uint32_t unread;
  size_t item_size;
  size_t first_bytes;
  size_t second_bytes;

  if ((read_index == NULL) || (buffer == NULL) || (output_data == NULL) ||
      (ringBuffer_IsConfigValid(element_size, max_elements) == 0U)) {
    return RING_BUFFER_ERROR;
  }

  if (elem_count == 0U) {
    return RING_BUFFER_SUCCESS;
  }

  unread = write_index - *read_index;
  if ((unread > max_elements) || (unread < elem_count)) {
    return RING_BUFFER_ERROR;
  }

  item_size = (size_t)element_size;
  read_pos = ringBuffer_WrapIndex(*read_index, max_elements);
  first_chunk = ringBuffer_MinU32(elem_count, max_elements - read_pos);
  second_chunk = elem_count - first_chunk;

  first_bytes = (size_t)first_chunk * item_size;
  (void)memcpy(output_data, buffer + ((size_t)read_pos * item_size),
               first_bytes);

  if (second_chunk > 0U) {
    second_bytes = (size_t)second_chunk * item_size;
    (void)memcpy(output_data + first_bytes, buffer, second_bytes);
  }

  *read_index += elem_count;
  return (second_chunk > 0U) ? RING_BUFFER_SUCCESS_COIL : RING_BUFFER_SUCCESS;
}

uint32_t ringBuffer_ReadAll(uint32_t *read_index, uint32_t write_index,
                            uint64_t element_size, uint32_t max_elements,
                            const uint8_t *buffer, uint8_t *output_data) {
  uint32_t elem_count;
  uint8_t status;

  if ((read_index == NULL) || (buffer == NULL) || (output_data == NULL) ||
      (ringBuffer_IsConfigValid(element_size, max_elements) == 0U)) {
    return 0U;
  }

  elem_count = write_index - *read_index;
  if ((elem_count == 0U) || (elem_count > max_elements)) {
    return 0U;
  }

  status = ringBuffer_Read(read_index, write_index, elem_count, element_size,
                           max_elements, buffer, output_data);
  if (status == RING_BUFFER_ERROR) {
    return 0U;
  }

  return elem_count;
}
