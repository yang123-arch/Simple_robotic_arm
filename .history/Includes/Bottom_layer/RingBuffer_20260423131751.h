#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 返回值定义
 */
enum {
  RING_BUFFER_ERROR = 0U,
  RING_BUFFER_SUCCESS = 1U,      // 成功且未跨尾部
  RING_BUFFER_SUCCESS_COIL = 2U, // 成功且发生回卷
};

typedef struct {
  uint8_t *buffer;       // 环形缓冲区起始地址
  uint32_t read_index;   // 读索引（单调递增计数器）
  uint32_t write_index;  // 写索引（单调递增计数器）
  uint32_t max_elements; // 环形缓冲区元素最大容量
  size_t element_size;   // 单个元素大小（字节）
} RingBuffer;

/**
 * @brief 初始化环形缓冲区
 * @param control 环形缓冲区控制结构体指针
 * @param max_elements 最大元素数量
 * @param element_size 单个元素大小（字节）
 * @param buffer 外部提供的缓冲区指针（长度需 >= max_elements * element_size）
 * @return RING_BUFFER_ERROR / RING_BUFFER_SUCCESS
 */
uint8_t RingBuffer_Init(RingBuffer *control, uint32_t max_elements,
                        size_t element_size, uint8_t *buffer);

/**
 * @brief 获取未读元素个数
 */
uint32_t RingBuffer_GetUnreadCount(const RingBuffer *control);

/**
 * @brief 获取剩余可写元素个数
 */
uint32_t RingBuffer_GetFreeCount(const RingBuffer *control);

/**
 * @brief 写入多个元素
 * @param control 环形缓冲区控制结构体指针
 * @param src_data 源数据指针
 * @param elem_count 写入元素个数
 * @return RING_BUFFER_ERROR / RING_BUFFER_SUCCESS / RING_BUFFER_SUCCESS_COIL
 */
uint8_t RingBuffer_Write(RingBuffer *control, const uint8_t *src_data,
                         uint32_t elem_count);

/**
 * @brief 读取指定数量元素
 * @param control 环形缓冲区控制结构体指针
 * @param elem_count 读取元素个数
 * @param output_data 输出缓冲区指针
 * @return RING_BUFFER_ERROR / RING_BUFFER_SUCCESS / RING_BUFFER_SUCCESS_COIL
 */
uint8_t RingBuffer_Read(RingBuffer *control, uint32_t elem_count,
                        uint8_t *output_data);

/**
 * @brief 一次性读取所有未读元素
 * @return 实际读取元素个数
 */
uint32_t RingBuffer_ReadAll(RingBuffer *control, uint8_t *output_data);

#ifdef __cplusplus
}
#endif

#endif /* RING_BUFFER_H */
