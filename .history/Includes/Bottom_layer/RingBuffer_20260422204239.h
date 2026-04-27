#ifndef RING_BUFFER_H
#define RING_BUFFER_H

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

/**
 * @brief 判断是否为 2 的幂（Linux CIRC_* 推荐容量）
 */
#define RING_BUFFER_IS_POWER_OF_2(x)                                          \
  (((x) != 0U) && (((x) & ((x)-1U)) == 0U))

/**
 * @brief 静态定义一组环形缓冲资源（字节缓冲 + 读写索引）
 *
 * @note 该宏不会动态分配内存，适合裸机/RTOS 场景。
 */
#define RING_BUFFER_DEFINE_STATIC(name, element_type, element_count)          \
  static uint8_t                                                             \
      name[(uint32_t)(element_count) * (uint32_t)sizeof(element_type)] =     \
          {0U};                                                               \
  static uint32_t name##_write_index = 0U;                                   \
  static uint32_t name##_read_index = 0U

/**
 * @brief 获取未读元素个数
 *
 * @note read_index / write_index 为“单调递增计数器”（不强制模回卷）。
 */
uint32_t ringBuffer_GetUnreadCount(uint32_t read_index, uint32_t write_index,
                                   uint32_t max_elements);

/**
 * @brief 获取剩余可写元素个数
 */
uint32_t ringBuffer_GetFreeCount(uint32_t read_index, uint32_t write_index,
                                 uint32_t max_elements);

/**
 * @brief 写入多个元素
 *
 * @param write_index 写索引（单调递增计数器，函数内部自增）
 * @param read_index 读索引（单调递增计数器）
 * @param src_data 源数据
 * @param elem_count 写入元素个数
 * @param element_size 每个元素大小（字节）
 * @param max_elements 环形缓冲区元素容量
 * @param buffer 缓冲区起始地址（长度需 >= max_elements * element_size）
 * @return RING_BUFFER_ERROR / RING_BUFFER_SUCCESS / RING_BUFFER_SUCCESS_COIL
 */
uint8_t ringBuffer_Write(uint32_t *write_index, uint32_t read_index,
                         const uint8_t *src_data, uint32_t elem_count,
                         uint64_t element_size, uint32_t max_elements,
                         uint8_t *buffer);

/**
 * @brief 读取指定数量元素
 *
 * @param read_index 读索引（单调递增计数器，函数内部自增）
 * @param write_index 写索引（单调递增计数器）
 * @param elem_count 读取元素个数
 * @param element_size 每个元素大小（字节）
 * @param max_elements 环形缓冲区元素容量
 * @param buffer 缓冲区起始地址
 * @param output_data 输出缓冲区
 * @return RING_BUFFER_ERROR / RING_BUFFER_SUCCESS / RING_BUFFER_SUCCESS_COIL
 */
uint8_t ringBuffer_Read(uint32_t *read_index, uint32_t write_index,
                        uint32_t elem_count, uint64_t element_size,
                        uint32_t max_elements, const uint8_t *buffer,
                        uint8_t *output_data);

/**
 * @brief 一次性读取所有未读元素
 * @return 实际读取元素个数
 */
uint32_t ringBuffer_ReadAll(uint32_t *read_index, uint32_t write_index,
                            uint64_t element_size, uint32_t max_elements,
                            const uint8_t *buffer, uint8_t *output_data);

#ifdef __cplusplus
}
#endif

#endif /* RING_BUFFER_H */
