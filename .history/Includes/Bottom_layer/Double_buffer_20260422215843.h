#ifndef BOTTOM_LAYER_DOUBLE_BUFFER_H
#define BOTTOM_LAYER_DOUBLE_BUFFER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 通用双缓冲上下文
 *
 * 设计目标：
 * 1. 与具体外设解耦，仅负责双缓冲切换与事件发布/提取
 * 2. 兼容 ISR(生产者) + 任务(消费者) 的典型使用模式
 */
typedef struct {
  uint8_t *buffer[2];
  uint16_t buffer_size;

  volatile uint8_t active_buffer_index;

  volatile uint8_t event_ready;
} DoubleBuffer_t;

#ifdef __cplusplus
}
#endif

#endif /* BOTTOM_LAYER_DOUBLE_BUFFER_H */
