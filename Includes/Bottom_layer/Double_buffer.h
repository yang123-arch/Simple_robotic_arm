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

  volatile uint8_t
      free_buffer_index; //无数据的缓冲区索引，ISR写入数据后切换到另一个缓冲区
  volatile uint8_t
      busy_buffer_index; //填充完数据的缓冲区索引，任务读取数据后切换到另一个缓冲区
  volatile uint8_t event_ready;
} DoubleBuffer_t;

typedef struct {
  void (*init_Buffer)(
      DoubleBuffer_t *ctx, void *buffer0, void *buffer1,
      uint16_t buffer_size); //初始化双缓冲上下文，设置缓冲区指针和大小
  void (*reset_Buffer)(DoubleBuffer_t *ctx); //重置双缓冲状态，清空数据
  void (*Buffer_event)(
      DoubleBuffer_t *ctx); //切换缓冲区并获取当前活动缓冲区指针

} DoubleBuffer_Control;

extern const DoubleBuffer_Control DoubleBuffer_Template;

#ifdef __cplusplus
}
#endif

#endif /* BOTTOM_LAYER_DOUBLE_BUFFER_H */
