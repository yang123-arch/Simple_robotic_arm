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

  uint8_t active_buffer_index;

  volatile uint8_t event_buffer_index;
  volatile uint16_t event_length;
  volatile uint32_t event_type;
  volatile uint8_t event_ready;
} DoubleBuffer_t;

/**
 * @brief 初始化双缓冲上下文
 */
void DoubleBuffer_Init(DoubleBuffer_t *ctx, uint8_t *buffer0, uint8_t *buffer1,
                       uint16_t buffer_size);

/**
 * @brief 仅重置运行态状态（不修改 buffer 指针与容量）
 */
void DoubleBuffer_Reset(DoubleBuffer_t *ctx);

/**
 * @brief 按索引获取缓冲区（0/1）
 * @return 缓冲区指针，参数错误返回NULL
 */
uint8_t *DoubleBuffer_GetBufferByIndex(DoubleBuffer_t *ctx, uint8_t index);

/**
 * @brief 按索引获取只读缓冲区（0/1）
 * @return 缓冲区指针，参数错误返回NULL
 */
const uint8_t *DoubleBuffer_GetBufferByIndexConst(const DoubleBuffer_t *ctx,
                                                  uint8_t index);

/**
 * @brief 获取当前活动缓冲区
 */
uint8_t *DoubleBuffer_GetActiveBuffer(DoubleBuffer_t *ctx);

/**
 * @brief 获取当前活动缓冲区（只读）
 */
const uint8_t *DoubleBuffer_GetActiveBufferConst(const DoubleBuffer_t *ctx);

/**
 * @brief 获取活动缓冲区索引（0/1）
 */
uint8_t DoubleBuffer_GetActiveIndex(const DoubleBuffer_t *ctx);

/**
 * @brief 设置活动缓冲区索引（仅允许0/1）
 */
void DoubleBuffer_SetActiveIndex(DoubleBuffer_t *ctx, uint8_t index);

/**
 * @brief 切换活动缓冲区（0<->1）
 */
void DoubleBuffer_SwitchActiveBuffer(DoubleBuffer_t *ctx);

/**
 * @brief 发布一次事件
 *
 * 语义：默认将“当前活动缓冲区”作为本次完成缓冲区。
 * 常用于中断中“DMA 本轮接收完成”后的事件登记。
 *
 * @param length 本次有效数据长度
 * @param event_type 业务自定义事件类型（通用 uint32_t）
 * @param switch_active_buffer 非0时，发布事件后切换活动缓冲区
 */
void DoubleBuffer_PublishEvent(DoubleBuffer_t *ctx, uint16_t length,
                               uint32_t event_type,
                               uint8_t switch_active_buffer);

/**
 * @brief 获取一次事件并清除事件就绪标志
 * @return 1 成功，0 无事件或参数错误
 */
uint8_t DoubleBuffer_GetEvent(DoubleBuffer_t *ctx, uint8_t **buffer,
                              uint16_t *length, uint32_t *event_type);

/**
 * @brief 查询是否有事件就绪
 * @return 1 有事件，0 无事件或参数错误
 */
uint8_t DoubleBuffer_IsEventReady(const DoubleBuffer_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* BOTTOM_LAYER_DOUBLE_BUFFER_H */
