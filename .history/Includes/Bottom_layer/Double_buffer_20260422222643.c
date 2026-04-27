#include "Double_buffer.h"

#include "stm32f4xx_hal.h"
#include <string.h>

void DoubleBuffer_Reset(DoubleBuffer_t *ctx) {
  if (ctx == NULL) {
    return;
  }

  ctx->free_buffer_index = 0U;
  ctx->busy_buffer_index = 1U;
  ctx->event_ready = 0U;
  memset(ctx->buffer[0], 0, ctx->buffer_size);
  memset(ctx->buffer[1], 0, ctx->buffer_size);
}

void DoubleBuffer_Init(DoubleBuffer_t *ctx, uint8_t *buffer0, uint8_t *buffer1,
                       uint16_t buffer_size) {
  if (ctx == NULL) {
    return;
  }

  ctx->buffer[0] = buffer0;
  ctx->buffer[1] = buffer1;
  ctx->buffer_size = buffer_size;

  DoubleBuffer_Reset(ctx);
}

static inline void DoubleBuffer_SwitchActiveBuffer(DoubleBuffer_t *ctx) {
  if (ctx == NULL) {
    return;
  }

  ctx->free_buffer_index ^= 1U;
  ctx->busy_buffer_index ^= 1U;
}

void DoubleBuffer_Event(DoubleBuffer_t *ctx) {
  if (ctx == NULL) {
    return;
  }

  DoubleBuffer_SwitchActiveBuffer(ctx);

  ctx->event_ready = 1U;
}

const DoubleBuffer_Control DoubleBuffer_Template = {
    .init_Buffer = DoubleBuffer_Init,
    .reset_Buffer = DoubleBuffer_Reset,
    .Buffer_event = DoubleBuffer_Event,
};