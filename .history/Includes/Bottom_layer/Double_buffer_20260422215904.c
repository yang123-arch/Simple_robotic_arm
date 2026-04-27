#include "Double_buffer.h"

#include <stddef.h>

void DoubleBuffer_Reset(DoubleBuffer_t *ctx) {
  if (ctx == NULL) {
    return;
  }

  ctx->active_buffer_index = 0U;
  ctx->event_ready = 0U;
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

  ctx->active_buffer_index ^= 1U;
}

void DoubleBuffer_Event(DoubleBuffer_t *ctx, uint8_t **buffer) {
  if (ctx == NULL) {
    return;
  }

  ctx->event_buffer_index = ctx->active_buffer_index;

  ctx->event_ready = 1U;

  DoubleBuffer_SwitchActiveBuffer(ctx);
}
