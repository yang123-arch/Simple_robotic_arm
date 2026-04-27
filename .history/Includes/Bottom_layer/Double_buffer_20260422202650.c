#include "Double_buffer.h"

#include <stddef.h>

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

void DoubleBuffer_Reset(DoubleBuffer_t *ctx) {
  if (ctx == NULL) {
    return;
  }

  ctx->active_buffer_index = 0U;
  ctx->event_buffer_index = 0U;
  ctx->event_length = 0U;
  ctx->event_type = 0U;
  ctx->event_ready = 0U;
}

uint8_t *DoubleBuffer_GetBufferByIndex(DoubleBuffer_t *ctx, uint8_t index) {
  if ((ctx == NULL) || (index > 1U)) {
    return NULL;
  }

  return ctx->buffer[index];
}

const uint8_t *DoubleBuffer_GetBufferByIndexConst(const DoubleBuffer_t *ctx,
                                                  uint8_t index) {
  if ((ctx == NULL) || (index > 1U)) {
    return NULL;
  }

  return ctx->buffer[index];
}

uint8_t *DoubleBuffer_GetActiveBuffer(DoubleBuffer_t *ctx) {
  if (ctx == NULL) {
    return NULL;
  }

  return DoubleBuffer_GetBufferByIndex(ctx, ctx->active_buffer_index);
}

const uint8_t *DoubleBuffer_GetActiveBufferConst(const DoubleBuffer_t *ctx) {
  if (ctx == NULL) {
    return NULL;
  }

  return DoubleBuffer_GetBufferByIndexConst(ctx, ctx->active_buffer_index);
}

uint8_t DoubleBuffer_GetActiveIndex(const DoubleBuffer_t *ctx) {
  if (ctx == NULL) {
    return 0U;
  }
  return ctx->active_buffer_index;
}

void DoubleBuffer_SetActiveIndex(DoubleBuffer_t *ctx, uint8_t index) {
  if ((ctx == NULL) || (index > 1U)) {
    return;
  }

  ctx->active_buffer_index = index;
}

void DoubleBuffer_SwitchActiveBuffer(DoubleBuffer_t *ctx) {
  if (ctx == NULL) {
    return;
  }

  ctx->active_buffer_index ^= 1U;
}

void DoubleBuffer_PublishEvent(DoubleBuffer_t *ctx, uint16_t length,
                               uint32_t event_type,
                               uint8_t switch_active_buffer) {
  uint16_t event_length;

  if (ctx == NULL) {
    return;
  }

  event_length = length;
  if ((ctx->buffer_size > 0U) && (event_length > ctx->buffer_size)) {
    event_length = ctx->buffer_size;
  }

  ctx->event_buffer_index = ctx->active_buffer_index;
  ctx->event_length = event_length;
  ctx->event_type = event_type;
  ctx->event_ready = 1U;

  if (switch_active_buffer != 0U) {
    DoubleBuffer_SwitchActiveBuffer(ctx);
  }
}

uint8_t DoubleBuffer_GetEvent(DoubleBuffer_t *ctx, uint8_t **buffer,
                              uint16_t *length, uint32_t *event_type) {
  if ((ctx == NULL) || (buffer == NULL) || (length == NULL) ||
      (ctx->event_ready == 0U)) {
    return 0U;
  }

  *buffer = DoubleBuffer_GetBufferByIndex(ctx, ctx->event_buffer_index);
  if (*buffer == NULL) {
    return 0U;
  }

  *length = ctx->event_length;
  if (event_type != NULL) {
    *event_type = ctx->event_type;
  }

  ctx->event_ready = 0U;
  return 1U;
}

uint8_t DoubleBuffer_IsEventReady(const DoubleBuffer_t *ctx) {
  if (ctx == NULL) {
    return 0U;
  }

  return (ctx->event_ready != 0U) ? 1U : 0U;
}
