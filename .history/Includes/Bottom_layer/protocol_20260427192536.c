#include "protocol.h"
#include "CRC_check.h"

#include <string.h>

/* ==========================================================================
 * 机械臂 HIL 仿真串口通信协议 — 实现
 *
 * 数据流向:
 *   STM32 ──torque cmd(16B)──► Gazebo
 *   STM32 ◄──motor fb(32B)──── Gazebo
 *
 * CRC16 覆盖范围: SEQ + CMD + Payload (不含 SOF)
 * 多字节字段均为小端序
 * ========================================================================== */

/* 根据命令字获取对应帧长度 */
uint8_t protocol_get_frame_size(uint8_t cmd) {
  switch (cmd) {
  case PROTOCOL_CMD_TORQUE_CMD:
    return PROTOCOL_TORQUE_CMD_FRAME_SIZE;
  case PROTOCOL_CMD_MOTOR_FB:
    return PROTOCOL_MOTOR_FB_FRAME_SIZE;
  default:
    return 0U;
  }
}

/* 将 int16_t 按小端序写入 2 字节 */
static inline void write_int16_le(uint8_t *dst, int16_t val) {
  dst[0] = (uint8_t)(val & 0xFF);
  dst[1] = (uint8_t)((val >> 8) & 0xFF);
}

/* 从 2 字节小端序读取 int16_t */
static inline int16_t read_int16_le(const uint8_t *src) {
  return (int16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8));
}

/* 将 float 按小端序写入 4 字节 */
static inline void write_float_le(uint8_t *dst, float val) {
  uint32_t bits;
  memcpy(&bits, &val, sizeof(bits));
  dst[0] = (uint8_t)(bits & 0xFF);
  dst[1] = (uint8_t)((bits >> 8) & 0xFF);
  dst[2] = (uint8_t)((bits >> 16) & 0xFF);
  dst[3] = (uint8_t)((bits >> 24) & 0xFF);
}

/* 从 4 字节小端序读取 float */
static inline float read_float_le(const uint8_t *src) {
  uint32_t bits = (uint32_t)src[0] | ((uint32_t)src[1] << 8) |
                  ((uint32_t)src[2] << 16) | ((uint32_t)src[3] << 24);
  float val;
  memcpy(&val, &bits, sizeof(val));
  return val;
}

/**
 * @brief 打包命令帧
 *
 * @param frame
 * @param seq
 * @param torques
 * @note 帧结构: SOF0 | SOF1 | SEQ | CMD | joint1_torque(2B) | ...
 * gripper_torque(2B) | CRC16(2B)
 */
void protocol_pack_torque_cmd(uint8_t frame[PROTOCOL_TORQUE_CMD_FRAME_SIZE],
                              uint8_t seq,
                              const int16_t torques[PROTOCOL_MOTOR_COUNT]) {
  frame[0] = PROTOCOL_SOF0;
  frame[1] = PROTOCOL_SOF1;
  frame[2] = seq;
  frame[3] = PROTOCOL_CMD_TORQUE_CMD;

  /* Payload: 5 × int16_t = 10 bytes */
  for (uint8_t i = 0; i < PROTOCOL_MOTOR_COUNT; i++) {
    write_int16_le(&frame[4 + i * 2], torques[i]);
  }

  /* CRC16 覆盖 SEQ + CMD + Payload (14 bytes) */
  crc16_append_checksum(frame, PROTOCOL_TORQUE_CMD_FRAME_SIZE);
}

/* 打包反馈帧 (Gazebo → STM32)
 *
 * 帧结构: SOF0 | SOF1 | SEQ | CMD | 5×float_angle(20B) | reserved(6B) |
 * CRC16(2B)
 */
void protocol_pack_motor_fb(uint8_t frame[PROTOCOL_MOTOR_FB_FRAME_SIZE],
                            uint8_t seq,
                            const float angles[PROTOCOL_MOTOR_COUNT]) {
  frame[0] = PROTOCOL_SOF0;
  frame[1] = PROTOCOL_SOF1;
  frame[2] = seq;
  frame[3] = PROTOCOL_CMD_MOTOR_FB;

  /* Payload: 5 × float = 20 bytes (仅角度) */
  for (uint8_t i = 0; i < PROTOCOL_MOTOR_COUNT; i++) {
    write_float_le(&frame[4 + i * 4], angles[i]);
  }

  /* 保留扩展区填零 (6 bytes at offset 24..29) */
  for (uint8_t i = 24; i < 30; i++) {
    frame[i] = 0U;
  }

  /* CRC16 覆盖 SOF + SEQ + CMD + Payload + Reserved (30 bytes) */
  crc16_append_checksum(frame, PROTOCOL_MOTOR_FB_FRAME_SIZE);
}

/* 解包命令帧，校验 SOF/CMD/CRC 后提取数据 */
int protocol_unpack_torque_cmd(
    const uint8_t frame[PROTOCOL_TORQUE_CMD_FRAME_SIZE],
    int16_t torques[PROTOCOL_MOTOR_COUNT], uint8_t *seq) {
  if (frame[0] != PROTOCOL_SOF0 || frame[1] != PROTOCOL_SOF1)
    return 0;
  if (frame[3] != PROTOCOL_CMD_TORQUE_CMD)
    return 0;
  if (!crc16_verify_checksum(frame, PROTOCOL_TORQUE_CMD_FRAME_SIZE))
    return 0;

  *seq = frame[2];
  for (uint8_t i = 0; i < PROTOCOL_MOTOR_COUNT; i++) {
    torques[i] = read_int16_le(&frame[4 + i * 2]);
  }
  return 1;
}

/* 解包反馈帧，校验 SOF/CMD/CRC 后提取数据 */
int protocol_unpack_motor_fb(const uint8_t frame[PROTOCOL_MOTOR_FB_FRAME_SIZE],
                             float angles[PROTOCOL_MOTOR_COUNT], uint8_t *seq) {
  if (frame[0] != PROTOCOL_SOF0 || frame[1] != PROTOCOL_SOF1)
    return 0;
  if (frame[3] != PROTOCOL_CMD_MOTOR_FB)
    return 0;
  if (!crc16_verify_checksum(frame, PROTOCOL_MOTOR_FB_FRAME_SIZE))
    return 0;

  *seq = frame[2];
  for (uint8_t i = 0; i < PROTOCOL_MOTOR_COUNT; i++) {
    angles[i] = read_float_le(&frame[4 + i * 4]);
  }
  return 1;
}

/* 验证帧 CRC16 */
int protocol_verify_frame(const uint8_t *frame, uint8_t frame_size) {
  if (frame == NULL || frame_size <= PROTOCOL_CRC_SIZE)
    return 0;
  if (frame[0] != PROTOCOL_SOF0 || frame[1] != PROTOCOL_SOF1)
    return 0;
  return crc16_verify_checksum(frame, frame_size);
}
