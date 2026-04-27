#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 机械臂 HIL 仿真串口通信协议
 *
 * 架构: STM32(PID控制器) ←→ UART ←→ ROS2节点 ←→ Gazebo(电调仿真)
 * 控制频率: 1kHz
 *
 * --- 命令帧 (STM32 → Gazebo) 总长 16 字节 ---
 * ┌────────┬────────┬─────┬─────┬────────────────────────┬───────────┐
 * │ SOF0   │ SOF1   │ SEQ │ CMD │  Payload (5×int16)     │ CRC16     │
 * │ 0xA5   │ 0x5A   │ 1B  │ 1B  │  10B (LE)              │ 2B (LE)   │
 * └────────┴────────┴─────┴─────┴────────────────────────┴───────────┘
 * Offset: 0        1        2     3     4..13                  14..15
 *
 * --- 反馈帧 (Gazebo → STM32) 总长 32 字节 ---
 * ┌────────┬────────┬─────┬─────┬──────────────────────────┬────────────┬───────────┐
 * │ SOF0   │ SOF1   │ SEQ │ CMD │  Payload (5×float 角度)   │ 保留扩展    │ CRC16     │
 * │ 0xA5   │ 0x5A   │ 1B  │ 1B  │  20B (LE)                │ 6B         │ 2B (LE)   │
 * └────────┴────────┴─────┴─────┴──────────────────────────┴────────────┴───────────┘
 * Offset: 0        1        2     3     4..23                  24..29       30..31
 *
 * CRC16 覆盖: 全部字段 (含 SOF，不含 CRC 自身)
 * 保留扩展区 (offset 24..29): 当前填零，留给未来扩展（如速度反馈）
 * 多字节字段均为小端序 (Little-Endian)
 * CRC 多项式: 0x8005, 初始值 0xFFFF (与 CRC_check.h 一致)
 * 多字节字段均为小端序 (Little-Endian)
 * ========================================================================== */

/* 帧起始符 */
#define PROTOCOL_SOF0 0xA5U
#define PROTOCOL_SOF1 0x5AU

/* 命令 ID */
#define PROTOCOL_CMD_TORQUE_CMD 0x01U // STM32→Gazebo: 力矩命令
#define PROTOCOL_CMD_MOTOR_FB   0x02U // Gazebo→STM32: 电机角度反馈

/* 帧长度 */
#define PROTOCOL_TORQUE_CMD_FRAME_SIZE 16U
#define PROTOCOL_MOTOR_FB_FRAME_SIZE   32U
#define PROTOCOL_MAX_FRAME_SIZE        32U

/* 帧头长度: SOF(2) + SEQ(1) + CMD(1) */
#define PROTOCOL_HEADER_SIZE 4U
/* CRC16 长度 */
#define PROTOCOL_CRC_SIZE 2U

/* 电机数量 */
#define PROTOCOL_MOTOR_COUNT 5U

/* 根据命令字获取对应帧长度 */
uint8_t protocol_get_frame_size(uint8_t cmd);

/* 打包命令帧 (STM32 → Gazebo) */
void protocol_pack_torque_cmd(uint8_t frame[PROTOCOL_TORQUE_CMD_FRAME_SIZE],
                              uint8_t seq, const int16_t torques[PROTOCOL_MOTOR_COUNT]);

/* 打包反馈帧 (Gazebo → STM32) */
void protocol_pack_motor_fb(uint8_t frame[PROTOCOL_MOTOR_FB_FRAME_SIZE],
                            uint8_t seq, const float angles[PROTOCOL_MOTOR_COUNT]);

/* 解包命令帧，成功返回 1 */
int protocol_unpack_torque_cmd(const uint8_t frame[PROTOCOL_TORQUE_CMD_FRAME_SIZE],
                               int16_t torques[PROTOCOL_MOTOR_COUNT], uint8_t *seq);

/* 解包反馈帧，成功返回 1 */
int protocol_unpack_motor_fb(const uint8_t frame[PROTOCOL_MOTOR_FB_FRAME_SIZE],
                             float angles[PROTOCOL_MOTOR_COUNT], uint8_t *seq);

/* 验证帧 CRC16，成功返回 1 */
int protocol_verify_frame(const uint8_t *frame, uint8_t frame_size);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H */
