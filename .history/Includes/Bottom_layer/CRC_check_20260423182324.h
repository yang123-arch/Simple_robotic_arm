#ifndef CRC16_H
#define CRC16_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CRC16_INIT 0xFFFF

/* CRC16 查找表 */
extern const uint16_t CRC16_TABLE[256];

/**
 * @brief 计算 CRC16 校验值
 * @param pchMessage 待校验数据缓冲区
 * @param dwLength   数据长度（字节数）
 * @param wCRC       初始值（通常传入 CRC16_INIT）
 * @return CRC16 校验值
 */
uint16_t crc16_get_checksum(const uint8_t *pchMessage, uint32_t dwLength,
                            uint16_t wCRC);

/**
 * @brief 校验 CRC16 是否正确
 * @param pchMessage 包含数据及尾部 2 字节 CRC 的完整帧缓冲区
 * @param dwLength   完整帧长度（数据 + CRC）
 * @return 1 表示校验通过，0 表示失败
 */
int crc16_verify_checksum(const uint8_t *pchMessage, uint32_t dwLength);

/**
 * @brief 在数据末尾追加 CRC16 校验值
 * @param pchMessage 数据缓冲区（需预留末尾 2 字节空间）
 * @param dwLength   完整帧长度（数据 + CRC）
 */
void crc16_append_checksum(uint8_t *pchMessage, uint32_t dwLength);

#ifdef __cplusplus
}
#endif

#endif /* CRC16_H */