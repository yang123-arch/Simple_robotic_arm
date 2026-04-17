// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "rm_serial_driver/crc.hpp"

namespace rm_serial_driver
{
constexpr uint8_t kCmdPacketHeader = 0xA5;
constexpr uint8_t kFeedbackPacketHeader = 0xA6;
constexpr uint8_t kCmdPacketLength = 32;
constexpr uint8_t kFeedbackPacketLength = 32;

enum class Mode : uint8_t { kNoOp = 0, kJointPosition = 1, kGripperOnly = 2, kHoming = 3 };

enum class CmdAction : uint8_t {
  kNone = 0,
  kExecute = 1,
  kPause = 2,
  kResume = 3,
  kClearFault = 4,
  kEstop = 5
};

enum class Status : uint8_t {
  kInit = 0,
  kReady = 1,
  kRunning = 2,
  kPaused = 3,
  kFault = 4,
  kEstop = 5
};

enum class ErrorCode : uint8_t {
  kOk = 0,
  kCmdCrcFail = 1,
  kCmdLengthInvalid = 2,
  kCmdTimeout = 3,
  kJointStateMissing = 4,
  kControllerInactive = 5,
  kLimitViolation = 6,
  kEstopTriggered = 7,
  kCmdHeaderInvalid = 8,
  kCmdEnumInvalid = 9,
  kCmdValueInvalid = 10,
  kCmdSeqJump = 11
};

enum class Online : uint8_t { kOffline = 0, kOnline = 1 };

struct CmdPacket
{
  uint8_t header = kCmdPacketHeader;
  uint8_t length = kCmdPacketLength;
  uint8_t seq = 0;
  float j1 = 0.0F;
  float j2 = 0.0F;
  float j3 = 0.0F;
  float j4 = 0.0F;
  float gripper = 0.0F;
  uint8_t mode = 0;
  uint8_t cmd_action = 0;
  uint8_t reserved[5] = {0};
  uint16_t checksum = 0;
} __attribute__((packed));
static_assert(sizeof(CmdPacket) == kCmdPacketLength, "CmdPacket size must be 32 bytes");
static_assert(
  offsetof(CmdPacket, checksum) == sizeof(CmdPacket) - sizeof(uint16_t),
  "CmdPacket checksum must be the last 2 bytes");

struct FeedbackPacket
{
  uint8_t header = kFeedbackPacketHeader;
  uint8_t length = kFeedbackPacketLength;
  uint8_t seq = 0;
  float j1 = 0.0F;
  float j2 = 0.0F;
  float j3 = 0.0F;
  float j4 = 0.0F;
  float gripper = 0.0F;
  uint8_t online = 0;
  uint8_t status = 0;
  uint8_t error_code = 0;
  uint8_t reserved[4] = {0};
  uint16_t checksum = 0;
} __attribute__((packed));
static_assert(
  sizeof(FeedbackPacket) == kFeedbackPacketLength, "FeedbackPacket size must be 32 bytes");
static_assert(
  offsetof(FeedbackPacket, checksum) == sizeof(FeedbackPacket) - sizeof(uint16_t),
  "FeedbackPacket checksum must be the last 2 bytes");

inline CmdPacket fromVector(const std::vector<uint8_t> & data)
{
  CmdPacket packet;
  if (data.size() == sizeof(CmdPacket)) {
    std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  }
  return packet;
}

inline std::vector<uint8_t> toVector(const CmdPacket & data)
{
  std::vector<uint8_t> packet(sizeof(CmdPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(CmdPacket), packet.begin());
  return packet;
}

inline std::vector<uint8_t> toVector(const FeedbackPacket & data)
{
  std::vector<uint8_t> packet(sizeof(FeedbackPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(FeedbackPacket), packet.begin());
  return packet;
}

// CRC rule: cover all bytes except the final checksum field.
inline bool verifyFrameCrc(const std::vector<uint8_t> & frame)
{
  if (frame.size() < sizeof(uint16_t)) {
    return false;
  }
  return crc16::Verify_CRC16_Check_Sum(frame.data(), static_cast<uint32_t>(frame.size()));
}

template <typename PacketT>
inline void appendPacketCrc(PacketT & packet)
{
  crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(PacketT));
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
