#ifndef UART_TASK_H
#define UART_TASK_H

#include "stm32f4xx_hal.h"

const uint8_t kCmdPacketHeader = 0x5A;
const uint8_t kFeedbackPacketHeader = 0xA6;
const uint8_t kCmdPacketLength = 32;
const uint8_t kFeedbackPacketLength = 32;

typedef enum {
  kNoOp = 0,
  kJointPosition = 1,
  kGripperOnly = 2,
  kHoming = 3
} Mode;

typedef enum {
  kNone = 0,
  kExecute = 1,
  kPause = 2,
  kResume = 3,
  kClearFault = 4,
  kEstop = 5
} CmdAction;

typedef enum {
  kInit = 0,
  kReady = 1,
  kRunning = 2,
  kPaused = 3,
  kFault = 4,
  kEstop = 5
} Status;

typedef enum {
  kOk = 0,
  kCmdCrcFail = 1,
  kCmdLengthInvalid = 2,
  kCmdTimeout = 3,
  kJointStateMissing = 4,
  kControllerInactive = 5,
  kLimitViolation = 6,
  kEstopTriggered = 7
} ErrorCode;

typedef enum { kOffline = 0, kOnline = 1 } Online;

struct CmdPacket {
  uint8_t header = kCmdPacketHeader;
  uint8_t length = kCmdPacketLength;
  uint8_t seq = 0;
  float j1 = 0.0F;
  float j2 = 0.0F;
  float j3 = 0.0F;
  float j4 = 0.0F;
  float gripper = 0.0F;
  uint8_t mode = static_cast<uint8_t>(Mode::kNoOp);
  uint8_t cmd_action = static_cast<uint8_t>(CmdAction::kNone);
  uint8_t reserved[5] = {0};
  uint16_t checksum = 0;
} __attribute__((packed));
static_assert(sizeof(CmdPacket) == kCmdPacketLength,
              "CmdPacket size must be 32 bytes");

struct FeedbackPacket {
  uint8_t header = kFeedbackPacketHeader;
  uint8_t length = kFeedbackPacketLength;
  uint8_t seq = 0;
  float j1 = 0.0F;
  float j2 = 0.0F;
  float j3 = 0.0F;
  float j4 = 0.0F;
  float gripper = 0.0F;
  uint8_t online = static_cast<uint8_t>(Online::kOnline);
  uint8_t status = static_cast<uint8_t>(Status::kInit);
  uint8_t error_code = static_cast<uint8_t>(ErrorCode::kOk);
  uint8_t reserved[4] = {0};
  uint16_t checksum = 0;
} __attribute__((packed));
static_assert(sizeof(FeedbackPacket) == kFeedbackPacketLength,
              "FeedbackPacket size must be 32 bytes");

#endif