// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "rm_serial_driver/packet.hpp"

namespace rm_serial_driver
{
TEST(PacketProtocolTest, FixedLengthIs32Bytes)
{
  CmdPacket cmd_packet;
  FeedbackPacket feedback_packet;

  EXPECT_EQ(sizeof(CmdPacket), 32U);
  EXPECT_EQ(sizeof(FeedbackPacket), 32U);
  EXPECT_EQ(cmd_packet.length, kCmdPacketLength);
  EXPECT_EQ(feedback_packet.length, kFeedbackPacketLength);
}

TEST(PacketProtocolTest, CmdPacketCrcAppendAndVerify)
{
  CmdPacket cmd_packet;
  cmd_packet.seq = 42;
  cmd_packet.j1 = 1.2F;
  cmd_packet.j2 = -0.6F;
  cmd_packet.j3 = 0.8F;
  cmd_packet.j4 = -1.1F;
  cmd_packet.gripper = 0.015F;
  cmd_packet.mode = static_cast<uint8_t>(Mode::kJointPosition);
  cmd_packet.cmd_action = static_cast<uint8_t>(CmdAction::kExecute);

  appendPacketCrc(cmd_packet);
  auto frame = toVector(cmd_packet);
  ASSERT_EQ(frame.size(), sizeof(CmdPacket));
  EXPECT_TRUE(verifyFrameCrc(frame));

  frame[10] ^= 0x01U;
  EXPECT_FALSE(verifyFrameCrc(frame));
}

TEST(PacketProtocolTest, FeedbackPacketCrcAppendAndVerify)
{
  FeedbackPacket feedback_packet;
  feedback_packet.seq = 9;
  feedback_packet.j1 = 0.1F;
  feedback_packet.j2 = 0.2F;
  feedback_packet.j3 = 0.3F;
  feedback_packet.j4 = 0.4F;
  feedback_packet.gripper = 0.01F;
  feedback_packet.online = static_cast<uint8_t>(Online::kOnline);
  feedback_packet.status = static_cast<uint8_t>(Status::kRunning);
  feedback_packet.error_code = static_cast<uint8_t>(ErrorCode::kOk);

  appendPacketCrc(feedback_packet);
  auto frame = toVector(feedback_packet);
  ASSERT_EQ(frame.size(), sizeof(FeedbackPacket));
  EXPECT_TRUE(verifyFrameCrc(frame));

  frame[0] = 0x00U;
  EXPECT_FALSE(verifyFrameCrc(frame));
}

TEST(PacketProtocolTest, CmdPacketRoundTripFromVector)
{
  CmdPacket source;
  source.seq = 255;
  source.j1 = -3.0F;
  source.j2 = 1.0F;
  source.j3 = -2.0F;
  source.j4 = 2.0F;
  source.gripper = 0.02F;
  source.mode = static_cast<uint8_t>(Mode::kJointPosition);
  source.cmd_action = static_cast<uint8_t>(CmdAction::kExecute);
  appendPacketCrc(source);

  const std::vector<uint8_t> frame = toVector(source);
  CmdPacket parsed = fromVector(frame);

  EXPECT_EQ(parsed.header, kCmdPacketHeader);
  EXPECT_EQ(parsed.length, kCmdPacketLength);
  EXPECT_EQ(parsed.seq, 255U);
  EXPECT_FLOAT_EQ(parsed.j1, -3.0F);
  EXPECT_FLOAT_EQ(parsed.j2, 1.0F);
  EXPECT_FLOAT_EQ(parsed.j3, -2.0F);
  EXPECT_FLOAT_EQ(parsed.j4, 2.0F);
  EXPECT_FLOAT_EQ(parsed.gripper, 0.02F);
  EXPECT_EQ(parsed.mode, static_cast<uint8_t>(Mode::kJointPosition));
  EXPECT_EQ(parsed.cmd_action, static_cast<uint8_t>(CmdAction::kExecute));
  EXPECT_EQ(parsed.checksum, source.checksum);
}

}  // namespace rm_serial_driver
