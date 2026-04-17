// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

// clang-format off
// C++ system
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <serial_driver/serial_driver.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "rm_serial_driver/packet.hpp"
// clang-format on

namespace rm_serial_driver
{
class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);

  ~RMSerialDriver() override;

private:
  void getParams();

  void receiveData();

  bool isValidMode(uint8_t mode) const;

  bool isValidCmdAction(uint8_t cmd_action) const;

  void setRuntimeState(Status status, ErrorCode error_code);

  void updateCmdTimeoutState(const rclcpp::Time & now);

  void executeCmdPacket(const CmdPacket & packet);

  void publishArmTrajectory(double j1, double j2, double j3, double j4, double duration_sec);

  void publishGripperTrajectory(double gripper, double duration_sec);

  void sendArmFeedback(sensor_msgs::msg::JointState::SharedPtr msg);

  void reopenPort();

  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_cmd_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_cmd_pub_;

  std::string arm_cmd_topic_ = "/arm_controller/joint_trajectory";
  std::string gripper_cmd_topic_ = "/gripper_controller/joint_trajectory";
  double cmd_duration_sec_ = 0.25;
  int cmd_timeout_ms_ = 300;

  double feedback_rate_hz_ = 50.0;
  uint8_t feedback_status_ = static_cast<uint8_t>(Status::kReady);
  uint8_t feedback_error_code_ = static_cast<uint8_t>(ErrorCode::kOk);
  uint8_t runtime_status_ = static_cast<uint8_t>(Status::kReady);
  uint8_t runtime_error_code_ = static_cast<uint8_t>(ErrorCode::kOk);
  uint8_t feedback_seq_ = 0;
  uint8_t last_cmd_seq_ = 0;
  bool has_last_cmd_seq_ = false;
  bool paused_by_cmd_ = false;
  bool estop_latched_ = false;
  bool has_latest_joint_state_ = false;
  float latest_joint_1_ = 0.0F;
  float latest_joint_2_ = 0.0F;
  float latest_joint_3_ = 0.0F;
  float latest_joint_4_ = 0.0F;
  float latest_gripper_ = 0.0F;
  CmdPacket last_execute_cmd_{};
  bool has_last_execute_cmd_ = false;
  CmdPacket last_safe_cmd_{};
  bool has_last_safe_cmd_ = false;
  bool cmd_timeout_armed_ = false;
  bool cmd_timeout_active_ = false;
  rclcpp::Time last_valid_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_feedback_send_time_{0, 0, RCL_ROS_TIME};

  std::thread receive_thread_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
