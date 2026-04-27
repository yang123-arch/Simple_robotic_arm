// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include "rm_serial_driver/rm_serial_driver.hpp"

// clang-format off
// C++ system
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>

#include "rm_serial_driver/packet.hpp"
// clang-format on

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();

  feedback_rate_hz_ = this->declare_parameter("feedback_rate_hz", 50.0);
  const int feedback_status_param =
    this->declare_parameter("feedback_status", static_cast<int>(Status::kReady));
  const int feedback_error_code_param =
    this->declare_parameter("feedback_error_code", static_cast<int>(ErrorCode::kOk));
  if (feedback_status_param < 0 || feedback_status_param > 255) {
    RCLCPP_WARN(
      get_logger(), "feedback_status out of range [0,255], clamped: %d", feedback_status_param);
  }
  if (feedback_error_code_param < 0 || feedback_error_code_param > 255) {
    RCLCPP_WARN(
      get_logger(), "feedback_error_code out of range [0,255], clamped: %d",
      feedback_error_code_param);
  }
  feedback_status_ = static_cast<uint8_t>(std::clamp(feedback_status_param, 0, 255));
  feedback_error_code_ = static_cast<uint8_t>(std::clamp(feedback_error_code_param, 0, 255));
  runtime_status_ = feedback_status_;
  runtime_error_code_ = feedback_error_code_;
  arm_cmd_topic_ = this->declare_parameter("arm_cmd_topic", arm_cmd_topic_);
  gripper_cmd_topic_ = this->declare_parameter("gripper_cmd_topic", gripper_cmd_topic_);
  cmd_duration_sec_ = this->declare_parameter("cmd_duration_sec", cmd_duration_sec_);
  cmd_timeout_ms_ = this->declare_parameter("cmd_timeout_ms", cmd_timeout_ms_);

  if (cmd_duration_sec_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "cmd_duration_sec must be > 0, fallback to 0.25");
    cmd_duration_sec_ = 0.25;
  }
  if (cmd_timeout_ms_ <= 0) {
    RCLCPP_WARN(get_logger(), "cmd_timeout_ms must be > 0, fallback to 300");
    cmd_timeout_ms_ = 300;
  }

  arm_cmd_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(arm_cmd_topic_, 10);
  gripper_cmd_pub_ =
    this->create_publisher<trajectory_msgs::msg::JointTrajectory>(gripper_cmd_topic_, 10);
  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendArmFeedback, this, std::placeholders::_1));
}

RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(CmdPacket));

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(header);

      if (header[0] == kCmdPacketHeader) {
        data.resize(sizeof(CmdPacket) - 1);
        serial_driver_->port()->receive(data);

        data.insert(data.begin(), header[0]);
        const CmdPacket packet = fromVector(data);

        // Validation order: header -> length -> CRC -> enum
        if (packet.length != static_cast<uint8_t>(sizeof(CmdPacket))) {
          setRuntimeState(Status::kFault, ErrorCode::kCmdLengthInvalid);
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "CmdPacket length mismatch. expect=%u, got=%u",
            static_cast<unsigned int>(sizeof(CmdPacket)), static_cast<unsigned int>(packet.length));
          continue;
        }

        if (!verifyFrameCrc(data)) {
          setRuntimeState(Status::kFault, ErrorCode::kCmdCrcFail);
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "CmdPacket CRC error");
          continue;
        }

        if (!isValidMode(packet.mode) || !isValidCmdAction(packet.cmd_action)) {
          setRuntimeState(Status::kFault, ErrorCode::kCmdEnumInvalid);
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "CmdPacket enum invalid. mode=%u, cmd_action=%u",
            static_cast<unsigned int>(packet.mode), static_cast<unsigned int>(packet.cmd_action));
          continue;
        }

        const auto now = this->now();
        updateCmdTimeoutState(now);

        bool seq_jump = false;
        if (!has_last_cmd_seq_) {
          has_last_cmd_seq_ = true;
          last_cmd_seq_ = packet.seq;
        } else {
          const uint8_t expected_seq = static_cast<uint8_t>(last_cmd_seq_ + 1U);
          if (packet.seq == last_cmd_seq_) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000, "Duplicate CmdPacket seq=%u, ignored",
              static_cast<unsigned int>(packet.seq));
            continue;
          }
          const bool seq_restarted = (packet.seq == 0U && expected_seq != 0U);
          if (seq_restarted) {
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "CmdPacket seq restart detected (sender restart). rebase to seq=0");
          } else if (packet.seq != expected_seq) {
            seq_jump = true;
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000, "CmdPacket seq jump detected. expect=%u, got=%u",
              static_cast<unsigned int>(expected_seq), static_cast<unsigned int>(packet.seq));
          }
          last_cmd_seq_ = packet.seq;
        }

        const auto cmd_action = static_cast<CmdAction>(packet.cmd_action);
        const bool is_execute_cmd = (cmd_action == CmdAction::kExecute);
        if (!is_execute_cmd && cmd_timeout_armed_ && !cmd_timeout_active_) {
          last_valid_cmd_time_ = now;
        }

        RCLCPP_DEBUG(
          get_logger(), "Receive CmdPacket: seq=%u, mode=%u, cmd_action=%u",
          static_cast<unsigned int>(packet.seq), static_cast<unsigned int>(packet.mode),
          static_cast<unsigned int>(packet.cmd_action));
        last_safe_cmd_ = packet;
        has_last_safe_cmd_ = true;

        const auto nominal_error = seq_jump ? ErrorCode::kCmdSeqJump : ErrorCode::kOk;
        const auto publish_hold_trajectory = [&]() {
          if (!has_latest_joint_state_) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "No latest joint state cached, cannot publish hold trajectory");
            return;
          }
          constexpr double hold_duration_sec = 0.05;
          publishArmTrajectory(
            latest_joint_1_, latest_joint_2_, latest_joint_3_, latest_joint_4_, hold_duration_sec);
          publishGripperTrajectory(latest_gripper_, hold_duration_sec);
        };

        if (estop_latched_ && cmd_action != CmdAction::kClearFault) {
          setRuntimeState(Status::kEstop, ErrorCode::kEstopTriggered);
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Estop latched. Ignore cmd_action=%u until ClearFault",
            static_cast<unsigned int>(packet.cmd_action));
          continue;
        }

        switch (cmd_action) {
          case CmdAction::kExecute:
            if (estop_latched_) {
              setRuntimeState(Status::kEstop, ErrorCode::kEstopTriggered);
              RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "Execute rejected: estop latched");
              continue;
            }
            if (paused_by_cmd_) {
              setRuntimeState(Status::kPaused, nominal_error);
              RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "Execute rejected: paused_by_cmd is active");
              continue;
            }
            // Timeout freeze is released only by next valid Execute.
            cmd_timeout_armed_ = true;
            cmd_timeout_active_ = false;
            last_valid_cmd_time_ = now;
            executeCmdPacket(packet);
            last_execute_cmd_ = packet;
            has_last_execute_cmd_ = true;
            setRuntimeState(Status::kRunning, nominal_error);
            break;

          case CmdAction::kPause:
            paused_by_cmd_ = true;
            publish_hold_trajectory();
            setRuntimeState(Status::kPaused, nominal_error);
            break;

          case CmdAction::kResume:
            if (estop_latched_) {
              setRuntimeState(Status::kEstop, ErrorCode::kEstopTriggered);
              RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "Resume rejected: estop latched");
              continue;
            }
            paused_by_cmd_ = false;
            if (cmd_timeout_active_) {
              // Shared pause gate: timeout still active, remain paused.
              setRuntimeState(Status::kPaused, ErrorCode::kCmdTimeout);
              RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Resume ignored: cmd timeout active, waiting for next valid Execute");
              continue;
            }
            if (has_last_execute_cmd_) {
              executeCmdPacket(last_execute_cmd_);
              setRuntimeState(Status::kRunning, nominal_error);
            } else {
              setRuntimeState(Status::kReady, nominal_error);
            }
            break;

          case CmdAction::kClearFault:
            estop_latched_ = false;
            paused_by_cmd_ = false;
            if (cmd_timeout_active_) {
              // Keep timeout freeze until next valid Execute.
              setRuntimeState(Status::kPaused, ErrorCode::kCmdTimeout);
            } else {
              last_valid_cmd_time_ = now;
              setRuntimeState(Status::kReady, ErrorCode::kOk);
            }
            break;

          case CmdAction::kEstop:
            cmd_timeout_active_ = false;
            estop_latched_ = true;
            paused_by_cmd_ = true;
            publish_hold_trajectory();
            setRuntimeState(Status::kEstop, ErrorCode::kEstopTriggered);
            break;

          case CmdAction::kNone:
          default:
            setRuntimeState(Status::kReady, nominal_error);
            break;
        }
      } else {
        setRuntimeState(Status::kFault, ErrorCode::kCmdHeaderInvalid);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
      }
    } catch (const std::exception & ex) {
      setRuntimeState(Status::kFault, ErrorCode::kControllerInactive);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

bool RMSerialDriver::isValidMode(uint8_t mode) const
{
  switch (static_cast<Mode>(mode)) {
    case Mode::kNoOp:
    case Mode::kJointPosition:
    case Mode::kGripperOnly:
    case Mode::kHoming:
      return true;
    default:
      return false;
  }
}

bool RMSerialDriver::isValidCmdAction(uint8_t cmd_action) const
{
  switch (static_cast<CmdAction>(cmd_action)) {
    case CmdAction::kNone:
    case CmdAction::kExecute:
    case CmdAction::kPause:
    case CmdAction::kResume:
    case CmdAction::kClearFault:
    case CmdAction::kEstop:
      return true;
    default:
      return false;
  }
}

void RMSerialDriver::setRuntimeState(Status status, ErrorCode error_code)
{
  runtime_status_ = static_cast<uint8_t>(status);
  runtime_error_code_ = static_cast<uint8_t>(error_code);
}

void RMSerialDriver::updateCmdTimeoutState(const rclcpp::Time & now)
{
  if (estop_latched_) {
    return;
  }
  if (!cmd_timeout_armed_) {
    return;
  }
  if (cmd_timeout_ms_ <= 0) {
    return;
  }
  if (last_valid_cmd_time_.nanoseconds() == 0) {
    last_valid_cmd_time_ = now;
    return;
  }

  const double elapsed_ms = (now - last_valid_cmd_time_).seconds() * 1000.0;
  if (elapsed_ms > static_cast<double>(cmd_timeout_ms_)) {
    if (!cmd_timeout_active_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cmd timeout. elapsed=%.1f ms, threshold=%d ms",
        elapsed_ms, cmd_timeout_ms_);
    }
    cmd_timeout_active_ = true;
    setRuntimeState(Status::kPaused, ErrorCode::kCmdTimeout);
  }
}

void RMSerialDriver::executeCmdPacket(const CmdPacket & packet)
{
  const auto mode = static_cast<Mode>(packet.mode);
  const auto cmd_action = static_cast<CmdAction>(packet.cmd_action);

  if (cmd_action == CmdAction::kNone) {
    return;
  }

  if (cmd_action != CmdAction::kExecute) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "CmdPacket action=%u currently does not publish trajectory command",
      static_cast<unsigned int>(packet.cmd_action));
    return;
  }

  switch (mode) {
    case Mode::kJointPosition:
      publishArmTrajectory(packet.j1, packet.j2, packet.j3, packet.j4, cmd_duration_sec_);
      publishGripperTrajectory(packet.gripper, cmd_duration_sec_);
      break;
    case Mode::kGripperOnly:
      publishGripperTrajectory(packet.gripper, cmd_duration_sec_);
      break;
    case Mode::kHoming:
      publishArmTrajectory(0.0, 0.0, 0.0, 0.0, cmd_duration_sec_);
      publishGripperTrajectory(0.0, cmd_duration_sec_);
      break;
    case Mode::kNoOp:
      break;
    default:
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Unsupported mode=%u in CmdPacket",
        static_cast<unsigned int>(packet.mode));
      break;
  }
}

void RMSerialDriver::publishArmTrajectory(
  double j1, double j2, double j3, double j4, double duration_sec)
{
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp = this->now();
  msg.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4"};

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {j1, j2, j3, j4};
  const double safe_duration = duration_sec > 0.0 ? duration_sec : 0.25;
  const int32_t sec = static_cast<int32_t>(safe_duration);
  const uint32_t nsec = static_cast<uint32_t>((safe_duration - static_cast<double>(sec)) * 1e9);
  point.time_from_start.sec = sec;
  point.time_from_start.nanosec = nsec;
  msg.points.push_back(point);

  arm_cmd_pub_->publish(msg);
}

void RMSerialDriver::publishGripperTrajectory(double gripper, double duration_sec)
{
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp = this->now();
  msg.joint_names = {"left_finger_joint"};

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {gripper};
  const double safe_duration = duration_sec > 0.0 ? duration_sec : 0.25;
  const int32_t sec = static_cast<int32_t>(safe_duration);
  const uint32_t nsec = static_cast<uint32_t>((safe_duration - static_cast<double>(sec)) * 1e9);
  point.time_from_start.sec = sec;
  point.time_from_start.nanosec = nsec;
  msg.points.push_back(point);

  gripper_cmd_pub_->publish(msg);
}

void RMSerialDriver::sendArmFeedback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  try {
    const auto now = this->now();
    updateCmdTimeoutState(now);

    if (feedback_rate_hz_ > 0.0) {
      if (last_feedback_send_time_.nanoseconds() != 0) {
        const double min_dt = 1.0 / feedback_rate_hz_;
        if ((now - last_feedback_send_time_).seconds() < min_dt) {
          return;
        }
      }
      last_feedback_send_time_ = now;
    }

    std::map<std::string, size_t> joint_index;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      joint_index[msg->name[i]] = i;
    }

    FeedbackPacket packet;

    const auto read_joint = [&](const std::string & joint_name, float & value) -> bool {
      const auto it = joint_index.find(joint_name);
      if (it == joint_index.end() || it->second >= msg->position.size()) {
        return false;
      }
      value = static_cast<float>(msg->position[it->second]);
      return true;
    };

    float j1 = 0.0F;
    float j2 = 0.0F;
    float j3 = 0.0F;
    float j4 = 0.0F;
    float gripper = 0.0F;
    const bool j1_ok = read_joint("joint_1", j1);
    const bool j2_ok = read_joint("joint_2", j2);
    const bool j3_ok = read_joint("joint_3", j3);
    const bool j4_ok = read_joint("joint_4", j4);
    const bool gripper_ok = read_joint("left_finger_joint", gripper);
    const bool joints_ok = j1_ok && j2_ok && j3_ok && j4_ok && gripper_ok;

    if (joints_ok) {
      // Cache latest valid pose for future Pause/Estop hold trajectory (e.g. 50 ms hold).
      latest_joint_1_ = j1;
      latest_joint_2_ = j2;
      latest_joint_3_ = j3;
      latest_joint_4_ = j4;
      latest_gripper_ = gripper;
      has_latest_joint_state_ = true;
    }

    packet.j1 = j1;
    packet.j2 = j2;
    packet.j3 = j3;
    packet.j4 = j4;
    packet.gripper = gripper;
    packet.seq = feedback_seq_++;
    packet.length = static_cast<uint8_t>(sizeof(FeedbackPacket));

    if (!joints_ok) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "JointState missing required joints. Require: joint_1~joint_4, left_finger_joint");
    }

    packet.online = serial_driver_->port()->is_open() ? static_cast<uint8_t>(Online::kOnline)
                                                      : static_cast<uint8_t>(Online::kOffline);
    packet.status = runtime_status_;
    packet.error_code = runtime_error_code_;
    // Keep command-path fault visible in feedback. Only report JointStateMissing when runtime
    // path is still nominal.
    if (!joints_ok && packet.error_code == static_cast<uint8_t>(ErrorCode::kOk)) {
      packet.error_code = static_cast<uint8_t>(ErrorCode::kJointStateMissing);
    }
    appendPacketCrc(packet);

    std::vector<uint8_t> data = toVector(packet);
    serial_driver_->port()->send(data);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending arm feedback: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
