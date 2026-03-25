// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "openarm_hardware/v10_simple_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openarm_hardware {

OpenArm_v10HW::OpenArm_v10HW() = default;

bool OpenArm_v10HW::parse_config(const hardware_interface::HardwareInfo& info) {
  // Parse CAN interface (default: can0)
  auto it = info.hardware_parameters.find("can_interface");
  can_interface_ = (it != info.hardware_parameters.end()) ? it->second : "can0";

  // Parse arm prefix (default: empty for single arm, "left_" or "right_" for
  // bimanual)
  it = info.hardware_parameters.find("arm_prefix");
  arm_prefix_ = (it != info.hardware_parameters.end()) ? it->second : "";

  // Parse end-effector type
  it = info.hardware_parameters.find("ee_type");
  ee_type_ = (it != info.hardware_parameters.end()) ? it->second : "default";
  has_leap_hand_ = (ee_type_ == "leap_hand_right" && arm_prefix_ == "right_");

  // Parse gripper enable (default: true for V10)
  it = info.hardware_parameters.find("hand");
  if (it == info.hardware_parameters.end()) {
    hand_ = true;  // Default to true for V10
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    hand_ = (value == "true");
  }

  // Parse CAN-FD enable (default: true for V10)
  it = info.hardware_parameters.find("can_fd");
  if (it == info.hardware_parameters.end()) {
    can_fd_ = true;  // Default to true for V10
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    can_fd_ = (value == "true");
  }

  // Parse control gains
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    it = info.hardware_parameters.find("kp" + std::to_string(i));
    if (it != info.hardware_parameters.end()) {
      kp_[i - 1] = std::stod(it->second);
    }
    it = info.hardware_parameters.find("kd" + std::to_string(i));
    if (it != info.hardware_parameters.end()) {
      kd_[i - 1] = std::stod(it->second);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Configuration: CAN=%s, arm_prefix=%s, ee_type=%s, hand=%s, can_fd=%s, leap_hand=%s",
              can_interface_.c_str(), arm_prefix_.c_str(), ee_type_.c_str(),
              hand_ ? "enabled" : "disabled", can_fd_ ? "enabled" : "disabled",
              has_leap_hand_ ? "enabled" : "disabled");
  return true;
}

void OpenArm_v10HW::generate_joint_names() {
  joint_names_.clear();
  // TODO: read from urdf properly and sort in the future.
  // Currently, the joint names are hardcoded for order consistency to align
  // with hardware. Generate arm joint names: openarm_{arm_prefix}joint{N}
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    std::string joint_name =
        "openarm_" + arm_prefix_ + "joint" + std::to_string(i);
    joint_names_.push_back(joint_name);
  }

  // Generate gripper joint name if enabled
  if (hand_) {
    std::string gripper_joint_name = "openarm_" + arm_prefix_ + "finger_joint1";
    joint_names_.push_back(gripper_joint_name);
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Added gripper joint: %s",
                gripper_joint_name.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Gripper joint NOT added because hand_=false");
  }

  // Generate leap_hand finger joint names if enabled (right hand only)
  if (has_leap_hand_) {
    // Index finger
    joint_names_.push_back("right_index_mcp_side");
    joint_names_.push_back("right_index_mcp_forward");
    joint_names_.push_back("right_index_pip");
    joint_names_.push_back("right_index_dip");
    // Middle finger
    joint_names_.push_back("right_middle_mcp_side");
    joint_names_.push_back("right_middle_mcp_forward");
    joint_names_.push_back("right_middle_pip");
    joint_names_.push_back("right_middle_dip");
    // Ring finger
    joint_names_.push_back("right_ring_mcp_side");
    joint_names_.push_back("right_ring_mcp_forward");
    joint_names_.push_back("right_ring_pip");
    joint_names_.push_back("right_ring_dip");
    // Thumb
    joint_names_.push_back("right_thumb_mcp_side");
    joint_names_.push_back("right_thumb_mcp_forward");
    joint_names_.push_back("right_thumb_pip_joint");
    joint_names_.push_back("right_thumb_dip_joint");
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Added 16 leap_hand finger joints");
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Generated %zu joint names for arm prefix '%s'",
              joint_names_.size(), arm_prefix_.c_str());
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // Parse configuration
  if (!parse_config(info)) {
    return CallbackReturn::ERROR;
  }

  // Generate joint names based on arm prefix
  generate_joint_names();

  // Validate joint count (7 arm joints + optional gripper + optional leap_hand)
  size_t expected_joints = ARM_DOF + (hand_ ? 1 : 0) + (has_leap_hand_ ? LEAP_HAND_DOF : 0);
  if (joint_names_.size() != expected_joints) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Generated %zu joint names, expected %zu", joint_names_.size(),
                 expected_joints);
    return CallbackReturn::ERROR;
  }

  // Initialize OpenArm with configurable CAN-FD setting
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Initializing OpenArm on %s with CAN-FD %s...",
              can_interface_.c_str(), can_fd_ ? "enabled" : "disabled");
  openarm_ =
      std::make_unique<openarm::can::socket::OpenArm>(can_interface_, can_fd_);

  // Initialize arm motors with V10 defaults
  openarm_->init_arm_motors(DEFAULT_MOTOR_TYPES, DEFAULT_SEND_CAN_IDS,
                            DEFAULT_RECV_CAN_IDS);

  // Initialize gripper if enabled
  if (hand_) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Initializing gripper...");
    openarm_->init_gripper_motor(DEFAULT_GRIPPER_MOTOR_TYPE,
                                 DEFAULT_GRIPPER_SEND_CAN_ID,
                                 DEFAULT_GRIPPER_RECV_CAN_ID);
  }

  // Initialize state and command vectors based on generated joint count
  const size_t total_joints = joint_names_.size();
  pos_commands_.resize(total_joints, 0.0);
  vel_commands_.resize(total_joints, 0.0);
  tau_commands_.resize(total_joints, 0.0);
  pos_states_.resize(total_joints, 0.0);
  vel_states_.resize(total_joints, 0.0);
  tau_states_.resize(total_joints, 0.0);

  // Initialize LEAP Hand if enabled
  leap_connected_ = false;
  if (has_leap_hand_) {
    // Get serial port parameter (default: /dev/ttyUSB0)
    auto it = info.hardware_parameters.find("serial_port");
    leap_serial_port_ = (it != info.hardware_parameters.end()) ? it->second : "/dev/ttyUSB0";
    
    // Get baudrate parameter (default: 4000000)
    it = info.hardware_parameters.find("baudrate");
    leap_baudrate_ = (it != info.hardware_parameters.end()) ? std::stoi(it->second) : 4000000;
    
    // Initialize motor IDs (0-15)
    leap_motor_ids_.clear();
    for (size_t i = 0; i < LEAP_HAND_DOF; i++) {
      leap_motor_ids_.push_back(static_cast<uint8_t>(i));
    }
    
    // Initialize Dynamixel SDK objects
    leap_port_handler_ = std::shared_ptr<dynamixel::PortHandler>(
      dynamixel::PortHandler::getPortHandler(leap_serial_port_.c_str()));
    leap_packet_handler_ = std::shared_ptr<dynamixel::PacketHandler>(
      dynamixel::PacketHandler::getPacketHandler(LEAP_PROTOCOL_VERSION));
    
    leap_group_sync_write_ = std::make_shared<dynamixel::GroupSyncWrite>(
      leap_port_handler_.get(), leap_packet_handler_.get(), 
      LEAP_ADDR_GOAL_POSITION, LEAP_LEN_GOAL_POSITION);
    leap_group_sync_read_pos_ = std::make_shared<dynamixel::GroupSyncRead>(
      leap_port_handler_.get(), leap_packet_handler_.get(), 
      LEAP_ADDR_PRESENT_POSITION, LEAP_LEN_PRESENT_POSITION);
    
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "LEAP Hand configured: port=%s, baudrate=%d", 
                leap_serial_port_.c_str(), leap_baudrate_);
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "OpenArm V10 Simple HW initialized successfully");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Set callback mode to ignore during configuration
  openarm_->refresh_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenArm_v10HW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenArm_v10HW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // TODO: consider exposing only needed interfaces to avoid undefined behavior.
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Activating OpenArm V10...");
  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
  openarm_->enable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  // Connect to LEAP Hand if enabled
  if (has_leap_hand_) {
    if (!connect_leap_hand()) {
      RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                  "Failed to connect to LEAP Hand, will operate in simulation mode");
    }
  }

  // Return to zero position
  return_to_zero();

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "OpenArm V10 activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Deactivating OpenArm V10...");

  // Disconnect LEAP Hand if connected
  if (has_leap_hand_) {
    disconnect_leap_hand();
  }

  // Disable all motors (like full_arm.cpp exit)
  openarm_->disable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "OpenArm V10 deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArm_v10HW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Receive all motor states
  openarm_->refresh_all();
  openarm_->recv_all();

  // Read arm joint states
  const auto& arm_motors = openarm_->get_arm().get_motors();
  for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
    pos_states_[i] = arm_motors[i].get_position();
    vel_states_[i] = arm_motors[i].get_velocity();
    tau_states_[i] = arm_motors[i].get_torque();
  }

  // Read gripper state if enabled
  if (hand_ && joint_names_.size() > ARM_DOF) {
    const auto& gripper_motors = openarm_->get_gripper().get_motors();
    if (!gripper_motors.empty()) {
      // TODO the mappings are approximates
      // Convert motor position (radians) to joint value (0-0.044m)
      double motor_pos = gripper_motors[0].get_position();
      pos_states_[ARM_DOF] = motor_radians_to_joint(motor_pos);

      // Unimplemented: Velocity and torque mapping
      vel_states_[ARM_DOF] = 0;  // gripper_motors[0].get_velocity();
      tau_states_[ARM_DOF] = 0;  // gripper_motors[0].get_torque();
    }
  }

  // Read LEAP Hand states if connected
  if (has_leap_hand_ && leap_connected_) {
    size_t leap_start_idx = ARM_DOF + (hand_ ? 1 : 0);
    if (!read_leap_hand_states(pos_states_, leap_start_idx)) {
      static auto last_warn_time = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_warn_time).count() > 1000) {
        RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                    "Failed to read LEAP Hand states");
        last_warn_time = now;
      }
    }
    // Velocity and effort are not read from LEAP Hand
    for (size_t i = leap_start_idx; i < leap_start_idx + LEAP_HAND_DOF && i < vel_states_.size(); ++i) {
      vel_states_[i] = 0.0;
      tau_states_[i] = 0.0;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10HW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Control arm motors with MIT control + closed-loop feedback
  std::vector<openarm::damiao_motor::MITParam> arm_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    // Closed-loop PD control using position and velocity feedback
    double pos_error = pos_commands_[i] - pos_states_[i];
    double vel_error = 0.0 - vel_states_[i];  // Target velocity = 0 (stop at goal)
    
    // Calculate feedforward torque to reduce steady-state error
    double feedforward_tau = kp_[i] * pos_error + kd_[i] * vel_error;
    
    arm_params.push_back(
        {kp_[i], kd_[i], pos_commands_[i], 0.0, feedforward_tau + tau_commands_[i]});
  }
  openarm_->get_arm().mit_control_all(arm_params);
  // Control gripper if enabled
  if (hand_ && joint_names_.size() > ARM_DOF) {
    // TODO the true mappings are unimplemented.
    double motor_command = joint_to_motor_radians(pos_commands_[ARM_DOF]);
    openarm_->get_gripper().mit_control_all(
        {{GRIPPER_KP, GRIPPER_KD, motor_command, 0, 0}});
  }
  
  // Control LEAP Hand if connected
  if (has_leap_hand_ && leap_connected_) {
    size_t leap_start_idx = ARM_DOF + (hand_ ? 1 : 0);
    if (!send_leap_hand_command(pos_commands_, leap_start_idx)) {
      static auto last_warn_time = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_warn_time).count() > 1000) {
        RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                    "Failed to send LEAP Hand commands");
        last_warn_time = now;
      }
    }
  }
  
  openarm_->recv_all(1000);
  return hardware_interface::return_type::OK;
}

void OpenArm_v10HW::return_to_zero() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Returning to zero position...");

  // Return arm to zero with MIT control
  std::vector<openarm::damiao_motor::MITParam> arm_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    arm_params.push_back({kp_[i], kd_[i], 0.0, 0.0, 0.0});
  }
  openarm_->get_arm().mit_control_all(arm_params);

  // Return gripper to zero if enabled
  if (hand_) {
    openarm_->get_gripper().mit_control_all(
        {{GRIPPER_KP, GRIPPER_KD, GRIPPER_JOINT_0_POSITION, 0.0, 0.0}});
  }
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  openarm_->recv_all();
}

// Gripper mapping helper functions
double OpenArm_v10HW::joint_to_motor_radians(double joint_value) {
  // Joint 0=closed -> motor 0 rad, Joint 0.044=open -> motor -1.0472 rad
  return (joint_value / GRIPPER_JOINT_0_POSITION) *
         GRIPPER_MOTOR_1_RADIANS;  // Scale from 0-0.044 to 0 to -1.0472
}

double OpenArm_v10HW::motor_radians_to_joint(double motor_radians) {
  // Motor 0 rad=closed -> joint 0, Motor -1.0472 rad=open -> joint 0.044
  return GRIPPER_JOINT_0_POSITION *
         (motor_radians /
          GRIPPER_MOTOR_1_RADIANS);  // Scale from 0 to -1.0472 to 0-0.044
}

// LEAP Hand Dynamixel functions
bool OpenArm_v10HW::connect_leap_hand() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Connecting to LEAP Hand on %s at %d baud",
              leap_serial_port_.c_str(), leap_baudrate_);

  // Open port
  if (!leap_port_handler_->openPort()) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Failed to open LEAP Hand port %s", leap_serial_port_.c_str());
    return false;
  }

  // Set baudrate
  if (!leap_port_handler_->setBaudRate(leap_baudrate_)) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Failed to set LEAP Hand baudrate to %d", leap_baudrate_);
    leap_port_handler_->closePort();
    return false;
  }

  // Add motors to sync read group
  for (uint8_t motor_id : leap_motor_ids_) {
    if (!leap_group_sync_read_pos_->addParam(motor_id)) {
      RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                  "Failed to add LEAP motor %d to sync read", motor_id);
    }
  }

  // Enable torque for all motors
  for (uint8_t motor_id : leap_motor_ids_) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = leap_packet_handler_->write1ByteTxRx(
      leap_port_handler_.get(), motor_id, LEAP_ADDR_TORQUE_ENABLE, 1, &dxl_error);
    
    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                  "Failed to enable torque for LEAP motor %d: %s",
                  motor_id, leap_packet_handler_->getTxRxResult(dxl_comm_result));
    }
  }

  leap_connected_ = true;
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "LEAP Hand connected successfully");
  return true;
}

void OpenArm_v10HW::disconnect_leap_hand() {
  if (!leap_connected_) {
    return;
  }

  // Disable torque for all motors
  for (uint8_t motor_id : leap_motor_ids_) {
    uint8_t dxl_error = 0;
    leap_packet_handler_->write1ByteTxRx(
      leap_port_handler_.get(), motor_id, LEAP_ADDR_TORQUE_ENABLE, 0, &dxl_error);
  }

  leap_port_handler_->closePort();
  leap_connected_ = false;
  
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "LEAP Hand disconnected");
}

bool OpenArm_v10HW::send_leap_hand_command(const std::vector<double>& positions, size_t start_idx) {
  if (!leap_connected_ || start_idx + LEAP_HAND_DOF > positions.size()) {
    return false;
  }

  // Clear previous sync write data
  leap_group_sync_write_->clearParam();

  // Convert URDF coordinates to LEAP coordinates, then to Dynamixel ticks
  for (size_t i = 0; i < LEAP_HAND_DOF; ++i) {
    double leap_pos = urdf_to_leap(positions[start_idx + i]);
    int32_t position_ticks = static_cast<int32_t>(leap_pos / LEAP_POS_SCALE);
    
    // Add position goal to sync write
    uint8_t param_goal_position[4];
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(position_ticks));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(position_ticks));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(position_ticks));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(position_ticks));
    
    if (!leap_group_sync_write_->addParam(leap_motor_ids_[i], param_goal_position)) {
      RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                  "Failed to add goal position for LEAP motor %d", leap_motor_ids_[i]);
      return false;
    }
  }

  // Transmit sync write packet
  int dxl_comm_result = leap_group_sync_write_->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                "LEAP Hand sync write failed: %s",
                leap_packet_handler_->getTxRxResult(dxl_comm_result));
    return false;
  }

  return true;
}

bool OpenArm_v10HW::read_leap_hand_states(std::vector<double>& positions, size_t start_idx) {
  if (!leap_connected_ || start_idx + LEAP_HAND_DOF > positions.size()) {
    return false;
  }

  // Read positions
  int dxl_comm_result = leap_group_sync_read_pos_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    return false;
  }

  // Extract data for each motor
  for (size_t i = 0; i < LEAP_HAND_DOF; ++i) {
    uint8_t motor_id = leap_motor_ids_[i];
    
    if (leap_group_sync_read_pos_->isAvailable(motor_id, LEAP_ADDR_PRESENT_POSITION, LEAP_LEN_PRESENT_POSITION)) {
      int32_t position_ticks = leap_group_sync_read_pos_->getData(
        motor_id, LEAP_ADDR_PRESENT_POSITION, LEAP_LEN_PRESENT_POSITION);
      double leap_pos = static_cast<double>(position_ticks) * LEAP_POS_SCALE;
      positions[start_idx + i] = leap_to_urdf(leap_pos);
    }
  }

  return true;
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_v10HW,
                       hardware_interface::SystemInterface)
