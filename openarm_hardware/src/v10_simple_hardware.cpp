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
#include <cmath>
#include <fstream>
#include <filesystem>
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

  // Parse frequency diagnostics enable (default: false)
  it = info.hardware_parameters.find("enable_frequency_diagnostics");
  if (it == info.hardware_parameters.end()) {
    enable_frequency_diagnostics_ = false;
  } else {
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    enable_frequency_diagnostics_ = (value == "true");
  }

  // Load parameters from YAML file using ROS2 package resource lookup
  try {
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("openarm_hardware");
    std::string yaml_path = package_share_dir + "/config/parameters.yaml";
    loadParametersFromYAML(yaml_path);
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Loaded parameters from %s", yaml_path.c_str());
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "kp=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
                kp_[0], kp_[1], kp_[2], kp_[3], kp_[4], kp_[5], kp_[6]);
  } catch (const std::exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                "Failed to load parameters.yaml: %s, using defaults", e.what());
  }

  // Parse control gains from hardware parameters (overrides YAML if present)
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
              "Configuration: CAN=%s, arm_prefix=%s, ee_type=%s, hand=%s, can_fd=%s, leap_hand=%s, freq_diag=%s",
              can_interface_.c_str(), arm_prefix_.c_str(), ee_type_.c_str(),
              hand_ ? "enabled" : "disabled", can_fd_ ? "enabled" : "disabled",
              has_leap_hand_ ? "enabled" : "disabled",
              enable_frequency_diagnostics_ ? "enabled" : "disabled");
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

  // Initialize thread control flags
  arm_thread_running_ = false;
  leap_thread_running_ = false;
  
  // Initialize CSV logging variables
  csv_initialized_ = false;
  csv_sample_count_ = 0;
  
  // Initialize arm thread buffers (7 DOF + optional gripper)
  size_t arm_size = ARM_DOF + (hand_ ? 1 : 0);
  arm_pos_cmd_buffer_.resize(arm_size, 0.0);
  arm_vel_cmd_buffer_.resize(arm_size, 0.0);
  arm_tau_cmd_buffer_.resize(arm_size, 0.0);
  arm_pos_state_buffer_.resize(arm_size, 0.0);
  arm_vel_state_buffer_.resize(arm_size, 0.0);
  arm_tau_state_buffer_.resize(arm_size, 0.0);
  
  // Initialize LEAP Hand thread buffers if enabled
  if (has_leap_hand_) {
    leap_pos_cmd_buffer_.resize(LEAP_HAND_DOF, 0.0);
    leap_pos_state_buffer_.resize(LEAP_HAND_DOF, 0.0);
  }

  // Initialize LEAP Hand if enabled
  leap_connected_ = false;
  if (has_leap_hand_) {
    // Get serial port parameter (default: /dev/ttyUSB0)
    auto it = info.hardware_parameters.find("serial_port");
    leap_serial_port_ = (it != info.hardware_parameters.end()) ? it->second : "/dev/leaphand";
    
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

  // Initialize gravity compensation (default: enabled)
  auto it = info.hardware_parameters.find("use_gravity_compensation");
  use_gravity_compensation_ = (it == info.hardware_parameters.end() || it->second != "false");
  
  // Initialize friction compensation (default: enabled)
  it = info.hardware_parameters.find("use_friction_compensation");
  use_friction_compensation_ = (it == info.hardware_parameters.end() || it->second != "false");
  
  if (use_gravity_compensation_) {
    // Try to get URDF string from robot_description parameter (passed by controller_manager)
    // Note: This is typically not available in hardware_parameters during on_init
    // For now, we'll initialize KDL in on_configure when we can access the node
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Gravity compensation will be initialized in on_configure");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Gravity compensation disabled");
  }
  
  if (use_friction_compensation_) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Friction compensation enabled (LuGre model)");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Friction compensation disabled");
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

  // Initialize KDL dynamics if gravity compensation is enabled
  if (use_gravity_compensation_) {
    // Try to get robot_description from ROS2 parameter
    try {
      auto node = rclcpp::Node::make_shared("_openarm_hw_temp_node");
      node->declare_parameter("robot_description", "");
      std::string urdf_string = node->get_parameter("robot_description").as_string();
      
      if (!urdf_string.empty()) {
        if (init_kdl_dynamics(urdf_string)) {
          gravity_torques_.resize(ARM_DOF);
          RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                      "Gravity compensation enabled successfully");
        } else {
          RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                      "Failed to initialize KDL dynamics, gravity compensation disabled");
          use_gravity_compensation_ = false;
        }
      } else {
        RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                    "robot_description parameter is empty, gravity compensation disabled");
        use_gravity_compensation_ = false;
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                  "Failed to get robot_description: %s, gravity compensation disabled", e.what());
      use_gravity_compensation_ = false;
    }
  }

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

  // Start high-frequency arm control thread (500Hz)
  arm_thread_running_ = true;
  arm_control_thread_ = std::thread(&OpenArm_v10HW::arm_control_loop, this);
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Arm control thread started at 500Hz");
  
  // Start LEAP Hand control thread (100Hz) if enabled
  if (has_leap_hand_ && leap_connected_) {
    leap_thread_running_ = true;
    leap_control_thread_ = std::thread(&OpenArm_v10HW::leap_control_loop, this);
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "LEAP Hand control thread started at 100Hz");
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "OpenArm V10 activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Deactivating OpenArm V10...");
  RCLCPP_INFO(rclcpp::get_logger("Safety closing"),"Returning to safe zero position before shutdown...");
  return_to_zero();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // Stop arm control thread
  if (arm_thread_running_) {
    arm_thread_running_ = false;
    if (arm_control_thread_.joinable()) {
      arm_control_thread_.join();
    }
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Arm control thread stopped");
  }
  
  // Stop LEAP Hand control thread
  if (leap_thread_running_) {
    leap_thread_running_ = false;
    if (leap_control_thread_.joinable()) {
      leap_control_thread_.join();
    }
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "LEAP Hand control thread stopped");
  }

  // Disconnect LEAP Hand if connected
  if (has_leap_hand_) {
    disconnect_leap_hand();
  }

  // Disable all motors (like full_arm.cpp exit)
  openarm_->disable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();
  // Close debug CSV if open
  if (debug_csv_.is_open()) {
    debug_csv_.flush();
    debug_csv_.close();
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Closed debug CSV file on deactivate");
  }
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "OpenArm V10 deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArm_v10HW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Copy arm states from high-frequency thread buffer (thread-safe)
  {
    std::lock_guard<std::mutex> lock(arm_state_mutex_);
    size_t arm_size = ARM_DOF + (hand_ ? 1 : 0);
    for (size_t i = 0; i < arm_size; ++i) {
      pos_states_[i] = arm_pos_state_buffer_[i];
      vel_states_[i] = arm_vel_state_buffer_[i];
      tau_states_[i] = arm_tau_state_buffer_[i];
    }
  }

  // Copy LEAP Hand states from thread buffer if enabled (thread-safe)
  if (has_leap_hand_ && leap_connected_) {
    std::lock_guard<std::mutex> lock(leap_state_mutex_);
    size_t leap_start_idx = ARM_DOF + (hand_ ? 1 : 0);
    for (size_t i = 0; i < LEAP_HAND_DOF; ++i) {
      pos_states_[leap_start_idx + i] = leap_pos_state_buffer_[i];
      vel_states_[leap_start_idx + i] = 0.0;  // LEAP Hand doesn't provide velocity
      tau_states_[leap_start_idx + i] = 0.0;  // LEAP Hand doesn't provide torque
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10HW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Update arm command buffers (thread-safe)
  {
    std::lock_guard<std::mutex> lock(arm_command_mutex_);
    size_t arm_size = ARM_DOF + (hand_ ? 1 : 0);
    for (size_t i = 0; i < arm_size; ++i) {
      arm_pos_cmd_buffer_[i] = pos_commands_[i];
      arm_vel_cmd_buffer_[i] = vel_commands_[i];
      arm_tau_cmd_buffer_[i] = tau_commands_[i];
    }
  }

  // Update LEAP Hand command buffers if enabled (thread-safe)
  if (has_leap_hand_ && leap_connected_) {
    std::lock_guard<std::mutex> lock(leap_command_mutex_);
    size_t leap_start_idx = ARM_DOF + (hand_ ? 1 : 0);
    for (size_t i = 0; i < LEAP_HAND_DOF; ++i) {
      leap_pos_cmd_buffer_[i] = pos_commands_[leap_start_idx + i];
    }
  }

  return hardware_interface::return_type::OK;
}

void OpenArm_v10HW::return_to_zero() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Returning to zero position...");

  // Return arm to zero with MIT control
  std::vector<openarm::damiao_motor::MITParam> arm_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    arm_params.push_back({kp_[i]/100000, kd_[i]/100000, 0.0, 0.0, 0.0});
  }
  openarm_->get_arm().mit_control_all(arm_params);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Return gripper to zero if enabled
  if (hand_) {
    openarm_->get_gripper().mit_control_all(
        {{GRIPPER_KP, GRIPPER_KD, GRIPPER_JOINT_0_POSITION, 0.0, 0.0}});
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));
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

// Initialize KDL dynamics for gravity compensation
bool OpenArm_v10HW::init_kdl_dynamics(const std::string& urdf_content) {
  // Build KDL tree directly from URDF string
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromString(urdf_content, kdl_tree)) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Failed to construct KDL tree from URDF");
    return false;
  }

  // Extract chain for this arm (only arm links, exclude hand/gripper for dynamic loads)
  std::string root_link = "openarm_body_link0";
  std::string tip_link = "openarm_" + arm_prefix_ + "link7";  // End at link7, before hand
  
  if (!kdl_tree.getChain(root_link, tip_link, kdl_chain_)) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                 "Failed to get KDL chain from %s to %s, gravity compensation will be disabled",
                 root_link.c_str(), tip_link.c_str());
    return false;
  }

  if (kdl_chain_.getNrOfJoints() != ARM_DOF) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "KDL chain has %u joints, expected %zu",
                 kdl_chain_.getNrOfJoints(), ARM_DOF);
    return false;
  }

  // Create dynamics solver with gravity vector (0, 0, -9.81)
  kdl_solver_ = std::make_unique<KDL::ChainDynParam>(
      kdl_chain_, KDL::Vector(0.0, 0.0, -9.81));

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "KDL dynamics initialized: chain from %s to %s with %u joints",
              root_link.c_str(), tip_link.c_str(), kdl_chain_.getNrOfJoints());

  return true;
}

// Compute gravity compensation torques
void OpenArm_v10HW::compute_gravity_compensation(std::vector<double>& gravity_torques) {
  if (!kdl_solver_ || gravity_torques.size() != ARM_DOF) {
    return;
  }

  // Convert current positions to KDL format
  KDL::JntArray q(ARM_DOF);
  for (size_t i = 0; i < ARM_DOF; ++i) {
    q(i) = pos_states_[i];
  }

  // Compute gravity torques
  kdl_solver_->JntToGravity(q, gravity_torques_);

  // Copy results
  for (size_t i = 0; i < ARM_DOF; ++i) {
    gravity_torques[i] = gravity_torques_(i);
  }
}

// Compute friction compensation torques using LuGre model
// tau_friction = Fc * tanh(k * dq) + Fv * dq + Fo
void OpenArm_v10HW::compute_friction_compensation(std::vector<double>& friction_torques) {
  if (friction_torques.size() != ARM_DOF) {
    return;
  }

  for (size_t i = 0; i < ARM_DOF; ++i) {
    double dq = vel_states_[i];  // Current joint velocity
    
    // LuGre friction model:
    // - Fc * tanh(k * dq): Smooth Coulomb friction (avoids discontinuity at dq=0)
    // - Fv * dq: Viscous friction (proportional to velocity)
    // - Fo: Static offset (compensates for asymmetries)
    friction_torques[i] = Fc_[i] * std::tanh(k_[i] * dq) + Fv_[i] * dq + Fo_[i];
  }
}

// High-frequency arm control loop (500Hz)
void OpenArm_v10HW::arm_control_loop() {
  using namespace std::chrono;
  const auto loop_period = microseconds(1000);  // 1000Hz = 1000us

  auto next_cycle = steady_clock::now() + loop_period;
  
  if (enable_frequency_diagnostics_) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Thread"), 
                "Arm control loop started (target: 500Hz, diagnostics: ON)");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Thread"), 
                "Arm control loop started (target: 500Hz)");
  }
  
  // Local command buffers
  std::vector<double> pos_cmd(ARM_DOF + (hand_ ? 1 : 0), 0.0);
  std::vector<double> vel_cmd(ARM_DOF + (hand_ ? 1 : 0), 0.0);
  std::vector<double> tau_cmd(ARM_DOF + (hand_ ? 1 : 0), 0.0);
  
  // Local state buffers
  std::vector<double> pos_state(ARM_DOF + (hand_ ? 1 : 0), 0.0);
  std::vector<double> vel_state(ARM_DOF + (hand_ ? 1 : 0), 0.0);
  std::vector<double> tau_state(ARM_DOF + (hand_ ? 1 : 0), 0.0);
  
  // Performance monitoring (only if diagnostics enabled)
  size_t loop_count = 0;
  auto stats_start = steady_clock::now();
  duration<double, std::micro> max_loop_time(0);
  duration<double, std::micro> min_loop_time(999999);
  duration<double, std::micro> total_loop_time(0);
  
  while (arm_thread_running_) {
    auto cycle_start = steady_clock::now();
    // Copy commands from buffer (thread-safe)
    {
      std::lock_guard<std::mutex> lock(arm_command_mutex_);
      pos_cmd = arm_pos_cmd_buffer_;
      vel_cmd = arm_vel_cmd_buffer_;
      tau_cmd = arm_tau_cmd_buffer_;
    }
    
    // Read current states
    openarm_->refresh_all();
    openarm_->recv_all();
    
    const auto& arm_motors = openarm_->get_arm().get_motors();
    for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
      pos_state[i] = arm_motors[i].get_position();
      vel_state[i] = arm_motors[i].get_velocity();
      tau_state[i] = arm_motors[i].get_torque();
    }
    
    // Read gripper state if enabled
    if (hand_) {
      const auto& gripper_motors = openarm_->get_gripper().get_motors();
      if (!gripper_motors.empty()) {
        double motor_pos = gripper_motors[0].get_position();
        pos_state[ARM_DOF] = motor_radians_to_joint(motor_pos);
        vel_state[ARM_DOF] = 0.0;
        tau_state[ARM_DOF] = 0.0;
      }
    }
    
    // Update state buffer (thread-safe)
    {
      std::lock_guard<std::mutex> lock(arm_state_mutex_);
      arm_pos_state_buffer_ = pos_state;
      arm_vel_state_buffer_ = vel_state;
      arm_tau_state_buffer_ = tau_state;
    }
    
    // Compute gravity compensation
    std::vector<double> gravity_comp(ARM_DOF, 0.0);
    if (use_gravity_compensation_) {
      // Use current state for gravity computation
      KDL::JntArray q(ARM_DOF);
      for (size_t i = 0; i < ARM_DOF; ++i) {
        q(i) = pos_state[i];
      }
      if (kdl_solver_) {
        kdl_solver_->JntToGravity(q, gravity_torques_);
        for (size_t i = 0; i < ARM_DOF; ++i) {
          gravity_comp[i] = gravity_torques_(i);
        }
      }
    }
    
    // Compute friction compensation
    std::vector<double> friction_comp(ARM_DOF, 0.0);
    if (use_friction_compensation_) {
      for (size_t i = 0; i < ARM_DOF; ++i) {
        double dq = vel_state[i];
        friction_comp[i] = Fc_[i] * std::tanh(k_[i] * dq) + Fv_[i] * dq + Fo_[i];
      }
    }
    
    // Debug logging: save data to CSV for analysis (one CSV per arm)
    if (!csv_initialized_) {
      // Get install directory path (workspace/install/openarm_hardware/share/openarm_hardware)
      std::string package_share_dir;
      try {
        package_share_dir = ament_index_cpp::get_package_share_directory("openarm_hardware");
      } catch (const std::exception& e) {
        package_share_dir = "/tmp";  // Fallback to /tmp if package not found
      }
      
      // Create filename with arm prefix and timestamp
      auto now = std::chrono::system_clock::now();
      auto time_t_now = std::chrono::system_clock::to_time_t(now);
      std::tm tm_now;
      localtime_r(&time_t_now, &tm_now);
      
      std::ostringstream tmp_date;
      tmp_date << std::put_time(&tm_now, "%Y%m%d");
      std::string date_str = tmp_date.str();

      std::string arm_name = arm_prefix_.empty() ? "arm" : arm_prefix_;
      // Remove trailing underscore if present
      if (!arm_name.empty() && arm_name.back() == '_') {
        arm_name.pop_back();
      }

      // Build directory: <package_share_dir>/debug_csvs/YYYYMMDD
      std::filesystem::path dir_path = std::filesystem::path(package_share_dir) / "debug_csvs" / date_str;
      try {
        std::filesystem::create_directories(dir_path);
      } catch (const std::exception &e) {
        RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW_Thread"), "Failed to create debug CSV directory '%s': %s", dir_path.c_str(), e.what());
      }

      std::ostringstream oss;
      oss << dir_path.string() << "/debug_" << arm_name << "_"
          << std::put_time(&tm_now, "%Y%m%d_%H%M%S") << ".csv";
      std::string csv_filename = oss.str();
      debug_csv_.open(csv_filename);
      if (!debug_csv_.is_open()) {
        RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW_Thread"), "Failed to open debug CSV: %s", csv_filename.c_str());
      } else {
        debug_csv_ << "timestamp,joint_id,pos_cmd,vel_cmd,tau_cmd,pos_state,vel_state,tau_state,"
          << "pos_error,vel_error,gravity_comp,friction_comp,software_feedback,feedforward_tau,"
          << "kp,kd\n";
      }
      csv_initialized_ = true;
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Thread"), 
          "Debug CSV created: %s", csv_filename.c_str());
    }
    
    // Send arm commands with compensation
    std::vector<openarm::damiao_motor::MITParam> arm_params;
    for (size_t i = 0; i < ARM_DOF; ++i) {
      // Software-layer feedforward with additional error-based correction
      // This adds a software PD term on top of hardware PD for enhanced tracking
      double pos_error = pos_cmd[i] - pos_state[i];
      double vel_error = vel_cmd[i] - vel_state[i];
      // double software_feedback = kp_[i] * pos_error *0.3 + kd_[i] * vel_error *0.3;
      double software_feedback = 0;
      // Combined feedforward: compensation + software feedback + user torque command
      double feedforward_tau = tau_cmd[i] + gravity_comp[i] + friction_comp[i] + software_feedback;
      
      // MIT controller will add its own hardware PD on top of this
      // Total control: hardware_PD + (gravity + friction + software_PD + tau_cmd)
      
      // Log data every 100 iterations (5Hz) to reduce file size
      if (csv_sample_count_ % 100 == 0) {
        auto timestamp = duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
        if (debug_csv_.is_open()) {
          debug_csv_ << timestamp << "," << i << ","
            << pos_cmd[i] << "," << vel_cmd[i] << "," << tau_cmd[i] << ","
            << pos_state[i] << "," << vel_state[i] << "," << tau_state[i] << ","
            << pos_error << "," << vel_error << ","
            << gravity_comp[i] << "," << friction_comp[i] << "," << software_feedback << ","
            << feedforward_tau << "," << kp_[i] << "," << kd_[i] << "\n";
        }
      }
      
      arm_params.push_back({kp_[i], kd_[i], pos_cmd[i], vel_cmd[i], feedforward_tau});
    }
    csv_sample_count_++;
    openarm_->get_arm().mit_control_all(arm_params);
    
    // Send gripper command if enabled
    if (hand_) {
      double motor_command = joint_to_motor_radians(pos_cmd[ARM_DOF]);
      openarm_->get_gripper().mit_control_all(
          {{GRIPPER_KP, GRIPPER_KD, motor_command, 0, 0}});
    }
    
    // Frequency diagnostics (only if enabled)
    if (enable_frequency_diagnostics_) {
      auto cycle_end = steady_clock::now();
      auto loop_time = duration_cast<duration<double, std::micro>>(cycle_end - cycle_start);
      
      // Update statistics
      max_loop_time = std::max(max_loop_time, loop_time);
      min_loop_time = std::min(min_loop_time, loop_time);
      total_loop_time += loop_time;
      loop_count++;
      
      // Print statistics every 5 seconds
      auto elapsed = duration_cast<seconds>(cycle_end - stats_start);
      if (elapsed.count() >= 5) {
        double avg_loop_time = total_loop_time.count() / loop_count;
        double actual_frequency = loop_count / elapsed.count();
        
        RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Thread"),
                    "Loop stats: freq=%.1f Hz (target=500), "
                    "exec_time: avg=%.0f us, min=%.0f us, max=%.0f us",
                    actual_frequency, avg_loop_time, 
                    min_loop_time.count(), max_loop_time.count());
        
        // Reset statistics
        loop_count = 0;
        stats_start = cycle_end;
        max_loop_time = duration<double, std::micro>(0);
        min_loop_time = duration<double, std::micro>(999999);
        total_loop_time = duration<double, std::micro>(0);
      }
    }
    
    // Sleep until next cycle
    std::this_thread::sleep_until(next_cycle);
    next_cycle += loop_period;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Thread"), 
              "Arm control loop stopped");
}

// LEAP Hand control loop (100Hz)
void OpenArm_v10HW::leap_control_loop() {
  using namespace std::chrono;
  const auto loop_period = milliseconds(10);  // 100Hz = 10ms
  auto next_cycle = steady_clock::now() + loop_period;
  
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Thread"), 
              "LEAP Hand control loop started (100Hz)");
  
  std::vector<double> pos_cmd(LEAP_HAND_DOF, 0.0);
  std::vector<double> pos_state(LEAP_HAND_DOF, 0.0);
  
  while (leap_thread_running_) {
    // Copy commands from buffer (thread-safe)
    {
      std::lock_guard<std::mutex> lock(leap_command_mutex_);
      pos_cmd = leap_pos_cmd_buffer_;
    }
    
    // Send LEAP Hand commands
    if (leap_connected_) {
      // Clear previous sync write data
      leap_group_sync_write_->clearParam();
      
      // Convert URDF coordinates to LEAP coordinates and send
      for (size_t i = 0; i < LEAP_HAND_DOF; ++i) {
        double leap_pos = urdf_to_leap(pos_cmd[i]);
        int32_t position_ticks = static_cast<int32_t>(leap_pos / LEAP_POS_SCALE);
        
        uint8_t param_goal_position[4];
        param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(position_ticks));
        param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(position_ticks));
        param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(position_ticks));
        param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(position_ticks));
        
        leap_group_sync_write_->addParam(leap_motor_ids_[i], param_goal_position);
      }
      
      leap_group_sync_write_->txPacket();
      
      // Read LEAP Hand states
      int dxl_comm_result = leap_group_sync_read_pos_->txRxPacket();
      if (dxl_comm_result == COMM_SUCCESS) {
        for (size_t i = 0; i < LEAP_HAND_DOF; ++i) {
          uint8_t motor_id = leap_motor_ids_[i];
          if (leap_group_sync_read_pos_->isAvailable(motor_id, LEAP_ADDR_PRESENT_POSITION, 
                                                      LEAP_LEN_PRESENT_POSITION)) {
            int32_t position_ticks = leap_group_sync_read_pos_->getData(
              motor_id, LEAP_ADDR_PRESENT_POSITION, LEAP_LEN_PRESENT_POSITION);
            double leap_pos = static_cast<double>(position_ticks) * LEAP_POS_SCALE;
            pos_state[i] = leap_to_urdf(leap_pos);
          }
        }
        
        // Update state buffer (thread-safe)
        {
          std::lock_guard<std::mutex> lock(leap_state_mutex_);
          leap_pos_state_buffer_ = pos_state;
        }
      }
    }
    
    // Sleep until next cycle
    std::this_thread::sleep_until(next_cycle);
    next_cycle += loop_period;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Thread"), 
              "LEAP Hand control loop stopped");
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_v10HW,
                       hardware_interface::SystemInterface)
