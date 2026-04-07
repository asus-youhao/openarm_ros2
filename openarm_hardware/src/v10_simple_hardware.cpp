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
#include <iomanip>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"

namespace openarm_hardware {

OpenArm_v10HW::OpenArm_v10HW() = default;

bool OpenArm_v10HW::parse_config(const hardware_interface::HardwareInfo& info) {
  // Parse simulation_mode FIRST — must happen before any CAN socket construction
  {
    auto sim_it = info.hardware_parameters.find("simulation_mode");
    if (sim_it != info.hardware_parameters.end()) {
      std::string value = sim_it->second;
      std::transform(value.begin(), value.end(), value.begin(), ::tolower);
      simulation_mode_ = (value == "true");
    }
  }
  {
    auto it2 = info.hardware_parameters.find("sim_use_cmd_feedback");
    if (it2 != info.hardware_parameters.end()) {
      std::string value = it2->second;
      std::transform(value.begin(), value.end(), value.begin(), ::tolower);
      sim_use_cmd_feedback_ = (value == "true");
    }
  }
  // Parse hand mass for chain solver (default: 0.0 = no hand mass)
  {
    auto it_hand_mass = info.hardware_parameters.find("hand_mass_kg");
    if (it_hand_mass != info.hardware_parameters.end()) {
      try {
        hand_mass_kg_ = std::stod(it_hand_mass->second);
        RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                    "[Chain Solver] Hand mass parameter: %.3f kg", hand_mass_kg_);
      } catch (const std::exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                    "Failed to parse hand_mass_kg parameter: %s", e.what());
      }
    }
  }
  if (simulation_mode_) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                "[SIMULATION MODE] CAN hardware disabled — %s",
                sim_use_cmd_feedback_
                    ? "state = commanded positions (controller-driven)"
                    : "KDL runs on sine-wave joint positions");
  }

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
  // O6 hand detection: supports o6_hand (bimanual), o6_hand_left, o6_hand_right, o6_left, o6_right
  has_o6_hand_ = (ee_type_.find("o6") != std::string::npos);

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

  // simulation_mode_ already parsed at the top of parse_config

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
              "Configuration: CAN=%s, arm_prefix=%s, ee_type=%s, hand=%s, can_fd=%s, leap_hand=%s, o6_hand=%s, freq_diag=%s",
              can_interface_.c_str(), arm_prefix_.c_str(), ee_type_.c_str(),
              hand_ ? "enabled" : "disabled", can_fd_ ? "enabled" : "disabled",
              has_leap_hand_ ? "enabled" : "disabled",
              has_o6_hand_ ? "enabled" : "disabled",
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

  // Generate gripper joint name if enabled (only for simple gripper, not O6/LEAP)
  // O6 and LEAP hands have their own joint definitions below
  if (hand_ && !has_o6_hand_ && !has_leap_hand_) {
    std::string gripper_joint_name = "openarm_" + arm_prefix_ + "finger_joint1";
    joint_names_.push_back(gripper_joint_name);
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Added gripper joint: %s",
                gripper_joint_name.c_str());
  } else if (hand_ && (has_o6_hand_ || has_leap_hand_)) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Gripper joint NOT added (using %s hand instead)",
                has_o6_hand_ ? "O6" : "LEAP");
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

  // Generate O6 hand joint names if enabled
  if (has_o6_hand_) {
    // In bimanual setup: arm_prefix is "left_" or "right_", but O6 joints in URDF use "L_" or "R_"
    // Determine O6 prefix based on arm_prefix (not o6_hand_type_ which might not match)
    std::string hand_prefix;
    if (arm_prefix_.find("left") != std::string::npos) {
      hand_prefix = "L_";
    } else if (arm_prefix_.find("right") != std::string::npos) {
      hand_prefix = "R_";
    } else {
      // Single arm case - use o6_hand_type_ directly
      hand_prefix = (o6_hand_type_ == "left") ? "L_" : "R_";
    }
    
    // Active joints (6)
    joint_names_.push_back(hand_prefix + "thumb_cmc_yaw");
    joint_names_.push_back(hand_prefix + "thumb_cmc_pitch");
    joint_names_.push_back(hand_prefix + "index_mcp_pitch");
    joint_names_.push_back(hand_prefix + "middle_mcp_pitch");
    joint_names_.push_back(hand_prefix + "ring_mcp_pitch");
    joint_names_.push_back(hand_prefix + "pinky_mcp_pitch");
    // Passive/coupled joints (5) - matching external openarm_description URDF
    joint_names_.push_back(hand_prefix + "thumb_dip");
    joint_names_.push_back(hand_prefix + "index_dip");
    joint_names_.push_back(hand_prefix + "middle_dip");
    joint_names_.push_back(hand_prefix + "ring_dip");
    joint_names_.push_back(hand_prefix + "pinky_dip");
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Added 11 O6 hand joints with prefix '%s' (arm_prefix='%s')", 
                hand_prefix.c_str(), arm_prefix_.c_str());
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Generated %zu joint names for arm prefix '%s'",
              joint_names_.size(), arm_prefix_.c_str());
  
  // Debug: Print all joint names
  std::string joint_list = "";
  for (const auto& name : joint_names_) {
    joint_list += name + ", ";
  }
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Joint names: %s", joint_list.c_str());
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

  // Validate joint count
  // gripper_joint: only added if hand_=true AND no O6/LEAP hand
  size_t gripper_joints = (hand_ && !has_o6_hand_ && !has_leap_hand_) ? 1 : 0;
  size_t expected_joints = ARM_DOF + gripper_joints + 
                          (has_leap_hand_ ? LEAP_HAND_DOF : 0) + 
                          (has_o6_hand_ ? O6_HAND_DOF : 0);
  if (joint_names_.size() != expected_joints) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Generated %zu joint names, expected %zu (arm=%zu, gripper=%zu, leap=%zu, o6=%zu)", 
                 joint_names_.size(), expected_joints, ARM_DOF, gripper_joints,
                 has_leap_hand_ ? LEAP_HAND_DOF : 0, has_o6_hand_ ? O6_HAND_DOF : 0);
    return CallbackReturn::ERROR;
  }

  // Initialize OpenArm (skip in simulation mode — no CAN socket)
  if (simulation_mode_) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "[SIMULATION MODE] Skipping CAN socket init (can_interface=%s)",
                can_interface_.c_str());
  } else {
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
  state_read_thread_running_ = false;
  
  // Initialize low-pass filters for state smoothing
  // arm_size includes gripper only if no O6/LEAP hand
  size_t arm_size = ARM_DOF + (hand_ && !has_o6_hand_ && !has_leap_hand_ ? 1 : 0);
  arm_state_filter_.init(arm_size, STATE_FILTER_CUTOFF_HZ, CONTROL_READ_RATE_HZ);
  if (has_leap_hand_) {
    leap_state_filter_.init(LEAP_HAND_DOF, STATE_FILTER_CUTOFF_HZ, CONTROL_READ_RATE_HZ);
  }
  if (has_o6_hand_) {
    o6_state_filter_.init(O6_HAND_DOF, STATE_FILTER_CUTOFF_HZ, 60.0);  // O6 runs at 60Hz
  }
  
  // Initialize CSV logging variables
  csv_initialized_ = false;
  csv_sample_count_ = 0;
  leap_csv_initialized_ = false;
  leap_csv_sample_count_ = 0;
  
  // Initialize arm thread buffers (7 DOF + optional gripper) - arm_size already declared above
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

  // Initialize O6 Hand thread buffers if enabled
  if (has_o6_hand_) {
    // Command buffer: only 6 active joints (passive joints don't receive commands)
    o6_pos_cmd_buffer_.resize(6, 0.0);
    // State buffer: all 11 joints (6 active + 5 passive for full kinematics)
    o6_pos_state_buffer_.resize(O6_HAND_DOF, 0.0);
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

  // Initialize O6 Hand if enabled
  o6_connected_ = false;
  if (has_o6_hand_) {
    // Get O6 CAN interface parameter (default: can2 for right, can3 for left)
    auto it = info.hardware_parameters.find("o6_can_interface");
    if (it != info.hardware_parameters.end()) {
      o6_can_interface_ = it->second;
    } else {
      // Default based on arm_prefix or ee_type
      if (arm_prefix_ == "left_") {
        o6_can_interface_ = "can1";
      } else if (arm_prefix_ == "right_") {
        o6_can_interface_ = "can0";
      } else {
        // Single arm: use ee_typeCOMM_TYPE
        o6_can_interface_ = (ee_type_ == "o6_hand_left" || ee_type_ == "o6_left") ? "can3" : "can2";
      }
    }
    
    // Determine hand type from arm_prefix or ee_type
    if (arm_prefix_ == "left_") {
      o6_hand_type_ = "left";
    } else if (arm_prefix_ == "right_") {
      o6_hand_type_ = "right";
    } else {
      // Single arm: use ee_type
      o6_hand_type_ = (ee_type_ == "o6_hand_left" || ee_type_ == "o6_left") ? "left" : "right";
    }
    
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "O6 Hand configured: can=%s, hand_type=%s", 
                o6_can_interface_.c_str(), o6_hand_type_.c_str());
  }

  // Initialize health monitoring
  health_monitor_running_ = false;
  health_status_.system_healthy = true;
  health_status_.emergency_stop_triggered = false;
  health_status_.total_read_operations = 0;
  health_status_.total_write_operations = 0;
  health_status_.total_errors = 0;
  health_status_.average_read_latency_ms = 0.0;
  health_status_.average_write_latency_ms = 0.0;

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
              "OpenArm V10 Simple HW initialized successfully with %zu joints ready for export",
              joint_names_.size());

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Set callback mode to ignore during configuration
  if (!simulation_mode_) {
    openarm_->refresh_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    openarm_->recv_all();
  }

  // Initialize KDL dynamics if gravity compensation is enabled
  if (use_gravity_compensation_) {
    // Try to get robot_description from ROS2 parameter using a ParameterClient
    // Typical providers of this parameter are `robot_state_publisher` or the
    // launch process that loaded the URDF. Creating a temporary node and
    // declaring the parameter locally will NOT read another node's parameter
    // (that was why the previous implementation often found an empty string).
    try {
      auto node = rclcpp::Node::make_shared("_openarm_hw_temp_node");

      // First try to query the commonly used provider `robot_state_publisher`.
      rclcpp::SyncParametersClient param_client(node, "robot_state_publisher");
      std::string urdf_string;

      if (param_client.wait_for_service(std::chrono::seconds(1))) {
        auto params = param_client.get_parameters({"robot_description"});
        if (!params.empty() && params[0].get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          urdf_string = params[0].as_string();
        }
      }

      // Fallback: if we couldn't get it from robot_state_publisher, try a
      // local parameter declaration (preserves previous behavior).
      if (urdf_string.empty()) {
        node->declare_parameter("robot_description", "");
        urdf_string = node->get_parameter("robot_description").as_string();
      }

      if (!urdf_string.empty()) {
        if (init_kdl_dynamics(urdf_string)) {
          RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                      "Gravity compensation enabled successfully");
          // Print diagnostics to verify tree structure
          print_kdl_tree_diagnostics();
        } else {
          RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                      "Failed to initialize KDL dynamics, gravity compensation disabled");
          use_gravity_compensation_ = false;
        }
      } else {
        RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                    "robot_description parameter is empty (not found on robot_state_publisher), gravity compensation disabled");
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
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Exporting %zu state interfaces for %zu joints",
              joint_names_.size() * 3, joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]));
  }
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Successfully exported %zu state interfaces", state_interfaces.size());
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenArm_v10HW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Exporting %zu command interfaces for %zu joints",
              joint_names_.size() * 3, joint_names_.size());
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
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Successfully exported %zu command interfaces", command_interfaces.size());
  return command_interfaces;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Activating OpenArm V10...");
  if (!simulation_mode_) {
    openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
    openarm_->enable_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    openarm_->recv_all();
  }

  // Connect to LEAP Hand if enabled
  if (has_leap_hand_) {
    if (!connect_leap_hand()) {
      RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                  "Failed to connect to LEAP Hand, will operate in simulation mode");
    }
  }

  // Connect to O6 Hand if enabled
  if (has_o6_hand_) {
    if (!connect_o6_hand()) {
      RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                  "Failed to connect to O6 Hand, will operate in simulation mode");
    }
    
    // Initialize O6 hand to open position (0.0 rad = open)
    size_t o6_start_idx = ARM_DOF + (hand_ ? 1 : 0) + (has_leap_hand_ ? LEAP_HAND_DOF : 0);
    
    // Set command and state buffers to open position
    for (size_t i = 0; i < 6; ++i) {
      pos_commands_[o6_start_idx + i] = 0.0;  // Open hand position
      pos_states_[o6_start_idx + i] = 0.0;    // Reflect open state
    }
    
    // Immediately send open command to hardware to prevent controller from reading old state
    if (o6_connected_) {
      std::vector<double> open_pos(6, 0.0);
      std::vector<uint8_t> motor_cmds(6, 255);  // 0 rad -> 255 motor (open)
      try {
        o6_hand_api_->fingerMove(motor_cmds);
        RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                    "O6 Hand set to open position [arm_prefix=%s]", arm_prefix_.c_str());
      } catch (const std::exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                    "Failed to send initial open command to O6 [arm_prefix=%s]: %s",
                    arm_prefix_.c_str(), e.what());
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Wait for hand to open
    }
  }

  // Return to zero position
  return_to_zero();

  // Start high-frequency arm control thread (500Hz, write-only)
  arm_thread_running_ = true;
  arm_control_thread_ = std::thread(&OpenArm_v10HW::arm_control_loop, this);
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Arm control thread started at 500Hz (write-only)");
  
  // Start LEAP Hand control thread (500Hz, write-only) if enabled
  if (has_leap_hand_ && leap_connected_) {
    leap_thread_running_ = true;
    leap_control_thread_ = std::thread(&OpenArm_v10HW::leap_control_loop, this);
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "LEAP Hand control thread started at 500Hz (write-only)");
  }

  // Start O6 Hand control thread (60Hz) if enabled
  if (has_o6_hand_ && o6_connected_) {
    // Pre-fill O6 state buffer with open position to prevent controller from reading old closed state
    {
      std::lock_guard<std::mutex> lock(o6_state_mutex_);
      for (size_t i = 0; i < O6_HAND_DOF; ++i) {
        o6_pos_state_buffer_[i] = 0.0;  // All joints at open position
      }
    }
    
    o6_thread_running_ = true;
    o6_control_thread_ = std::thread(&OpenArm_v10HW::o6_control_loop, this);
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "O6 Hand control thread started at 60Hz [arm_prefix=%s]", arm_prefix_.c_str());
  } else if (has_o6_hand_ && !o6_connected_) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                "O6 Hand enabled but not connected, control thread NOT started [arm_prefix=%s]",
                arm_prefix_.c_str());
  }

  // Start decoupled state read thread (reads CAN arm + LEAP serial + applies LPF)
  state_read_thread_running_ = true;
  state_read_thread_ = std::thread(&OpenArm_v10HW::state_read_loop, this);
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "State read thread started at %.0fHz (decoupled R/W architecture)",
              CONTROL_READ_RATE_HZ);

  // Start health monitoring thread
  health_monitor_running_ = true;
  health_monitor_thread_ = std::thread(&OpenArm_v10HW::health_monitor_loop, this);
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Health monitoring thread started");

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

  // Stop O6 Hand control thread
  if (o6_thread_running_) {
    o6_thread_running_ = false;
    if (o6_control_thread_.joinable()) {
      o6_control_thread_.join();
    }
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "O6 Hand control thread stopped");
  }

  // Stop decoupled state read thread
  if (state_read_thread_running_) {
    state_read_thread_running_ = false;
    if (state_read_thread_.joinable()) {
      state_read_thread_.join();
    }
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "State read thread stopped");
  }

  // Disconnect LEAP Hand if connected
  if (has_leap_hand_) {
    disconnect_leap_hand();
  }

  // Disconnect O6 Hand if connected
  if (has_o6_hand_) {
    disconnect_o6_hand();
  }

  // Disable all motors (like full_arm.cpp exit)
  if (!simulation_mode_) {
    openarm_->disable_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    openarm_->recv_all();
  }
  
  // Close debug CSVs if open
  if (debug_csv_.is_open()) {
    debug_csv_.flush();
    debug_csv_.close();
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Closed arm debug CSV file on deactivate");
  }
  if (leap_debug_csv_.is_open()) {
    leap_debug_csv_.flush();
    leap_debug_csv_.close();
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Closed LEAP Hand debug CSV file on deactivate");
  }
  if (kdl_bench_csv_.is_open()) {
    kdl_bench_csv_.flush();
    kdl_bench_csv_.close();
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "[%s] Closed KDL benchmark CSV", arm_prefix_.c_str());
  }
  
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "OpenArm V10 deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArm_v10HW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Copy arm states from high-frequency thread buffer (thread-safe)
  {
    std::lock_guard<std::mutex> lock(arm_state_mutex_);
    size_t arm_size = ARM_DOF + (hand_ && !has_o6_hand_ && !has_leap_hand_ ? 1 : 0);
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

  // Copy O6 Hand states from thread buffer if enabled (thread-safe)
  if (has_o6_hand_ && o6_connected_) {
    std::lock_guard<std::mutex> lock(o6_state_mutex_);
    size_t o6_start_idx = ARM_DOF + (hand_ ? 1 : 0) + (has_leap_hand_ ? LEAP_HAND_DOF : 0);
    for (size_t i = 0; i < O6_HAND_DOF; ++i) {
      pos_states_[o6_start_idx + i] = o6_pos_state_buffer_[i];
      vel_states_[o6_start_idx + i] = 0.0;  // O6 Hand doesn't provide velocity feedback
      tau_states_[o6_start_idx + i] = 0.0;  // O6 Hand doesn't provide torque feedback
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10HW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Update arm command buffers (thread-safe)
  {
    std::lock_guard<std::mutex> lock(arm_command_mutex_);
    size_t arm_size = ARM_DOF + (hand_ && !has_o6_hand_ && !has_leap_hand_ ? 1 : 0);
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

  // Update O6 Hand command buffers if enabled (thread-safe)
  // Only copy 6 active joint commands (passive joints are mechanically coupled)
  if (has_o6_hand_ && o6_connected_) {
    std::lock_guard<std::mutex> lock(o6_command_mutex_);
    size_t o6_start_idx = ARM_DOF + (hand_ ? 1 : 0) + (has_leap_hand_ ? LEAP_HAND_DOF : 0);
    
    // Copy 6 active joint commands (indices 0-5 in O6 joint list)
    for (size_t i = 0; i < 6; ++i) {
      o6_pos_cmd_buffer_[i] = pos_commands_[o6_start_idx + i];
    }
  }

  return hardware_interface::return_type::OK;
}

void OpenArm_v10HW::return_to_zero() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Returning to zero position...");
  if (simulation_mode_) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "[SIMULATION MODE] return_to_zero skipped (no hardware)");
    return;
  }

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

// ============================================================================
// KDL Tree-based Dynamics Functions
// ============================================================================

// Build joint name to KDL tree index mapping
// KDL Tree stores joints in tree traversal order, we need to map our joint names to their indices
void OpenArm_v10HW::build_joint_index_map() {
  joint_name_to_kdl_idx_.clear();
  kdl_joint_names_.clear();
  
  // Traverse the KDL tree to extract joint names in tree order
  // KDL tree segments contain joints, we extract them in order
  KDL::SegmentMap::const_iterator root_seg = kdl_tree_.getRootSegment();
  
  // Recursive lambda to traverse tree and collect joint names
  std::function<void(const KDL::SegmentMap::const_iterator&)> traverse_tree;
  traverse_tree = [&](const KDL::SegmentMap::const_iterator& seg_it) {
    // Process current segment's joint
    const KDL::Joint& joint = seg_it->second.segment.getJoint();
    if (joint.getType() != KDL::Joint::None) {  // Skip fixed joints
      std::string joint_name = joint.getName();
      int kdl_idx = kdl_joint_names_.size();
      kdl_joint_names_.push_back(joint_name);
      joint_name_to_kdl_idx_[joint_name] = kdl_idx;
    }
    
    // Recursively process children segments
    // children is a vector of SegmentMap::const_iterator
    for (const auto& child_seg_it : seg_it->second.children) {
      traverse_tree(child_seg_it);  // child_seg_it is already a SegmentMap::const_iterator
    }
  };
  
  // Traverse from root
  traverse_tree(root_seg);
  
  // Count joints belonging to this arm
  size_t arm_joints = 0;
  for (const auto& jname : kdl_joint_names_) {
    bool is_mine = (jname.find("openarm_" + arm_prefix_) != std::string::npos) ||
                   (has_o6_hand_ && arm_prefix_.find("left") != std::string::npos && jname[0] == 'L') ||
                   (has_o6_hand_ && arm_prefix_.find("right") != std::string::npos && jname[0] == 'R') ||
                   (has_leap_hand_ && jname.find("right_") != std::string::npos);
    if (is_mine) arm_joints++;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Built KDL joint index map: %zu total joints in tree, %zu belong to '%s' arm",
              kdl_joint_names_.size(), arm_joints, arm_prefix_.c_str());
  
  // Debug: Print mapping (only show joints belonging to this arm)
  std::string mapping_info = "";
  size_t shown = 0;
  for (size_t i = 0; i < kdl_joint_names_.size() && shown < 10; ++i) {
    const auto& jname = kdl_joint_names_[i];
    bool is_mine = (jname.find("openarm_" + arm_prefix_) != std::string::npos) ||
                   (has_o6_hand_ && arm_prefix_.find("left") != std::string::npos && jname[0] == 'L') ||
                   (has_o6_hand_ && arm_prefix_.find("right") != std::string::npos && jname[0] == 'R') ||
                   (has_leap_hand_ && jname.find("right_") != std::string::npos);
    if (is_mine) {
      mapping_info += jname + ":" + std::to_string(i) + " ";
      shown++;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "This arm's joints: %s", mapping_info.c_str());
}

// Initialize KDL dynamics for gravity compensation using Tree-based solver
bool OpenArm_v10HW::init_kdl_dynamics(const std::string& urdf_content) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Initializing KDL Tree-based dynamics (supports branching structures)...");
  
  // Build full KDL tree from URDF string
  KDL::Tree full_tree;
  if (!kdl_parser::treeFromString(urdf_content, full_tree)) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Failed to construct KDL tree from URDF");
    return false;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Full KDL tree built: %u segments, %u joints (bimanual system)",
              full_tree.getNrOfSegments(), full_tree.getNrOfJoints());
  
  // Extract subtree for current arm only (left or right)
  // This ensures each hardware interface only computes gravity for its own arm
  std::string root_link = "openarm_body_link0";  // Common base for both arms
  std::string arm_base_link = "openarm_" + arm_prefix_ + "link0";  // Arm-specific starting point
  
  // Find the deepest link in this arm's branch to define the subtree boundary
  std::vector<std::string> tip_candidates;
  
  // Priority 1: Try to find the most distal link (fingertip or hand end)
  if (has_o6_hand_) {
    // O6 hand has multiple fingertips, choose one as representative
    std::string hand_prefix = (arm_prefix_.find("left") != std::string::npos) ? "L_" : "R_";
    tip_candidates.push_back(hand_prefix + "index_distal");
    tip_candidates.push_back(hand_prefix + "middle_distal");
    tip_candidates.push_back(hand_prefix + "thumb_distal");
  } else if (has_leap_hand_) {
    tip_candidates.push_back("right_index_tip_head");
    tip_candidates.push_back("right_fingertip");
  }
  
  // Priority 2: Standard arm link7
  tip_candidates.push_back("openarm_" + arm_prefix_ + "link7");
  
  // Try to extract subtree using getChain (single branch) or manual filtering
  bool subtree_ok = false;
  std::string tip_link;
  
  // Attempt 1: Try to extract a chain from root to tip
  KDL::Chain temp_chain;
  for (const auto& candidate : tip_candidates) {
    if (full_tree.getChain(root_link, candidate, temp_chain)) {
      tip_link = candidate;
      subtree_ok = true;
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                  "Using chain extraction: %s -> %s", root_link.c_str(), candidate.c_str());
      break;
    }
  }
  
  if (!subtree_ok) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                "Could not extract subtree, using full tree (may include other arm)");
    kdl_tree_ = full_tree;
  } else {
    // For now, still use full tree but we'll filter joints during computation
    // TODO: Implement proper subtree extraction or manual tree construction
    kdl_tree_ = full_tree;
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Note: Using full tree, will filter to arm '%s' joints during computation",
                arm_prefix_.c_str());
  }
  
  // Build joint name to index mapping (only for this arm's joints)
  build_joint_index_map();
  
  // Count joints that belong to this arm
  size_t arm_joints_count = 0;
  for (const auto& jname : kdl_joint_names_) {
    // Check if joint belongs to current arm
    bool is_arm_joint = (jname.find("openarm_" + arm_prefix_) != std::string::npos) ||
                        (has_o6_hand_ && arm_prefix_.find("left") != std::string::npos && jname[0] == 'L') ||
                        (has_o6_hand_ && arm_prefix_.find("right") != std::string::npos && jname[0] == 'R') ||
                        (has_leap_hand_ && jname.find("right_") != std::string::npos);
    if (is_arm_joint) arm_joints_count++;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "KDL tree for '%s' arm: %u total segments, %u total joints, %zu belong to this arm",
              arm_prefix_.c_str(), kdl_tree_.getNrOfSegments(), 
              kdl_tree_.getNrOfJoints(), arm_joints_count);
  
  // Validate that we have at least the arm joints
  if (arm_joints_count < ARM_DOF) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "KDL tree has %zu joints for this arm, expected at least %zu",
                 arm_joints_count, ARM_DOF);
    return false;
  }
  
  // Create Tree-based recursive Newton-Euler inverse dynamics solver
  KDL::Vector gravity_vector(0.0, 0.0, -9.81);  // Earth gravity in base frame
  kdl_tree_solver_ = std::make_unique<KDL::TreeIdSolver_RNE>(kdl_tree_, gravity_vector);

  // Pre-allocate ALL hot-loop buffers ONCE here — never allocate inside the 500Hz control loop
  const unsigned int tree_n = kdl_tree_.getNrOfJoints();
  gravity_torques_.resize(tree_n);
  kdl_q_buf_.resize(tree_n);    KDL::SetToZero(kdl_q_buf_);
  kdl_qdot_buf_.resize(tree_n); KDL::SetToZero(kdl_qdot_buf_);
  kdl_qddot_buf_.resize(tree_n); KDL::SetToZero(kdl_qddot_buf_);
  kdl_tau_buf_.resize(tree_n);  KDL::SetToZero(kdl_tau_buf_);
  // kdl_f_ext_buf_ is a std::map — default-constructed empty, no further init needed

  // ── Benchmark: also build a Chain solver (arm + palm, no finger branches) ───────────────
  // Priority: palm_base (includes palm mass) → link7 (arm-only, no palm)
  std::vector<std::string> bench_tip_candidates;
  if (has_o6_hand_) {
    std::string hp = (arm_prefix_.find("left") != std::string::npos) ? "L_" : "R_";
    bench_tip_candidates.push_back(hp + "hand_base_link");
    bench_tip_candidates.push_back(hp + "palm");
  } else if (has_leap_hand_) {
    bench_tip_candidates.push_back("right_palm_lower");
    bench_tip_candidates.push_back("right_hand_base_link");
  }
  bench_tip_candidates.push_back("openarm_" + arm_prefix_ + "link7");  // final fallback

  // Use the actual tree root (not hardcoded "openarm_body_link0" which only exists
  // in bimanual URDF — standalone URDFs use "palm_mount" etc. as root)
  const std::string bench_root = full_tree.getRootSegment()->first;
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "[Benchmark] Using tree root '%s' for chain extraction", bench_root.c_str());

  for (const auto& cand : bench_tip_candidates) {
    KDL::Chain trial;
    if (!full_tree.getChain(bench_root, cand, trial)) {
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                  "[Benchmark] getChain(%s → %s) failed, trying next",
                  bench_root.c_str(), cand.c_str());
      continue;
    }
    // Success — optionally append virtual hand mass segment
    kdl_bench_chain_     = trial;
    kdl_bench_chain_tip_ = cand;
    
    // If hand_mass_kg > 0, add a virtual fixed segment at the chain tip with hand mass
    if (hand_mass_kg_ > 1e-6) {
      // Create a fixed joint (Joint::None) with hand mass as point mass at origin
      KDL::Joint fixed_joint(KDL::Joint::None);
      KDL::Frame tip_frame = KDL::Frame::Identity(); // No translation/rotation
      KDL::RigidBodyInertia hand_inertia(hand_mass_kg_); // Point mass at origin
      KDL::Segment hand_segment("virtual_hand_mass", fixed_joint, tip_frame, hand_inertia);
      kdl_bench_chain_.addSegment(hand_segment);
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                  "[Benchmark] Added virtual hand mass: %.3f kg at chain tip '%s'",
                  hand_mass_kg_, cand.c_str());
    }
    
    const unsigned int cn = kdl_bench_chain_.getNrOfJoints();
    kdl_bench_chain_solver_ =
        std::make_unique<KDL::ChainDynParam>(kdl_bench_chain_, KDL::Vector(0.0, 0.0, -9.81));
    kdl_bench_q_buf_.resize(cn);    KDL::SetToZero(kdl_bench_q_buf_);
    kdl_bench_grav_buf_.resize(cn); KDL::SetToZero(kdl_bench_grav_buf_);
    kdl_bench_chain_ok_ = true;
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "[Benchmark] Chain built: %s → %s  (%u joints) — timing-only, not used for output",
                bench_root.c_str(), cand.c_str(), cn);
    break;
  }
  if (!kdl_bench_chain_ok_) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                "[Benchmark] Could not build arm chain — chain timing comparison unavailable");
  }
  
  // Diagnostic: Print mass information from segments to verify URDF parsing
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "=== Verifying URDF mass data in KDL tree ===");
  
  // Traverse segments and show mass info for this arm's links
  KDL::SegmentMap::const_iterator root_seg = kdl_tree_.getRootSegment();
  std::function<void(const KDL::SegmentMap::const_iterator&, int)> check_mass;
  size_t segments_with_mass = 0;
  double total_mass = 0.0;
  
  check_mass = [&](const KDL::SegmentMap::const_iterator& seg_it, int depth) {
    const KDL::Segment& seg = seg_it->second.segment;
    std::string seg_name = seg.getName();
    
    // Check if this segment belongs to current arm
    bool is_mine = (seg_name.find("openarm_" + arm_prefix_) != std::string::npos) ||
                   (has_o6_hand_ && arm_prefix_.find("left") != std::string::npos && seg_name[0] == 'L') ||
                   (has_o6_hand_ && arm_prefix_.find("right") != std::string::npos && seg_name[0] == 'R') ||
                   (has_leap_hand_ && seg_name.find("right_") != std::string::npos);
    
    if (is_mine) {
      const KDL::RigidBodyInertia& inertia = seg.getInertia();
      double mass = inertia.getMass();
      if (mass > 1e-6) {  // Non-zero mass
        segments_with_mass++;
        total_mass += mass;
        RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                    "  Segment '%s': mass=%.4f kg", seg_name.c_str(), mass);
      }
    }
    
    // Recurse to children
    for (const auto& child_seg_it : seg_it->second.children) {
      check_mass(child_seg_it, depth + 1);
    }
  };
  
  check_mass(root_seg, 0);
  
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Found %zu segments with mass for '%s' arm, total mass: %.4f kg",
              segments_with_mass, arm_prefix_.c_str(), total_mass);
  
  if (segments_with_mass == 0 || total_mass < 0.01) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                "WARNING: No significant mass data found in URDF! Gravity compensation may not work!");
  }
  
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "KDL Tree dynamics initialized successfully for '%s' arm",
              arm_prefix_.c_str());
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "This arm: arm (7) + hand (%s) joints",
              has_leap_hand_ ? "16 leap" : (has_o6_hand_ ? "11 o6" : (hand_ ? "1 gripper" : "0")));
  
  return true;
}

// Compute gravity compensation torques using KDL Tree-based solver
// Tree solver computes gravity for ALL joints in the tree, including hand joints
void OpenArm_v10HW::compute_gravity_compensation(std::vector<double>& gravity_torques,
                                                  const std::vector<double>& q_current) {
  if (!kdl_tree_solver_ || gravity_torques.size() != ARM_DOF) {
    return;
  }

  // Use pre-allocated member buffers — zero heap allocation in the 500Hz hot loop
  const size_t tree_njoints = static_cast<size_t>(kdl_q_buf_.rows());

  // Reset positions to zero, then fill with the current-cycle joint states
  // q_current is the pos_state already locked from arm_state_mutex this cycle
  // (avoids using stale global pos_states_ which is written by read() on a different path)
  KDL::SetToZero(kdl_q_buf_);
  for (size_t i = 0; i < joint_names_.size() && i < q_current.size(); ++i) {
    auto it = joint_name_to_kdl_idx_.find(joint_names_[i]);
    if (it != joint_name_to_kdl_idx_.end()) {
      int kdl_idx = it->second;
      if (kdl_idx >= 0 && kdl_idx < static_cast<int>(tree_njoints)) {
        kdl_q_buf_(kdl_idx) = q_current[i];
      }
    }
  }
  // kdl_qdot_buf_ and kdl_qddot_buf_ remain zero (gravity-only: no vel/accel terms)
  // kdl_f_ext_buf_ remains empty (no external wrenches)

  // Compute: tau = G(q)  (gravity torques only, since q_dot=0, q_dotdot=0)
  // --- KDL timing: measure CartToJnt wall time ---
  const auto kdl_t0 = std::chrono::steady_clock::now();
  int result = kdl_tree_solver_->CartToJnt(
      kdl_q_buf_, kdl_qdot_buf_, kdl_qddot_buf_, kdl_f_ext_buf_, kdl_tau_buf_);
  const auto kdl_t1 = std::chrono::steady_clock::now();
  const double kdl_us =
      std::chrono::duration<double, std::micro>(kdl_t1 - kdl_t0).count();

  // Accumulate running stats (reset every 500 calls)
  kdl_timing_count_++;
  kdl_timing_sum_us_ += kdl_us;
  if (kdl_us < kdl_timing_min_us_) kdl_timing_min_us_ = kdl_us;
  if (kdl_us > kdl_timing_max_us_) kdl_timing_max_us_ = kdl_us;

  if (enable_frequency_diagnostics_ && kdl_timing_count_ >= 500) {
    if (!kdl_bench_chain_ok_) {
      // No benchmark chain → print Tree-only log and reset here
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_KDL"),
                  "[%s] CartToJnt timing over %u calls | "
                  "avg=%.2f µs  min=%.2f µs  max=%.2f µs  (budget=2000 µs @ 500Hz)",
                  arm_prefix_.c_str(), kdl_timing_count_,
                  kdl_timing_sum_us_ / kdl_timing_count_,
                  kdl_timing_min_us_, kdl_timing_max_us_);
      kdl_timing_count_ = 0;
      kdl_timing_sum_us_ = 0.0;
      kdl_timing_min_us_ = 1e9;
      kdl_timing_max_us_ = 0.0;
    }
    // else: bench block below will read these Tree stats and reset both counters together
  }

  // ── Benchmark: run Chain solver side-by-side (diagnostics mode only) ─────────────────────
  if (enable_frequency_diagnostics_ && kdl_bench_chain_ok_) {
    // Fill chain q buffer with arm joints (link0-indexed order = first ARM_DOF joints)
    const size_t chain_n = static_cast<size_t>(kdl_bench_q_buf_.rows());
    KDL::SetToZero(kdl_bench_q_buf_);
    for (size_t i = 0; i < chain_n && i < q_current.size(); ++i) {
      kdl_bench_q_buf_(i) = q_current[i];
    }

    const auto bench_t0 = std::chrono::steady_clock::now();
    kdl_bench_chain_solver_->JntToGravity(kdl_bench_q_buf_, kdl_bench_grav_buf_);
    const auto bench_t1 = std::chrono::steady_clock::now();
    const double bench_us =
        std::chrono::duration<double, std::micro>(bench_t1 - bench_t0).count();

    kdl_bench_count_++;
    kdl_bench_sum_us_ += bench_us;
    if (bench_us < kdl_bench_min_us_) kdl_bench_min_us_ = bench_us;
    if (bench_us > kdl_bench_max_us_) kdl_bench_max_us_ = bench_us;

    // Track per-joint torque difference (Tree vs Chain)
    for (size_t i = 0; i < ARM_DOF && i < joint_names_.size(); ++i) {
      double tree_tau = 0.0;
      auto kt = joint_name_to_kdl_idx_.find(joint_names_[i]);
      if (kt != joint_name_to_kdl_idx_.end() && kt->second >= 0 &&
          kt->second < static_cast<int>(kdl_tau_buf_.rows())) {
        tree_tau = kdl_tau_buf_(kt->second);
      }
      double chain_tau = (i < static_cast<size_t>(kdl_bench_grav_buf_.rows()))
                             ? kdl_bench_grav_buf_(i)
                             : 0.0;
      kdl_bench_last_tree_tau_[i]  = tree_tau;
      kdl_bench_last_chain_tau_[i] = chain_tau;
      double diff = std::abs(tree_tau - chain_tau);
      if (diff > kdl_bench_max_torque_diff_[i]) kdl_bench_max_torque_diff_[i] = diff;
    }

    if (kdl_bench_count_ >= 500) {
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_KDL"),
                  "[%s] ── TIMING COMPARISON (500 calls each) ──────────────────────────────",
                  arm_prefix_.c_str());
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_KDL"),
                  "[%s]  Tree  TreeIdSolver_RNE  (%2u joints, full bimanual) │"
                  " avg=%6.2f µs  min=%6.2f µs  max=%6.2f µs",
                  arm_prefix_.c_str(),
                  kdl_tree_.getNrOfJoints(),
                  kdl_timing_sum_us_ / std::max(kdl_timing_count_, 1u),
                  kdl_timing_min_us_, kdl_timing_max_us_);
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_KDL"),
                  "[%s]  Chain ChainDynParam     (%2zu joints, arm+palm '%s') │"
                  " avg=%6.2f µs  min=%6.2f µs  max=%6.2f µs",
                  arm_prefix_.c_str(),
                  chain_n, kdl_bench_chain_tip_.c_str(),
                  kdl_bench_sum_us_ / kdl_bench_count_,
                  kdl_bench_min_us_, kdl_bench_max_us_);
      const double speedup =
          (kdl_bench_count_ > 0 && kdl_bench_sum_us_ > 0.0)
              ? (kdl_timing_sum_us_ / std::max(kdl_timing_count_, 1u)) /
                    (kdl_bench_sum_us_ / kdl_bench_count_)
              : 1.0;
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_KDL"),
                  "[%s]  → Chain is %.1fx %s than Tree (avg)",
                  arm_prefix_.c_str(), speedup,
                  speedup >= 1.0 ? "FASTER" : "slower");
      // ── Torque accuracy comparison ───────────────────────────────────────────
      // Print q (input) alongside Tree vs Chain torques so comparison is self-contained
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_KDL"),
                  "[%s]  q [rad] (last call):  "
                  "j1=%+.3f  j2=%+.3f  j3=%+.3f  j4=%+.3f  j5=%+.3f  j6=%+.3f  j7=%+.3f",
                  arm_prefix_.c_str(),
                  (ARM_DOF > 0 ? q_current[0] : 0.0),
                  (ARM_DOF > 1 ? q_current[1] : 0.0),
                  (ARM_DOF > 2 ? q_current[2] : 0.0),
                  (ARM_DOF > 3 ? q_current[3] : 0.0),
                  (ARM_DOF > 4 ? q_current[4] : 0.0),
                  (ARM_DOF > 5 ? q_current[5] : 0.0),
                  (ARM_DOF > 6 ? q_current[6] : 0.0));
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_KDL"),
                  "[%s]  Torque [Nm]            Tree       Chain      diff   maxDiff(500)",
                  arm_prefix_.c_str());
      double overall_max_diff = 0.0;
      for (size_t i = 0; i < ARM_DOF; ++i) {
        double diff_now = std::abs(kdl_bench_last_tree_tau_[i] - kdl_bench_last_chain_tau_[i]);
        if (kdl_bench_max_torque_diff_[i] > overall_max_diff)
          overall_max_diff = kdl_bench_max_torque_diff_[i];
        RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_KDL"),
                    "[%s]   j%zu: %+9.4f  %+9.4f  %+7.4f  %7.4f",
                    arm_prefix_.c_str(), i + 1,
                    kdl_bench_last_tree_tau_[i], kdl_bench_last_chain_tau_[i],
                    diff_now, kdl_bench_max_torque_diff_[i]);
      }
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_KDL"),
                  "[%s]  Overall max torque diff over 500 calls: %.4f Nm",
                  arm_prefix_.c_str(), overall_max_diff);
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_KDL"),
                  "[%s] ───────────────────────────────────────────────────────────────────",
                  arm_prefix_.c_str());

      // ── Write CSV row ───────────────────────────────────────────────────────
      {
        // Lazy open: create file on first batch, name encodes arm + wall-clock time
        if (!kdl_bench_csv_initialized_) {
          const auto now_sys = std::chrono::system_clock::now();
          const auto now_t   = std::chrono::system_clock::to_time_t(now_sys);
          char ts_buf[32];
          std::strftime(ts_buf, sizeof(ts_buf), "%Y%m%d_%H%M%S", std::localtime(&now_t));
          const std::string csv_path =
              std::string("/tmp/kdl_bench_") + arm_prefix_ + ts_buf + ".csv";
          kdl_bench_csv_.open(csv_path, std::ios::out | std::ios::trunc);
          if (kdl_bench_csv_.is_open()) {
            // Header row
            kdl_bench_csv_
                << "batch,timestamp_s,arm"
                << ",tree_avg_us,tree_min_us,tree_max_us"
                << ",chain_avg_us,chain_min_us,chain_max_us,speedup"
                << ",q1,q2,q3,q4,q5,q6,q7"
                << ",tree_tau1,tree_tau2,tree_tau3,tree_tau4,tree_tau5,tree_tau6,tree_tau7"
                << ",chain_tau1,chain_tau2,chain_tau3,chain_tau4,chain_tau5,chain_tau6,chain_tau7"
                << ",diff1,diff2,diff3,diff4,diff5,diff6,diff7"
                << ",max_diff1,max_diff2,max_diff3,max_diff4,max_diff5,max_diff6,max_diff7"
                << ",overall_max_diff_nm\n";
            kdl_bench_csv_initialized_ = true;
            RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_KDL"),
                        "[%s] KDL benchmark CSV: %s", arm_prefix_.c_str(), csv_path.c_str());
          } else {
            RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW_KDL"),
                        "[%s] Failed to open KDL benchmark CSV at %s",
                        arm_prefix_.c_str(), csv_path.c_str());
          }
        }
        if (kdl_bench_csv_.is_open()) {
          kdl_bench_batch_count_++;
          const double now_s = std::chrono::duration<double>(
              std::chrono::steady_clock::now().time_since_epoch()).count();
          const double tree_avg = kdl_timing_sum_us_ / std::max(kdl_timing_count_, 1u);
          const double chain_avg = kdl_bench_sum_us_ / kdl_bench_count_;
          kdl_bench_csv_ << std::fixed << std::setprecision(6)
              << kdl_bench_batch_count_ << ","
              << now_s << ","
              << arm_prefix_ << ","
              << tree_avg << "," << kdl_timing_min_us_ << "," << kdl_timing_max_us_ << ","
              << chain_avg << "," << kdl_bench_min_us_ << "," << kdl_bench_max_us_ << ","
              << speedup << ",";
          // q values
          for (size_t i = 0; i < 7; ++i)
            kdl_bench_csv_ << (i < q_current.size() ? q_current[i] : 0.0)
                           << (i < 6 ? "," : ",");
          // tree_tau
          for (size_t i = 0; i < 7; ++i)
            kdl_bench_csv_ << kdl_bench_last_tree_tau_[i]  << (i < 6 ? "," : ",");
          // chain_tau
          for (size_t i = 0; i < 7; ++i)
            kdl_bench_csv_ << kdl_bench_last_chain_tau_[i] << (i < 6 ? "," : ",");
          // diff
          for (size_t i = 0; i < 7; ++i) {
            double d = std::abs(kdl_bench_last_tree_tau_[i] - kdl_bench_last_chain_tau_[i]);
            kdl_bench_csv_ << d << (i < 6 ? "," : ",");
          }
          // max_diff
          for (size_t i = 0; i < 7; ++i)
            kdl_bench_csv_ << kdl_bench_max_torque_diff_[i] << (i < 6 ? "," : ",");
          // overall_max_diff
          kdl_bench_csv_ << overall_max_diff << "\n";
          kdl_bench_csv_.flush();
        }
      }

      // Reset bench counters
      kdl_bench_count_ = 0;
      kdl_bench_sum_us_ = 0.0;
      kdl_bench_min_us_ = 1e9;
      kdl_bench_max_us_ = 0.0;
      kdl_bench_max_torque_diff_.fill(0.0);
      // Reset Tree counters here (not reset above when bench is ok)
      kdl_timing_count_ = 0;
      kdl_timing_sum_us_ = 0.0;
      kdl_timing_min_us_ = 1e9;
      kdl_timing_max_us_ = 0.0;
    }
  }

  if (result != 0) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("OpenArm_v10HW"),
                         *rclcpp::Clock::make_shared(), 5000,
                         "KDL TreeIdSolver failed with code %d", result);
    return;
  }

  // Map results back to ARM_DOF output vector
  for (size_t i = 0; i < ARM_DOF && i < joint_names_.size(); ++i) {
    auto it = joint_name_to_kdl_idx_.find(joint_names_[i]);
    if (it != joint_name_to_kdl_idx_.end()) {
      int kdl_idx = it->second;
      gravity_torques[i] = (kdl_idx >= 0 && kdl_idx < static_cast<int>(tree_njoints))
                               ? kdl_tau_buf_(kdl_idx)
                               : 0.0;
    } else {
      gravity_torques[i] = 0.0;
    }
  }

  // Copy to diagnostics buffer (gravity_torques_ already sized at init, no resize)
  for (size_t i = 0; i < tree_njoints; ++i) {
    gravity_torques_(i) = kdl_tau_buf_(i);
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

// Print KDL Tree diagnostics for debugging and verification
void OpenArm_v10HW::print_kdl_tree_diagnostics() {
  if (!kdl_tree_solver_) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW_Diagnostics"),
                "KDL Tree solver not initialized");
    return;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Diagnostics"),
              "========== KDL Tree Diagnostics (%s arm) ==========",
              arm_prefix_.c_str());
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Diagnostics"),
              "Tree structure: %u segments, %u joints (NOTE: may include other arm in bimanual)",
              kdl_tree_.getNrOfSegments(), kdl_tree_.getNrOfJoints());
  
  // Print tree structure showing segments and their joint connections
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Diagnostics"),
              "=== Segment Tree Structure for this arm ===");
  
  std::function<void(const KDL::SegmentMap::const_iterator&, int)> print_tree;
  print_tree = [&](const KDL::SegmentMap::const_iterator& seg_it, int depth) {
    const KDL::Segment& seg = seg_it->second.segment;
    const KDL::Joint& joint = seg.getJoint();
    std::string seg_name = seg.getName();
    
    // Check if this segment belongs to current arm
    bool is_mine = (seg_name.find("openarm_" + arm_prefix_) != std::string::npos) ||
                   (has_o6_hand_ && arm_prefix_.find("left") != std::string::npos && seg_name[0] == 'L') ||
                   (has_o6_hand_ && arm_prefix_.find("right") != std::string::npos && seg_name[0] == 'R') ||
                   (has_leap_hand_ && seg_name.find("right_") != std::string::npos);
    
    if (is_mine) {
      std::string indent(depth * 2, ' ');
      std::string joint_type_str;
      switch (joint.getType()) {
        case KDL::Joint::None: 
          joint_type_str = "Fixed"; 
          break;
        case KDL::Joint::RotAxis: 
          joint_type_str = "Revolute"; 
          break;
        case KDL::Joint::RotX: 
          joint_type_str = "RotX"; 
          break;
        case KDL::Joint::RotY: 
          joint_type_str = "RotY"; 
          break;
        case KDL::Joint::RotZ: 
          joint_type_str = "RotZ"; 
          break;
        case KDL::Joint::TransAxis: 
          joint_type_str = "Prismatic"; 
          break;
        default: 
          joint_type_str = "Unknown"; 
          break;
      }
      
      double mass = seg.getInertia().getMass();
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Diagnostics"),
                  "%s└─ Segment: '%s' | Joint: '%s' (%s) | Mass: %.4f kg",
                  indent.c_str(), seg_name.c_str(), joint.getName().c_str(), 
                  joint_type_str.c_str(), mass);
    }
    
    // Recurse to children
    for (const auto& child_seg_it : seg_it->second.children) {
      print_tree(child_seg_it, depth + 1);
    }
  };
  
  KDL::SegmentMap::const_iterator root_seg = kdl_tree_.getRootSegment();
  print_tree(root_seg, 0);
  
  // Count and print only this arm's joints
  std::vector<std::pair<size_t, std::string>> arm_joints;
  for (size_t i = 0; i < kdl_joint_names_.size(); ++i) {
    const std::string& jname = kdl_joint_names_[i];
    bool is_mine = (jname.find("openarm_" + arm_prefix_) != std::string::npos) ||
                   (has_o6_hand_ && arm_prefix_.find("left") != std::string::npos && jname[0] == 'L') ||
                   (has_o6_hand_ && arm_prefix_.find("right") != std::string::npos && jname[0] == 'R') ||
                   (has_leap_hand_ && jname.find("right_") != std::string::npos);
    if (is_mine) {
      arm_joints.push_back({i, jname});
    }
  }
  
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Diagnostics"),
              "This arm's joint mapping (%zu joints):", arm_joints.size());
  for (const auto& [idx, name] : arm_joints) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Diagnostics"),
                "  [%02zu] %s", idx, name.c_str());
  }
  
  // Print current gravity torques if available
  if (gravity_torques_.rows() > 0) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Diagnostics"),
                "Current gravity torques (Nm) - at ZERO position:");
    for (size_t i = 0; i < static_cast<size_t>(gravity_torques_.rows()); ++i) {
      if (i < kdl_joint_names_.size()) {
        RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Diagnostics"),
                    "  %s: %.4f Nm", kdl_joint_names_[i].c_str(), gravity_torques_(i));
      }
    }
    
    // Test gravity calculation with non-zero position (one-time diagnostic, reuse member buffers)
    if (kdl_tree_solver_) {
      // Save current content and temporarily set joint2 to 45 deg
      std::string test_joint = "openarm_" + arm_prefix_ + "joint2";
      auto it = joint_name_to_kdl_idx_.find(test_joint);
      if (it != joint_name_to_kdl_idx_.end()) {
        KDL::SetToZero(kdl_q_buf_);
        kdl_q_buf_(it->second) = 0.785;  // 45 degrees
        // kdl_qdot/qddot/f_ext buffers are already zero/empty
        int result = kdl_tree_solver_->CartToJnt(
            kdl_q_buf_, kdl_qdot_buf_, kdl_qddot_buf_, kdl_f_ext_buf_, kdl_tau_buf_);
        if (result == 0) {
          double test_torque = kdl_tau_buf_(it->second);
          RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Diagnostics"),
                      "Test: %s at 45deg -> gravity torque = %.4f Nm (should be non-zero if mass data exists)",
                      test_joint.c_str(), test_torque);
          if (std::abs(test_torque) < 0.001) {
            RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW_Diagnostics"),
                        "WARNING: Gravity torque is near zero even at 45deg! Check URDF mass/inertia data!");
          }
        }
        KDL::SetToZero(kdl_q_buf_);  // restore to zero after test
      }
    }
  }
  
  // Print controlled joints mapping
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Diagnostics"),
              "Controlled joints (hardware interface):");
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    auto it = joint_name_to_kdl_idx_.find(joint_names_[i]);
    if (it != joint_name_to_kdl_idx_.end()) {
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Diagnostics"),
                  "  HW[%02zu] %s -> KDL[%d]", i, joint_names_[i].c_str(), it->second);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW_Diagnostics"),
                  "  HW[%02zu] %s -> NOT FOUND in KDL", i, joint_names_[i].c_str());
    }
  }
  
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Diagnostics"),
              "==========================================");
}


// High-frequency arm control loop (WRITE ONLY @ 500Hz)
// This loop handles ONLY command sending, NOT reading (reading is done in state_read_loop)
void OpenArm_v10HW::arm_control_loop() {
  using namespace std::chrono;
  // Use constant from header: CONTROL_WRITE_RATE_HZ = 500Hz -> 2000us period
  const auto loop_period = microseconds(static_cast<int>(1000000.0 / CONTROL_WRITE_RATE_HZ));

  auto next_cycle = steady_clock::now() + loop_period;
  
  if (enable_frequency_diagnostics_) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Thread"), 
                "Arm control loop started (target: %.0fHz, diagnostics: ON)", CONTROL_WRITE_RATE_HZ);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Thread"), 
                "Arm control loop started (target: %.0fHz - WRITE ONLY)", CONTROL_WRITE_RATE_HZ);
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
    
    // Copy latest states from decoupled state_read_loop @ 200Hz (thread-safe).
    // state_read_loop applies LPF and updates these buffers independently.
    {
      std::lock_guard<std::mutex> lock(arm_state_mutex_);
      pos_state = arm_pos_state_buffer_;
      vel_state = arm_vel_state_buffer_;
      tau_state = arm_tau_state_buffer_;
    }
    
    // Compute gravity compensation using Tree-based solver
    // Pass pos_state (this cycle's mutex-locked snapshot) so KDL uses fresh, consistent data
    std::vector<double> gravity_comp(ARM_DOF, 0.0);
    if (use_gravity_compensation_ && kdl_tree_solver_) {
      compute_gravity_compensation(gravity_comp, pos_state);
    }
    
    // Compute friction compensation
    std::vector<double> friction_comp(ARM_DOF, 0.0);
    if (use_friction_compensation_) {
      compute_friction_compensation(friction_comp);
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
    if (!simulation_mode_) {
      openarm_->get_arm().mit_control_all(arm_params);
    }
    
    // Send gripper command if enabled
    if (hand_ && !simulation_mode_) {
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

// LEAP Hand control loop (500Hz - synchronized with arm)
void OpenArm_v10HW::leap_control_loop() {
  using namespace std::chrono;
  const auto loop_period = microseconds(2000);  // 500Hz = 2ms (matches arm control timing)
  auto next_cycle = steady_clock::now() + loop_period;
  
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Thread"), 
              "LEAP Hand control loop started (500Hz, synchronized with arm)");
  
  std::vector<double> pos_cmd(LEAP_HAND_DOF, 0.0);
  
  while (leap_thread_running_) {
    // Copy commands from buffer (thread-safe)
    {
      std::lock_guard<std::mutex> lock(leap_command_mutex_);
      pos_cmd = leap_pos_cmd_buffer_;
    }
    
    // Initialize CSV file on first iteration
    if (!leap_csv_initialized_) {
      std::string package_share_dir;
      try {
        package_share_dir = ament_index_cpp::get_package_share_directory("openarm_hardware");
      } catch (const std::exception& e) {
        package_share_dir = "/tmp";
      }
      
      auto now = std::chrono::system_clock::now();
      auto time_t_now = std::chrono::system_clock::to_time_t(now);
      std::tm tm_now;
      localtime_r(&time_t_now, &tm_now);
      
      std::ostringstream tmp_date;
      tmp_date << std::put_time(&tm_now, "%Y%m%d");
      std::string date_str = tmp_date.str();
      
      std::filesystem::path dir_path = std::filesystem::path(package_share_dir) / "debug_csvs" / date_str;
      try {
        std::filesystem::create_directories(dir_path);
      } catch (const std::exception &e) {
        RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW_Thread"), 
                    "Failed to create LEAP debug CSV directory '%s': %s", 
                    dir_path.c_str(), e.what());
      }
      
      std::ostringstream oss;
      oss << dir_path.string() << "/debug_leap_hand_"
          << std::put_time(&tm_now, "%Y%m%d_%H%M%S") << ".csv";
      std::string csv_filename = oss.str();
      leap_debug_csv_.open(csv_filename);
      
      if (!leap_debug_csv_.is_open()) {
        RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW_Thread"), 
                    "Failed to open LEAP debug CSV: %s", csv_filename.c_str());
      } else {
        leap_debug_csv_ << "timestamp,motor_id,pos_cmd_urdf,pos_cmd_leap,pos_state_urdf,"
                        << "pos_state_leap,pos_error_urdf,motor_name\n";
        RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Thread"), 
                    "LEAP Hand debug CSV created: %s", csv_filename.c_str());
      }
      leap_csv_initialized_ = true;
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
      
      // Protect serial write operation with mutex (RS-485 is half-duplex)
      {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        leap_group_sync_write_->txPacket();
      }
      
      // State reading is handled by the decoupled state_read_loop() @ 200Hz.
    }
    
    // Sleep until next cycle
    std::this_thread::sleep_until(next_cycle);
    next_cycle += loop_period;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Thread"), 
              "LEAP Hand control loop stopped");
}

// Decoupled state read loop — reads both CAN arm and LEAP Hand serial, applies LPF.
// Runs at CONTROL_READ_RATE_HZ (200Hz) independently of the 500Hz write loops.
// This prevents RS-485 read latency from stalling CAN command sending.
void OpenArm_v10HW::state_read_loop() {
  using namespace std::chrono;
  const auto loop_period =
    microseconds(static_cast<int>(1000000.0 / CONTROL_READ_RATE_HZ));
  auto next_cycle = steady_clock::now() + loop_period;

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Thread"),
              "State read loop started (target: %.0f Hz)%s", CONTROL_READ_RATE_HZ,
              simulation_mode_
                  ? (sim_use_cmd_feedback_ ? " [SIMULATION MODE — cmd feedback]" : " [SIMULATION MODE — sine-wave joints]")
                  : "");

  const size_t arm_buf_size = ARM_DOF + (hand_ && !has_o6_hand_ && !has_leap_hand_ ? 1 : 0);
  std::vector<double> pos_state(arm_buf_size, 0.0);
  std::vector<double> vel_state(arm_buf_size, 0.0);
  std::vector<double> tau_state(arm_buf_size, 0.0);
  std::vector<double> leap_pos_state(LEAP_HAND_DOF, 0.0);

  // For simulation mode: log joint positions every 5 seconds
  auto sim_log_time = steady_clock::now();

  while (state_read_thread_running_) {
    // ---- ARM STATE READ (CAN-FD) ----
    if (simulation_mode_) {
      if (sim_use_cmd_feedback_) {
        // Reflect commanded positions directly as state — perfect tracking
        // This allows any ros2_controller (joint_trajectory, etc.) to drive the sim
        {
          std::lock_guard<std::mutex> lk(arm_state_mutex_);
          for (size_t i = 0; i < ARM_DOF && i < arm_pos_cmd_buffer_.size(); ++i) {
            pos_state[i] = arm_pos_cmd_buffer_[i];
            vel_state[i] = 0.0;
            tau_state[i] = 0.0;
          }
        }
      } else {
        // Generate sine-wave joint positions to exercise KDL with non-trivial inputs
        // Each joint gets a different frequency and amplitude for realistic coverage
        sim_time_ += 1.0 / CONTROL_READ_RATE_HZ;
        const double amp[7] = {0.3, 0.5, 0.4, 0.6, 0.3, 0.4, 0.2};  // [rad]
        const double freq[7] = {0.2, 0.15, 0.25, 0.1, 0.3, 0.2, 0.35}; // [Hz]
        for (size_t i = 0; i < ARM_DOF; ++i) {
          pos_state[i] = amp[i] * std::sin(2.0 * M_PI * freq[i] * sim_time_);
          vel_state[i] = amp[i] * 2.0 * M_PI * freq[i] *
                         std::cos(2.0 * M_PI * freq[i] * sim_time_);
          tau_state[i] = 0.0;
        }
      }
      // Log simulated joint positions every 5 seconds
      if (duration_cast<seconds>(steady_clock::now() - sim_log_time).count() >= 5) {
        sim_log_time = steady_clock::now();
        RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Thread"),
                    "[%s][SIM%s t=%.1fs] q[rad]: j1=%+.3f j2=%+.3f j3=%+.3f "
                    "j4=%+.3f j5=%+.3f j6=%+.3f j7=%+.3f",
                    arm_prefix_.c_str(),
                    sim_use_cmd_feedback_ ? "(cmd)" : "(sin)",
                    sim_time_,
                    pos_state[0], pos_state[1], pos_state[2], pos_state[3],
                    pos_state[4], pos_state[5], pos_state[6]);
      }
    } else {
      openarm_->refresh_all();
      openarm_->recv_all();

      const auto& arm_motors = openarm_->get_arm().get_motors();
      for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
        pos_state[i] = arm_motors[i].get_position();
        vel_state[i] = arm_motors[i].get_velocity();
        tau_state[i] = arm_motors[i].get_torque();
      }

      // Read gripper state if enabled
      if (hand_ && arm_buf_size > ARM_DOF) {
        const auto& gripper_motors = openarm_->get_gripper().get_motors();
        if (!gripper_motors.empty()) {
          pos_state[ARM_DOF] = motor_radians_to_joint(gripper_motors[0].get_position());
          vel_state[ARM_DOF] = 0.0;
          tau_state[ARM_DOF] = 0.0;
        }
      }
    }

    // Apply low-pass filter to arm position states (LPF cutoff=30Hz, sample=200Hz, alpha≈0.49)
    arm_state_filter_.update(pos_state);
    const std::vector<double>& filtered_pos = arm_state_filter_.get();

    // Update arm state buffers (thread-safe, read by arm_control_loop and read())
    {
      std::lock_guard<std::mutex> lock(arm_state_mutex_);
      arm_pos_state_buffer_ = filtered_pos;  // LPF-smoothed positions
      arm_vel_state_buffer_ = vel_state;     // Raw velocities (LPF on pos is sufficient)
      arm_tau_state_buffer_ = tau_state;
    }

    // ---- LEAP HAND STATE READ (RS-485) ----
    if (has_leap_hand_ && leap_connected_) {
      int dxl_comm_result;
      {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        dxl_comm_result = leap_group_sync_read_pos_->txRxPacket();
      }
      if (dxl_comm_result == COMM_SUCCESS) {
        for (size_t i = 0; i < LEAP_HAND_DOF; ++i) {
          uint8_t motor_id = leap_motor_ids_[i];
          if (leap_group_sync_read_pos_->isAvailable(
                motor_id, LEAP_ADDR_PRESENT_POSITION, LEAP_LEN_PRESENT_POSITION)) {
            int32_t ticks = leap_group_sync_read_pos_->getData(
              motor_id, LEAP_ADDR_PRESENT_POSITION, LEAP_LEN_PRESENT_POSITION);
            leap_pos_state[i] = leap_to_urdf(static_cast<double>(ticks) * LEAP_POS_SCALE);
          }
        }

        // Apply low-pass filter to LEAP position states
        leap_state_filter_.update(leap_pos_state);
        const std::vector<double>& filtered_leap = leap_state_filter_.get();

        // Update LEAP state buffer (thread-safe)
        {
          std::lock_guard<std::mutex> lock(leap_state_mutex_);
          leap_pos_state_buffer_ = filtered_leap;
        }

        // Health: successful read
        health_status_.consecutive_read_failures = 0;
        health_status_.leap_healthy = true;
      } else {
        health_status_.consecutive_read_failures++;
        if (health_status_.consecutive_read_failures > MAX_CONSECUTIVE_FAILURES) {
          health_status_.leap_healthy = false;
          RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW_Thread"),
                      "State read loop: %zu consecutive LEAP read failures",
                      health_status_.consecutive_read_failures.load());
        }
        
        // Log data to CSV every 10 iterations (10Hz) to reduce file size
        if (leap_csv_sample_count_ % 1 == 0 && leap_debug_csv_.is_open()) {
          auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now().time_since_epoch()).count();
          // Motor names for readability
          const char* motor_names[] = {
            "index_side", "index_fwd", "index_pip", "index_dip",
            "middle_side", "middle_fwd", "middle_pip", "middle_dip",
            "ring_side", "ring_fwd", "ring_pip", "ring_dip",
            "thumb_side", "thumb_fwd", "thumb_pip", "thumb_dip"
          };
          // Get latest command buffer (thread-safe)
          std::vector<double> pos_cmd(LEAP_HAND_DOF, 0.0);
          {
            std::lock_guard<std::mutex> lock(leap_command_mutex_);
            pos_cmd = leap_pos_cmd_buffer_;
          }
          for (size_t i = 0; i < LEAP_HAND_DOF; ++i) {
            double pos_cmd_urdf = pos_cmd[i];
            double pos_cmd_leap = urdf_to_leap(pos_cmd_urdf);
            double pos_state_urdf = pos_state[i];
            double pos_state_leap = urdf_to_leap(pos_state_urdf);
            double pos_error = pos_cmd_urdf - pos_state_urdf;
            leap_debug_csv_ << timestamp << "," << static_cast<int>(leap_motor_ids_[i]) << ","
                           << pos_cmd_urdf << "," << pos_cmd_leap << ","
                           << pos_state_urdf << "," << pos_state_leap << ","
                           << pos_error << "," << motor_names[i] << "\n";
          }
        }
        leap_csv_sample_count_++;
      }
    }

    std::this_thread::sleep_until(next_cycle);
    next_cycle += loop_period;
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Thread"), "State read loop stopped");
}

// Health monitoring functions
void OpenArm_v10HW::check_health() {
  // Check arm health based on consecutive failures
  if (health_status_.consecutive_read_failures > MAX_CONSECUTIVE_FAILURES ||
      health_status_.consecutive_write_failures > MAX_CONSECUTIVE_FAILURES) {
    health_status_.arm_healthy = false;
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                "Arm health check failed: read_failures=%zu, write_failures=%zu",
                health_status_.consecutive_read_failures.load(),
                health_status_.consecutive_write_failures.load());
  }
  
  // Check LEAP health
  if (!health_status_.leap_healthy) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                "LEAP Hand health check failed");
  }
}

void OpenArm_v10HW::report_health_status() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "=== Health Status ===");
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Arm healthy: %s, LEAP healthy: %s",
              health_status_.arm_healthy ? "YES" : "NO",
              health_status_.leap_healthy ? "YES" : "NO");
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Consecutive read failures: %zu",
              health_status_.consecutive_read_failures.load());
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Consecutive write failures: %zu",
              health_status_.consecutive_write_failures.load());
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Last read latency: %.2f ms",
              health_status_.last_read_latency_ms.load());
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Last write latency: %.2f ms",
              health_status_.last_write_latency_ms.load());
}

bool OpenArm_v10HW::is_healthy() const {
  return health_status_.arm_healthy && health_status_.leap_healthy;
}

// Health monitoring thread implementation
void OpenArm_v10HW::health_monitor_loop() {
  using namespace std::chrono;
  const auto loop_period = seconds(1);  // Check health every second
  auto next_cycle = steady_clock::now() + loop_period;
  
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Health"), "Health monitoring thread started");
  
  while (health_monitor_running_) {
    // Check system health
    check_health();
    
    // Update system health status
    health_status_.system_healthy = is_healthy();
    
    // Log health status periodically
    if (enable_frequency_diagnostics_) {
      report_health_status();
    }
    
    // Check for emergency stop conditions
    if (!health_status_.system_healthy && !health_status_.emergency_stop_triggered) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW_Health"),
                   "System health check failed, triggering emergency stop");
      health_status_.emergency_stop_triggered = true;
      
      // Trigger emergency stop by disabling all motors
      if (openarm_) {
        openarm_->disable_all();
        RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Health"),
                    "Emergency stop executed: all motors disabled");
      }
    }
    
    std::this_thread::sleep_until(next_cycle);
    next_cycle += loop_period;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW_Health"), "Health monitoring thread stopped");
}

// ============================================================================
// O6 Hand Functions
// ============================================================================

bool OpenArm_v10HW::connect_o6_hand() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Connecting to O6 Hand on %s (%s hand) [arm_prefix=%s]...",
              o6_can_interface_.c_str(), o6_hand_type_.c_str(), arm_prefix_.c_str());

  try {
    // Determine CAN channel from interface name
    COMM_TYPE channel;
    if (o6_can_interface_ == "can0") {
      channel = COMM_TYPE::COMM_CAN_0;
    } else if (o6_can_interface_ == "can1") {
      channel = COMM_TYPE::COMM_CAN_1;
    } else if (o6_can_interface_ == "can2") {
      // Map additional CAN interfaces to the available COMM channels
      channel = COMM_TYPE::COMM_CAN_0;  // can2 maps to COMM_CAN_0
    } else if (o6_can_interface_ == "can3") {
      channel = COMM_TYPE::COMM_CAN_1;  // can3 maps to COMM_CAN_1
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                   "Invalid O6 CAN interface: %s", o6_can_interface_.c_str());
      return false;
    }

    // Determine hand type
    HAND_TYPE hand_type = (o6_hand_type_ == "left") ? HAND_TYPE::LEFT : HAND_TYPE::RIGHT;

    // Create O6 hand API instance
    o6_hand_api_ = std::make_unique<LinkerHandApi>(
      LINKER_HAND::O6, hand_type, channel);

    // IMPORTANT: Enable the hand FIRST before setting speed/torque
    o6_hand_api_->setEnable();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Then set speed and torque for all 6 motors (0-255)
    std::vector<uint8_t> speed(6, 200);    // Default speed: 200/255 (~78%)
    o6_hand_api_->setSpeed(speed);
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Set O6 Hand speed to 200 [arm_prefix=%s]", arm_prefix_.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::vector<uint8_t> torque(6, 200);   // Default torque: 200/255 (~78%)
    o6_hand_api_->setTorque(torque);
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Set O6 Hand torque to 200 [arm_prefix=%s]", arm_prefix_.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Get version info
    std::string version = o6_hand_api_->getVersion();
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "O6 Hand connected successfully [arm_prefix=%s], version: %s",
                arm_prefix_.c_str(), version.c_str());

    o6_connected_ = true;
    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Failed to connect O6 Hand [arm_prefix=%s]: %s",
                 arm_prefix_.c_str(), e.what());
    o6_connected_ = false;
    return false;
  }
}

void OpenArm_v10HW::disconnect_o6_hand() {
  if (!o6_connected_ || !o6_hand_api_) {
    return;
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Disconnecting O6 Hand [arm_prefix=%s]...", arm_prefix_.c_str());

  try {
    // Disable the hand
    o6_hand_api_->setDisable();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    o6_hand_api_.reset();
    o6_connected_ = false;
    
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "O6 Hand disconnected");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Error disconnecting O6 Hand: %s", e.what());
  }
}

bool OpenArm_v10HW::send_o6_hand_command(const std::vector<double>& positions, size_t start_idx) {
  if (!o6_connected_ || !o6_hand_api_) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("OpenArm_v10HW"),
                         *rclcpp::Clock::make_shared(), 5000,
                         "O6 not connected, skipping command");
    return false;
  }

  try {
    // Convert radians to O6 motor units (0-255)
    // Correct mapping for O6: 
    //   0 rad (min, open hand) -> 255 motor (open)
    //   max rad (closed hand) -> 0 motor (closed)
    std::vector<uint8_t> motor_cmds(6);

    // Only send commands for the 6 active joints
    for (size_t i = 0; i < 6; ++i) {
      double pos_rad = positions[start_idx + i];
      // Clamp to joint limits
      pos_rad = std::max(O6_JOINT_MIN[i], std::min(pos_rad, O6_JOINT_MAX[i]));

      double min_angle = O6_JOINT_MIN[i];
      double max_angle = O6_JOINT_MAX[i];
      double normalized = 0.0;
      if (max_angle > min_angle) {
        normalized = (pos_rad - min_angle) / (max_angle - min_angle);
      }

      // Direct mapping: 0 rad -> 255 motor (open), max rad -> 0 motor (closed)
      double val = 255.0 - (normalized * 255.0);
      val = std::max(0.0, std::min(255.0, val));
      motor_cmds[i] = static_cast<uint8_t>(std::round(val));
    }

    // Commands are sent at 60Hz to O6 hand hardware

    // Send command to O6 hand via SDK (fingerMove expects 6 values 0-255)
    // Passive joints (6-10) are mechanically coupled and don't need commands
    o6_hand_api_->fingerMove(motor_cmds);
    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("OpenArm_v10HW"),
                          *rclcpp::Clock::make_shared(), 1000,
                          "Error sending O6 command: %s", e.what());
    return false;
  }
}

bool OpenArm_v10HW::read_o6_hand_states(std::vector<double>& positions, size_t start_idx) {
  if (!o6_connected_ || !o6_hand_api_) {
    return false;
  }

  try {
    // Read current positions from O6 hand (range values 0-255)
    auto motor_positions = o6_hand_api_->getState();
    
    // O6 Hand has 6 active motors
    if (motor_positions.size() != 6) {
      return false;
    }

    // Convert motor units to radians for 6 active joints
    // Reversed mapping (matching send command): motor 255 -> 0 rad (open), motor 0 -> max rad (closed)
    for (size_t i = 0; i < 6; ++i) {
      // Reverse the motor value: actual motor value is inverted
      double reversed_motor = 255.0 - motor_positions[i];
      double normalized = reversed_motor / 255.0;
      positions[start_idx + i] = O6_JOINT_MIN[i] + normalized * (O6_JOINT_MAX[i] - O6_JOINT_MIN[i]);
    }
    
    // Simulate passive/coupled DIP joints (joints 6-10)
    // These mimic relationships are defined in the URDF with specific multipliers
    // Reference: external openarm_description/urdf/ros2_control/openarm.bimanual.ros2_control.xacro
    positions[start_idx + 6] = positions[start_idx + 1] * 1.86;  // thumb_dip mimics thumb_cmc_pitch (multiplier=1.86)
    positions[start_idx + 7] = positions[start_idx + 2] * 0.89;  // index_dip mimics index_mcp_pitch (multiplier=0.89)
    positions[start_idx + 8] = positions[start_idx + 3] * 0.89;  // middle_dip mimics middle_mcp_pitch (multiplier=0.89)
    positions[start_idx + 9] = positions[start_idx + 4] * 0.89;  // ring_dip mimics ring_mcp_pitch (multiplier=0.89)
    positions[start_idx + 10] = positions[start_idx + 5] * 0.89; // pinky_dip mimics pinky_mcp_pitch (multiplier=0.89)

    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("OpenArm_v10HW"),
                          *rclcpp::Clock::make_shared(), 1000,
                          "Error reading O6 states: %s", e.what());
    return false;
  }
}

void OpenArm_v10HW::o6_control_loop() {
  std::string logger_name = "OpenArm_v10HW_O6_" + arm_prefix_;  // Add arm prefix to logger
  RCLCPP_INFO(rclcpp::get_logger(logger_name), "O6 Hand control thread started for %s", arm_prefix_.c_str());

  const auto loop_period = std::chrono::microseconds(16667);  // ~60Hz
  auto next_cycle = std::chrono::steady_clock::now() + loop_period;

  while (o6_thread_running_) {
    // Read current O6 positions (all 11: 6 active + 5 passive)
    std::vector<double> temp_positions(O6_HAND_DOF);
    if (read_o6_hand_states(temp_positions, 0)) {
      // Apply low-pass filter and update state buffer
      o6_state_filter_.update(temp_positions);
      const auto &filtered = o6_state_filter_.get();
      {
        std::lock_guard<std::mutex> lock(o6_state_mutex_);
        o6_pos_state_buffer_ = filtered;
      }
    }

    // Get command from buffer (thread-safe) - only 6 active joints
    std::vector<double> cmd_positions(6);
    {
      std::lock_guard<std::mutex> lock(o6_command_mutex_);
      cmd_positions = o6_pos_cmd_buffer_;  // Copy 6 active joint commands
    }

    // Send command to O6 hand (only 6 active joints)
    send_o6_hand_command(cmd_positions, 0);

    std::this_thread::sleep_until(next_cycle);
    next_cycle += loop_period;
  }

  RCLCPP_INFO(rclcpp::get_logger(logger_name), "O6 Hand control thread stopped for %s", arm_prefix_.c_str());
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_v10HW,
                       hardware_interface::SystemInterface)
