// Copyright 2026 Enactic, Inc.
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

#include "openarm_hardware/o6_hand_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// Include LinkerHandApi SDK
#include "LinkerHandApi.h"

namespace openarm_hardware
{

// O6 hand constants and joint limits (from LinkerHand SDK)
static constexpr size_t NUM_JOINTS = 6;
static constexpr std::array<double, NUM_JOINTS> JOINT_MIN = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static constexpr std::array<double, NUM_JOINTS> JOINT_MAX = {0.58, 1.36, 1.6, 1.6, 1.6, 1.6};

hardware_interface::CallbackReturn O6HandHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get CAN interface parameter (default: can2 for right, can3 for left)
  can_interface_ = "can2";
  if (info_.hardware_parameters.find("can_interface") != info_.hardware_parameters.end())
  {
    can_interface_ = info_.hardware_parameters["can_interface"];
  }

  // Get hand type parameter (left or right)
  hand_type_ = "right";
  if (info_.hardware_parameters.find("hand_type") != info_.hardware_parameters.end())
  {
    hand_type_ = info_.hardware_parameters["hand_type"];
  }

  // Get hand prefix (e.g., "right_" or "left_")
  hand_prefix_ = "";
  if (info_.hardware_parameters.find("hand_prefix") != info_.hardware_parameters.end())
  {
    hand_prefix_ = info_.hardware_parameters["hand_prefix"];
  }

  // Get move_to_home parameter (default: false to avoid sudden movements)
  move_to_home_on_activate_ = false;
  if (info_.hardware_parameters.find("move_to_home") != info_.hardware_parameters.end())
  {
    std::string move_to_home_str = info_.hardware_parameters["move_to_home"];
    move_to_home_on_activate_ = (move_to_home_str == "true" || move_to_home_str == "True");
  }

  // Get init_speed parameter (0-255, default: 150)
  init_speed_ = 150;
  if (info_.hardware_parameters.find("init_speed") != info_.hardware_parameters.end())
  {
    try {
      int speed = std::stoi(info_.hardware_parameters["init_speed"]);
      init_speed_ = static_cast<uint8_t>(std::max(0, std::min(255, speed)));
    } catch (const std::exception& e) {
      RCLCPP_WARN(rclcpp::get_logger("O6HandHardware"),
                  "Invalid init_speed parameter, using default: 150");
    }
  }

  // Get init_torque parameter (0-255, default: 150)
  init_torque_ = 150;
  if (info_.hardware_parameters.find("init_torque") != info_.hardware_parameters.end())
  {
    try {
      int torque = std::stoi(info_.hardware_parameters["init_torque"]);
      init_torque_ = static_cast<uint8_t>(std::max(0, std::min(255, torque)));
    } catch (const std::exception& e) {
      RCLCPP_WARN(rclcpp::get_logger("O6HandHardware"),
                  "Invalid init_torque parameter, using default: 150");
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"), 
              "O6 Hand CAN interface: %s, hand_type: %s, prefix: %s, move_to_home: %s", 
              can_interface_.c_str(), hand_type_.c_str(), hand_prefix_.c_str(),
              move_to_home_on_activate_ ? "true" : "false");
  RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"),
              "O6 Hand init_speed: %d, init_torque: %d", 
              init_speed_, init_torque_);

  // Initialize state and command vectors (O6 has 6 DOF)
  hw_states_position_.resize(info_.joints.size(), 0.0);
  hw_states_velocity_.resize(info_.joints.size(), 0.0);
  hw_states_effort_.resize(info_.joints.size(), 0.0);
  hw_commands_position_.resize(info_.joints.size(), 0.0);
  hw_commands_velocity_.resize(info_.joints.size(), 0.0);
  hw_commands_effort_.resize(info_.joints.size(), 0.0);

  // Store joint names and log order
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    joint_names_.push_back(info_.joints[i].name);
    RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"), 
                "Joint[%zu] = %s (min=%.3f, max=%.3f)", 
                i, info_.joints[i].name.c_str(), JOINT_MIN[i], JOINT_MAX[i]);
  }

  RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"), 
              "Initialized O6 Hand with %zu joints", info_.joints.size());

  is_connected_ = false;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn O6HandHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"), "Configuring O6 Hand hardware...");
  
  // Create ROS2 node for publishing touch sensor data
  // Use hand_prefix_ to create appropriate topic names
  std::string node_name = hand_type_ + "_hand_touch_publisher";
  node_ = std::make_shared<rclcpp::Node>(node_name);
  
  // Create topic names based on hand type (left or right)
  std::string prefix = "cb_" + hand_type_ + "_hand";
  std::string touch_topic = prefix + "_matrix_touch";
  std::string mass_topic = prefix + "_matrix_touch_mass";
  std::string pc_topic = prefix + "_matrix_touch_pc";
  
  // Initialize publishers
  touch_matrix_pub_ = node_->create_publisher<std_msgs::msg::String>(touch_topic, 10);
  touch_mass_pub_ = node_->create_publisher<std_msgs::msg::String>(mass_topic, 10);
  touch_pc_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(pc_topic, 10);
  
  RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"), 
              "Created touch sensor publishers: %s, %s, %s",
              touch_topic.c_str(), mass_topic.c_str(), pc_topic.c_str());
  
  RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"), "O6 Hand configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> O6HandHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> O6HandHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_effort_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn O6HandHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"), "Activating O6 Hand hardware...");

  // Try to connect to hand
  if (!connect_hand())
  {
    RCLCPP_WARN(rclcpp::get_logger("O6HandHardware"), 
                "Failed to connect to O6 Hand on %s, entering simulation mode", can_interface_.c_str());
    // Initialize to home position for simulation mode
    for (size_t i = 0; i < hw_states_position_.size(); i++)
    {
      hw_states_position_[i] = 0.0;
      hw_commands_position_[i] = 0.0;
    }
    RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"), 
                "Simulation mode initialized to home position (0.0 rad)");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Read current actual position from hardware to avoid sudden jumps
  RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"), "Reading current position from hardware...");
  
  // Try multiple times to ensure stable reading
  bool read_success = false;
  for (int attempt = 0; attempt < 3; attempt++)
  {
    if (attempt > 0)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"), "Retry reading position (attempt %d/3)...", attempt + 1);
    }
    
    if (read_joint_states(hw_states_position_, hw_states_velocity_, hw_states_effort_))
    {
      // Initialize command to current position
      hw_commands_position_ = hw_states_position_;
      
      RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"), 
                  "Position read successfully - Joint 0: %.3f rad", 
                  hw_states_position_[0]);
      read_success = true;
      break;
    }
  }
  
  if (!read_success)
  {
    RCLCPP_ERROR(rclcpp::get_logger("O6HandHardware"), 
                 "Failed to read initial position after 3 attempts!");
    // Initialize to home position as fallback
    for (size_t i = 0; i < hw_states_position_.size(); i++)
    {
      hw_states_position_[i] = 0.0;
      hw_commands_position_[i] = 0.0;
    }
  }

  // Optionally move to home position
  if (move_to_home_on_activate_)
  {
    RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"), "Moving to home position...");
    return_to_home();
  }

  RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"), "O6 Hand activated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn O6HandHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"), "Deactivating O6 Hand hardware...");
  
  disconnect_hand();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type O6HandHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (is_connected_)
  {
    // Read actual joint states from hardware
    // Use temporary vectors to avoid corrupting state on failure
    std::vector<double> temp_pos = hw_states_position_;
    std::vector<double> temp_vel = hw_states_velocity_;
    std::vector<double> temp_eff = hw_states_effort_;
    
    if (read_joint_states(temp_pos, temp_vel, temp_eff))
    {
      // Only update if read was successful
      hw_states_position_ = temp_pos;
      hw_states_velocity_ = temp_vel;
      hw_states_effort_ = temp_eff;
      
      // Read and publish touch sensor data
      try {
        auto force_data = hand_api_->getForce();
        
        // Debug: Log force_data structure
        static int debug_count = 0;
        if (debug_count < 5) {  // Only log first 5 times
          RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"),
                      "DEBUG: getForce() returned %zu fingers", force_data.size());
          for (size_t i = 0; i < force_data.size(); ++i) {
            RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"),
                        "  Finger[%zu]: %zu rows", i, force_data[i].size());
            if (!force_data[i].empty()) {
              RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"),
                          "    Row[0]: %zu columns", force_data[i][0].size());
              // Print first few values
              std::stringstream ss;
              for (size_t j = 0; j < std::min(size_t(6), force_data[i][0].size()); ++j) {
                ss << static_cast<int>(force_data[i][0][j]) << " ";
              }
              RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"),
                          "    First values: %s", ss.str().c_str());
            }
          }
          debug_count++;
        }
        
        // Check if force_data is valid and has expected size (5 fingers for O6 hand)
        if (force_data.empty()) {
          static auto last_empty_warn = std::chrono::steady_clock::now();
          auto now = std::chrono::steady_clock::now();
          if (std::chrono::duration_cast<std::chrono::seconds>(now - last_empty_warn).count() >= 5) {
            RCLCPP_WARN(rclcpp::get_logger("O6HandHardware"),
                        "getForce() returned empty data - touch sensor may not be available");
            last_empty_warn = now;
          }
          return hardware_interface::return_type::OK;
        }
        
        // Publish touch matrix data (JSON format matching Python SDK)
        if (touch_matrix_pub_ && touch_matrix_pub_->get_subscription_count() > 0) {
          std_msgs::msg::String touch_msg;
          std::stringstream ss;
          
          // Get current time
          auto now = node_->get_clock()->now();
          
          // Build JSON: {"stamp": {"secs": ..., "nsecs": ...}, "thumb_matrix": [...], ...}
          ss << "{";
          ss << "\"stamp\":{\"secs\":" << now.seconds() << ",\"nsecs\":" << now.nanoseconds() % 1000000000 << "},";
          
          // Finger names matching Python SDK
          const std::vector<std::string> finger_names = {
            "thumb_matrix", "index_matrix", "middle_matrix", "ring_matrix", "little_matrix"
          };
          
          for (size_t finger = 0; finger < std::min(force_data.size(), finger_names.size()); ++finger) {
            if (finger > 0) ss << ",";
            ss << "\"" << finger_names[finger] << "\":[";
            for (size_t row = 0; row < force_data[finger].size(); ++row) {
              if (row > 0) ss << ",";
              ss << "[";
              for (size_t col = 0; col < force_data[finger][row].size(); ++col) {
                if (col > 0) ss << ",";
                ss << static_cast<int>(force_data[finger][row][col]);
              }
              ss << "]";
            }
            ss << "]";
          }
          ss << "}";
          touch_msg.data = ss.str();
          touch_matrix_pub_->publish(touch_msg);
        }
        
        // Publish touch mass data (JSON format matching Python SDK)
        if (touch_mass_pub_ && touch_mass_pub_->get_subscription_count() > 0) {
          std_msgs::msg::String mass_msg;
          std::stringstream ss;
          
          // Get current time
          auto now = node_->get_clock()->now();
          
          // Build JSON: {"stamp": {...}, "unit": "g", "thumb_mass": ..., ...}
          ss << "{";
          ss << "\"stamp\":{\"secs\":" << now.seconds() << ",\"nsecs\":" << now.nanoseconds() % 1000000000 << "},";
          ss << "\"unit\":\"g\",";
          
          // Finger mass names matching Python SDK
          const std::vector<std::string> mass_names = {
            "thumb_mass", "index_mass", "middle_mass", "ring_mass", "little_mass"
          };
          
          for (size_t finger = 0; finger < std::min(force_data.size(), mass_names.size()); ++finger) {
            if (finger > 0) ss << ",";
            uint32_t total_force = 0;
            for (const auto& row : force_data[finger]) {
              for (const auto& val : row) {
                total_force += val;
              }
            }
            ss << "\"" << mass_names[finger] << "\":" << total_force;
          }
          ss << "}";
          mass_msg.data = ss.str();
          touch_mass_pub_->publish(mass_msg);
        }
        
        // Publish touch point cloud data (PointCloud2 format)
        if (touch_pc_pub_ && touch_pc_pub_->get_subscription_count() > 0) {
          sensor_msgs::msg::PointCloud2 pc_msg;
          pc_msg.header.stamp = node_->get_clock()->now();
          pc_msg.header.frame_id = hand_prefix_ + "palm_link";
          
          // Setup point cloud fields: x, y, z, intensity
          pc_msg.fields.resize(4);
          pc_msg.fields[0].name = "x";
          pc_msg.fields[0].offset = 0;
          pc_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
          pc_msg.fields[0].count = 1;
          
          pc_msg.fields[1].name = "y";
          pc_msg.fields[1].offset = 4;
          pc_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
          pc_msg.fields[1].count = 1;
          
          pc_msg.fields[2].name = "z";
          pc_msg.fields[2].offset = 8;
          pc_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
          pc_msg.fields[2].count = 1;
          
          pc_msg.fields[3].name = "intensity";
          pc_msg.fields[3].offset = 12;
          pc_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
          pc_msg.fields[3].count = 1;
          
          // Calculate total number of points
          size_t total_points = 0;
          for (const auto& finger : force_data) {
            for (const auto& row : finger) {
              for (const auto& val : row) {
                if (val > 0) total_points++; // Only include active touch points
              }
            }
          }
          
          pc_msg.point_step = 16; // 4 floats * 4 bytes
          pc_msg.row_step = pc_msg.point_step * total_points;
          pc_msg.width = total_points;
          pc_msg.height = 1;
          pc_msg.is_dense = true;
          pc_msg.data.resize(pc_msg.row_step);
          
          // Fill point cloud data
          size_t point_idx = 0;
          for (size_t finger = 0; finger < force_data.size(); ++finger) {
            for (size_t row = 0; row < force_data[finger].size(); ++row) {
              for (size_t col = 0; col < force_data[finger][row].size(); ++col) {
                uint8_t force_val = force_data[finger][row][col];
                if (force_val > 0) {
                  float* point = reinterpret_cast<float*>(&pc_msg.data[point_idx * pc_msg.point_step]);
                  point[0] = static_cast<float>(finger) * 0.02f; // x: finger spacing
                  point[1] = static_cast<float>(row) * 0.01f;    // y: row spacing
                  point[2] = static_cast<float>(col) * 0.01f;    // z: column spacing
                  point[3] = static_cast<float>(force_val);       // intensity: force value
                  point_idx++;
                }
              }
            }
          }
          
          touch_pc_pub_->publish(pc_msg);
        }
      } catch (const std::exception& e) {
        static auto last_touch_warn_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_touch_warn_time).count() >= 5) {
          RCLCPP_WARN(rclcpp::get_logger("O6HandHardware"),
                      "Failed to read touch sensor data: %s", e.what());
          last_touch_warn_time = now;
        }
      }
    }
    else
    {
      // Keep previous values on read failure
      static auto last_warn_time = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - last_warn_time).count() >= 5) {
        RCLCPP_WARN(rclcpp::get_logger("O6HandHardware"),
                    "Failed to read O6 Hand joint states, keeping previous values");
        last_warn_time = now;
      }
    }
  }
  else
  {
    // Simulation mode - mirror commands to states
    hw_states_position_ = hw_commands_position_;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type O6HandHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (is_connected_)
  {
    // Send position commands to hardware
    if (!send_position_command(hw_commands_position_))
    {
      RCLCPP_WARN(rclcpp::get_logger("O6HandHardware"),
                  "Failed to write O6 Hand commands");
    }
  }

  return hardware_interface::return_type::OK;
}

bool O6HandHardware::connect_hand()
{
  RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"), 
              "Attempting to connect to O6 Hand on %s (type: %s)", 
              can_interface_.c_str(), hand_type_.c_str());

  // Map CAN interface string to COMM_TYPE enum
  COMM_TYPE channel;
  if (can_interface_ == "can0") {
    channel = COMM_TYPE::COMM_CAN_0;
  } else if (can_interface_ == "can1") {
    channel = COMM_TYPE::COMM_CAN_1;
  } else if (can_interface_ == "can2") {
    channel = COMM_TYPE::COMM_CAN_0;  // Assuming can2 maps to COMM_CAN_0
  } else if (can_interface_ == "can3") {
    channel = COMM_TYPE::COMM_CAN_1;  // Assuming can3 maps to COMM_CAN_1
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("O6HandHardware"),
                 "Invalid CAN interface: %s", can_interface_.c_str());
    return false;
  }

  // Map hand_type string to HAND_TYPE enum
  HAND_TYPE hand_enum;
  if (hand_type_ == "left") {
    hand_enum = HAND_TYPE::LEFT;
  } else if (hand_type_ == "right") {
    hand_enum = HAND_TYPE::RIGHT;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("O6HandHardware"),
                 "Invalid hand type: %s", hand_type_.c_str());
    return false;
  }

  try {
    // Create LinkerHandApi instance for O6 hand
    hand_api_ = std::make_unique<LinkerHandApi>(LINKER_HAND::O6, hand_enum, channel);
    
    // Enable the hand first
    hand_api_->setEnable();
    
    // Delay to ensure hand is fully enabled and ready
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Initialize speed and torque settings from parameters (one-time setup)
    std::vector<uint8_t> init_speed(NUM_JOINTS, init_speed_);
    hand_api_->setSpeed(init_speed);
    RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"), 
                "Set initial speed to %d for all %zu joints", init_speed_, NUM_JOINTS);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::vector<uint8_t> init_torque(NUM_JOINTS, init_torque_);
    hand_api_->setTorque(init_torque);
    RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"), 
                "Set initial torque to %d for all %zu joints", init_torque_, NUM_JOINTS);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    is_connected_ = true;
    RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"), 
                "Successfully connected to O6 Hand");
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("O6HandHardware"),
                 "Failed to connect to O6 Hand: %s", e.what());
    return false;
  }
}

void O6HandHardware::disconnect_hand()
{
  if (!is_connected_)
  {
    return;
  }

  try {
    // Disable the hand
    if (hand_api_) {
      hand_api_->setDisable();
    }
    
    hand_api_.reset();
    is_connected_ = false;
    
    RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"), "Disconnected from O6 Hand");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("O6HandHardware"),
                 "Error during disconnect: %s", e.what());
  }
}

bool O6HandHardware::send_position_command(const std::vector<double> & positions)
{
  if (!is_connected_ || !hand_api_)
  {
    return false;
  }

  try {
    // Convert radians to range values (0-255)
    auto range_values = radians_to_range(positions);
    
    // Send command using SDK fingerMove API
    hand_api_->fingerMove(range_values);
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("O6HandHardware"),
                "Failed to send position command: %s", e.what());
    return false;
  }
}

bool O6HandHardware::read_joint_states(std::vector<double> & positions,
                                       std::vector<double> & velocities,
                                       std::vector<double> & efforts)
{
  if (!is_connected_ || !hand_api_)
  {
    static auto last_warn_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_warn_time).count() >= 5) {
      RCLCPP_WARN(rclcpp::get_logger("O6HandHardware"), 
                  "read_joint_states: not connected or no hand_api");
      last_warn_time = now;
    }
    return false;
  }

  try {
    // Read position (range values 0-250)
    auto range_values = hand_api_->getState();
    
    // Validate data size
    if (range_values.size() != NUM_JOINTS)
    {
      static auto last_error_time = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - last_error_time).count() >= 1) {
        RCLCPP_ERROR(rclcpp::get_logger("O6HandHardware"),
                     "Invalid getState() size: expected %zu, got %zu", 
                     NUM_JOINTS, range_values.size());
        last_error_time = now;
      }
      return false;
    }
    
    // Convert and validate
    auto new_positions = range_to_radians(range_values);
    
    // Sanity check: all values should be within reasonable bounds
    bool all_valid = true;
    for (size_t i = 0; i < new_positions.size(); i++) {
      if (std::isnan(new_positions[i]) || std::isinf(new_positions[i]) ||
          new_positions[i] < -10.0 || new_positions[i] > 10.0) {
        RCLCPP_ERROR(rclcpp::get_logger("O6HandHardware"),
                     "Invalid position[%zu]: %.3f (range_value: %d)", 
                     i, new_positions[i], range_values[i]);
        all_valid = false;
      }
    }
    
    if (!all_valid) {
      return false;
    }
    
    // Data is valid, update positions
    positions = new_positions;
    
    // Read velocity (range values 0-250)
    auto speed_values = hand_api_->getSpeed();
    if (speed_values.size() == velocities.size()) {
      for (size_t i = 0; i < speed_values.size(); i++) {
        velocities[i] = static_cast<double>(speed_values[i]) / 250.0;  // Normalize
      }
    }
    
    // Read effort/torque (range values 0-250)
    auto torque_values = hand_api_->getTorque();
    if (torque_values.size() == efforts.size()) {
      for (size_t i = 0; i < torque_values.size(); i++) {
        efforts[i] = static_cast<double>(torque_values[i]) / 250.0;  // Normalize
      }
    }
    
    return true;
  } catch (const std::exception& e) {
    static auto last_exception_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_exception_time).count() >= 1) {
      RCLCPP_ERROR(rclcpp::get_logger("O6HandHardware"),
                   "Exception in read_joint_states: %s", e.what());
      last_exception_time = now;
    }
    return false;
  }
}

void O6HandHardware::return_to_home()
{
  if (!is_connected_)
  {
    RCLCPP_WARN(rclcpp::get_logger("O6HandHardware"), 
                "Cannot move to home - not connected to hardware");
    return;
  }

  RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"), 
              "Moving O6 Hand to home position (0.0 rad)...");

  // Home position is 0.0 rad for all joints
  std::vector<double> home_position(O6_HAND_DOF, 0.0);
  
  // Send home position command
  if (!send_position_command(home_position))
  {
    RCLCPP_ERROR(rclcpp::get_logger("O6HandHardware"), 
                 "Failed to send home position command");
    return;
  }

  // Wait for movement to complete
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  
  // Read back position to verify
  if (read_joint_states(hw_states_position_, hw_states_velocity_, hw_states_effort_))
  {
    hw_commands_position_ = hw_states_position_;
    RCLCPP_INFO(rclcpp::get_logger("O6HandHardware"), 
                "Reached home position (first joint: %.3f rad)", hw_states_position_[0]);
  }
}

std::vector<double> O6HandHardware::range_to_radians(const std::vector<uint8_t> & range_values)
{
  // O6 hand joint limits from LinkerHand SDK (radians)
  // SDK mapping (CORRECTED): 
  //   range 0   -> max_angle (closed/握拳)
  //   range 255 -> min_angle (0.0 rad, open/張開)
  // (Using namespace-level constants defined above)
  
  std::vector<double> radians;
  radians.reserve(range_values.size());
  
  for (size_t i = 0; i < range_values.size() && i < NUM_JOINTS; ++i) {
    double val = std::max(0.0, std::min(255.0, static_cast<double>(range_values[i])));
    double min_angle = JOINT_MIN[i];  // 0.0 rad (open)
    double max_angle = JOINT_MAX[i];  // max rad (closed)
    
    // REVERSED mapping: 0->max (closed), 255->min (open)
    double rad = max_angle - (val / 255.0) * (max_angle - min_angle);
    radians.push_back(rad);
  }
  
  return radians;
}

std::vector<uint8_t> O6HandHardware::radians_to_range(const std::vector<double> & radians)
{
  // O6 hand joint limits (using namespace-level constants)
  
  std::vector<uint8_t> range_values;
  range_values.reserve(radians.size());
  
  for (size_t i = 0; i < radians.size() && i < NUM_JOINTS; ++i) {
    double min_angle = JOINT_MIN[i];  // 0.0 rad (open)
    double max_angle = JOINT_MAX[i];  // max rad (closed)
    
    // Clamp radians to valid range with warning
    double rad = radians[i];
    if (rad < min_angle || rad > max_angle) {
      RCLCPP_WARN(rclcpp::get_logger("O6HandHardware"),
                  "Joint %zu command %.3f rad clamped to [%.3f, %.3f]",
                  i, rad, min_angle, max_angle);
      rad = std::max(min_angle, std::min(max_angle, rad));
    }
    
    // REVERSED mapping: min(0 rad)->255 (open), max->0 (closed)
    double value = 255.0 - ((rad - min_angle) / (max_angle - min_angle) * 255.0);
    value = std::max(0.0, std::min(255.0, value));
    range_values.push_back(static_cast<uint8_t>(value));
  }
  
  return range_values;
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  openarm_hardware::O6HandHardware, hardware_interface::SystemInterface)
