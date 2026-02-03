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

#include "openarm_hardware/leap_hand_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openarm_hardware
{

hardware_interface::CallbackReturn LeapHandHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get serial port parameter (default: /dev/ttyUSB0)
  serial_port_ = "/dev/ttyUSB0";
  if (info_.hardware_parameters.find("serial_port") != info_.hardware_parameters.end())
  {
    serial_port_ = info_.hardware_parameters["serial_port"];
  }

  // Get baudrate parameter (default: 4000000 for Dynamixel XH)
  baudrate_ = 4000000;
  if (info_.hardware_parameters.find("baudrate") != info_.hardware_parameters.end())
  {
    baudrate_ = std::stoi(info_.hardware_parameters["baudrate"]);
  }

  // Get move_to_home parameter (default: false to avoid sudden movements)
  move_to_home_on_activate_ = false;
  if (info_.hardware_parameters.find("move_to_home") != info_.hardware_parameters.end())
  {
    std::string move_to_home_str = info_.hardware_parameters["move_to_home"];
    move_to_home_on_activate_ = (move_to_home_str == "true" || move_to_home_str == "True");
  }

  RCLCPP_INFO(rclcpp::get_logger("LeapHandHardware"), 
              "LEAP Hand serial port: %s, baudrate: %d, move_to_home: %s", 
              serial_port_.c_str(), baudrate_, 
              move_to_home_on_activate_ ? "true" : "false");

  // Initialize motor IDs (default: 0-15 for 16 motors)
  motor_ids_.clear();
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    motor_ids_.push_back(static_cast<uint8_t>(i));
  }

  // Initialize Dynamixel SDK objects
  port_handler_ = std::shared_ptr<dynamixel::PortHandler>(
    dynamixel::PortHandler::getPortHandler(serial_port_.c_str()));
  packet_handler_ = std::shared_ptr<dynamixel::PacketHandler>(
    dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION));
  
  group_sync_write_ = std::make_shared<dynamixel::GroupSyncWrite>(
    port_handler_.get(), packet_handler_.get(), ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
  group_sync_read_pos_ = std::make_shared<dynamixel::GroupSyncRead>(
    port_handler_.get(), packet_handler_.get(), ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
  group_sync_read_vel_ = std::make_shared<dynamixel::GroupSyncRead>(
    port_handler_.get(), packet_handler_.get(), ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
  group_sync_read_cur_ = std::make_shared<dynamixel::GroupSyncRead>(
    port_handler_.get(), packet_handler_.get(), ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);

  // Initialize joint arrays
  hw_states_position_.resize(info_.joints.size(), 0.0);
  hw_states_velocity_.resize(info_.joints.size(), 0.0);
  hw_states_effort_.resize(info_.joints.size(), 0.0);
  hw_commands_position_.resize(info_.joints.size(), 0.0);
  hw_commands_velocity_.resize(info_.joints.size(), 0.0);
  hw_commands_effort_.resize(info_.joints.size(), 0.0);

  // Store joint names
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
  }

  RCLCPP_INFO(rclcpp::get_logger("LeapHandHardware"), 
              "Initialized LEAP Hand with %zu joints", info_.joints.size());

  is_connected_ = false;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LeapHandHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("LeapHandHardware"), "Configuring LEAP Hand hardware...");
  
  // Don't reset positions here - will be read from hardware in on_activate()
  // Only initialize if values are uninitialized (NaN or invalid)
  
  RCLCPP_INFO(rclcpp::get_logger("LeapHandHardware"), "LEAP Hand configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LeapHandHardware::export_state_interfaces()
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

std::vector<hardware_interface::CommandInterface> LeapHandHardware::export_command_interfaces()
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

hardware_interface::CallbackReturn LeapHandHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("LeapHandHardware"), "Activating LEAP Hand hardware...");

  // Try to connect to serial port
  if (!connect_serial())
  {
    RCLCPP_WARN(rclcpp::get_logger("LeapHandHardware"), 
                "Failed to connect to LEAP Hand on %s, entering simulation mode", serial_port_.c_str());
    // Don't return ERROR - allow activation even if hardware not connected
    // This enables simulation mode
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Read current actual position from hardware to avoid sudden jumps
  RCLCPP_INFO(rclcpp::get_logger("LeapHandHardware"), "Reading current position from hardware...");
  
  // Try multiple times to ensure stable reading
  bool read_success = false;
  for (int attempt = 0; attempt < 3; attempt++)
  {
    if (attempt > 0)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      RCLCPP_INFO(rclcpp::get_logger("LeapHandHardware"), "Retry reading position (attempt %d/3)...", attempt + 1);
    }
    
    if (read_joint_states(hw_states_position_, hw_states_velocity_, hw_states_effort_))
    {
      // Initialize command to current position
      hw_commands_position_ = hw_states_position_;
      
      RCLCPP_INFO(rclcpp::get_logger("LeapHandHardware"), 
                  "Position read successfully - Joint 0: %.3f rad, Joint 8: %.3f rad", 
                  hw_states_position_[0], 
                  hw_states_position_.size() > 8 ? hw_states_position_[8] : 0.0);
      read_success = true;
      break;
    }
  }
  
  if (!read_success)
  {
    RCLCPP_ERROR(rclcpp::get_logger("LeapHandHardware"), 
                 "Failed to read initial position after 3 attempts!");
    // Initialize to home position as fallback
    for (size_t i = 0; i < hw_states_position_.size(); i++)
    {
      hw_states_position_[i] = 0.0;
      hw_commands_position_[i] = 0.0;
    }
  }

  // Optionally move to home position (0 in URDF = 3.14 in LEAP coords)
  if (move_to_home_on_activate_)
  {
    RCLCPP_INFO(rclcpp::get_logger("LeapHandHardware"), "Moving to home position...");
    return_to_home();
  }

  RCLCPP_INFO(rclcpp::get_logger("LeapHandHardware"), "LEAP Hand activated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LeapHandHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("LeapHandHardware"), "Deactivating LEAP Hand hardware...");
  
  disconnect_serial();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type LeapHandHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (is_connected_)
  {
    // Read actual joint states from hardware
    if (!read_joint_states(hw_states_position_, hw_states_velocity_, hw_states_effort_))
    {
      RCLCPP_WARN(rclcpp::get_logger("LeapHandHardware"),
                  "Failed to read LEAP Hand joint states");
    }
  }
  else
  {
    // Simulation mode - mirror commands to states
    hw_states_position_ = hw_commands_position_;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LeapHandHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (is_connected_)
  {
    // Send position commands to hardware
    if (!send_position_command(hw_commands_position_))
    {
      RCLCPP_WARN(rclcpp::get_logger("LeapHandHardware"),
                  "Failed to write LEAP Hand commands");
    }
  }

  return hardware_interface::return_type::OK;
}

bool LeapHandHardware::connect_serial()
{
  RCLCPP_INFO(rclcpp::get_logger("LeapHandHardware"), 
              "Attempting to connect to LEAP Hand on %s at %d baud", 
              serial_port_.c_str(), baudrate_);

  // Open serial port
  if (!port_handler_->openPort())
  {
    RCLCPP_ERROR(rclcpp::get_logger("LeapHandHardware"),
                 "Failed to open port %s", serial_port_.c_str());
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("LeapHandHardware"), "Opened port %s", serial_port_.c_str());

  // Set baudrate
  if (!port_handler_->setBaudRate(baudrate_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("LeapHandHardware"),
                 "Failed to set baudrate to %d", baudrate_);
    port_handler_->closePort();
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("LeapHandHardware"), "Set baudrate to %d", baudrate_);

  // Add all motors to sync read groups
  for (uint8_t motor_id : motor_ids_)
  {
    if (!group_sync_read_pos_->addParam(motor_id))
    {
      RCLCPP_WARN(rclcpp::get_logger("LeapHandHardware"),
                  "Failed to add motor %d to position sync read", motor_id);
    }
    if (!group_sync_read_vel_->addParam(motor_id))
    {
      RCLCPP_WARN(rclcpp::get_logger("LeapHandHardware"),
                  "Failed to add motor %d to velocity sync read", motor_id);
    }
    if (!group_sync_read_cur_->addParam(motor_id))
    {
      RCLCPP_WARN(rclcpp::get_logger("LeapHandHardware"),
                  "Failed to add motor %d to current sync read", motor_id);
    }
  }

  // Enable torque for all motors
  for (uint8_t motor_id : motor_ids_)
  {
    uint8_t dxl_error = 0;
    int dxl_comm_result = packet_handler_->write1ByteTxRx(
      port_handler_.get(), motor_id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    
    if (dxl_comm_result != COMM_SUCCESS)
    {
      RCLCPP_WARN(rclcpp::get_logger("LeapHandHardware"),
                  "Failed to enable torque for motor %d: %s",
                  motor_id, packet_handler_->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      RCLCPP_WARN(rclcpp::get_logger("LeapHandHardware"),
                  "Motor %d error: %s",
                  motor_id, packet_handler_->getRxPacketError(dxl_error));
    }
  }

  is_connected_ = true;
  RCLCPP_INFO(rclcpp::get_logger("LeapHandHardware"), 
              "Successfully connected to LEAP Hand with %zu motors", motor_ids_.size());
  return true;
}

void LeapHandHardware::disconnect_serial()
{
  if (!is_connected_)
  {
    return;
  }

  // Disable torque for all motors
  for (uint8_t motor_id : motor_ids_)
  {
    uint8_t dxl_error = 0;
    packet_handler_->write1ByteTxRx(
      port_handler_.get(), motor_id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  }

  // Close port
  port_handler_->closePort();
  is_connected_ = false;
  
  RCLCPP_INFO(rclcpp::get_logger("LeapHandHardware"), "Disconnected from LEAP Hand");
}

bool LeapHandHardware::send_position_command(const std::vector<double> & positions)
{
  if (!is_connected_)
  {
    return false;
  }

  // Clear previous sync write data
  group_sync_write_->clearParam();

  // Convert URDF coordinates to LEAP coordinates, then to Dynamixel ticks
  for (size_t i = 0; i < positions.size() && i < motor_ids_.size(); i++)
  {
    double leap_pos = urdf_to_leap(positions[i]);
    int32_t position_ticks = static_cast<int32_t>(leap_pos / POS_SCALE);
    
    // Add position goal to sync write
    uint8_t param_goal_position[4];
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(position_ticks));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(position_ticks));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(position_ticks));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(position_ticks));
    
    if (!group_sync_write_->addParam(motor_ids_[i], param_goal_position))
    {
      RCLCPP_WARN(rclcpp::get_logger("LeapHandHardware"),
                  "Failed to add goal position for motor %d", motor_ids_[i]);
      return false;
    }
  }

  // Transmit sync write packet
  int dxl_comm_result = group_sync_write_->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    RCLCPP_WARN(rclcpp::get_logger("LeapHandHardware"),
                "Sync write failed: %s", packet_handler_->getTxRxResult(dxl_comm_result));
    return false;
  }

  return true;
}

bool LeapHandHardware::read_joint_states(std::vector<double> & positions,
                                         std::vector<double> & velocities,
                                         std::vector<double> & efforts)
{
  if (!is_connected_)
  {
    return false;
  }

  // Read positions
  int dxl_comm_result = group_sync_read_pos_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    RCLCPP_WARN(rclcpp::get_logger("LeapHandHardware"),
                "Position sync read failed: %s", 
                packet_handler_->getTxRxResult(dxl_comm_result));
    return false;
  }

  // Read velocities
  dxl_comm_result = group_sync_read_vel_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    RCLCPP_WARN(rclcpp::get_logger("LeapHandHardware"),
                "Velocity sync read failed: %s",
                packet_handler_->getTxRxResult(dxl_comm_result));
  }

  // Read currents (effort)
  dxl_comm_result = group_sync_read_cur_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    RCLCPP_WARN(rclcpp::get_logger("LeapHandHardware"),
                "Current sync read failed: %s",
                packet_handler_->getTxRxResult(dxl_comm_result));
  }

  // Extract data for each motor
  for (size_t i = 0; i < motor_ids_.size() && i < positions.size(); i++)
  {
    uint8_t motor_id = motor_ids_[i];
    
    // Get position
    if (group_sync_read_pos_->isAvailable(motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION))
    {
      int32_t position_ticks = group_sync_read_pos_->getData(
        motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
      double leap_pos = static_cast<double>(position_ticks) * POS_SCALE;
      positions[i] = leap_to_urdf(leap_pos);
    }
    
    // Get velocity
    if (group_sync_read_vel_->isAvailable(motor_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY))
    {
      int32_t velocity_ticks = group_sync_read_vel_->getData(
        motor_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
      velocities[i] = static_cast<double>(velocity_ticks) * VEL_SCALE;
    }
    
    // Get current (effort)
    if (group_sync_read_cur_->isAvailable(motor_id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT))
    {
      int16_t current_ticks = group_sync_read_cur_->getData(
        motor_id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);
      efforts[i] = static_cast<double>(current_ticks) * CUR_SCALE / 1000.0;  // Convert mA to A
    }
  }

  return true;
}

void LeapHandHardware::return_to_home()
{
  if (!is_connected_)
  {
    RCLCPP_WARN(rclcpp::get_logger("LeapHandHardware"), 
                "Cannot move to home - not connected to hardware");
    return;
  }

  RCLCPP_INFO(rclcpp::get_logger("LeapHandHardware"), 
              "Moving LEAP Hand to home position (0.0 rad in URDF = 3.14 rad in LEAP)...");

  // Home position is 0.0 in URDF coordinates
  std::vector<double> home_position(motor_ids_.size(), 0.0);
  
  // Send home position command
  if (!send_position_command(home_position))
  {
    RCLCPP_ERROR(rclcpp::get_logger("LeapHandHardware"), 
                 "Failed to send home position command");
    return;
  }

  // Wait for movement to complete (adjust time as needed)
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  
  // Read back position to verify
  if (read_joint_states(hw_states_position_, hw_states_velocity_, hw_states_effort_))
  {
    hw_commands_position_ = hw_states_position_;
    RCLCPP_INFO(rclcpp::get_logger("LeapHandHardware"), 
                "Reached home position (first joint: %.3f rad)", hw_states_position_[0]);
  }
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  openarm_hardware::LeapHandHardware, hardware_interface::SystemInterface)
