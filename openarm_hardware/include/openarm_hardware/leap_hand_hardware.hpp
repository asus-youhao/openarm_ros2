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

#ifndef OPENARM_HARDWARE__LEAP_HAND_HARDWARE_HPP_
#define OPENARM_HARDWARE__LEAP_HAND_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

#include "openarm_hardware/visibility_control.h"

namespace openarm_hardware
{

class LeapHandHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LeapHandHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr size_t LEAP_HAND_DOF = 16;
  
  // LEAP Hand coordinate offset (3.14 rad = home pose in LEAP convention)
  static constexpr double LEAP_HOME_OFFSET = 3.14159265359;

  // Dynamixel protocol settings (XH series motors)
  static constexpr float PROTOCOL_VERSION = 2.0;
  static constexpr int ADDR_TORQUE_ENABLE = 64;
  static constexpr int ADDR_GOAL_POSITION = 116;
  static constexpr int ADDR_PRESENT_POSITION = 132;
  static constexpr int ADDR_PRESENT_VELOCITY = 128;
  static constexpr int ADDR_PRESENT_CURRENT = 126;
  static constexpr int LEN_GOAL_POSITION = 4;
  static constexpr int LEN_PRESENT_POSITION = 4;
  static constexpr int LEN_PRESENT_VELOCITY = 4;
  static constexpr int LEN_PRESENT_CURRENT = 2;
  static constexpr double POS_SCALE = 2.0 * 3.141592653589793 / 4096.0;  // radians per tick
  static constexpr double VEL_SCALE = 0.229 * 2.0 * 3.141592653589793 / 60.0;  // rad/s per tick
  static constexpr double CUR_SCALE = 1.34;  // mA per tick

  // LEAP Hand motor IDs (16 motors total)
  std::vector<uint8_t> motor_ids_;

  // Serial port communication
  std::string serial_port_;
  int baudrate_;
  bool is_connected_;
  bool move_to_home_on_activate_;  // Whether to move to home position on activation
  
  // Dynamixel SDK objects
  std::shared_ptr<dynamixel::PortHandler> port_handler_;
  std::shared_ptr<dynamixel::PacketHandler> packet_handler_;
  std::shared_ptr<dynamixel::GroupSyncWrite> group_sync_write_;
  std::shared_ptr<dynamixel::GroupSyncRead> group_sync_read_pos_;
  std::shared_ptr<dynamixel::GroupSyncRead> group_sync_read_vel_;
  std::shared_ptr<dynamixel::GroupSyncRead> group_sync_read_cur_;

  // Joint states
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_states_effort_;
  std::vector<double> hw_commands_position_;
  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_commands_effort_;

  // Joint names
  std::vector<std::string> joint_names_;

  // Internal methods
  bool connect_serial();
  void disconnect_serial();
  bool send_position_command(const std::vector<double> & positions);
  bool read_joint_states(std::vector<double> & positions, 
                         std::vector<double> & velocities,
                         std::vector<double> & efforts);
  void return_to_home();
  
  // Convert between URDF (0 = home) and LEAP (3.14 = home) coordinates
  inline double urdf_to_leap(double urdf_pos) const { return urdf_pos + LEAP_HOME_OFFSET; }
  inline double leap_to_urdf(double leap_pos) const { return leap_pos - LEAP_HOME_OFFSET; }
};

}  // namespace openarm_hardware

#endif  // OPENARM_HARDWARE__LEAP_HAND_HARDWARE_HPP_
