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

#ifndef OPENARM_HARDWARE__O6_HAND_HARDWARE_HPP_
#define OPENARM_HARDWARE__O6_HAND_HARDWARE_HPP_

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

#include "openarm_hardware/visibility_control.h"

// Forward declare LinkerHandApi to avoid including the header here
class LinkerHandApi;

namespace openarm_hardware
{

class O6HandHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(O6HandHardware)

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
  // O6 hand has 6 degrees of freedom
  static constexpr size_t O6_HAND_DOF = 6;

  // Hardware parameters
  std::string can_interface_;     // CAN interface (can0, can1, can2, can3)
  std::string hand_type_;         // "left" or "right"
  std::string hand_prefix_;       // Joint name prefix (e.g., "right_")
  bool is_connected_;
  bool move_to_home_on_activate_;
  
  // Speed and torque parameters (0-250)
  uint8_t init_speed_;            // Initial speed (default: 150)
  uint8_t init_torque_;           // Initial torque (default: 150)

  // LinkerHandApi SDK interface
  std::unique_ptr<LinkerHandApi> hand_api_;

  // Joint states and commands
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_states_effort_;
  std::vector<double> hw_commands_position_;
  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_commands_effort_;

  // Joint names
  std::vector<std::string> joint_names_;

  // Internal methods
  bool connect_hand();
  void disconnect_hand();
  bool send_position_command(const std::vector<double> & positions);
  bool read_joint_states(std::vector<double> & positions, 
                         std::vector<double> & velocities,
                         std::vector<double> & efforts);
  void return_to_home();
  
  // Convert between range (0-250) and radians
  std::vector<double> range_to_radians(const std::vector<uint8_t> & range_values);
  std::vector<uint8_t> radians_to_range(const std::vector<double> & radians);
};

}  // namespace openarm_hardware

#endif  // OPENARM_HARDWARE__O6_HAND_HARDWARE_HPP_
