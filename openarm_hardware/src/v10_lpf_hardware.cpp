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

// ============================================================================
// OpenArm V10 LPF-Enhanced Hardware Interface — Implementation
// ============================================================================
//
// Only the THREE methods that differ from OpenArm_v10HW are implemented here:
//   on_init()     — chains to base, then initialises command LPF filters
//   on_activate() — chains to base, then seeds cmd filters with arm position
//   write()       — applies command LPF before updating thread-safe buffers
//
// All other methods (on_configure, on_deactivate, read, export_*_interfaces,
// arm_control_loop, state_read_loop, etc.) are inherited from OpenArm_v10HW
// and compiled from v10_simple_hardware.cpp.
// ============================================================================

#include "openarm_hardware/v10_lpf_hardware.hpp"

#include <string>
#include <vector>

#include "rclcpp/logging.hpp"

namespace openarm_hardware {

// ============================================================================
// on_init  — base init + command LPF filter initialisation
// ============================================================================
hardware_interface::CallbackReturn OpenArm_v10LPF_HW::on_init(
    const hardware_interface::HardwareInfo& info) {

  // 1. Run base-class on_init (parses config, generates joint names, inits
  //    CAN/O6, allocates state/command vectors, inits state LPF filters).
  auto result = OpenArm_v10HW::on_init(info);
  if (result != hardware_interface::CallbackReturn::SUCCESS) {
    return result;
  }

  // 2. Read optional command filter cutoff frequency from hardware parameters.
  //    Add to your URDF:  <param name="cmd_filter_cutoff_hz">10.0</param>
  auto it = info.hardware_parameters.find("cmd_filter_cutoff_hz");
  if (it != info.hardware_parameters.end()) {
    try {
      cmd_filter_cutoff_hz_ = std::stod(it->second);
    } catch (const std::exception& e) {
      RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10LPF_HW"),
                  "Invalid cmd_filter_cutoff_hz '%s', using default %.1f Hz: %s",
                  it->second.c_str(), cmd_filter_cutoff_hz_, e.what());
    }
  }

  // 3. Initialise ARM command LPF.
  //    Size = ARM_DOF (7) + 1 gripper slot when a CAN gripper is configured.
  //    Sample rate = CONTROL_WRITE_RATE_HZ (500 Hz) — the rate at which
  //    arm_control_loop() consumes arm_pos_cmd_buffer_.
  //
  //    alpha = dt / (rc + dt),  rc = 1 / (2π * cutoff_hz)
  //    e.g. cutoff=10 Hz, sample=500 Hz  →  alpha ≈ 0.111  (smooth)
  //         cutoff=20 Hz, sample=500 Hz  →  alpha ≈ 0.200  (less smooth)
  {
    const size_t arm_size = ARM_DOF + (hand_ && !has_o6_hand_ ? 1 : 0);
    arm_cmd_filter_.init(arm_size, cmd_filter_cutoff_hz_, CONTROL_WRITE_RATE_HZ);
  }

  // 4. Initialise O6 Hand command LPF (6 active joints only — passive joints
  //    are mechanically coupled and receive no direct commands).
  if (has_o6_hand_) {
    o6_cmd_filter_.init(6, cmd_filter_cutoff_hz_, 60.0);  // O6 runs at ~60 Hz
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10LPF_HW"),
              "=== Command LPF initialised ===\n"
              "  cmd_filter_cutoff_hz : %.1f Hz\n"
              "  state_filter_cutoff  : %.1f Hz  (inherited, state_read_loop)\n"
              "  arm cmd filter size  : %zu joints\n"
              "  O6  cmd filter       : %s",
              cmd_filter_cutoff_hz_,
              STATE_FILTER_CUTOFF_HZ,
              ARM_DOF + (hand_ && !has_o6_hand_ ? 1 : 0),
              has_o6_hand_   ? "enabled (6 active DOF)" : "disabled");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_activate  — base activate + seed cmd filters with current arm position
// ============================================================================
hardware_interface::CallbackReturn OpenArm_v10LPF_HW::on_activate(
    const rclcpp_lifecycle::State& previous_state) {

  // 1. Run base-class on_activate:
  //    - sets motor callback mode
  //    - enables all motors
  //    - connects O6 hand
  //    - calls return_to_zero() (arm moves to zero position)
  //    - starts arm_control_thread_, state_read_thread_, health_monitor_thread_
  auto result = OpenArm_v10HW::on_activate(previous_state);
  if (result != hardware_interface::CallbackReturn::SUCCESS) {
    return result;
  }

  // 2. Seed command filter with the actual current arm position.
  //    By the time base on_activate() returns, state_read_thread_ has started
  //    and arm_pos_state_buffer_ is populated.
  //    Seeding prevents the filter from ramping up from 0.0 to the first
  //    commanded position, which would cause an unintended slow motion.
  {
    std::lock_guard<std::mutex> lock(arm_state_mutex_);
    if (!arm_pos_state_buffer_.empty()) {
      // Only seed the arm portion (ignore gripper if beyond ARM_DOF)
      const size_t arm_size = arm_cmd_filter_.filtered_values.size();
      for (size_t i = 0; i < arm_size && i < arm_pos_state_buffer_.size(); ++i) {
        arm_cmd_filter_.filtered_values[i] = arm_pos_state_buffer_[i];
      }
    }
  }

  // The O6 command filter is left at 0.0 because the base class already
  // sends an open-position command to the hand during activation.

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10LPF_HW"),
              "Command LPF seeded with current arm position (avoids initial ramp)");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// write  — apply command LPF then forward to thread-safe buffers
// ============================================================================
//
// Called by controller_manager at the control loop rate (~1 kHz by default).
// The base-class arm_control_loop() runs at 500 Hz and reads
// arm_pos_cmd_buffer_ under arm_command_mutex_.  This override applies the LPF
// to pos_commands_ BEFORE writing to that buffer.
//
// Why only position commands are filtered:
//   - Velocity (vel_commands_) and torque (tau_commands_) feed-forward terms
//     are already small correction values; filtering them would introduce
//     phase lag that degrades dynamics compensation.
//   - Position is the primary trajectory tracking reference, and smoothing it
//     prevents jerk caused by discrete waypoint steps from the JTC.
// ============================================================================
hardware_interface::return_type OpenArm_v10LPF_HW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {

  // =========================================================================
  // [1] ARM (+ optional CAN gripper) — Command LPF on position
  // =========================================================================
  {
    std::lock_guard<std::mutex> lock(arm_command_mutex_);
    const size_t arm_size = ARM_DOF + (hand_ && !has_o6_hand_ ? 1 : 0);

    // Build raw command vector, applying joint-direction sign correction.
    // Vel/tau are written directly (no LPF).
    std::vector<double> raw_pos_cmd(arm_size);
    for (size_t i = 0; i < arm_size; ++i) {
      if (i < ARM_DOF && joint_direction_[i] < 0) {
        raw_pos_cmd[i]         = -pos_commands_[i];
        arm_vel_cmd_buffer_[i] = -vel_commands_[i];
        arm_tau_cmd_buffer_[i] = -tau_commands_[i];
      } else {
        raw_pos_cmd[i]         = pos_commands_[i];
        arm_vel_cmd_buffer_[i] = vel_commands_[i];
        arm_tau_cmd_buffer_[i] = tau_commands_[i];
      }
    }

    // Apply command LPF to position.
    // filtered[i] = filtered[i] + alpha * (raw[i] - filtered[i])
    arm_cmd_filter_.update(raw_pos_cmd);
    const std::vector<double>& filtered_pos = arm_cmd_filter_.get();
    for (size_t i = 0; i < arm_size; ++i) {
      arm_pos_cmd_buffer_[i] = filtered_pos[i];
    }
  }

  // =========================================================================
  // [2] O6 Hand — Command LPF on position (6 active joints only)
  //     Passive/coupled joints (indices 6-10) receive no direct commands.
  // =========================================================================
  if (has_o6_hand_ && o6_connected_) {
    std::lock_guard<std::mutex> lock(o6_command_mutex_);
    const size_t o6_start =
        ARM_DOF +
        (hand_ && !has_o6_hand_ ? 1 : 0);

    std::vector<double> raw_o6_cmd(6);
    for (size_t i = 0; i < 6; ++i) {
      raw_o6_cmd[i] = pos_commands_[o6_start + i];
    }

    o6_cmd_filter_.update(raw_o6_cmd);
    const std::vector<double>& filtered_o6 = o6_cmd_filter_.get();
    for (size_t i = 0; i < 6; ++i) {
      o6_pos_cmd_buffer_[i] = filtered_o6[i];
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace openarm_hardware

// Register as a ros2_control hardware plugin.
// Plugin name used in URDF: openarm_hardware/OpenArm_v10LPF_HW
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_v10LPF_HW,
                       hardware_interface::SystemInterface)
