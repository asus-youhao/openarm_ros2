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

#pragma once

// ============================================================================
// OpenArm V10 LPF-Enhanced Hardware Interface
// ============================================================================
//
// This class extends OpenArm_v10HW to add a Low-Pass Filter (LPF) on the
// COMMAND side.  The state-side LPF already exists in the base class
// (arm_state_filter_, o6_state_filter_).
//
// Data-flow overview
// ------------------
//
//  [Command path]
//  JointTrajectoryController
//    ↓  write() @ controller_manager rate
//  pos_commands_[i]          ← CommandInterface pointer (in base class)
//    ↓  [NEW] arm_cmd_filter_.update()   ← Command LPF (this class)
//  arm_pos_cmd_buffer_[i]    ← thread-safe buffer
//    ↓  arm_control_loop() @ 500 Hz
//  MIT control → CAN-FD → Motor
//
//  [State path – unchanged from base class]
//  Motor CAN-FD
//    ↓  state_read_loop() @ 200 Hz
//  raw pos/vel/tau
//    ↓  arm_state_filter_.update()  ← State LPF (already in base class)
//  arm_pos_state_buffer_[i]
//    ↓  read()
//  pos_states_[i]            ← StateInterface pointer
//    ↓
//  JointTrajectoryController
//
// Configuration (add to your ros2_control URDF hardware parameters):
//   <param name="cmd_filter_cutoff_hz">10.0</param>
//
// Default cutoff frequency: 10 Hz for commands (20 Hz is also reasonable).
// State filter cutoff is inherited from base class: STATE_FILTER_CUTOFF_HZ = 50 Hz.
//
// Plugin name: openarm_hardware/OpenArm_v10LPF_HW
// ============================================================================

#include "openarm_hardware/v10_simple_hardware.hpp"

namespace openarm_hardware {

/**
 * @brief LPF-enhanced OpenArm V10 Hardware Interface.
 *
 * Inherits all motor control, thread management, gravity/friction compensation,
 * and O6 Hand functionality from OpenArm_v10HW.
 *
 * Only three methods are overridden:
 *   - on_init()     : adds cmd filter initialisation after base on_init()
 *   - on_activate() : seeds cmd filter with current arm position after base
 *                     on_activate() to avoid an initial position spike
 *   - write()       : applies LPF to pos_commands_ before writing to the
 *                     thread-safe command buffers read by arm_control_loop()
 */
class OpenArm_v10LPF_HW : public OpenArm_v10HW {
 public:
  OpenArm_v10LPF_HW() = default;

  // ---- Overridden lifecycle hooks ----

  /**
   * Calls OpenArm_v10HW::on_init() then initialises the command LPF filters.
   * Reads the optional hardware parameter "cmd_filter_cutoff_hz" (default 10 Hz).
   */
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  /**
   * Calls OpenArm_v10HW::on_activate() (return-to-zero, starts threads, etc.)
   * then seeds the command filters with the current arm position so the first
   * filtered command equals the actual arm position instead of 0.
   */
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  /**
   * Applies LPF to position commands before writing them to the thread-safe
   * cmd buffers consumed by arm_control_loop() / o6_control_loop().
   *
   * Velocity and torque commands are passed through unfiltered to preserve
   * feed-forward dynamics accuracy.
   */
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  // ---- Command LPF filters ----
  // (State LPF filters arm_state_filter_ / o6_state_filter_
  //  are inherited from OpenArm_v10HW and applied in state_read_loop().)

  LowPassFilter arm_cmd_filter_;   ///< LPF for arm (+ gripper) position commands
  LowPassFilter o6_cmd_filter_;    ///< LPF for O6 Hand (6 active joints) position commands

  /// Cutoff frequency for command LPF (Hz). Loaded from hardware parameter
  /// "cmd_filter_cutoff_hz". Typical values: 10–20 Hz.
  double cmd_filter_cutoff_hz_ = 10.0;
};

}  // namespace openarm_hardware
