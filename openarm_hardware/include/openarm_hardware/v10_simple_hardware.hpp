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

#include <atomic>
#include <chrono>
#include <fstream>
#include <memory>
#include <mutex>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <string>
#include <thread>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "openarm_hardware/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

// KDL headers for dynamics computation
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <yaml-cpp/yaml.h>

namespace openarm_hardware {

/**
 * @brief Simplified OpenArm V10 Hardware Interface
 *
 * This is a simplified version that uses the OpenArm CAN API directly,
 * following the pattern from full_arm.cpp example. Much simpler than
 * the original implementation.
 */
class OpenArm_v10HW : public hardware_interface::SystemInterface {
 public:
  OpenArm_v10HW();

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  // V10 default configuration
  static constexpr size_t ARM_DOF = 7;
  static constexpr size_t LEAP_HAND_DOF = 16;
  static constexpr bool ENABLE_GRIPPER = true;

  // ========== CONFIGURABLE CONTROL RATES ==========
  // These can be tuned based on system performance requirements
  // Write rate: Motor command frequency (Hz)
  static constexpr double CONTROL_WRITE_RATE_HZ = 500.0;  // 500Hz write-only (CAN-FD/Serial TX)
  // Read rate: Decoupled state read thread frequency (Hz) - separate from write loop
  static constexpr double CONTROL_READ_RATE_HZ = 200.0;   // 200Hz decoupled state read thread

  // Low-pass filter cutoff frequency for state smoothing (Hz)
  static constexpr double STATE_FILTER_CUTOFF_HZ = 50.0;  // Smooth states for VLA feedback
  
  // Health monitoring thresholds
  static constexpr double MAX_COMM_LATENCY_MS = 5.0;      // Max allowed communication latency
  static constexpr size_t MAX_CONSECUTIVE_FAILURES = 10;  // Max failures before error

  // Default motor configuration for V10
  const std::vector<openarm::damiao_motor::MotorType> DEFAULT_MOTOR_TYPES = {
      openarm::damiao_motor::MotorType::DM8009,  // Joint 1
      openarm::damiao_motor::MotorType::DM8009,  // Joint 2
      openarm::damiao_motor::MotorType::DM4340,  // Joint 3
      openarm::damiao_motor::MotorType::DM4340,  // Joint 4
      openarm::damiao_motor::MotorType::DM4310,  // Joint 5
      openarm::damiao_motor::MotorType::DM4310,  // Joint 6
      openarm::damiao_motor::MotorType::DM4310   // Joint 7
  };

  const std::vector<uint32_t> DEFAULT_SEND_CAN_IDS = {0x01, 0x02, 0x03, 0x04,
                                                      0x05, 0x06, 0x07};
  const std::vector<uint32_t> DEFAULT_RECV_CAN_IDS = {0x11, 0x12, 0x13, 0x14,
                                                      0x15, 0x16, 0x17};

  const openarm::damiao_motor::MotorType DEFAULT_GRIPPER_MOTOR_TYPE =
      openarm::damiao_motor::MotorType::DM4310;
  const uint32_t DEFAULT_GRIPPER_SEND_CAN_ID = 0x08;
  const uint32_t DEFAULT_GRIPPER_RECV_CAN_ID = 0x18;

  // Control gains and compensation parameters
  // Will be loaded from parameters.yaml, these are just fallback defaults
  std::vector<double> kp_ = {50.0, 50.0, 50.0, 60.0, 24.0, 31.0, 10.0};
  std::vector<double> kd_ = {2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5};

  // Friction compensation parameters (LuGre model)
  // tau_friction = Fc * tanh(k * dq) + Fv * dq + Fo
  std::vector<double> Fc_ = {0.306, 0.306, 0.40, 0.166, 0.050, 0.093, 0.172};  // Coulomb friction
  std::vector<double> k_  = {28.417, 28.417, 29.065, 130.038, 151.771, 242.287, 7.888};  // Stiffness
  std::vector<double> Fv_ = {0.063, 0.063, 0.604, 0.813, 0.029, 0.072, 0.084};  // Viscous friction
  std::vector<double> Fo_ = {0.088, 0.088, 0.008, -0.058, 0.005, 0.009, -0.059};  // Offset

  const double GRIPPER_JOINT_0_POSITION = 0.044;
  const double GRIPPER_JOINT_1_POSITION = 0.0;
  const double GRIPPER_MOTOR_0_RADIANS = 0.0;
  const double GRIPPER_MOTOR_1_RADIANS = -1.0472;
  const double GRIPPER_KP = 5.0;
  const double GRIPPER_KD = 0.1;

  // Configuration
  std::string can_interface_;
  std::string arm_prefix_;
  std::string ee_type_;
  bool hand_;
  bool can_fd_;
  bool has_leap_hand_;
  bool enable_frequency_diagnostics_;  // Enable performance monitoring

  // OpenArm instance
  std::unique_ptr<openarm::can::socket::OpenArm> openarm_;

  // Generated joint names for this arm instance
  std::vector<std::string> joint_names_;

  // ROS2 control state and command vectors
  std::vector<double> pos_commands_;
  std::vector<double> vel_commands_;
  std::vector<double> tau_commands_;
  std::vector<double> pos_states_;
  std::vector<double> vel_states_;
  std::vector<double> tau_states_;

  // High-frequency control thread (500Hz for arm)
  std::thread arm_control_thread_;
  std::atomic<bool> arm_thread_running_;
  std::mutex arm_command_mutex_;
  std::mutex arm_state_mutex_;
  
  // Command buffers (thread-safe copy)
  std::vector<double> arm_pos_cmd_buffer_;
  std::vector<double> arm_vel_cmd_buffer_;
  std::vector<double> arm_tau_cmd_buffer_;
  
  // State buffers (thread-safe copy)
  std::vector<double> arm_pos_state_buffer_;
  std::vector<double> arm_vel_state_buffer_;
  std::vector<double> arm_tau_state_buffer_;
  
  // LEAP Hand control thread (now 500Hz for synchronized execution)
  std::thread leap_control_thread_;
  std::atomic<bool> leap_thread_running_;
  std::mutex leap_command_mutex_;
  std::mutex leap_state_mutex_;
  std::mutex serial_mutex_;  // Protect RS-485 serial port access (half-duplex)
  
  // LEAP Hand command/state buffers
  std::vector<double> leap_pos_cmd_buffer_;
  std::vector<double> leap_pos_state_buffer_;
  
  // ========== DECOUPLED STATE READ THREAD ==========
  // Reads CAN arm + LEAP serial states at CONTROL_READ_RATE_HZ, applies LPF.
  // Decoupled from write loops so RS-485 read latency does not stall CAN commands.
  std::thread state_read_thread_;
  std::atomic<bool> state_read_thread_running_;
  
  // Debug CSV logging (one file per arm instance for arm, one for LEAP Hand)
  std::ofstream debug_csv_;
  bool csv_initialized_;
  size_t csv_sample_count_;
  
  std::ofstream leap_debug_csv_;
  bool leap_csv_initialized_;
  size_t leap_csv_sample_count_;

  // Note: State reading is handled exclusively by state_read_loop().
  // arm_control_loop() and leap_control_loop() are pure write-only threads.
  
  // ========== HEALTH MONITORING ==========
  struct HealthStatus {
    std::atomic<size_t> consecutive_read_failures{0};
    std::atomic<size_t> consecutive_write_failures{0};
    std::atomic<double> last_read_latency_ms{0.0};
    std::atomic<double> last_write_latency_ms{0.0};
    std::atomic<bool> arm_healthy{true};
    std::atomic<bool> leap_healthy{true};
    std::atomic<uint64_t> last_successful_read_time{0};
    std::atomic<uint64_t> last_successful_write_time{0};
    std::atomic<size_t> total_read_operations{0};
    std::atomic<size_t> total_write_operations{0};
    std::atomic<size_t> total_errors{0};
    std::atomic<double> average_read_latency_ms{0.0};
    std::atomic<double> average_write_latency_ms{0.0};
    std::atomic<bool> system_healthy{true};
    std::atomic<bool> emergency_stop_triggered{false};
  } health_status_;

  // Health monitoring thread
  std::thread health_monitor_thread_;
  std::atomic<bool> health_monitor_running_;
  std::mutex health_mutex_;
  
  // ========== LOW-PASS FILTER FOR STATE SMOOTHING ==========
  struct LowPassFilter {
    double alpha;          // Filter coefficient
    std::vector<double> filtered_values;
    bool initialized = false;
    
    void init(size_t size, double cutoff_hz, double sample_hz) {
      // Calculate alpha from cutoff frequency
      double rc = 1.0 / (2.0 * M_PI * cutoff_hz);
      double dt = 1.0 / sample_hz;
      alpha = dt / (rc + dt);
      filtered_values.resize(size, 0.0);
      initialized = true;
    }
    
    void update(const std::vector<double>& new_values) {
      if (!initialized || new_values.size() != filtered_values.size()) return;
      for (size_t i = 0; i < filtered_values.size(); ++i) {
        filtered_values[i] = filtered_values[i] + alpha * (new_values[i] - filtered_values[i]);
      }
    }
    
    const std::vector<double>& get() const { return filtered_values; }
  };
  
  LowPassFilter arm_state_filter_;
  LowPassFilter leap_state_filter_;

  // Helper methods
  void return_to_zero();
  bool parse_config(const hardware_interface::HardwareInfo& info);
  void generate_joint_names();
  
  // Thread control loops
  void arm_control_loop();        // Write-only loop @ 500Hz (CAN-FD MIT commands)
  void leap_control_loop();       // Write-only loop @ 500Hz (LEAP Hand serial TX)
  void state_read_loop();         // Read-only loop @ CONTROL_READ_RATE_HZ (CAN + serial + LPF)
  
  // Health monitoring methods
  void check_health();
  void report_health_status();
  bool is_healthy() const;
  void health_monitor_loop();

  // Gripper mapping functions
  double joint_to_motor_radians(double joint_value);
  double motor_radians_to_joint(double motor_radians);

  // LEAP Hand Dynamixel support
  std::string leap_serial_port_;
  int leap_baudrate_;
  std::vector<uint8_t> leap_motor_ids_;
  bool leap_connected_;
  
  std::shared_ptr<dynamixel::PortHandler> leap_port_handler_;
  std::shared_ptr<dynamixel::PacketHandler> leap_packet_handler_;
  std::shared_ptr<dynamixel::GroupSyncWrite> leap_group_sync_write_;
  std::shared_ptr<dynamixel::GroupSyncRead> leap_group_sync_read_pos_;
  
  // Dynamixel Protocol 2.0 addresses (XH series)
  static constexpr uint8_t LEAP_ADDR_TORQUE_ENABLE = 64;
  static constexpr uint8_t LEAP_ADDR_GOAL_POSITION = 116;
  static constexpr uint8_t LEAP_ADDR_PRESENT_POSITION = 132;
  static constexpr uint8_t LEAP_LEN_GOAL_POSITION = 4;
  static constexpr uint8_t LEAP_LEN_PRESENT_POSITION = 4;
  static constexpr float LEAP_PROTOCOL_VERSION = 2.0;
  static constexpr double LEAP_POS_SCALE = 2.0 * M_PI / 4096.0;  // ticks to radians
  
  bool connect_leap_hand();
  void disconnect_leap_hand();
  bool send_leap_hand_command(const std::vector<double>& positions, size_t start_idx);
  bool read_leap_hand_states(std::vector<double>& positions, size_t start_idx);
  
  // LEAP coordinate conversion (URDF 0=home, LEAP 3.14=home)
  inline double urdf_to_leap(double urdf_pos) { return urdf_pos + M_PI; }
  inline double leap_to_urdf(double leap_pos) { return leap_pos - M_PI; }

  // Gravity compensation using KDL
  std::unique_ptr<KDL::ChainDynParam> kdl_solver_;
  KDL::Chain kdl_chain_;
  KDL::JntArray gravity_torques_;
  bool use_gravity_compensation_;
  bool use_friction_compensation_;
  std::string urdf_string_;
  
  bool init_kdl_dynamics(const std::string& urdf_content);
  void compute_gravity_compensation(std::vector<double>& gravity_torques);
  void compute_friction_compensation(std::vector<double>& friction_torques);

  // Function to load parameters from YAML file
  void loadParametersFromYAML(const std::string& yaml_file) {
    YAML::Node config = YAML::LoadFile(yaml_file);

    if (config["kp"]) {
        kp_ = config["kp"].as<std::vector<double>>();
    }
    if (config["kd"]) {
        kd_ = config["kd"].as<std::vector<double>>();
    }
    if (config["Fc"]) {
        Fc_ = config["Fc"].as<std::vector<double>>();
    }
    if (config["k"]) {
        k_ = config["k"].as<std::vector<double>>();
    }
    if (config["Fv"]) {
        Fv_ = config["Fv"].as<std::vector<double>>();
    }
    if (config["Fo"]) {
        Fo_ = config["Fo"].as<std::vector<double>>();
    }
}

};

}  // namespace openarm_hardware
