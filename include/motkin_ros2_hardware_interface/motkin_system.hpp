// Copyright 2026 LAAS-CNRS
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

#ifndef MOTKIN_ROS2_HARDWARE_INTERFACE__MOTKIN_SYSTEM_HPP_
#define MOTKIN_ROS2_HARDWARE_INTERFACE__MOTKIN_SYSTEM_HPP_

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "motkin_ros2_hardware_interface/serial_port.hpp"
#include "motkin_ros2_hardware_interface/usb_protocol.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace motkin_ros2_hardware_interface {

constexpr const char* kHwIfGainKp = "gain_kp";
constexpr const char* kHwIfGainKd = "gain_kd";

/// Per-joint command/state storage. `effort` carries the board's current
/// feedforward / measured current in Amps, `Kp`/`Kd` the PD gains in
/// A/rad and A/(rad/s) as defined by USB_PROTOCOL.md.
struct JointValues {
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  double Kp{0.0};
  double Kd{0.0};
};

/// Two joints are expected: index 0 drives motor M0, index 1 drives motor M1,
/// in the order the joints appear under the <ros2_control> tag.
constexpr std::size_t kNumMotors = 2;

class SystemMotkinHardware : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SystemMotkinHardware)

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& info)
      override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  enum class ControlMode {
    NO_VALID_MODE,
    POSITION,
    VELOCITY,
    EFFORT,
    POS_VEL_EFF_GAINS
  };

  // ---- Configuration read from <ros2_control> hardware parameters ----
  std::string serial_device_;
  unsigned int baud_rate_{115200};
  uint16_t watchdog_timeout_ms_{20};

  // ---- ros2_control bookkeeping, one slot per joint ----
  std::array<JointValues, kNumMotors> hw_commands_;
  std::array<JointValues, kNumMotors> hw_states_;
  std::array<ControlMode, kNumMotors> control_mode_{};
  std::array<std::string, kNumMotors> joint_names_;

  static const std::set<std::string>& expected_interfaces();

  // ---- Serial transport + background reader thread ----
  SerialPort serial_port_;
  std::thread rx_thread_;
  std::atomic<bool> rx_running_{false};

  std::mutex state_mutex_;
  std::condition_variable state_cv_;
  StatePacket latest_state_{};
  bool have_state_{false};
  uint16_t last_state_sequence_{0};

  uint32_t command_index_{1};
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};

  void rx_loop();
  uint32_t next_command_index();
  bool send_command(uint8_t flags, uint16_t timeout_ms, const MotorCommand& m0,
                    const MotorCommand& m1, uint32_t index);
  bool wait_for_command_echo(uint32_t index, double timeout_s);
};

}  // namespace motkin_ros2_hardware_interface

#endif  // MOTKIN_ROS2_HARDWARE_INTERFACE__MOTKIN_SYSTEM_HPP_
