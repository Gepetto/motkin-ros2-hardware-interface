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

#include "motkin_ros2_hardware_interface/pico_dual_drv8316c_system.hpp"

#include <chrono>
#include <cstdlib>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace motkin_ros2_hardware_interface {

namespace {
rclcpp::Logger logger() {
  return rclcpp::get_logger("SystemPicoDualDrv8316CHardware");
}

std::string get_param_or(const hardware_interface::HardwareInfo& info,
                         const std::string& key,
                         const std::string& default_value) {
  auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? default_value : it->second;
}

}  // namespace

const std::set<std::string>&
SystemPicoDualDrv8316CHardware::expected_interfaces() {
  static const std::set<std::string> interfaces{
      hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
      hardware_interface::HW_IF_EFFORT, kHwIfGainKp, kHwIfGainKd};
  return interfaces;
}

hardware_interface::CallbackReturn SystemPicoDualDrv8316CHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kNumMotors) {
    RCLCPP_FATAL(logger(), "Expected exactly %zu joints (M0, M1), got %zu.",
                 kNumMotors, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (std::size_t i = 0; i < kNumMotors; ++i) {
    joint_names_[i] = info_.joints[i].name;
  }

  serial_device_ = get_param_or(info_, "serial_port", "");
  baud_rate_ = static_cast<unsigned int>(std::strtoul(
      get_param_or(info_, "baud_rate", "115200").c_str(), nullptr, 10));
  watchdog_timeout_ms_ = static_cast<uint16_t>(std::strtoul(
      get_param_or(info_, "timeout_ms", "20").c_str(), nullptr, 10));

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SystemPicoDualDrv8316CHardware::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  for (std::size_t i = 0; i < kNumMotors; ++i) {
    const hardware_interface::ComponentInfo& joint = info_.joints[i];

    if (joint.command_interfaces.size() != expected_interfaces().size()) {
      RCLCPP_FATAL(logger(),
                   "Joint '%s' has %zu command interfaces, expected %zu.",
                   joint.name.c_str(), joint.command_interfaces.size(),
                   expected_interfaces().size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    for (const auto& cmd_if : joint.command_interfaces) {
      if (expected_interfaces().find(cmd_if.name) ==
          expected_interfaces().end()) {
        RCLCPP_FATAL(logger(),
                     "Joint '%s' has unexpected command interface '%s'.",
                     joint.name.c_str(), cmd_if.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    if (joint.state_interfaces.size() != expected_interfaces().size()) {
      RCLCPP_FATAL(logger(),
                   "Joint '%s' has %zu state interfaces, expected %zu.",
                   joint.name.c_str(), joint.state_interfaces.size(),
                   expected_interfaces().size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    for (const auto& state_if : joint.state_interfaces) {
      if (expected_interfaces().find(state_if.name) ==
          expected_interfaces().end()) {
        RCLCPP_FATAL(logger(),
                     "Joint '%s' has unexpected state interface '%s'.",
                     joint.name.c_str(), state_if.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    hw_states_[i] = JointValues{};
    hw_commands_[i] = JointValues{};
    control_mode_[i] = ControlMode::NO_VALID_MODE;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SystemPicoDualDrv8316CHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < kNumMotors; ++i) {
    state_interfaces.emplace_back(joint_names_[i],
                                  hardware_interface::HW_IF_POSITION,
                                  &hw_states_[i].position);
    state_interfaces.emplace_back(joint_names_[i],
                                  hardware_interface::HW_IF_VELOCITY,
                                  &hw_states_[i].velocity);
    state_interfaces.emplace_back(joint_names_[i],
                                  hardware_interface::HW_IF_EFFORT,
                                  &hw_states_[i].effort);
    state_interfaces.emplace_back(joint_names_[i], kHwIfGainKp,
                                  &hw_states_[i].Kp);
    state_interfaces.emplace_back(joint_names_[i], kHwIfGainKd,
                                  &hw_states_[i].Kd);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SystemPicoDualDrv8316CHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < kNumMotors; ++i) {
    command_interfaces.emplace_back(joint_names_[i],
                                    hardware_interface::HW_IF_POSITION,
                                    &hw_commands_[i].position);
    command_interfaces.emplace_back(joint_names_[i],
                                    hardware_interface::HW_IF_VELOCITY,
                                    &hw_commands_[i].velocity);
    command_interfaces.emplace_back(joint_names_[i],
                                    hardware_interface::HW_IF_EFFORT,
                                    &hw_commands_[i].effort);
    command_interfaces.emplace_back(joint_names_[i], kHwIfGainKp,
                                    &hw_commands_[i].Kp);
    command_interfaces.emplace_back(joint_names_[i], kHwIfGainKd,
                                    &hw_commands_[i].Kd);
  }
  return command_interfaces;
}

hardware_interface::return_type
SystemPicoDualDrv8316CHardware::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  std::array<ControlMode, kNumMotors> new_modes{};
  new_modes.fill(ControlMode::NO_VALID_MODE);

  for (const auto& key : start_interfaces) {
    for (std::size_t i = 0; i < kNumMotors; ++i) {
      const std::string& name = joint_names_[i];
      if (key == name + "/" + hardware_interface::HW_IF_POSITION) {
        new_modes[i] = ControlMode::POSITION;
      } else if (key == name + "/" + hardware_interface::HW_IF_VELOCITY) {
        new_modes[i] = ControlMode::VELOCITY;
      } else if (key == name + "/" + hardware_interface::HW_IF_EFFORT) {
        new_modes[i] = ControlMode::EFFORT;
      } else if (key == name + "/" + kHwIfGainKp ||
                 key == name + "/" + kHwIfGainKd) {
        new_modes[i] = ControlMode::POS_VEL_EFF_GAINS;
      }
    }
  }

  for (const auto& key : stop_interfaces) {
    for (std::size_t i = 0; i < kNumMotors; ++i) {
      if (key.rfind(joint_names_[i] + "/", 0) == 0) {
        hw_commands_[i].velocity = 0.0;
        hw_commands_[i].effort = 0.0;
        control_mode_[i] = ControlMode::NO_VALID_MODE;
      }
    }
  }

  for (std::size_t i = 0; i < kNumMotors; ++i) {
    if (control_mode_[i] == ControlMode::NO_VALID_MODE &&
        new_modes[i] == ControlMode::NO_VALID_MODE) {
      continue;  // Nothing claims this joint yet; leave it disabled, not an
                 // error.
    }
    if (new_modes[i] != ControlMode::NO_VALID_MODE) {
      control_mode_[i] = new_modes[i];
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn SystemPicoDualDrv8316CHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  std::string device = serial_device_;
  if (device.empty()) {
    device = SerialPort::find_default_device();
  }
  if (device.empty()) {
    RCLCPP_FATAL(logger(),
                 "No serial device found. Set the 'serial_port' hardware "
                 "parameter or plug in the board.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    serial_port_.open(device, baud_rate_);
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(logger(), "Could not open '%s': %s", device.c_str(),
                 ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger(), "Opened pico USB link on '%s'.", device.c_str());

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    have_state_ = false;
    last_state_sequence_ = 0;
  }
  command_index_ = 1;

  rx_running_ = true;
  rx_thread_ = std::thread(&SystemPicoDualDrv8316CHardware::rx_loop, this);

  // Recommended startup sequence (USB_PROTOCOL.md): send a zero-gain,
  // zero-timeout command and wait for core1 to echo its command index
  // before enabling the watchdog and real commands.
  for (std::size_t i = 0; i < kNumMotors; ++i) {
    hw_commands_[i] = JointValues{};
    control_mode_[i] = ControlMode::NO_VALID_MODE;
  }
  uint32_t handshake_index = next_command_index();
  if (!send_command(0, 0, MotorCommand{}, MotorCommand{}, handshake_index)) {
    RCLCPP_FATAL(logger(),
                 "Failed to write the initialization command to the board.");
    rx_running_ = false;
    if (rx_thread_.joinable()) {
      rx_thread_.join();
    }
    serial_port_.close();
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!wait_for_command_echo(handshake_index, 2.0)) {
    RCLCPP_FATAL(logger(),
                 "Board did not echo the initialization command within 2s.");
    rx_running_ = false;
    if (rx_thread_.joinable()) {
      rx_thread_.join();
    }
    serial_port_.close();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger(), "Board initialized and ready.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
SystemPicoDualDrv8316CHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (serial_port_.is_open()) {
    // flags = 0 forces zero Iq on both motors regardless of control mode.
    send_command(0, 0, MotorCommand{}, MotorCommand{}, next_command_index());
  }

  rx_running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  serial_port_.close();

  for (std::size_t i = 0; i < kNumMotors; ++i) {
    control_mode_[i] = ControlMode::NO_VALID_MODE;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SystemPicoDualDrv8316CHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  StatePacket state;
  bool has_state;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state = latest_state_;
    has_state = have_state_;
  }
  if (!has_state) {
    return hardware_interface::return_type::OK;
  }

  hw_states_[0].position = state.m0_q;
  hw_states_[0].velocity = state.m0_v;
  hw_states_[0].effort = state.m0_i;
  hw_states_[0].Kp = hw_commands_[0].Kp;
  hw_states_[0].Kd = hw_commands_[0].Kd;

  hw_states_[1].position = state.m1_q;
  hw_states_[1].velocity = state.m1_v;
  hw_states_[1].effort = state.m1_i;
  hw_states_[1].Kp = hw_commands_[1].Kp;
  hw_states_[1].Kd = hw_commands_[1].Kd;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SystemPicoDualDrv8316CHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // The firmware runs a single PD + feedforward law per motor:
  //   iq = iff + kp * (q_target - q) + kd * (v_target - v)
  // ros2_control "modes" here only decide whether a joint's command is
  // considered live; the actual gains/targets always come straight from
  // the claimed command interfaces.
  std::array<MotorCommand, kNumMotors> motor_cmd{};
  uint8_t flags = 0;
  static const std::array<uint8_t, kNumMotors> kReadyFlag{
      {kM0ReadyFlag, kM1ReadyFlag}};

  for (std::size_t i = 0; i < kNumMotors; ++i) {
    if (control_mode_[i] == ControlMode::NO_VALID_MODE) {
      continue;
    }
    flags = static_cast<uint8_t>(flags | kReadyFlag[i]);
    motor_cmd[i].kp = static_cast<float>(hw_commands_[i].Kp);
    motor_cmd[i].kd = static_cast<float>(hw_commands_[i].Kd);
    motor_cmd[i].iff = static_cast<float>(hw_commands_[i].effort);
    motor_cmd[i].q_target = static_cast<float>(hw_commands_[i].position);
    motor_cmd[i].v_target = static_cast<float>(hw_commands_[i].velocity);
  }

  uint32_t index = next_command_index();
  if (!send_command(flags, watchdog_timeout_ms_, motor_cmd[0], motor_cmd[1],
                    index)) {
    RCLCPP_ERROR_THROTTLE(logger(), throttle_clock_, 1000,
                          "Failed to write command to the board.");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

uint32_t SystemPicoDualDrv8316CHardware::next_command_index() {
  command_index_ = (command_index_ + 1) & 0xFFFFFFFFu;
  if (command_index_ == 0) {
    command_index_ = 1;
  }
  return command_index_;
}

bool SystemPicoDualDrv8316CHardware::send_command(uint8_t flags,
                                                  uint16_t timeout_ms,
                                                  const MotorCommand& m0,
                                                  const MotorCommand& m1,
                                                  uint32_t index) {
  CommandPacket packet = encode_command(index, flags, timeout_ms, m0, m1);
  return serial_port_.write(reinterpret_cast<const uint8_t*>(&packet),
                            sizeof(packet));
}

bool SystemPicoDualDrv8316CHardware::wait_for_command_echo(uint32_t index,
                                                           double timeout_s) {
  std::unique_lock<std::mutex> lock(state_mutex_);
  auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(timeout_s));
  return state_cv_.wait_until(lock, deadline, [this, index] {
    return have_state_ && latest_state_.latest_command_index == index;
  });
}

void SystemPicoDualDrv8316CHardware::rx_loop() {
  std::vector<uint8_t> buffer;
  buffer.reserve(4 * sizeof(StatePacket));
  uint8_t chunk[256];

  while (rx_running_) {
    int n = serial_port_.read_available(chunk, sizeof(chunk));
    if (n <= 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(200));
      continue;
    }
    buffer.insert(buffer.end(), chunk, chunk + n);

    while (buffer.size() >= sizeof(StatePacket)) {
      // Resynchronize on the two magic bytes.
      std::size_t magic_at = std::string::npos;
      for (std::size_t i = 0; i + 1 < buffer.size(); ++i) {
        if (buffer[i] == kMagic0 && buffer[i + 1] == kMagic1) {
          magic_at = i;
          break;
        }
      }
      if (magic_at == std::string::npos) {
        // Keep the last byte in case it is the first half of the magic.
        buffer.erase(buffer.begin(), buffer.end() - 1);
        break;
      }
      if (magic_at > 0) {
        buffer.erase(
            buffer.begin(),
            buffer.begin() +
                static_cast<std::vector<uint8_t>::difference_type>(magic_at));
      }
      if (buffer.size() < sizeof(StatePacket)) {
        break;
      }

      if (is_valid_state_packet(buffer.data(), sizeof(StatePacket))) {
        StatePacket decoded = decode_state(buffer.data());
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          latest_state_ = decoded;
          have_state_ = true;
          last_state_sequence_ = decoded.sequence;
        }
        state_cv_.notify_all();
        buffer.erase(buffer.begin(), buffer.begin() + sizeof(StatePacket));
      } else {
        // Bad frame at this offset: drop one byte and resync on the next
        // occurrence of the magic sequence.
        buffer.erase(buffer.begin());
      }
    }
  }
}

}  // namespace motkin_ros2_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    motkin_ros2_hardware_interface::SystemPicoDualDrv8316CHardware,
    hardware_interface::SystemInterface)
