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

#ifndef MOTKIN_ROS2_HARDWARE_INTERFACE__SERIAL_PORT_HPP_
#define MOTKIN_ROS2_HARDWARE_INTERFACE__SERIAL_PORT_HPP_

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace motkin_ros2_hardware_interface {

/// Minimal raw POSIX serial (termios) transport for the USB-CDC link exposed
/// by the pico_dual_PMSM_BUG79100G_DRV8316C firmware. The board ignores the
/// requested baud rate (USB CDC), but a real line coding is configured
/// anyway so the port behaves consistently across platforms.
class SerialPort {
 public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;

  /// Opens `device` and configures it as an 8N1 raw, non-blocking port.
  /// Throws std::runtime_error on failure.
  void open(const std::string& device, unsigned int baud_rate);

  void close();

  bool is_open() const { return fd_ >= 0; }

  /// Writes the full buffer, retrying on short writes. Returns false if the
  /// port is closed or a write error occurs.
  bool write(const uint8_t* data, std::size_t length);

  /// Non-blocking read of whatever is currently available, up to
  /// `max_length` bytes. Returns the number of bytes read (0 if none are
  /// available), or -1 on error.
  int read_available(uint8_t* buffer, std::size_t max_length);

  /// Finds a device matching the platform's default USB-serial locations
  /// (/dev/serial/by-id/*, then /dev/ttyACM*, then /dev/ttyUSB*), mirroring
  /// software/tools/usb_motor_protocol.py `default_port()`. Returns an
  /// empty string if none is found.
  static std::string find_default_device();

 private:
  int fd_{-1};
};

}  // namespace motkin_ros2_hardware_interface

#endif  // MOTKIN_ROS2_HARDWARE_INTERFACE__SERIAL_PORT_HPP_
