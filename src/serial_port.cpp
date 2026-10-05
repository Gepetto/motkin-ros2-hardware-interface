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

#include "motkin_ros2_hardware_interface/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <vector>

namespace motkin_ros2_hardware_interface {

namespace {

speed_t baud_to_speed_t(unsigned int baud_rate) {
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      return B115200;
  }
}

std::vector<std::string> list_directory(const std::string& dir) {
  namespace fs = std::filesystem;
  std::vector<std::string> entries;
  std::error_code ec;
  for (const auto& entry : fs::directory_iterator(dir, ec)) {
    entries.push_back(entry.path().string());
  }
  return entries;
}

}  // namespace

SerialPort::~SerialPort() { close(); }

void SerialPort::open(const std::string& device, unsigned int baud_rate) {
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw std::runtime_error("Failed to open serial device '" + device +
                             "': " + std::strerror(errno));
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    int saved_errno = errno;
    close();
    throw std::runtime_error("tcgetattr failed on '" + device +
                             "': " + std::strerror(saved_errno));
  }

  cfmakeraw(&tty);
  speed_t speed = baud_to_speed_t(baud_rate);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  // Non-blocking reads: the hardware interface polls the port itself.
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    int saved_errno = errno;
    close();
    throw std::runtime_error("tcsetattr failed on '" + device +
                             "': " + std::strerror(saved_errno));
  }

  tcflush(fd_, TCIOFLUSH);
}

void SerialPort::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::write(const uint8_t* data, std::size_t length) {
  if (fd_ < 0) {
    return false;
  }
  std::size_t written = 0;
  while (written < length) {
    ssize_t result = ::write(fd_, data + written, length - written);
    if (result < 0) {
      if (errno == EAGAIN || errno == EINTR) {
        continue;
      }
      return false;
    }
    written += static_cast<std::size_t>(result);
  }
  return true;
}

int SerialPort::read_available(uint8_t* buffer, std::size_t max_length) {
  if (fd_ < 0) {
    return -1;
  }
  ssize_t result = ::read(fd_, buffer, max_length);
  if (result < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return 0;
    }
    return -1;
  }
  return static_cast<int>(result);
}

std::string SerialPort::find_default_device() {
  auto by_id = list_directory("/dev/serial/by-id");
  if (!by_id.empty()) {
    std::sort(by_id.begin(), by_id.end());
    return by_id.front();
  }

  std::vector<std::string> candidates;
  for (const auto& path : list_directory("/dev")) {
    std::string name = std::filesystem::path(path).filename().string();
    if (name.rfind("ttyACM", 0) == 0 || name.rfind("ttyUSB", 0) == 0) {
      candidates.push_back(path);
    }
  }
  if (candidates.empty()) {
    return "";
  }
  std::sort(candidates.begin(), candidates.end());
  return candidates.front();
}

}  // namespace motkin_ros2_hardware_interface
