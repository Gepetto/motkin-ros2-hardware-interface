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

// Binary layout mirrors firmware/USB_PROTOCOL.md from
// https://github.com/tflayols/pico_dual_PMSM_BUG79100G_DRV8316C (version 2).
// Python reference: software/tools/usb_motor_protocol.py.

#ifndef MOTKIN_ROS2_HARDWARE_INTERFACE__USB_PROTOCOL_HPP_
#define MOTKIN_ROS2_HARDWARE_INTERFACE__USB_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace motkin_ros2_hardware_interface {

constexpr uint8_t kMagic0 = 0xA5;
constexpr uint8_t kMagic1 = 0x5A;
constexpr uint8_t kProtocolVersion = 2;
constexpr uint8_t kPacketTypeCommand = 'C';
constexpr uint8_t kPacketTypeState = 'S';

constexpr uint8_t kM0ReadyFlag = 1u << 0;
constexpr uint8_t kM1ReadyFlag = 1u << 1;
constexpr uint8_t kBothMotorsReady = kM0ReadyFlag | kM1ReadyFlag;

#pragma pack(push, 1)

// PC -> board, 53 bytes. Format string `<BBBBBIBH10fB>`.
struct CommandPacket {
  uint8_t magic0{kMagic0};
  uint8_t magic1{kMagic1};
  uint8_t type{kPacketTypeCommand};
  uint8_t version{kProtocolVersion};
  uint8_t length{0};
  uint32_t command_index{0};
  uint8_t flags{0};
  uint16_t timeout_ms{0};
  float m0_kp{0.0f};
  float m0_kd{0.0f};
  float m0_iff{0.0f};
  float m0_q_target{0.0f};
  float m0_v_target{0.0f};
  float m1_kp{0.0f};
  float m1_kd{0.0f};
  float m1_iff{0.0f};
  float m1_q_target{0.0f};
  float m1_v_target{0.0f};
  uint8_t checksum{0};
};

// Board -> PC, 61 bytes. Format string `<BBBBBHIIf10fBB>`.
struct StatePacket {
  uint8_t magic0;
  uint8_t magic1;
  uint8_t type;
  uint8_t version;
  uint8_t length;
  uint16_t sequence;
  uint32_t t_us;
  uint32_t latest_command_index;
  float control_loop_us;
  float m0_q;
  float m0_v;
  float m0_v_highfrequency;
  float m0_i;
  float m0_i_target;
  float m1_q;
  float m1_v;
  float m1_v_highfrequency;
  float m1_i;
  float m1_i_target;
  uint8_t flags;
  uint8_t checksum;
};

#pragma pack(pop)

static_assert(sizeof(CommandPacket) == 53,
              "CommandPacket layout must match USB_PROTOCOL.md");
static_assert(sizeof(StatePacket) == 61,
              "StatePacket layout must match USB_PROTOCOL.md");

/// XOR of every byte in [data, data + length - 1), i.e. every byte except the
/// trailing checksum byte itself.
inline uint8_t compute_checksum(const uint8_t* data, std::size_t length) {
  uint8_t value = 0;
  for (std::size_t i = 0; i + 1 < length; ++i) {
    value ^= data[i];
  }
  return value;
}

struct MotorCommand {
  float kp{0.0f};
  float kd{0.0f};
  float iff{0.0f};
  float q_target{0.0f};
  float v_target{0.0f};
};

/// Builds a checksummed command packet ready to write on the wire.
inline CommandPacket encode_command(uint32_t command_index, uint8_t flags,
                                    uint16_t timeout_ms, const MotorCommand& m0,
                                    const MotorCommand& m1) {
  CommandPacket packet;
  packet.length = sizeof(CommandPacket);
  packet.command_index = command_index;
  packet.flags = flags;
  packet.timeout_ms = timeout_ms;
  packet.m0_kp = m0.kp;
  packet.m0_kd = m0.kd;
  packet.m0_iff = m0.iff;
  packet.m0_q_target = m0.q_target;
  packet.m0_v_target = m0.v_target;
  packet.m1_kp = m1.kp;
  packet.m1_kd = m1.kd;
  packet.m1_iff = m1.iff;
  packet.m1_q_target = m1.q_target;
  packet.m1_v_target = m1.v_target;
  packet.checksum = compute_checksum(reinterpret_cast<const uint8_t*>(&packet),
                                     sizeof(CommandPacket));
  return packet;
}

/// Validates the fixed header fields and checksum of a candidate state frame.
inline bool is_valid_state_packet(const uint8_t* data, std::size_t length) {
  if (length != sizeof(StatePacket)) {
    return false;
  }
  if (data[0] != kMagic0 || data[1] != kMagic1) {
    return false;
  }
  if (data[2] != kPacketTypeState || data[3] != kProtocolVersion) {
    return false;
  }
  if (data[4] != sizeof(StatePacket)) {
    return false;
  }
  return compute_checksum(data, length) == data[length - 1];
}

/// Reinterprets an already-validated buffer as a StatePacket.
inline StatePacket decode_state(const uint8_t* data) {
  StatePacket packet;
  std::memcpy(&packet, data, sizeof(StatePacket));
  return packet;
}

}  // namespace motkin_ros2_hardware_interface

#endif  // MOTKIN_ROS2_HARDWARE_INTERFACE__USB_PROTOCOL_HPP_
