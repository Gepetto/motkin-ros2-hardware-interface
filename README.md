# motkin_ros2_hardware_interface

`ros2_control` `SystemInterface` hardware plugin for the
[`motkin-pcb`](https://github.com/Gepetto/MOTKIN-PCB)
board: a Raspberry Pi Pico driving two PMSM motors through BUG79100G quadrature
readers and DRV8316C gate drivers, exposed to the PC over USB-CDC with the
binary protocol documented in `firmware/USB_PROTOCOL.md` of that repository.

It is the real-hardware counterpart to `motkin_dual_motor_testbed_gazebo` /
`ros2_hardware_interface_odri`: same `ros2_control` integration pattern
(xacro macro -> `<ros2_control>` block -> `hardware_interface::SystemInterface`
plugin), but talking to the motkin board over a serial link instead of Gazebo
or an ODRI master board over Ethernet.

## Interfaces

Each of the two joints (motor `M0` and `M1`, in the order the `<joint>` tags
appear under `<ros2_control>`) exposes the same five command/state
interfaces used by `ros2_hardware_interface_odri`, so the existing
`motkin_forward_command_controller` works unmodified:

| Interface   | Direction | Firmware field           | Unit          |
| ----------- | --------- | ------------------------ | ------------- |
| `position`  | cmd+state | `q_target` / `q`         | rad           |
| `velocity`  | cmd+state | `v_target` / `v`         | rad/s         |
| `effort`    | cmd+state | `iff` / `i` (current)    | A             |
| `gain_kp`   | cmd+state | `kp`                     | A/rad         |
| `gain_kd`   | cmd+state | `kd`                     | A/(rad/s)     |

The firmware runs a single control law per motor:

```
iq = iff + kp * (q_target - q) + kd * (v_target - v)
```

There is no separate "position mode" or "velocity mode" on the board itself:
whichever of the five command interfaces a controller claims, the values
currently held in the *other*, unclaimed interfaces are still sent as-is
(and default to `0.0`). In particular, a plain `forward_position_controller`
that only claims `position` will produce **zero torque**, because
`gain_kp`/`gain_kd` stay at `0`. Use `motkin_forward_command_controller` (or
any controller that sets `gain_kp`/`gain_kd` alongside the targets) for
actual motion.

## Hardware parameters

Set on the `<hardware>` block in the `ros2_control` xacro:

| Parameter     | Default                       | Meaning                                    |
| ------------- | ------------------------------ | ------------------------------------------- |
| `serial_port` | auto-detect                    | e.g. `/dev/ttyACM0`. Empty scans `/dev/serial/by-id/*`, then `/dev/ttyACM*`, then `/dev/ttyUSB*`. |
| `baud_rate`   | `115200`                       | Ignored by USB-CDC but still set on the port. |
| `timeout_ms`  | `20`                           | Board-side command watchdog; both motors are forced to zero current if no valid command arrives within this window. |

## Usage

```xml
<xacro:include filename="$(find motkin_ros2_hardware_interface)/ros2_control/system_motkin.ros2_control.xacro" />
<xacro:motkin_ros2_control
  name="motkin"
  left_joint_name="motkin_dm_tb_kt_left_joint"
  right_joint_name="motkin_dm_tb_kt_right_joint"
  serial_port="/dev/ttyACM0" />
```

To use this hardware interface in place of Gazebo or the ODRI master board
for `motkin_dual_motor_testbed`, include this macro from
`motkin_dual_motor_testbed_description` instead of
`system_dual_motor_testbed.ros2_control.xacro` — the joint names and command
interfaces match, so `motkin_dual_motor_testbed_bringup`'s existing controllers
config keeps working.


## Startup / shutdown behavior

* `on_activate` opens the serial port, starts a background reader thread,
  and performs the protocol's recommended startup sequence: send a
  zero-gain, zero-timeout command and block (up to 2s) until the board
  echoes that command's index in a state packet, before enabling the
  watchdog. Activation fails if the board never responds.
* `on_deactivate` sends a command with `flags = 0` (forcing zero current on
  both motors regardless of last commanded gains), stops the reader thread,
  and closes the port.
* If `write()` fails to reach the board (e.g. the USB cable is unplugged),
  `read`/`write` return `ERROR` so the controller manager can react; the
  board's own watchdog independently zeroes torque if it stops receiving
  valid commands within `timeout_ms`.
