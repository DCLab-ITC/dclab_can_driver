# DCLab CAN driver for ROS 2

New here? Start with the [user guide](USER_GUIDE.md): setup, driving each kind of motor, reading sensors, troubleshooting.

ROS 2 packages that connect a SocketCAN bus to the DCLab boards and to common motor controllers:

- `can_driver`: the Python node.
- `can_driver_cpp`: the same node in C++, faster (see [C++ node](#c-node-can_driver_cpp)).
- `custom_messages`: their messages.

The node supports:

- DCLab boards: Smart Driver, Motor Controller, Sensor, Controller, Absolute Encoder and IMU.
- Other motor controllers: VESC and DJI RoboMaster C610/C620.

## Install

```bash
cd ~/ros2_ws/src
git clone https://github.com/DCLab-ITC/dclab_can_driver.git
sudo apt install python3-can
cd ~/ros2_ws && colcon build && source install/setup.bash
ros2 run can_driver can_driver_node      # Python
ros2 run can_driver_cpp can_driver_node    # or C++
```

The node opens `can0` at 1 Mbit/s. If the interface is down, it brings it up with `sudo -n ip link set can0 up type can bitrate 1000000`. That needs passwordless `sudo` for `ip`. Alternatively, bring the interface up yourself and set `configure_interface:=false`. If the bus disappears (for example, a USB adapter is unplugged), the node reopens it every 2 s.

## C++ node (can_driver_cpp)

`ros2 run can_driver_cpp can_driver_node` runs the C++ build. It has the same node name, topics,
parameters and frames as the Python node, so it replaces it with no other change (run one of the two,
not both). It also runs DCLab Link and takes the CANable back like the Python node. Its frame code is
checked against the Python node's (`can_driver_cpp/test`), and both nodes gave identical messages and
frames for the same input.

Measured on a Jetson (4 cores, ROS 2 Humble): a bus of encoder frames (DLC 8, two positions each, so
two messages per frame) for 10 s on a virtual CAN interface; latency is from the frame on the bus to
the message in a C++ subscriber.

| frames/s | node | CPU (one core = 100%) | messages lost | latency p50 | p99 | max |
|---|---|---|---|---|---|---|
| 1000 | Python | 59% | 0% | 0.58 ms | 6.4 ms | 15 ms |
| 1000 | C++ | 38% | 0% | 0.59 ms | 5.0 ms | 13 ms |
| 3000 | Python | 109% | 2.5% | 38 ms | 170 ms | 197 ms |
| 3000 | C++ | 53% | 0% | 0.24 ms | 6.4 ms | 16 ms |
| 5000 | Python | 113% (saturated) | 37% | 86 ms | 128 ms | 142 ms |
| 5000 | C++ | 61% | 0% | 0.17 ms | 10 ms | 22 ms |

Below about 1000 frames/s both are fast enough; on a busy bus the Python node falls behind (its
latency grows and frames are dropped), the C++ node does not.

## Update

Run `./update.sh` in the `dclab_can_driver` folder. It pulls the latest version from GitHub (a
git clone) or replaces the folder with it (a copy from a .zip), then rebuilds the packages with colcon. A robot without internet: copy `dclab_can_driver.zip` from the Super App's Downloads to it
and run `./update.sh dclab_can_driver.zip`. Restart the node afterwards.

## CANable and the Super App on the same computer

The Super App's Linux setup (`/etc/udev/rules.d/60-dclab-usb.rules`) takes the CANable away from
Linux at every plug-in so the browser can use it, which leaves no `can0`. The node has priority:
when its interface is missing it gives every CANable back to Linux (ending a browser session on it)
and opens it, with no root needed (the same rule lets every user open the adapter). To configure the
boards while the node runs, use the Super App's Robot connection: it shares `can0` with the node.
Turn the takeover off with `-p take_canable:=false`.

## Parameters

| name | default | meaning |
|---|---|---|
| `channel` | `can0` | SocketCAN interface |
| `bitrate` | `1000000` | used when the node brings the interface up |
| `configure_interface` | `true` | bring the interface up when it is down |
| `take_canable` | `true` | take a CANable back from the Super App's browser when the interface is missing |
| `encoder_ids` | `[100, 199, 250, 299]` | `[first, last]` pairs of position feedback IDs (encoders, Smart Driver, Motor Controller) |
| `speed_filter_alpha` | `0.0` | smoothing of the estimated speed (0 = raw) |
| `sensor_input_ids` | `[500, 510]` | Sensor digital/analog frame IDs |
| `imu_id` | `1000` | IMU quaternion ID |
| `log_rates` | `false` | log frames per second |
| `superapp_link` | `true` | run DCLab Link for the Super App's Robot connection (see below) |
| `superapp_link_lan` | `true` | DCLab Link reachable on the robot's network (login with the robot computer's account); `false` = through SSH only |

Example: `ros2 run can_driver can_driver_node --ros-args -p encoder_ids:="[101, 104, 151, 152]"`

## Super App (Robot)

While the node runs, it also runs DCLab Link (`can_driver/dclab_link.py`), so the Super App can
configure and update the robot's boards from a laptop on the same network, with nothing to install
on the laptop and no internet on the robot. In the Super App choose Robot, type the robot's address
(the node logs it, e.g. `192.168.1.20`) and Connect: a small window from the robot asks for the
robot computer's login - the account that runs this node, checked by the robot's own system as SSH
does. Keep that window open while you work (it can be minimised); closing it ends the link.

Without the window, through an SSH tunnel (also works with `superapp_link_lan:=false`): keep this
open on the laptop, choose Robot with the address left empty, Connect:

    ssh -N -L 48791:127.0.0.1:48791 USER@ROBOT

DCLab Link is a separate process at low CPU priority, so it does not slow the node. It opens the
CAN interface only while a page is connected and after the robot's login (repeated wrong passwords
are slowed down), takes pages only from the Super App's site or its own window, and the kernel
passes it only the boards' answers to the Super App, not the robot's own traffic. It stops with
the node. Measured on a Jetson with 3000 frames/s on the bus and a page connected: no change in
the node's receive latency (p99 about 1.5 ms either way), no frames lost. The login crosses the
network unencrypted (plain http on the robot): use it on the lab network, or the SSH tunnel
elsewhere. Turn it off with `-p superapp_link:=false`; without ROS, run it alone with
`ros2 run can_driver dclab_link` or `python3 dclab_link.py`.

## Topics

| topic | type | direction |
|---|---|---|
| `/dclab/motor_command` | `DCLabMotorCommand` | to the Smart Driver and Motor Controller motors on one receive ID |
| `/encoder_feedback` | `EncoderFeedback` | from Smart Driver, Motor Controller, Sensor and Absolute Encoder |
| `/digital_analog_feedback` | `DigitalAndAnalogFeedback` | from the Sensor board |
| `/publish_pwm`, `/publish_servo`, `/publish_digital_solenoid` | Controller board | to the board |
| `/imu/quaternion` | `geometry_msgs/Quaternion` | from the IMU board |
| `/publish_vesc`, `/vesc_status1..4` | VESC | both |
| `/publish_robomaster_current`, `/robomaster_feedback` | RoboMaster | both |

## Commanding motors

`/dclab/motor_command` (`DCLabMotorCommand`) sends the motors on one receive ID. M1 and M2 are the first and second motor on `can_id`, not motor 1 and 2 of the board.

| `m1_enable` | `m2_enable` | sent on `can_id` |
|---|---|---|
| true | false | DLC 4: M1 |
| false | true | DLC 8: M1 disabled, M2 |
| true | true | DLC 8: M1, M2 |
| false | false | nothing (warning) |

An unknown mode, a goal that is not finite or a `can_id` above `0x7FF` is refused with a warning. If you enable M2 on an ID that has only one motor, the board ignores the frame. The driver cannot check this.

```bash
# both Smart Driver motors on ID 101: motor 1 at 3 V, motor 2 at -2 rad/s
ros2 topic pub --once /dclab/motor_command custom_messages/msg/DCLabMotorCommand \
  "{can_id: 101, m1_enable: true, mode1: 0, goal1: 3.0, m2_enable: true, mode2: 2, goal2: -2.0}"
# only motor 2 on ID 101 (motor 1 keeps its command)
ros2 topic pub --once /dclab/motor_command custom_messages/msg/DCLabMotorCommand \
  "{can_id: 101, m2_enable: true, mode2: 2, goal2: -2.0}"
# a motor alone on ID 102
ros2 topic pub --once /dclab/motor_command custom_messages/msg/DCLabMotorCommand \
  "{can_id: 102, m1_enable: true, mode1: 1, goal1: 1.57}"
```

## DCLab frame formats (firmware V2)

These formats apply to Smart Driver 2.13, Motor Controller 2.22, Sensor 2.7 and Absolute Encoder 2.5, and to later versions.

**Motor command (Smart Driver and Motor Controller).** Each motor takes 4 bytes: a little-endian `uint32` word.

- Bits 2..0 are the mode:
  - `0` voltage (V; on the Motor Controller, the voltage or current of its motor)
  - `1` position (rad)
  - `2` speed (rad/s)
  - `3`/`4`/`5` set the position, speed or voltage ramp rate. The rate lasts until the board restarts; the saved settings are the default.
  - `6` and `7` are ignored.
- Bit 3 is the enable flag. A word without it is skipped: that motor keeps its command and its CAN timeout keeps running.
- Bits 31..4 are the goal: a `float32` with its 4 lowest mantissa bits dropped, which leaves about 6 significant digits.
- On one receive ID, a DLC 4 frame carries the first motor (M1). A DLC 8 frame carries M1 in bytes 0-3 and the second motor (M2) in bytes 4-7.
- M1 is the lower-numbered motor (or Motor Controller channel) on that ID. A motor alone on its ID is M1 and takes DLC 4 only.
- To stop a motor, send voltage 0. A disabled motor is not stopped. There is no reset flag; reset the position from the Super App.

**Encoder feedback.** Each encoder sends a `float32` position in 4 bytes (little-endian); the boards no longer send speed.

- Two encoders that share a transmit ID send one DLC 8 frame. The lower-numbered encoder comes first (`slot` 0).
- Both positions are sampled at the same instant.
- Encoders sharing an ID always share one transmit rate.
- The Sensor allows at most two encoders per ID.
- The driver estimates `speed` from successive positions.

**Motor Controller (V2, replaces the Swerve Controller):** it takes the same motor commands on each channel's receive ID (up to two channels per ID). Its positions arrive like encoder feedback on each channel's transmit ID (defaults 201/202 and 251/252, inside `encoder_ids`).

These formats need Smart Driver 2.13 and Motor Controller 2.22 or later. Older boards ignore the commands (no enable flag), and these boards ignore commands from older drivers, so update both together.

**Unchanged from V1:** the Sensor digital/analog frame, the Controller PWM/servo/digital frames and the IMU frame. The V1 Swerve Controller frames are gone since 2.6.0.

## Changes from the January 2026 version

- 2.6.0: removed the V1 Swerve Controller support (`/publish_swerve`, `/swerve_feedback`,
  `swerve_feedback_ids`, `SwerveCommand`, `SwerveFeedback`) and the unused `RobomasterCommand` message. The V2
  Motor Controller replaces the Swerve Controller and runs RoboMaster motors itself (`/dclab/motor_command`).
- 2.5.2: DCLab Link 3.3 - fixes the robot window: its script never ran (a raw carriage return in a
  regular expression), so Connect only reloaded the form and the Super App waited. Update every robot.
- 2.5.1: DCLab Link 3.2 - a refused robot login is shown clearly in the robot window and in the Super App, and names the
  account that works (the one running can_driver; only its password can be checked).
- 2.5.0: the C++ package is named `can_driver_cpp` (was `can_driver_c`): run it with
  `ros2 run can_driver_cpp can_driver_node`. After updating, delete `build/can_driver_c` and
  `install/can_driver_c` in the workspace.
- 2.4.0: `can_driver_c`, the C++ node (same topics and parameters, about half the CPU, no losses
  at 5000 frames/s); `update.sh` builds it too.
- 2.3.0: the node takes the CANable back from the Super App's browser when `can0` is missing
  (parameter `take_canable`); `update.sh` updates and rebuilds the driver.
- 2.2.1: DCLab Link 3.1 (review #3): login checks one at a time with per-address backoff, no empty
  passwords, account checks (expired/locked), the login only for this computer's own name or address;
  size and connection limits; a lost CAN adapter is reported; a window that was taken over no longer
  logs in again.
- 2.2.0: DCLab Link 3: the Super App connects from the robot's login window, nothing to install on
  the laptop (parameter `superapp_link_lan`).
- 2.1.0: the node runs DCLab Link for the Super App (parameter `superapp_link`).

- The package runs with `ros2 run`. It uses package-relative imports; the old ones only worked from inside the source folder.
- It uses the V2 motor commands (`/dclab/motor_command`, one topic for the Smart Driver and the Motor Controller) and the position-only feedback. V1 frames, `/publish_motor` and the `v1_frames` parameter were removed.
- ID ranges and the interface are ROS parameters.
- The CAN interface is reopened on a timer. Before, a failed setup retried by endless recursion, and a send error could crash the node because its error timer was never initialised.
- **VESC current and brake modes were sent in µA instead of mA** (×10⁶ instead of ×10³), which is 1000× the requested current. This is fixed. VESC position mode is in degrees, as the VESC firmware expects.
- A VESC command without a mode is now ignored with a warning. Before, it raised an exception.
- RoboMaster currents are clamped to the ESC limits, and the ESC temperature is published.
- Messages unrelated to DCLab boards, VESC and RoboMaster were removed (tractor, YOLO, debug,
  socket-heartbeat, BRT, raw CAN, motor-array and Damiao messages, and the ResetOdom service),
  together with the Damiao motor support.
- Every frame format lives in `can_driver/dclab_frames.py`, which has no ROS dependency. Unit tests: `python3 -m unittest discover -s can_driver/test -p test_frames.py`.
