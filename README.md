# DCLab CAN driver for ROS 2

Two ROS 2 packages that connect a SocketCAN bus to the DCLab boards and to common motor controllers:

- `can_driver`: the Python node.
- `custom_messages`: its messages.

The node supports:

- DCLab boards: Smart Driver, Motor Controller, Sensor, Controller, Swerve module, Absolute Encoder and IMU.
- Other motor controllers: VESC and DJI RoboMaster C610/C620.

## Install

```bash
cd ~/ros2_ws/src
git clone https://github.com/DCLab-ITC/dclab_can_driver.git
sudo apt install python3-can
cd ~/ros2_ws && colcon build && source install/setup.bash
ros2 run can_driver can_driver_node
```

The node opens `can0` at 1 Mbit/s. If the interface is down, it brings it up with `sudo -n ip link set can0 up type can bitrate 1000000`. That needs passwordless `sudo` for `ip`. Alternatively, bring the interface up yourself and set `configure_interface:=false`. If the bus disappears (for example, a USB adapter is unplugged), the node reopens it every 2 s.

## Parameters

| name | default | meaning |
|---|---|---|
| `channel` | `can0` | SocketCAN interface |
| `bitrate` | `1000000` | used when the node brings the interface up |
| `configure_interface` | `true` | bring the interface up when it is down |
| `encoder_ids` | `[100, 199, 250, 299]` | `[first, last]` pairs of position feedback IDs (encoders, Smart Driver, Motor Controller) |
| `speed_filter_alpha` | `0.0` | smoothing of the estimated speed (0 = raw) |
| `sensor_input_ids` | `[500, 510]` | Sensor digital/analog frame IDs |
| `swerve_feedback_ids` | `[0, -1]` (none) | V1 Swerve Controller feedback IDs |
| `imu_id` | `1000` | IMU quaternion ID |
| `log_rates` | `false` | log frames per second |
| `superapp_link` | `true` | run DCLab Link for the Super App's Robot over SSH (see below) |

Example: `ros2 run can_driver can_driver_node --ros-args -p encoder_ids:="[101, 104, 151, 152]"`

## Super App (Robot over SSH)

While the node runs, it also runs DCLab Link (`can_driver/dclab_link.py`), so the Super App can
configure and update the robot's boards from a laptop. On the laptop, keep this open and choose
Robot over SSH in the Super App:

    ssh -N -L 48791:127.0.0.1:48791 USER@ROBOT

DCLab Link is a separate process at low CPU priority, so it does not slow the node: it listens on
127.0.0.1 only (reachable through SSH), sends nothing until a page connects, and the kernel passes
it only the boards' answers to the Super App, not the robot's own traffic. It stops with the node.
Measured on a Jetson with 3000 frames/s on the bus and a page connected: no change in the node's
receive latency (p99 about 1.5 ms either way), no frames lost. Turn it off with
`-p superapp_link:=false`; without ROS, run it alone with `ros2 run can_driver dclab_link` or
`python3 dclab_link.py`.

## Topics

| topic | type | direction |
|---|---|---|
| `/dclab/motor_command` | `DCLabMotorCommand` | to the Smart Driver and Motor Controller motors on one receive ID |
| `/encoder_feedback` | `EncoderFeedback` | from Smart Driver, Motor Controller, Sensor and Absolute Encoder |
| `/digital_analog_feedback` | `DigitalAndAnalogFeedback` | from the Sensor board |
| `/publish_pwm`, `/publish_servo`, `/publish_digital_solenoid` | Controller board | to the board |
| `/publish_swerve`, `/swerve_feedback` | Swerve module | both |
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

**Unchanged from V1:** the Sensor digital/analog frame, the Controller PWM/servo/digital frames, the Swerve frames (V1 Swerve Controller only) and the IMU frame.

## Changes from the January 2026 version

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
