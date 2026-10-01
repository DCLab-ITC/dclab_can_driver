# DCLab CAN driver for ROS 2

Two ROS 2 packages that connect a SocketCAN bus to the DCLab boards and to common motor controllers:

- `can_driver`: the Python node.
- `custom_messages`: its messages.

The node supports:

- DCLab boards: Smart Driver, Sensor, Controller, Swerve module, Absolute Encoder and IMU.
- Other motor controllers: VESC and DJI RoboMaster C610/C620.

## Install

```bash
cd ~/ros2_ws/src
git clone https://github.com/soubun72/dclab_can_driver.git
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
| `v1_frames` | `false` | talk to DCLab boards still on V1 firmware (8-byte frames) |
| `encoder_ids` | `[100, 199, 250, 299]` | `[first, last]` pairs of position feedback IDs (encoders, Smart Driver, Motor Controller) |
| `speed_filter_alpha` | `0.0` | smoothing of the estimated speed (0 = raw) |
| `sensor_input_ids` | `[500, 510]` | Sensor digital/analog frame IDs |
| `swerve_feedback_ids` | `[0, -1]` (none) | V1 Swerve Controller feedback IDs |
| `imu_id` | `1000` | IMU quaternion ID |
| `log_rates` | `false` | log frames per second |

Example: `ros2 run can_driver can_driver_node --ros-args -p encoder_ids:="[101, 104, 151, 152]"`

## Topics

| topic | type | direction |
|---|---|---|
| `/smart_driver/command` | `SmartDriverCommand` | to one Smart Driver motor |
| `/smart_driver/pair_command` | `SmartDriverPairCommand` | to both motors of a Smart Driver whose motors share a receive ID |
| `/publish_motor` | `MotorCommand` | V1 message, still accepted |
| `/encoder_feedback` | `EncoderFeedback` | from Smart Driver, Sensor and Absolute Encoder |
| `/digital_analog_feedback` | `DigitalAndAnalogFeedback` | from the Sensor board |
| `/publish_pwm`, `/publish_servo`, `/publish_digital_solenoid` | Controller board | to the board |
| `/publish_swerve`, `/swerve_feedback` | Swerve module | both |
| `/imu/quaternion` | `geometry_msgs/Quaternion` | from the IMU board |
| `/publish_vesc`, `/vesc_status1..4` | VESC | both |
| `/publish_robomaster_current`, `/robomaster_feedback` | RoboMaster | both |

## DCLab frame formats (firmware V2)

These formats apply to Smart Driver 2.6, Sensor 2.7 and Absolute Encoder 2.5, and to later versions.

**Smart Driver command.** Each motor takes 4 bytes: a little-endian `uint32` word.

- Bits 3..0 are the mode:
  - `0` voltage (V)
  - `1` position (rad)
  - `2` speed (rad/s)
  - `3`/`4`/`5` set the position, speed or voltage ramp rate. The rate lasts until the board restarts; the saved settings are the default.
- Bits 31..4 are the goal: a `float32` with its 4 lowest mantissa bits dropped, which leaves about 6 significant digits.
- If the two motors have **different** receive IDs, each motor's ID takes a DLC 4 frame.
- If they have the **same** receive ID, one DLC 8 frame carries motor 1 in bytes 0-3 and motor 2 in bytes 4-7.
- The board drops frames with the wrong length.
- To stop a motor, send voltage 0. There is no separate stop or reset flag. Reset the position from the Super App.

**Encoder feedback.** Each encoder sends a `float32` position in 4 bytes (little-endian); the boards no longer send speed.

- Two encoders that share a transmit ID send one DLC 8 frame. The lower-numbered encoder comes first (`slot` 0).
- Both positions are sampled at the same instant.
- Encoders sharing an ID always share one transmit rate.
- The Sensor allows at most two encoders per ID.
- The driver estimates `speed` from successive positions.

**Motor Controller (V2, replaces the Swerve Controller):** it uses exactly the Smart Driver's frames. Send each channel's command like a Smart Driver motor to that channel's receive ID (two channels on one ID: the pair command, one DLC 8 frame); its positions arrive like encoder feedback on each channel's transmit ID (defaults 201/202 and 251/252, inside `encoder_ids`).

**Unchanged from V1:** the Sensor digital/analog frame, the Controller PWM/servo/digital frames, the Swerve frames (V1 Swerve Controller only) and the IMU frame.

## Changes from the January 2026 version

- The package runs with `ros2 run`. It uses package-relative imports; the old ones only worked from inside the source folder.
- It uses the V2 Smart Driver commands and the position-only feedback. With `v1_frames:=true` it still speaks V1.
- ID ranges and the interface are ROS parameters.
- The CAN interface is reopened on a timer. Before, a failed setup retried by endless recursion, and a send error could crash the node because its error timer was never initialised.
- **VESC current and brake modes were sent in µA instead of mA** (×10⁶ instead of ×10³), which is 1000× the requested current. This is fixed. VESC position mode is in degrees, as the VESC firmware expects.
- A VESC command without a mode is now ignored with a warning. Before, it raised an exception.
- RoboMaster currents are clamped to the ESC limits, and the ESC temperature is published.
- Messages unrelated to DCLab boards, VESC and RoboMaster were removed (tractor, YOLO, debug,
  socket-heartbeat, BRT, raw CAN, motor-array and Damiao messages, and the ResetOdom service),
  together with the Damiao motor support.
- Every frame format lives in `can_driver/dclab_frames.py`, which has no ROS dependency. Unit tests: `python3 -m unittest discover -s can_driver/test -p test_frames.py`.
