# DCLab CAN driver - user guide

How to set up the driver on a robot, drive motors and read sensors from ROS 2. The
[README](README.md) is the reference: every parameter, the frame formats and the change log.

```
your ROS 2 nodes  <-- topics -->  can_driver  <-- can0 (1 Mbit/s) -->  DCLab boards, VESC, RoboMaster
```

## 1. Before you start

- ROS 2 (tested with Humble) on Linux, and a colcon workspace (`~/ros2_ws`).
- A CAN adapter that Linux shows as `can0`: a CANable with candleLight firmware works out of the box.
- The bus wired at 1 Mbit/s with a 120 Ω terminator at **both** ends.
- The boards set up in the **Super App** first: each board's motor/encoder IDs, modes, gains and CAN
  timeout are saved on the board. The driver only sends commands to those IDs and reads what the
  boards send.

## 2. Install

```bash
cd ~/ros2_ws/src
git clone https://github.com/DCLab-ITC/dclab_can_driver.git
sudo apt install python3-can
cd ~/ros2_ws && colcon build && source install/setup.bash
```

Add `source ~/ros2_ws/install/setup.bash` to `~/.bashrc` so every terminal finds the packages.

Start the driver (one of the two, never both):

```bash
ros2 run can_driver_cpp can_driver_node   # C++: recommended, about half the CPU, no losses on a busy bus
ros2 run can_driver can_driver_node       # Python: same topics and parameters
```

The node brings `can0` up at 1 Mbit/s by itself. That needs passwordless `sudo` for `ip`; without
it, bring the interface up yourself and add `--ros-args -p configure_interface:=false`:

```bash
sudo ip link set can0 up type can bitrate 1000000
```

If the adapter is unplugged, the node keeps running and reopens the bus every 2 s.

## 3. Check that it works

```bash
ros2 topic list                    # /dclab/motor_command, /encoder_feedback, ...
ros2 topic echo /encoder_feedback  # positions from Smart Drivers, Motor Controllers, Sensors, encoders
ros2 topic hz /encoder_feedback
```

Nothing arrives? See [Troubleshooting](#11-troubleshooting). To see the raw frames, install
`can-utils` and run `candump can0`.

## 4. Drive DCLab motors (Smart Driver, Motor Controller)

One topic drives every DCLab motor: `/dclab/motor_command` (`custom_messages/msg/DCLabMotorCommand`).
Each message goes to one **receive ID** (`can_id`, set in the Super App). An ID carries one or two
motors: `M1` is the lower-numbered motor (or Motor Controller channel) on that ID, `M2` the other.

| mode | value | goal |
|---|---|---|
| `MODE_VOLTAGE` | 0 | volts (Motor Controller: its motor's voltage or current) |
| `MODE_POSITION` | 1 | rad |
| `MODE_SPEED` | 2 | rad/s |
| `MODE_POSITION_RAMP`, `MODE_SPEED_RAMP`, `MODE_VOLTAGE_RAMP` | 3, 4, 5 | ramp rate, until the board restarts |

```bash
# motor 1 of ID 101 at 2 rad/s
ros2 topic pub -r 50 /dclab/motor_command custom_messages/msg/DCLabMotorCommand \
  "{can_id: 101, m1_enable: true, mode1: 2, goal1: 2.0}"
# both motors on ID 101: M1 to 1.57 rad, M2 at -1 rad/s
ros2 topic pub -r 50 /dclab/motor_command custom_messages/msg/DCLabMotorCommand \
  "{can_id: 101, m1_enable: true, mode1: 1, goal1: 1.57, m2_enable: true, mode2: 2, goal2: -1.0}"
# stop motor 1 of ID 101
ros2 topic pub --once /dclab/motor_command custom_messages/msg/DCLabMotorCommand \
  "{can_id: 101, m1_enable: true, mode1: 0, goal1: 0.0}"
```

Rules worth knowing:

- **Keep sending** at a steady rate (20-100 Hz), not once. When a motor gets no command for longer
  than its CAN timeout (set in the Super App; Smart Driver default 500 ms, 0 = never), a Smart Driver
  switches it to 0 V; a Motor Controller channel holds its position, or goes to 0 speed or current.
- **To stop, send voltage 0.** `m1_enable: false` does not stop M1: it leaves M1's last command
  running and only updates M2.
- Enabling M2 on an ID that has only one motor: the board ignores the whole frame.
- An unknown mode, a goal that is not a number or a `can_id` above 2047 is refused with a warning.
- The board only accepts the modes enabled for that motor in the Super App.

From your own Python node:

```python
from custom_messages.msg import DCLabMotorCommand

pub = node.create_publisher(DCLabMotorCommand, "/dclab/motor_command", 10)
pub.publish(DCLabMotorCommand(can_id=101, m1_enable=True,
                              mode1=DCLabMotorCommand.MODE_SPEED, goal1=2.0))
```

C++: `#include "custom_messages/msg/dclab_motor_command.hpp"`, the same fields.

## 5. Read positions

`/encoder_feedback` (`EncoderFeedback`): one message per encoder per frame.

| field | meaning |
|---|---|
| `can_id` | the transmit ID the board sent on |
| `slot` | 0 = first (or only) encoder on that ID, 1 = the second |
| `position` | rad |
| `speed` | estimated by the driver from successive positions; smooth it with `speed_filter_alpha` (0..1, 0 = raw) |

The driver only reads IDs inside `encoder_ids` (default 100-199 and 250-299). If a board sends on
other IDs, list them:

```bash
ros2 run can_driver_cpp can_driver_node --ros-args -p encoder_ids:="[100, 199, 250, 299, 300, 310]"
```

## 6. RoboMaster motors (DJI C610 / C620)

Two ways:

**Through a DCLab Motor Controller (recommended).** Up to four RoboMasters per Motor Controller,
wired to its motor bus. In the Super App, set each channel to RoboMaster, pick the ESC type
(C610/C620) and the channel's **ESC ID, 1-4** - the ID set on the ESC itself (its button and LED
blinks), a different one on each channel. The Super App refuses an ID another channel already
uses. The board sends all four currents in one CAN frame at up to 1 kHz. Drive the channels with
`/dclab/motor_command` like any DCLab motor (section 4): position, speed or current (A), with the
control loop running on the board. Positions come back on `/encoder_feedback`.

Example: four M3508 wheels on one Motor Controller. Channels 1-4 = ESC IDs 1-4, receive IDs 201
(channels 1+2) and 202 (channels 3+4):

```bash
ros2 topic pub -r 50 /dclab/motor_command custom_messages/msg/DCLabMotorCommand \
  "{can_id: 201, m1_enable: true, mode1: 2, goal1: 10.0, m2_enable: true, mode2: 2, goal2: 10.0}"
ros2 topic pub -r 50 /dclab/motor_command custom_messages/msg/DCLabMotorCommand \
  "{can_id: 202, m1_enable: true, mode1: 2, goal1: -10.0, m2_enable: true, mode2: 2, goal2: -10.0}"
```

**Directly on the robot's bus (current only).** The driver passes currents straight to the ESCs;
any speed or position control is up to your code.

- `/publish_robomaster_current` (`RobomasterCurrentCommand`): `can_id` `0x200` (512) = ESC IDs 1-4,
  `0x1FF` (511) = ESC IDs 5-8. `current1..4` in A for the 4 ESCs of that group; `type1..4` = 0 for
  C610 (clamped to ±10 A), 1 for C620 (±20 A).
- One message sets all four ESCs of a group: an ESC you leave at 0 gets 0 A. Put every motor of a
  group in the same message, and keep sending at a steady rate.
- `/robomaster_feedback` (`RobomasterFeedback`) from each ESC: `motor_id` 1-8, `position` = raw
  rotor angle 0-8191 per rotor turn (before the gearbox), `speed` = rotor rpm, `current` = the ESC's
  raw torque-current value ÷ 1000, `temp` = °C.

```bash
# ESC 1 (C620) at 1.5 A, ESC 2 (C610) at -0.5 A, ESCs 3 and 4 at 0
ros2 topic pub -r 100 /publish_robomaster_current custom_messages/msg/RobomasterCurrentCommand \
  "{can_id: 512, type1: 1, current1: 1.5, type2: 0, current2: -0.5}"
```

ESCs wired to a Motor Controller are on its own motor bus: `/publish_robomaster_current` does not
reach them, and `/robomaster_feedback` does not show them.

## 7. VESC

`/publish_vesc` (`VescCommand`): set exactly one mode flag and `goal`. A message with no mode is
ignored with a warning.

| flag | goal |
|---|---|
| `pwm_mode` | duty -1..1 |
| `current_mode` | A |
| `current_brake_mode` | A |
| `speed_mode` | ERPM |
| `position_mode` | degrees |

```bash
ros2 topic pub -r 50 /publish_vesc custom_messages/msg/VescCommand "{vesc_id: 2, speed_mode: true, goal: 3000.0}"
```

Status: `/vesc_status1` (rpm, current, duty), `/vesc_status2` (Ah), `/vesc_status3` (Wh),
`/vesc_status4` (temperatures, input current, PID position). The VESC must have its CAN status
messages turned on (VESC Tool, App Settings > General).

## 8. Controller board outputs

| topic | message | values |
|---|---|---|
| `/publish_pwm` | `PwmCommand` | `pwm1..4_value` duty 0..1 |
| `/publish_servo` | `ServoCommand` | `servo1..4_value` 0..1 |
| `/publish_digital_solenoid` | `DigitalAndSolenoidCommand` | `digital1..4_value`, `solenoid1..6_value` true/false |

Values outside 0..1 are clamped. `can_id` is the Controller's receive ID.

```bash
ros2 topic pub --once /publish_digital_solenoid custom_messages/msg/DigitalAndSolenoidCommand \
  "{can_id: 600, solenoid1_value: true}"
```

## 9. Sensor inputs and IMU

- `/digital_analog_feedback` (`DigitalAndAnalogFeedback`): the Sensor board's 4 digital inputs and 5
  analog inputs (0..1 of full scale), from IDs in `sensor_input_ids` (default 500-510).
- `/imu/quaternion` (`geometry_msgs/Quaternion`): orientation from the IMU board on `imu_id`
  (default 1000).

## 10. Parameters in a launch file

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([Node(
        package="can_driver_cpp", executable="can_driver_node", output="screen",
        parameters=[{
            "channel": "can0",
            "encoder_ids": [100, 199, 250, 299],
            "speed_filter_alpha": 0.3,
            "sensor_input_ids": [500, 510],
            "imu_id": 1000,
        }])])
```

All parameters are listed in the [README](README.md#parameters).

### Configure the boards while the robot runs

The node also runs DCLab Link, so the Super App can reach the robot's boards from a laptop on the
same network. In the Super App choose **Robot**, type the robot's address (the node logs it) and
Connect, then log in with the robot computer's account (the one running the node). Keep the small
robot window open while you work. Leave "Remember this computer" ticked and later connects skip the
login; "Forget this computer" in that window, or `ros2 run can_driver dclab_link --forget-computers`
on the robot (all computers), cancels it. Without a shared network, use an SSH tunnel instead (see the
README). Turn it off with `-p superapp_link:=false`.

### Update

```bash
cd ~/ros2_ws/src/dclab_can_driver && ./update.sh            # from GitHub
./update.sh ~/Downloads/dclab_can_driver.zip                  # robot without internet (Super App Downloads)
```

Restart the node afterwards.

## 11. Troubleshooting

| symptom | check |
|---|---|
| `CAN can0 not available` in the log | adapter plugged in? `ip link show can0`; without passwordless `sudo`, bring it up yourself (section 2) |
| no messages on `/encoder_feedback` | `candump can0` shows frames? Their IDs inside `encoder_ids`? Board powered and set up in the Super App? |
| `candump` shows nothing | bitrate 1 Mbit/s on every device, terminators at both ends, CAN-H/CAN-L not swapped |
| a motor does not move | right `can_id`, mode enabled for that motor in the Super App, commands sent continuously (CAN timeout), board not in fault (Super App shows it) |
| a motor moves then stops | the commands stopped, or came slower than the CAN timeout |
| two motors on one ID: one ignores commands | M1 is the lower-numbered motor on that ID; a motor alone on its ID only takes M1 |
| RoboMaster motors jerk or stop | through a Motor Controller: each channel on its own ESC ID (1-4) matching the ESC. Directly: every ESC of a group in one message, sent continuously; nothing else sending `0x200`/`0x1FF` |
| `CAN transmit queue full, frame dropped` | the bus is overloaded or nothing acknowledges frames (no board powered, no terminator) |
| high CPU or lost messages on a busy bus | use the C++ node |
| the Super App's CANable connection ends when the node starts | expected: the node takes the CANable back; use the Super App's Robot connection while the node runs |

`-p log_rates:=true` logs frames per second sent, received and not understood.
