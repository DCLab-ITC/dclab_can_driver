"""CAN frame formats of the DCLab boards, without ROS or python-can.

Firmware V2 (Smart Driver 2.13, Motor Controller 2.22, Sensor 2.7, Absolute
Encoder 2.5 and later) uses 4 bytes per motor or encoder:

* Motor command (Smart Driver and Motor Controller): little-endian uint32,
  bits 2..0 = mode, bit 3 = enable, bits 31..4 = the goal float32 with its 4
  lowest mantissa bits dropped. On one receive ID, DLC 4 carries the first
  motor (M1) and DLC 8 M1 in bytes 0-3 and the second motor (M2) in bytes
  4-7; a half without the enable bit is skipped by the board (that motor
  keeps its command). A motor alone on its ID is M1 and takes DLC 4 only.
* Encoder feedback (Smart Driver motors, Motor Controller channels, Sensor
  encoders, Absolute Encoder): float32 position per encoder. Two encoders
  sharing an ID send one DLC 8 frame (lower-numbered encoder first);
  otherwise DLC 4. Speed is not sent.

Every function returns plain values so the formats can be tested without
hardware.
"""

import math
import struct

# Motor modes (bits 2..0 of the command word).
MODE_VOLTAGE = 0
MODE_POSITION = 1
MODE_SPEED = 2
MODE_POSITION_RAMP = 3  # goal = ramp rate, until the board restarts
MODE_SPEED_RAMP = 4
MODE_VOLTAGE_RAMP = 5
MODES = (MODE_VOLTAGE, MODE_POSITION, MODE_SPEED,
         MODE_POSITION_RAMP, MODE_SPEED_RAMP, MODE_VOLTAGE_RAMP)
COMMAND_ENABLE = 0x8
DISABLED_WORD = bytes(4)  # a half the board skips
MAX_STANDARD_ID = 0x7FF


def motor_word(mode, goal):
    """The 4 enabled command bytes for one motor."""
    if mode not in MODES:
        raise ValueError(f"unknown motor mode {mode}")
    goal = float(goal)
    if not math.isfinite(goal):
        raise ValueError("goal must be finite")
    bits = struct.unpack("<I", struct.pack("<f", goal))[0]
    return struct.pack("<I", (bits & ~0xF & 0xFFFFFFFF) | COMMAND_ENABLE | mode)


def decode_motor_word(data):
    """(enabled, mode, goal) from 4 command bytes, as the board reads them."""
    word = struct.unpack_from("<I", bytes(data), 0)[0]
    goal = struct.unpack("<f", struct.pack("<I", word & ~0xF & 0xFFFFFFFF))[0]
    return bool(word & COMMAND_ENABLE), word & 0x7, goal


def motor_command(m1_enable, mode1, goal1, m2_enable=False, mode2=0, goal2=0.0):
    """Frame data for the motors on one receive ID: DLC 4 for M1 alone,
    DLC 8 when M2 is enabled (M1's half disabled unless m1_enable).
    ValueError when neither is enabled or an enabled half is invalid."""
    if not m1_enable and not m2_enable:
        raise ValueError("neither motor is enabled")
    first = motor_word(mode1, goal1) if m1_enable else DISABLED_WORD
    if not m2_enable:
        return first
    return first + motor_word(mode2, goal2)


def encoder_positions(data):
    """Positions in a feedback frame: one for DLC 4, two for DLC 8."""
    data = bytes(data)
    if len(data) == 4:
        return [struct.unpack_from("<f", data, 0)[0]]
    if len(data) == 8:
        return list(struct.unpack_from("<ff", data, 0))
    raise ValueError(f"encoder feedback must be 4 or 8 bytes, got {len(data)}")


def sensor_inputs(data):
    """Sensor digital/analog frame: 5 analog values 0..1 (12-bit) and 4
    digital inputs."""
    d = bytes(data)
    if len(d) != 8:
        raise ValueError("sensor input frame must be 8 bytes")
    raw = [
        (d[0] << 4) | (d[1] >> 4),
        ((d[1] & 0x0F) << 8) | d[2],
        (d[3] << 4) | (d[4] >> 4),
        ((d[4] & 0x0F) << 8) | d[5],
        (d[6] << 4) | (d[7] >> 4),
    ]
    analog = [value / 4095.0 for value in raw]
    digital = [bool((d[7] >> i) & 1) for i in range(4)]
    return analog, digital


def _clamp01(value):
    value = float(value)
    if not math.isfinite(value):
        return 0.0
    return min(max(value, 0.0), 1.0)


def _controller_values(kind, values):
    if len(values) != 4:
        raise ValueError("four values are required")
    data = bytearray(8)
    for i, value in enumerate(values):
        raw = int(round(_clamp01(value) * 16383.0))
        data[2 * i] = raw >> 8
        data[2 * i + 1] = raw & 0xFF
    data[0] |= kind << 6
    return bytes(data)


def controller_pwm(values):
    """Controller PWM 1-4 duty (0..1)."""
    return _controller_values(2, values)


def controller_servo(values):
    """Controller servo 1-4 position (0..1 of the pulse range)."""
    return _controller_values(3, values)


def controller_digital(digital, solenoid):
    """Controller digital outputs 1-4 and solenoids 1-6."""
    if len(digital) != 4 or len(solenoid) != 6:
        raise ValueError("4 digital and 6 solenoid values are required")
    data = bytearray(8)
    data[0] = 1 << 6
    for i, on in enumerate(digital):
        data[1] |= int(bool(on)) << i
    for i, on in enumerate(solenoid):
        data[2] |= int(bool(on)) << i
    return bytes(data)


def swerve_command(position, speed):
    """Swerve module target: steering position (rad), wheel speed (rad/s)."""
    return struct.pack("<ff", float(position), float(speed))


def swerve_feedback(data):
    """Swerve module feedback: (position, speed)."""
    return struct.unpack_from("<ff", bytes(data), 0)


def imu_quaternion(data):
    """IMU board quaternion (w, x, y, z), sign-magnitude 16-bit each."""
    d = bytes(data)
    values = []
    for i in range(4):
        hi, lo = d[2 * i], d[2 * i + 1]
        magnitude = ((hi & 0x7F) << 8 | lo) / 32767.0
        values.append(-magnitude if hi & 0x80 else magnitude)
    return tuple(values)


class SpeedEstimator:
    """Speed from successive positions (the boards no longer send it).

    A plain difference over the receive time, optionally smoothed with
    exponential averaging (alpha = weight of the previous estimate)."""

    def __init__(self, alpha=0.0):
        self.alpha = alpha
        self._last = {}

    def update(self, key, position, time_s):
        last = self._last.get(key)
        speed = 0.0
        if last is not None:
            dt = time_s - last[1]
            if dt > 0.0:
                raw = (position - last[0]) / dt
                speed = self.alpha * last[2] + (1.0 - self.alpha) * raw
            else:
                speed = last[2]
        self._last[key] = (position, time_s, speed)
        return speed
