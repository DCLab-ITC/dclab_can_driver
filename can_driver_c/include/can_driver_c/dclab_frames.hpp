// CAN frame formats of the DCLab boards, VESC and RoboMaster, without ROS:
// the C++ twin of can_driver/dclab_frames.py (same bytes for the same
// values; test/test_frames.cpp compares them). See that file for the
// formats.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <unordered_map>

namespace dclab
{

enum MotorMode : uint8_t {
  MODE_VOLTAGE = 0,
  MODE_POSITION = 1,
  MODE_SPEED = 2,
  MODE_POSITION_RAMP = 3,  // goal = ramp rate, until the board restarts
  MODE_SPEED_RAMP = 4,
  MODE_VOLTAGE_RAMP = 5,
};
constexpr uint32_t COMMAND_ENABLE = 0x8;
constexpr uint32_t MAX_STANDARD_ID = 0x7FF;

struct Payload
{
  uint8_t len = 0;
  uint8_t data[8] = {};
};

inline uint32_t float_bits(float value)
{
  uint32_t bits;
  std::memcpy(&bits, &value, 4);
  return bits;
}

inline float bits_float(uint32_t bits)
{
  float value;
  std::memcpy(&value, &bits, 4);
  return value;
}

inline void put_le32(uint8_t * p, uint32_t v)
{
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
  p[2] = static_cast<uint8_t>(v >> 16);
  p[3] = static_cast<uint8_t>(v >> 24);
}

inline uint32_t get_le32(const uint8_t * p)
{
  return static_cast<uint32_t>(p[0]) | static_cast<uint32_t>(p[1]) << 8 |
         static_cast<uint32_t>(p[2]) << 16 | static_cast<uint32_t>(p[3]) << 24;
}

inline uint32_t get_be32(const uint8_t * p)
{
  return static_cast<uint32_t>(p[0]) << 24 | static_cast<uint32_t>(p[1]) << 16 |
         static_cast<uint32_t>(p[2]) << 8 | static_cast<uint32_t>(p[3]);
}

inline int16_t get_be16s(const uint8_t * p)
{
  return static_cast<int16_t>(static_cast<uint16_t>(p[0] << 8 | p[1]));
}

inline void put_be16s(uint8_t * p, int16_t v)
{
  const uint16_t u = static_cast<uint16_t>(v);
  p[0] = static_cast<uint8_t>(u >> 8);
  p[1] = static_cast<uint8_t>(u);
}

inline void put_be32s(uint8_t * p, int32_t v)
{
  const uint32_t u = static_cast<uint32_t>(v);
  p[0] = static_cast<uint8_t>(u >> 24);
  p[1] = static_cast<uint8_t>(u >> 16);
  p[2] = static_cast<uint8_t>(u >> 8);
  p[3] = static_cast<uint8_t>(u);
}

// The 4 enabled command bytes for one motor; nullptr, or why it is refused.
inline const char * motor_word(uint8_t mode, float goal, uint8_t * out)
{
  if (mode > MODE_VOLTAGE_RAMP) {return "unknown motor mode";}
  if (!std::isfinite(goal)) {return "goal must be finite";}
  put_le32(out, (float_bits(goal) & ~0xFu) | COMMAND_ENABLE | mode);
  return nullptr;
}

// The motors on one receive ID: DLC 4 for M1 alone, DLC 8 when M2 is
// enabled (M1's half disabled unless m1_enable).
inline const char * motor_command(
  bool m1_enable, uint8_t mode1, float goal1,
  bool m2_enable, uint8_t mode2, float goal2, Payload & out)
{
  out = Payload();
  if (!m1_enable && !m2_enable) {return "neither motor is enabled";}
  if (m1_enable) {
    const char * error = motor_word(mode1, goal1, out.data);
    if (error) {return error;}
  }
  out.len = 4;
  if (!m2_enable) {return nullptr;}
  out.len = 8;
  return motor_word(mode2, goal2, out.data + 4);
}

// Positions in a feedback frame: 1 for DLC 4, 2 for DLC 8, else 0.
inline int encoder_positions(const uint8_t * data, uint8_t len, float * out)
{
  if (len != 4 && len != 8) {return 0;}
  out[0] = bits_float(get_le32(data));
  if (len == 4) {return 1;}
  out[1] = bits_float(get_le32(data + 4));
  return 2;
}

// Sensor digital/analog frame (8 bytes): 5 analog 0..1, 4 digital inputs.
inline void sensor_inputs(const uint8_t * d, float * analog, bool * digital)
{
  const int raw[5] = {
    (d[0] << 4) | (d[1] >> 4), ((d[1] & 0x0F) << 8) | d[2],
    (d[3] << 4) | (d[4] >> 4), ((d[4] & 0x0F) << 8) | d[5],
    (d[6] << 4) | (d[7] >> 4),
  };
  for (int i = 0; i < 5; ++i) {analog[i] = static_cast<float>(raw[i] / 4095.0);}
  for (int i = 0; i < 4; ++i) {digital[i] = (d[7] >> i) & 1;}
}

inline double clamp01(double value)
{
  if (!std::isfinite(value)) {return 0.0;}
  return value < 0.0 ? 0.0 : (value > 1.0 ? 1.0 : value);
}

inline Payload controller_values(uint8_t kind, const float * values)
{
  Payload out;
  out.len = 8;
  for (int i = 0; i < 4; ++i) {
    // Python's round(): half to even, as nearbyint in the default mode
    const int raw = static_cast<int>(std::nearbyint(clamp01(values[i]) * 16383.0));
    out.data[2 * i] = static_cast<uint8_t>(raw >> 8);
    out.data[2 * i + 1] = static_cast<uint8_t>(raw);
  }
  out.data[0] |= static_cast<uint8_t>(kind << 6);
  return out;
}

inline Payload controller_pwm(const float * values) {return controller_values(2, values);}
inline Payload controller_servo(const float * values) {return controller_values(3, values);}

inline Payload controller_digital(const bool * digital, const bool * solenoid)
{
  Payload out;
  out.len = 8;
  out.data[0] = 1 << 6;
  for (int i = 0; i < 4; ++i) {out.data[1] |= static_cast<uint8_t>(digital[i] ? 1 << i : 0);}
  for (int i = 0; i < 6; ++i) {out.data[2] |= static_cast<uint8_t>(solenoid[i] ? 1 << i : 0);}
  return out;
}

inline Payload swerve_command(float position, float speed)
{
  Payload out;
  out.len = 8;
  put_le32(out.data, float_bits(position));
  put_le32(out.data + 4, float_bits(speed));
  return out;
}

// IMU quaternion (w, x, y, z), sign-magnitude 16-bit each.
inline void imu_quaternion(const uint8_t * d, double * wxyz)
{
  for (int i = 0; i < 4; ++i) {
    const double magnitude = (((d[2 * i] & 0x7F) << 8) | d[2 * i + 1]) / 32767.0;
    wxyz[i] = (d[2 * i] & 0x80) ? -magnitude : magnitude;
  }
}

// RoboMaster C610 (type 0, +-10 A) / C620 (type 1, +-20 A): 4 currents.
inline Payload robomaster_current(const float * currents, const uint8_t * types)
{
  Payload out;
  out.len = 8;
  for (int i = 0; i < 4; ++i) {
    const bool c620 = types[i] == 1;
    const double max_current = c620 ? 20.0 : 10.0, scale = c620 ? 819.2 : 1000.0;
    double value = std::isfinite(currents[i]) ? currents[i] : 0.0;
    value = value < -max_current ? -max_current : (value > max_current ? max_current : value);
    put_be16s(out.data + 2 * i, static_cast<int16_t>(value * scale));  // truncates, as int()
  }
  return out;
}

// VESC set command: packet 0 duty, 1 current, 2 brake, 3 ERPM, 4 position.
inline bool vesc_command(int packet, double goal, Payload & out)
{
  static const double scale[5] = {1e5, 1e3, 1e3, 1.0, 1e6};
  if (packet < 0 || packet > 4 || !std::isfinite(goal)) {return false;}
  if (packet == 0) {goal = goal < -1.0 ? -1.0 : (goal > 1.0 ? 1.0 : goal);}
  double value = std::trunc(goal * scale[packet]);
  const double lo = std::numeric_limits<int32_t>::min(), hi = std::numeric_limits<int32_t>::max();
  value = value < lo ? lo : (value > hi ? hi : value);
  out = Payload();
  out.len = 4;
  put_be32s(out.data, static_cast<int32_t>(value));
  return true;
}

// Speed from successive positions (the boards no longer send it): the
// difference over the receive time, optionally smoothed (alpha = weight of
// the previous estimate).
class SpeedEstimator
{
public:
  explicit SpeedEstimator(double alpha = 0.0)
  : alpha_(alpha) {}

  double update(uint32_t can_id, int slot, double position, double time_s)
  {
    const uint64_t key = static_cast<uint64_t>(can_id) << 1 | static_cast<uint64_t>(slot & 1);
    auto it = last_.find(key);
    double speed = 0.0;
    if (it != last_.end()) {
      const double dt = time_s - it->second.time;
      speed = dt > 0.0 ?
        alpha_ * it->second.speed + (1.0 - alpha_) * (position - it->second.position) / dt :
        it->second.speed;
    }
    last_[key] = {position, time_s, speed};
    return speed;
  }

private:
  struct Last
  {
    double position, time, speed;
  };
  double alpha_;
  std::unordered_map<uint64_t, Last> last_;
};

}  // namespace dclab
