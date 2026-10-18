// The C++ frames give the Python driver's bytes and values
// (frame_vectors.inc, from make_vectors.py). Runs without ROS:
//   g++ -std=c++17 -Iinclude test/test_frames.cpp && ./a.out
#include <cmath>
#include <cstdio>
#include <cstring>

#include "can_driver_cpp/dclab_frames.hpp"

struct MotorVector
{
  bool m1e; int mode1; float g1; bool m2e; int mode2; float g2; int ok; int len; unsigned char data[8];
};
struct ControllerVector {int kind; float values[4]; unsigned char data[8];};
struct DigitalVector {bool digital[4]; bool solenoid[6]; unsigned char data[8];};
struct RobomasterVector {float current[4]; unsigned char types[4]; unsigned char data[8];};
struct VescVector {int packet; float goal; unsigned char data[4];};
struct DecodeVector
{
  unsigned char data[8]; float analog[5]; bool digital[4]; double q[4]; float pos[2];
};
struct SpeedVector {unsigned can_id; int slot; float position; double time; float speed;};

#include "frame_vectors.inc"

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { ++failures; std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

static bool same(double a, double b) {return a == b || (std::isnan(a) && std::isnan(b));}

int main()
{
  int i = 0;
  for (const auto & v : motor_vectors) {
    dclab::Payload p;
    const char * error = dclab::motor_command(v.m1e, v.mode1, v.g1, v.m2e, v.mode2, v.g2, p);
    CHECK((error == nullptr) == (v.ok == 1), "motor %d: refused %s", i, error ? error : "-");
    if (!error && v.ok) {CHECK(p.len == v.len && !std::memcmp(p.data, v.data, p.len), "motor %d: bytes", i);}
    ++i;
  }
  i = 0;
  for (const auto & v : controller_vectors) {
    const auto p = v.kind == 2 ? dclab::controller_pwm(v.values) : dclab::controller_servo(v.values);
    CHECK(!std::memcmp(p.data, v.data, 8), "controller %d: bytes", i);
    ++i;
  }
  i = 0;
  for (const auto & v : digital_vectors) {
    CHECK(!std::memcmp(dclab::controller_digital(v.digital, v.solenoid).data, v.data, 8), "digital %d", i);
    ++i;
  }
  i = 0;
  for (const auto & v : robomaster_vectors) {
    CHECK(!std::memcmp(dclab::robomaster_current(v.current, v.types).data, v.data, 8), "robomaster %d", i);
    ++i;
  }
  i = 0;
  for (const auto & v : vesc_vectors) {
    dclab::Payload p;
    CHECK(dclab::vesc_command(v.packet, v.goal, p) && p.len == 4 && !std::memcmp(p.data, v.data, 4), "vesc %d", i);
    ++i;
  }
  i = 0;
  for (const auto & v : decode_vectors) {
    float analog[5], pos[2];
    bool digital[4];
    double q[4];
    dclab::sensor_inputs(v.data, analog, digital);
    dclab::imu_quaternion(v.data, q);
    CHECK(dclab::encoder_positions(v.data, 8, pos) == 2, "decode %d: count", i);
    for (int k = 0; k < 5; ++k) {CHECK(analog[k] == v.analog[k], "decode %d: analog %d", i, k);}
    for (int k = 0; k < 4; ++k) {CHECK(digital[k] == v.digital[k] && q[k] == v.q[k], "decode %d: %d", i, k);}
    for (int k = 0; k < 2; ++k) {CHECK(same(pos[k], v.pos[k]), "decode %d: position %d", i, k);}
    ++i;
  }
  dclab::SpeedEstimator speed(0.3);
  i = 0;
  for (const auto & v : speed_vectors) {
    const float got = static_cast<float>(speed.update(v.can_id, v.slot, v.position, v.time));
    CHECK(got == v.speed, "speed %d: %g != %g", i, got, v.speed);
    ++i;
  }
  dclab::Payload p;
  CHECK(!dclab::vesc_command(1, NAN, p), "vesc NaN refused");
  CHECK(dclab::vesc_command(4, 1e30, p) && p.data[0] == 0x7F, "vesc clamps to int32");
  std::printf(failures ? "test_frames: %d failures\n" : "test_frames: all passed\n", failures);
  return failures ? 1 : 0;
}
