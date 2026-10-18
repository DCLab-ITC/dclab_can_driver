// ROS 2 node bridging a SocketCAN bus and the DCLab boards (plus VESC and
// RoboMaster motors): the C++ build of can_driver, with the same node name,
// topics and parameters (README.md), so either one can run.
//
// One thread owns the bus: it opens it (again every 2 s while it is
// missing), takes every frame the moment the kernel has it (with the
// kernel's receive time, for the speed estimate) and publishes it. ROS
// callbacks send on the same socket. DCLab Link (dclab_link.py, Python) runs
// as before, as a separate low-priority process.

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fstream>
#include <memory>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "can_driver_cpp/dclab_frames.hpp"
#include "canable_reclaim.hpp"
#include "custom_messages/msg/dc_lab_motor_command.hpp"
#include "custom_messages/msg/digital_and_analog_feedback.hpp"
#include "custom_messages/msg/digital_and_solenoid_command.hpp"
#include "custom_messages/msg/encoder_feedback.hpp"
#include "custom_messages/msg/pwm_command.hpp"
#include "custom_messages/msg/robomaster_current_command.hpp"
#include "custom_messages/msg/robomaster_feedback.hpp"
#include "custom_messages/msg/servo_command.hpp"
#include "custom_messages/msg/swerve_command.hpp"
#include "custom_messages/msg/swerve_feedback.hpp"
#include "custom_messages/msg/vesc_command.hpp"
#include "custom_messages/msg/vesc_status_four.hpp"
#include "custom_messages/msg/vesc_status_one.hpp"
#include "custom_messages/msg/vesc_status_three.hpp"
#include "custom_messages/msg/vesc_status_two.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/rclcpp.hpp"

extern char ** environ;

namespace msg = custom_messages::msg;
using namespace std::chrono_literals;

namespace
{
using Ranges = std::vector<std::pair<int64_t, int64_t>>;

bool in_ranges(const Ranges & ranges, uint32_t id)
{
  for (const auto & range : ranges) {
    if (range.first <= id && id <= range.second) {return true;}
  }
  return false;
}

bool exists(const std::string & path)
{
  struct stat st;
  return stat(path.c_str(), &st) == 0;
}

double monotonic_s()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Starts argv; quiet = no output; returns the pid or -1.
pid_t spawn(const std::vector<std::string> & args, bool quiet)
{
  std::vector<char *> argv;
  for (const auto & arg : args) {argv.push_back(const_cast<char *>(arg.c_str()));}
  argv.push_back(nullptr);
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_addopen(&actions, 0, "/dev/null", O_RDONLY, 0);
  if (quiet) {
    posix_spawn_file_actions_addopen(&actions, 1, "/dev/null", O_WRONLY, 0);
    posix_spawn_file_actions_addopen(&actions, 2, "/dev/null", O_WRONLY, 0);
  }
  pid_t pid = -1;
  if (posix_spawnp(&pid, argv[0], &actions, nullptr, argv.data(), environ) != 0) {pid = -1;}
  posix_spawn_file_actions_destroy(&actions);
  return pid;
}

// Python's returncode: the exit status, or -signal
int return_code(int status)
{
  return WIFEXITED(status) ? WEXITSTATUS(status) : -WTERMSIG(status);
}
}  // namespace

class CanDriver : public rclcpp::Node
{
public:
  CanDriver()
  : Node("can_driver")
  {
    channel_ = declare_parameter<std::string>("channel", "can0");
    bitrate_ = declare_parameter<int64_t>("bitrate", 1000000);
    // Bring the interface up with `sudo ip link` when it is down (needs
    // passwordless sudo for `ip`); set false when the system does it.
    configure_interface_ = declare_parameter<bool>("configure_interface", true);
    // A CANable the Super App's Linux USB rule took is taken back when the
    // interface is missing (canable_reclaim.cpp).
    take_canable_ = declare_parameter<bool>("take_canable", true);
    log_rates_ = declare_parameter<bool>("log_rates", false);
    superapp_link_ = declare_parameter<bool>("superapp_link", true);
    superapp_link_lan_ = declare_parameter<bool>("superapp_link_lan", true);
    encoder_ranges_ = ranges("encoder_ids", {100, 199, 250, 299});
    speed_ = dclab::SpeedEstimator(declare_parameter<double>("speed_filter_alpha", 0.0));
    sensor_ranges_ = ranges("sensor_input_ids", {500, 510});
    swerve_ranges_ = ranges("swerve_feedback_ids", {0, -1});
    imu_id_ = declare_parameter<int64_t>("imu_id", 1000);

    encoder_pub_ = create_publisher<msg::EncoderFeedback>("/encoder_feedback", 10);
    sensor_pub_ = create_publisher<msg::DigitalAndAnalogFeedback>("/digital_analog_feedback", 10);
    swerve_pub_ = create_publisher<msg::SwerveFeedback>("/swerve_feedback", 10);
    imu_pub_ = create_publisher<geometry_msgs::msg::Quaternion>("/imu/quaternion", 10);
    robomaster_pub_ = create_publisher<msg::RobomasterFeedback>("/robomaster_feedback", 10);
    vesc1_pub_ = create_publisher<msg::VescStatusOne>("/vesc_status1", 10);
    vesc2_pub_ = create_publisher<msg::VescStatusTwo>("/vesc_status2", 10);
    vesc3_pub_ = create_publisher<msg::VescStatusThree>("/vesc_status3", 10);
    vesc4_pub_ = create_publisher<msg::VescStatusFour>("/vesc_status4", 10);

    subscriptions_.push_back(create_subscription<msg::DCLabMotorCommand>(
        "/dclab/motor_command", 10, [this](const msg::DCLabMotorCommand & m) {motor_command(m);}));
    subscriptions_.push_back(create_subscription<msg::PwmCommand>(
        "/publish_pwm", 10, [this](const msg::PwmCommand & m) {
          const float v[4] = {m.pwm1_value, m.pwm2_value, m.pwm3_value, m.pwm4_value};
          send(m.can_id, dclab::controller_pwm(v));
        }));
    subscriptions_.push_back(create_subscription<msg::ServoCommand>(
        "/publish_servo", 10, [this](const msg::ServoCommand & m) {
          const float v[4] = {m.servo1_value, m.servo2_value, m.servo3_value, m.servo4_value};
          send(m.can_id, dclab::controller_servo(v));
        }));
    subscriptions_.push_back(create_subscription<msg::DigitalAndSolenoidCommand>(
        "/publish_digital_solenoid", 10, [this](const msg::DigitalAndSolenoidCommand & m) {
          const bool d[4] = {m.digital1_value, m.digital2_value, m.digital3_value, m.digital4_value};
          const bool s[6] = {m.solenoid1_value, m.solenoid2_value, m.solenoid3_value,
            m.solenoid4_value, m.solenoid5_value, m.solenoid6_value};
          send(m.can_id, dclab::controller_digital(d, s));
        }));
    subscriptions_.push_back(create_subscription<msg::SwerveCommand>(
        "/publish_swerve", 10, [this](const msg::SwerveCommand & m) {
          send(m.can_id, dclab::swerve_command(m.position, m.speed));
        }));
    subscriptions_.push_back(create_subscription<msg::RobomasterCurrentCommand>(
        "/publish_robomaster_current", 10, [this](const msg::RobomasterCurrentCommand & m) {
          const float c[4] = {m.current1, m.current2, m.current3, m.current4};
          const uint8_t t[4] = {m.type1, m.type2, m.type3, m.type4};
          send(m.can_id, dclab::robomaster_current(c, t));
        }));
    subscriptions_.push_back(create_subscription<msg::VescCommand>(
        "/publish_vesc", 10, [this](const msg::VescCommand & m) {vesc_command(m);}));

    link_script_ = ament_index_cpp::get_package_prefix("can_driver_cpp") + "/lib/can_driver_cpp/dclab_link.py";
    bus_thread_ = std::thread(&CanDriver::bus_loop, this);
    rates_timer_ = create_wall_timer(1s, [this] {report_rates();});
    link_timer_ = create_wall_timer(2s, [this] {ensure_link();});
  }

  ~CanDriver() override
  {
    running_ = false;
    if (bus_thread_.joinable()) {bus_thread_.join();}
    stop_link();
    close_bus();
  }

private:
  Ranges ranges(const std::string & name, const std::vector<int64_t> & fallback)
  {
    const auto values = declare_parameter<std::vector<int64_t>>(name, fallback);
    if (values.size() % 2) {throw std::invalid_argument("parameter " + name + " needs [first, last] pairs");}
    Ranges out;
    for (size_t i = 0; i < values.size(); i += 2) {out.emplace_back(values[i], values[i + 1]);}
    return out;
  }

  // --- bus ---
  bool open_bus()
  {
    const std::string sys = "/sys/class/net/" + channel_;
    if (take_canable_ && !exists(sys) &&
      dclab::reclaim_canables([this](const std::string & m) {RCLCPP_WARN(get_logger(), "%s", m.c_str());}))
    {
      for (int i = 0; i < 20 && !exists(sys); ++i) {std::this_thread::sleep_for(100ms);}  // the kernel adds it
    }
    if (configure_interface_ && exists(sys)) {
      std::ifstream flags_file(sys + "/flags");
      unsigned long flags = 0;
      flags_file >> std::hex >> flags;
      if (!(flags & IFF_UP)) {
        const pid_t pid = spawn(
          {"sudo", "-n", "ip", "link", "set", channel_, "up", "type", "can", "bitrate",
            std::to_string(bitrate_)}, true);
        int status;
        if (pid > 0) {waitpid(pid, &status, 0);}
      }
    }
    const int fd = socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
    ifreq ifr{};
    std::strncpy(ifr.ifr_name, channel_.c_str(), IFNAMSIZ - 1);
    sockaddr_can address{};
    address.can_family = AF_CAN;
    const char * failed = nullptr;
    if (fd < 0) {
      failed = "socket";
    } else if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
      failed = "interface";
    } else {
      address.can_ifindex = ifr.ifr_ifindex;
      if (bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {failed = "bind";}
    }
    if (failed) {
      const int error = errno;
      if (fd >= 0) {close(fd);}
      const double now = monotonic_s();
      if (now - open_warned_ >= 30.0) {  // the 2 s retries, reported now and then
        open_warned_ = now;
        RCLCPP_WARN(get_logger(), "CAN %s not available: %s", channel_.c_str(), std::strerror(error));
      }
      return false;
    }
    const int on = 1, buffer = 1 << 20;
    setsockopt(fd, SOL_SOCKET, SO_TIMESTAMPNS, &on, sizeof(on));
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &buffer, sizeof(buffer));
    {
      std::unique_lock<std::shared_mutex> lock(fd_mutex_);
      fd_ = fd;
    }
    bus_open_ = true;
    open_warned_ = 0.0;
    RCLCPP_INFO(get_logger(), "CAN %s open", channel_.c_str());
    return true;
  }

  void close_bus()
  {
    std::unique_lock<std::shared_mutex> lock(fd_mutex_);
    if (fd_ >= 0) {close(fd_);}
    fd_ = -1;
    bus_open_ = false;
  }

  void bus_loop()
  {
    while (running_) {
      if (!bus_open_ && !open_bus()) {
        for (int i = 0; i < 20 && running_; ++i) {std::this_thread::sleep_for(100ms);}
        continue;
      }
      if (reopen_.exchange(false)) {
        close_bus();
        continue;
      }
      pollfd p{fd_, POLLIN, 0};
      if (poll(&p, 1, 200) <= 0) {continue;}
      for (;;) {  // everything waiting, then poll again
        can_frame frame;
        alignas(cmsghdr) char control[CMSG_SPACE(sizeof(timespec))];
        iovec io{&frame, sizeof(frame)};
        msghdr m{};
        m.msg_iov = &io;
        m.msg_iovlen = 1;
        m.msg_control = control;
        m.msg_controllen = sizeof(control);
        const ssize_t n = recvmsg(fd_, &m, MSG_DONTWAIT);
        if (n < 0) {
          if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            RCLCPP_ERROR(get_logger(), "CAN receive error: %s", std::strerror(errno));
            close_bus();
          }
          break;
        }
        if (n != sizeof(frame)) {continue;}
        double time_s = 0.0;
        for (cmsghdr * c = CMSG_FIRSTHDR(&m); c; c = CMSG_NXTHDR(&m, c)) {
          if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SO_TIMESTAMPNS) {
            timespec ts;
            std::memcpy(&ts, CMSG_DATA(c), sizeof(ts));
            time_s = ts.tv_sec + ts.tv_nsec * 1e-9;
          }
        }
        if (time_s == 0.0) {time_s = now().nanoseconds() * 1e-9;}
        try {
          receive(frame, time_s);
        } catch (const std::exception & error) {  // publishing while ROS shuts down
          if (!rclcpp::ok()) {return;}
          RCLCPP_ERROR(get_logger(), "%s", error.what());
        }
      }
    }
  }

  // Sends one frame; drops it (with a warning) when the bus is down or the
  // transmit queue is full.
  bool send(uint32_t id, const dclab::Payload & payload, bool extended = false)
  {
    can_frame frame{};
    frame.can_id = extended ? ((id & CAN_EFF_MASK) | CAN_EFF_FLAG) : (id & CAN_SFF_MASK);
    frame.can_dlc = payload.len;
    std::memcpy(frame.data, payload.data, payload.len);
    int error = 0;
    {
      std::shared_lock<std::shared_mutex> lock(fd_mutex_);
      if (fd_ < 0) {return false;}
      if (::send(fd_, &frame, sizeof(frame), MSG_DONTWAIT) == static_cast<ssize_t>(sizeof(frame))) {
        ++transmit_count_;
        return true;
      }
      error = errno;
    }
    if (error == ENOBUFS || error == EAGAIN) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "CAN transmit queue full, frame dropped");
    } else {
      RCLCPP_ERROR(get_logger(), "CAN send failed: %s", std::strerror(error));
      const double now = monotonic_s();
      if (now - last_recovery_ > 1.0) {
        last_recovery_ = now;
        reopen_ = true;
      }
    }
    return false;
  }

  // --- received frames, as the Python node's device modules in order ---
  void receive(const can_frame & frame, double time_s)
  {
    ++receive_count_;
    if (frame.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG)) {return;}
    const bool extended = frame.can_id & CAN_EFF_FLAG;
    const uint32_t id = frame.can_id & (extended ? CAN_EFF_MASK : CAN_SFF_MASK);
    const uint8_t len = frame.can_dlc > 8 ? 8 : frame.can_dlc;
    const uint8_t * d = frame.data;
    if (!extended && in_ranges(encoder_ranges_, id) && (len == 4 || len == 8)) {
      float positions[2];
      const int count = dclab::encoder_positions(d, len, positions);
      for (int slot = 0; slot < count; ++slot) {
        msg::EncoderFeedback out;
        out.can_id = static_cast<uint16_t>(id);
        out.slot = static_cast<uint8_t>(slot);
        out.position = positions[slot];
        out.speed = static_cast<float>(speed_.update(id, slot, positions[slot], time_s));
        encoder_pub_->publish(out);
      }
      return;
    }
    if (!extended && len == 8 && in_ranges(sensor_ranges_, id)) {
      float analog[5];
      bool digital[4];
      dclab::sensor_inputs(d, analog, digital);
      msg::DigitalAndAnalogFeedback out;
      out.can_id = static_cast<uint16_t>(id);
      out.analog1_value = analog[0];
      out.analog2_value = analog[1];
      out.analog3_value = analog[2];
      out.analog4_value = analog[3];
      out.analog5_value = analog[4];
      out.digital1_value = digital[0];
      out.digital2_value = digital[1];
      out.digital3_value = digital[2];
      out.digital4_value = digital[3];
      sensor_pub_->publish(out);
      return;
    }
    if (!extended && len == 8 && in_ranges(swerve_ranges_, id)) {
      msg::SwerveFeedback out;
      out.can_id = static_cast<uint16_t>(id);
      out.position = dclab::bits_float(dclab::get_le32(d));
      out.speed = dclab::bits_float(dclab::get_le32(d + 4));
      swerve_pub_->publish(out);
      return;
    }
    if (!extended && id == imu_id_ && len == 8) {
      double q[4];
      dclab::imu_quaternion(d, q);
      geometry_msgs::msg::Quaternion out;
      out.w = q[0];
      out.x = q[1];
      out.y = q[2];
      out.z = q[3];
      imu_pub_->publish(out);
      return;
    }
    if (!extended && len == 8 && id >= 0x201 && id <= 0x208) {
      msg::RobomasterFeedback out;
      out.motor_id = static_cast<uint16_t>(id - 0x200);
      out.position = dclab::get_be16s(d);
      out.speed = dclab::get_be16s(d + 2);
      out.current = static_cast<float>(dclab::get_be16s(d + 4) * 0.001);
      out.temp = d[6];
      robomaster_pub_->publish(out);
      return;
    }
    if (extended && len == 8 && vesc_status(id >> 8, static_cast<uint16_t>(id & 0xFF), d)) {return;}
    ++unhandled_count_;
  }

  bool vesc_status(uint32_t packet, uint16_t esc, const uint8_t * d)
  {
    const auto be32 = [](const uint8_t * p) {return static_cast<int32_t>(dclab::get_be32(p));};
    if (packet == 9) {
      msg::VescStatusOne out;
      out.esc_id = esc;
      out.rpm = static_cast<float>(be32(d));
      out.current = static_cast<float>(dclab::get_be16s(d + 4) * 0.1);
      out.duty = static_cast<float>(dclab::get_be16s(d + 6) * 0.001);
      vesc1_pub_->publish(out);
    } else if (packet == 14) {
      msg::VescStatusTwo out;
      out.esc_id = esc;
      out.amp_hours = static_cast<float>(be32(d) * 1e-4);
      out.amp_hours_charged = static_cast<float>(be32(d + 4) * 1e-4);
      vesc2_pub_->publish(out);
    } else if (packet == 15) {
      msg::VescStatusThree out;
      out.esc_id = esc;
      out.watt_hours = static_cast<float>(be32(d) * 1e-4);
      out.watt_hours_charged = static_cast<float>(be32(d + 4) * 1e-4);
      vesc3_pub_->publish(out);
    } else if (packet == 16) {
      msg::VescStatusFour out;
      out.esc_id = esc;
      out.fet_temp = static_cast<float>(dclab::get_be16s(d) * 0.1);
      out.motor_temp = static_cast<float>(dclab::get_be16s(d + 2) * 0.1);
      out.input_current = static_cast<float>(dclab::get_be16s(d + 4) * 0.1);
      out.pid_position = static_cast<float>(dclab::get_be16s(d + 6) * 0.02);
      vesc4_pub_->publish(out);
    } else {
      return false;
    }
    return true;
  }

  // --- commands ---
  void motor_command(const msg::DCLabMotorCommand & m)
  {
    dclab::Payload payload;
    const char * error = m.can_id > dclab::MAX_STANDARD_ID ? "can_id is above 0x7FF" :
      dclab::motor_command(m.m1_enable, m.mode1, m.goal1, m.m2_enable, m.mode2, m.goal2, payload);
    if (error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "motor command on %u refused: %s", m.can_id, error);
      return;
    }
    send(m.can_id, payload);
  }

  void vesc_command(const msg::VescCommand & m)
  {
    const int packet = m.pwm_mode ? 0 : m.current_mode ? 1 : m.current_brake_mode ? 2 :
      m.speed_mode ? 3 : m.position_mode ? 4 : -1;
    if (packet < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "VescCommand without a mode ignored");
      return;
    }
    dclab::Payload payload;
    if (!dclab::vesc_command(packet, m.goal, payload)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "VescCommand goal must be finite");
      return;
    }
    send((static_cast<uint32_t>(packet) << 8) | (m.vesc_id & 0xFF), payload, true);
  }

  void report_rates()
  {
    const auto tx = transmit_count_.exchange(0), rx = receive_count_.exchange(0);
    const auto unhandled = unhandled_count_.exchange(0);
    if (log_rates_) {
      RCLCPP_INFO(
        get_logger(), "CAN tx %llu/s rx %llu/s (unhandled %llu)", static_cast<unsigned long long>(tx),
        static_cast<unsigned long long>(rx), static_cast<unsigned long long>(unhandled));
    }
  }

  // --- DCLab Link (Super App's Robot connection) ---
  // Started once the bus is open and restarted if it stops; one that stops
  // at once (port 48791 taken, interface down) is retried less and less
  // often (30 s up to 10 min) and reported once.
  void ensure_link()
  {
    if (!superapp_link_ || !bus_open_) {return;}
    const double now = monotonic_s();
    if (link_pid_ > 0) {
      int status;
      if (waitpid(link_pid_, &status, WNOHANG) == 0) {return;}
      if (now - link_started_ < 10.0) {
        if (!link_warned_) {
          RCLCPP_WARN(
            get_logger(), "DCLab Link did not start (exit %d); retrying now and then", return_code(status));
          link_warned_ = true;
        }
        link_backoff_ = std::min(link_backoff_ * 2.0, 600.0);
      } else {
        RCLCPP_WARN(get_logger(), "DCLab Link stopped (exit %d)", return_code(status));
        link_backoff_ = 30.0;
      }
      link_pid_ = -1;
    }
    if (link_started_ > 0.0 && now - link_started_ < link_backoff_) {return;}
    link_started_ = now;
    if (!exists(link_script_)) {
      RCLCPP_WARN(get_logger(), "DCLab Link not started: %s is missing", link_script_.c_str());
      superapp_link_ = false;
      return;
    }
    link_pid_ = spawn(
      {"python3", link_script_, "--can", channel_, "--parent", std::to_string(getpid()),
        "--listen", superapp_link_lan_ ? "0.0.0.0" : "127.0.0.1"}, false);
    if (link_pid_ < 0) {RCLCPP_WARN(get_logger(), "DCLab Link not started: %s", std::strerror(errno));}
  }

  void stop_link()
  {
    if (link_pid_ <= 0) {return;}
    kill(link_pid_, SIGTERM);
    int status;
    for (int i = 0; i < 20; ++i) {
      if (waitpid(link_pid_, &status, WNOHANG) != 0) {
        link_pid_ = -1;
        return;
      }
      std::this_thread::sleep_for(100ms);
    }
    kill(link_pid_, SIGKILL);
    waitpid(link_pid_, &status, 0);
    link_pid_ = -1;
  }

  std::string channel_;
  int64_t bitrate_ = 1000000;
  bool configure_interface_ = true, take_canable_ = true, log_rates_ = false;
  bool superapp_link_ = true, superapp_link_lan_ = true;
  Ranges encoder_ranges_, sensor_ranges_, swerve_ranges_;
  int64_t imu_id_ = 1000;
  dclab::SpeedEstimator speed_;

  std::shared_mutex fd_mutex_;  // send() shares the socket, the bus thread closes it
  int fd_ = -1;
  std::atomic<bool> bus_open_{false}, reopen_{false}, running_{true};
  std::thread bus_thread_;
  double open_warned_ = 0.0, last_recovery_ = 0.0;
  std::atomic<uint64_t> transmit_count_{0}, receive_count_{0}, unhandled_count_{0};

  std::string link_script_;
  pid_t link_pid_ = -1;
  double link_started_ = 0.0, link_backoff_ = 30.0;
  bool link_warned_ = false;

  rclcpp::Publisher<msg::EncoderFeedback>::SharedPtr encoder_pub_;
  rclcpp::Publisher<msg::DigitalAndAnalogFeedback>::SharedPtr sensor_pub_;
  rclcpp::Publisher<msg::SwerveFeedback>::SharedPtr swerve_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr imu_pub_;
  rclcpp::Publisher<msg::RobomasterFeedback>::SharedPtr robomaster_pub_;
  rclcpp::Publisher<msg::VescStatusOne>::SharedPtr vesc1_pub_;
  rclcpp::Publisher<msg::VescStatusTwo>::SharedPtr vesc2_pub_;
  rclcpp::Publisher<msg::VescStatusThree>::SharedPtr vesc3_pub_;
  rclcpp::Publisher<msg::VescStatusFour>::SharedPtr vesc4_pub_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr rates_timer_, link_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    auto node = std::make_shared<CanDriver>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("can_driver"), "%s", error.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
