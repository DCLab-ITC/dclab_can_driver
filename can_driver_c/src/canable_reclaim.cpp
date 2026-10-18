#include "canable_reclaim.hpp"

#include <dirent.h>
#include <fcntl.h>
#include <linux/usbdevice_fs.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>

namespace dclab
{
namespace
{
const char * const USB_DEVICES = "/sys/bus/usb/devices/";

std::string read_line(const std::string & path)
{
  std::ifstream file(path);
  std::string line;
  std::getline(file, line);
  return line;
}

// "" when no driver has the interface
std::string driver_of(const std::string & interface)
{
  char target[512];
  const ssize_t n = readlink((interface + "/driver").c_str(), target, sizeof(target) - 1);
  if (n <= 0) {return "";}
  target[n] = '\0';
  const char * slash = std::strrchr(target, '/');
  return slash ? slash + 1 : target;
}

bool usb_request(int fd, int ifno, int code)
{
  usbdevfs_ioctl request{};
  request.ifno = ifno;
  request.ioctl_code = code;
  request.data = nullptr;
  return ioctl(fd, USBDEVFS_IOCTL, &request) >= 0;
}
}  // namespace

int reclaim_canables(const std::function<void(const std::string &)> & log)
{
  std::vector<std::string> names;
  if (DIR * dir = opendir(USB_DEVICES)) {
    while (dirent * entry = readdir(dir)) {names.emplace_back(entry->d_name);}
    closedir(dir);
  }
  std::sort(names.begin(), names.end());
  int count = 0;
  for (const auto & name : names) {
    if (name[0] == '.' || name.find(':') != std::string::npos) {continue;}
    const std::string device = USB_DEVICES + name;
    const std::string id = read_line(device + "/idVendor") + ":" + read_line(device + "/idProduct");
    if (id != "1d50:606f" && id != "1209:2323") {continue;}
    std::string config = read_line(device + "/bConfigurationValue");
    if (config.empty()) {config = "1";}
    const std::string interface = device + "/" + name + ":" + config + ".0";
    if (read_line(interface + "/bInterfaceClass") != "ff") {continue;}
    const std::string driver = driver_of(interface);
    if (driver == "gs_usb") {continue;}
    char node[32];
    std::snprintf(
      node, sizeof(node), "/dev/bus/usb/%03d/%03d",
      std::atoi(read_line(device + "/busnum").c_str()),
      std::atoi(read_line(device + "/devnum").c_str()));
    const int ifno = static_cast<int>(std::strtol(read_line(interface + "/bInterfaceNumber").c_str(), nullptr, 16));
    const int fd = open(node, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
      log(std::string("CANable ") + node + " not reclaimed: " + std::strerror(errno) +
        " (is the Super App's USB rule installed?)");
      continue;
    }
    // usbfs: a browser (Super App) has it
    const bool ok = (driver.empty() || usb_request(fd, ifno, USBDEVFS_DISCONNECT)) &&
      usb_request(fd, ifno, USBDEVFS_CONNECT);
    const int error = errno;
    close(fd);
    if (ok) {
      ++count;
      log(std::string("CANable ") + node + " given back to Linux's CAN driver" +
        (driver == "usbfs" ? " (taken from the browser)" : ""));
    } else {
      log(std::string("CANable ") + node + " not reclaimed: " + std::strerror(error));
    }
  }
  return count;
}

}  // namespace dclab
