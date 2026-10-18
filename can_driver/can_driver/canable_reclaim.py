"""Gives a CANable (candleLight) back to Linux's CAN driver, so it is can0
again.

The Super App's Linux setup (/etc/udev/rules.d/60-dclab-usb.rules) takes
the CANable from the kernel's gs_usb driver at every plug-in, so a browser
can use it; can0 is then missing. The node has priority over the browser
(owner 2026-10-18): when its interface is missing it hands every CANable
without gs_usb back to it, ending a browser's session on that adapter. The
same rule lets every user open the adapter, so no root is needed: these are
usbfs's own "disconnect driver" and "let kernel drivers bind" requests
(libusb's detach/attach kernel driver)."""

import ctypes
import fcntl
import os

CANABLE_IDS = {("1d50", "606f"), ("1209", "2323")}
USB_DEVICES = "/sys/bus/usb/devices"

# <linux/usbdevice_fs.h>
_IOC_WRITE, _IOC_READ = 1, 2


class _UsbdevfsIoctl(ctypes.Structure):
    _fields_ = [("ifno", ctypes.c_int), ("ioctl_code", ctypes.c_int),
                ("data", ctypes.c_void_p)]


def _ioc(direction, number, size):
    return (direction << 30) | (size << 16) | (ord("U") << 8) | number


USBDEVFS_IOCTL = _ioc(_IOC_READ | _IOC_WRITE, 18, ctypes.sizeof(_UsbdevfsIoctl))
USBDEVFS_DISCONNECT = _ioc(0, 22, 0)
USBDEVFS_CONNECT = _ioc(0, 23, 0)


def _read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return ""


def _driver(interface):
    link = os.path.join(interface, "driver")
    return os.path.basename(os.readlink(link)) if os.path.islink(link) else None


def released_canables(root=USB_DEVICES):
    """(device node, interface number, driver or None) of each CANable CAN
    interface (vendor class, interface 0) that gs_usb does not have."""
    found = []
    try:
        names = sorted(os.listdir(root))
    except OSError:
        return found
    for name in names:
        device = os.path.join(root, name)
        if ":" in name or (_read(device + "/idVendor"), _read(device + "/idProduct")) not in CANABLE_IDS:
            continue
        interface = os.path.join(device, "%s:%s.0" % (name, _read(device + "/bConfigurationValue") or "1"))
        if _read(interface + "/bInterfaceClass") != "ff":
            continue
        driver = _driver(interface)
        if driver == "gs_usb":
            continue
        node = "/dev/bus/usb/%03d/%03d" % (int(_read(device + "/busnum") or 0),
                                          int(_read(device + "/devnum") or 0))
        found.append((node, int(_read(interface + "/bInterfaceNumber") or "0", 16), driver))
    return found


def _request(fd, ifno, code):
    request = _UsbdevfsIoctl(ifno, code, None)
    fcntl.ioctl(fd, USBDEVFS_IOCTL, request)


def reclaim(log=None):
    """Hands each released CANable back to gs_usb; returns how many."""
    count = 0
    for node, ifno, driver in released_canables():
        try:
            fd = os.open(node, os.O_RDWR)
        except OSError as error:
            if log:
                log("CANable %s not reclaimed: %s (is the Super App's USB rule installed?)"
                    % (node, error))
            continue
        try:
            if driver is not None:  # usbfs: a browser (Super App) has it
                _request(fd, ifno, USBDEVFS_DISCONNECT)
            _request(fd, ifno, USBDEVFS_CONNECT)
            count += 1
            if log:
                log("CANable %s given back to Linux's CAN driver%s"
                    % (node, " (taken from the browser)" if driver == "usbfs" else ""))
        except OSError as error:
            if log:
                log("CANable %s not reclaimed: %s" % (node, error))
        finally:
            os.close(fd)
    return count


if __name__ == "__main__":
    print("%d reclaimed" % reclaim(print))
