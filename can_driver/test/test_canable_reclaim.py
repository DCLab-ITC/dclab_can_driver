"""canable_reclaim finds the CANables gs_usb does not have (fake sysfs)."""
import os
import tempfile
import unittest

from can_driver import canable_reclaim


def make_device(root, name, vendor, product, driver=None, busnum=1, devnum=27):
    device = os.path.join(root, name)
    interface = os.path.join(device, name + ":1.0")
    os.makedirs(interface)
    for key, value in (("idVendor", vendor), ("idProduct", product), ("bConfigurationValue", "1"),
                       ("busnum", str(busnum)), ("devnum", str(devnum))):
        with open(os.path.join(device, key), "w") as f:
            f.write(value + "\n")
    for key, value in (("bInterfaceClass", "ff"), ("bInterfaceNumber", "00")):
        with open(os.path.join(interface, key), "w") as f:
            f.write(value + "\n")
    if driver:
        target = os.path.join(root, "drivers", driver)
        os.makedirs(target, exist_ok=True)
        os.symlink(target, os.path.join(interface, "driver"))


class ReleasedCanables(unittest.TestCase):
    def test_only_canables_without_gs_usb(self):
        with tempfile.TemporaryDirectory() as root:
            make_device(root, "1-1", "1d50", "606f", driver="gs_usb", devnum=5)
            make_device(root, "1-2", "1d50", "606f", driver="usbfs", devnum=6)
            make_device(root, "1-3", "1209", "2323", devnum=7)
            make_device(root, "1-4", "0483", "3748", devnum=8)  # ST-Link
            self.assertEqual(canable_reclaim.released_canables(root), [
                ("/dev/bus/usb/001/006", 0, "usbfs"),
                ("/dev/bus/usb/001/007", 0, None),
            ])

    def test_missing_tree(self):
        self.assertEqual(canable_reclaim.released_canables("/nonexistent"), [])

    def test_ioctl_numbers(self):
        # <linux/usbdevice_fs.h> on a 64-bit system
        if canable_reclaim.ctypes.sizeof(canable_reclaim.ctypes.c_void_p) == 8:
            self.assertEqual(canable_reclaim.USBDEVFS_IOCTL, 0xC0105512)
        self.assertEqual(canable_reclaim.USBDEVFS_DISCONNECT, 0x5516)
        self.assertEqual(canable_reclaim.USBDEVFS_CONNECT, 0x5517)


if __name__ == "__main__":
    unittest.main()
