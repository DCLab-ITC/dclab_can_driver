#!/usr/bin/env python3
"""ROS 2 node bridging a SocketCAN bus and the DCLab boards (plus VESC,
RoboMaster motors).

Each device module registers the CAN IDs it handles; received frames are
dispatched to them and ROS commands are sent as CAN frames. See README.md
for topics, parameters and frame formats."""

import subprocess
import time

import can
import rclpy
from rclpy.node import Node

from .ControllerBoard import ControllerBoard
from .DCLabEncoders import DCLabEncoders
from .DCLabSensorInputs import DCLabSensorInputs
from .DCLabMotors import DCLabMotors
from .IMUBoard import IMUBoard
from .Robomaster import Robomaster
from .SwerveModule import SwerveModule
from .Vesc import Vesc


class CanDriver(Node, can.Listener):
    def __init__(self):
        super().__init__("can_driver")
        self.channel = self.declare_parameter("channel", "can0").value
        self.bitrate = int(self.declare_parameter("bitrate", 1000000).value)
        # Bring the interface up with `sudo ip link` when it is down (needs
        # passwordless sudo for `ip`); set false when the system does it.
        self.configure_interface = bool(self.declare_parameter("configure_interface", True).value)
        self.log_rates = bool(self.declare_parameter("log_rates", False).value)

        self.bus = None
        self.notifier = None
        self.transmit_count = 0
        self.receive_count = 0
        self.unhandled_count = 0
        self.last_recovery = 0.0

        self.devices = [
            DCLabMotors(self), DCLabEncoders(self), DCLabSensorInputs(self),
            ControllerBoard(self), SwerveModule(self), IMUBoard(self),
            Robomaster(self), Vesc(self),
        ]
        self.open_bus()
        self.create_timer(1.0, self.report_rates)
        self.create_timer(2.0, self.ensure_bus)

    # --- bus ---
    def open_bus(self):
        """Open the SocketCAN bus; on failure the 2 s timer retries."""
        self.close_bus()
        try:
            if self.configure_interface:
                state = subprocess.run(["ip", "link", "show", self.channel],
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                if b"state UP" not in state.stdout:
                    subprocess.run(["sudo", "-n", "ip", "link", "set", self.channel, "up",
                                    "type", "can", "bitrate", str(self.bitrate)],
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.bus = can.interface.Bus(interface="socketcan", channel=self.channel)
            self.notifier = can.Notifier(self.bus, [self])
            self.get_logger().info(f"CAN {self.channel} open")
        except Exception as error:  # interface missing, permissions, ...
            self.get_logger().warn(f"CAN {self.channel} not available: {error}")
            self.close_bus()

    def close_bus(self):
        if self.notifier is not None:
            try:
                self.notifier.stop()
            except Exception:
                pass
            self.notifier = None
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None

    def ensure_bus(self):
        if self.bus is None:
            self.open_bus()

    def send(self, arbitration_id, data, extended=False):
        """Send one frame; drops it (with a warning) when the bus is down or
        the transmit queue is full."""
        if self.bus is None:
            return False
        message = can.Message(arbitration_id=arbitration_id, data=bytes(data),
                              is_extended_id=extended)
        try:
            self.bus.send(message)
            self.transmit_count += 1
            return True
        except can.CanError as error:
            if "buffer" in str(error).lower():
                self.get_logger().warn("CAN transmit queue full, frame dropped",
                                       throttle_duration_sec=1.0)
            else:
                self.get_logger().error(f"CAN send failed: {error}")
                if time.monotonic() - self.last_recovery > 1.0:
                    self.last_recovery = time.monotonic()
                    self.open_bus()
            return False

    # --- can.Listener ---
    def on_message_received(self, msg):
        self.receive_count += 1
        if msg.is_error_frame or msg.is_remote_frame:
            return
        for device in self.devices:
            if device.process_can_msg(msg):
                return
        self.unhandled_count += 1

    def on_error(self, exc):
        self.get_logger().error(f"CAN receive error: {exc}")
        self.close_bus()

    def report_rates(self):
        if self.log_rates:
            self.get_logger().info(
                f"CAN tx {self.transmit_count}/s rx {self.receive_count}/s "
                f"(unhandled {self.unhandled_count})")
        self.transmit_count = self.receive_count = self.unhandled_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = CanDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_bus()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
