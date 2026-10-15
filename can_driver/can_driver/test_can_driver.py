#!/usr/bin/env python3
"""Example publisher for bench tests: ros2 run can_driver test_can_driver
--ros-args -p device:=motor (motor, motor_pair, motor_m2, controller,
swerve, vesc, robomaster)."""

import rclpy
from rclpy.node import Node

from custom_messages.msg import (DCLabMotorCommand, DigitalAndSolenoidCommand, PwmCommand,
                                 RobomasterCurrentCommand, ServoCommand, SwerveCommand,
                                 VescCommand)


class TestCanDriver(Node):
    def __init__(self):
        super().__init__("test_can_driver")
        self.device = self.declare_parameter("device", "motor").value
        self.pub = {
            "motor": self.create_publisher(DCLabMotorCommand, "/dclab/motor_command", 10),
            "pwm": self.create_publisher(PwmCommand, "/publish_pwm", 10),
            "servo": self.create_publisher(ServoCommand, "/publish_servo", 10),
            "digital": self.create_publisher(DigitalAndSolenoidCommand, "/publish_digital_solenoid", 10),
            "swerve": self.create_publisher(SwerveCommand, "/publish_swerve", 10),
            "vesc": self.create_publisher(VescCommand, "/publish_vesc", 10),
            "robomaster": self.create_publisher(RobomasterCurrentCommand, "/publish_robomaster_current", 10),
        }
        self.create_timer(0.01, self.tick)

    def tick(self):
        getattr(self, "send_" + self.device)()

    def send_motor(self):
        """M1 alone on ID 101: 5 rad/s (DLC 4)."""
        self.pub["motor"].publish(DCLabMotorCommand(
            can_id=101, m1_enable=True, mode1=DCLabMotorCommand.MODE_SPEED, goal1=5.0))

    def send_motor_pair(self):
        """Both motors on ID 101 (DLC 8)."""
        self.pub["motor"].publish(DCLabMotorCommand(
            can_id=101, m1_enable=True, mode1=DCLabMotorCommand.MODE_SPEED, goal1=5.0,
            m2_enable=True, mode2=DCLabMotorCommand.MODE_POSITION, goal2=3.14))

    def send_motor_m2(self):
        """M2 only on ID 101 (DLC 8, M1 untouched)."""
        self.pub["motor"].publish(DCLabMotorCommand(
            can_id=101, m2_enable=True, mode2=DCLabMotorCommand.MODE_VOLTAGE, goal2=2.0))

    def send_controller(self):
        self.pub["pwm"].publish(PwmCommand(can_id=600, pwm1_value=0.1, pwm2_value=0.2,
                                           pwm3_value=0.3, pwm4_value=0.4))
        self.pub["servo"].publish(ServoCommand(can_id=600, servo1_value=0.5, servo2_value=0.5,
                                               servo3_value=0.5, servo4_value=0.5))
        self.pub["digital"].publish(DigitalAndSolenoidCommand(can_id=600, solenoid1_value=True))

    def send_swerve(self):
        self.pub["swerve"].publish(SwerveCommand(can_id=201, position=1.57, speed=10.0))

    def send_vesc(self):
        self.pub["vesc"].publish(VescCommand(vesc_id=2, speed_mode=True, goal=1000.0))

    def send_robomaster(self):
        self.pub["robomaster"].publish(RobomasterCurrentCommand(can_id=0x200, current1=0.5, type1=0))



def main(args=None):
    rclpy.init(args=args)
    node = TestCanDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
