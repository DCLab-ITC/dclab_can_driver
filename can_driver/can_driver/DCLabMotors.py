"""DCLab motor commands: Smart Driver and Motor Controller, one topic.

/dclab/motor_command  DCLabMotorCommand  the motors on one receive ID:
    M1 only -> DLC 4; M2 enabled -> DLC 8 (M1's half disabled unless
    m1_enable). Firmware Smart Driver 2.13 / Motor Controller 2.22 and later.
Feedback arrives on /encoder_feedback (DCLabEncoders)."""

from custom_messages.msg import DCLabMotorCommand

from . import dclab_frames as frames


class DCLabMotors:
    def __init__(self, node):
        self.node = node
        node.create_subscription(DCLabMotorCommand, "/dclab/motor_command", self.command, 10)

    def process_can_msg(self, msg):
        return False  # feedback is handled by DCLabEncoders

    def command(self, msg):
        try:
            if msg.can_id > frames.MAX_STANDARD_ID:
                raise ValueError(f"can_id {msg.can_id} is above 0x7FF")
            data = frames.motor_command(msg.m1_enable, msg.mode1, msg.goal1,
                                        msg.m2_enable, msg.mode2, msg.goal2)
        except ValueError as error:
            self.node.get_logger().warn(f"motor command on {msg.can_id} refused: {error}",
                                        throttle_duration_sec=1.0)
            return
        self.node.send(msg.can_id, data)
