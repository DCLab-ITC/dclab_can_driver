"""Encoder feedback of the DCLab boards (Smart Driver motors, Sensor
encoders, Absolute Encoder), published on /encoder_feedback.

Firmware V2 sends float32 position only: DLC 4 for one encoder, DLC 8 for two
encoders sharing an ID (slot 0 = lower-numbered encoder, slot 1 = the other).
The speed field is estimated here from successive positions.

Parameters: encoder_ids ([first, last] pairs, default [100, 199, 250, 299]:
encoders and the Motor Controller's channel feedback) and
speed_filter_alpha (0 = raw difference, closer to 1 = smoother)."""

from custom_messages.msg import EncoderFeedback

from . import dclab_frames as frames
from .id_ranges import declare_ranges, in_ranges


class DCLabEncoders:
    def __init__(self, node):
        self.node = node
        self.ranges = declare_ranges(node, "encoder_ids", [100, 199, 250, 299])
        alpha = float(node.declare_parameter("speed_filter_alpha", 0.0).value)
        self.speed = frames.SpeedEstimator(alpha)
        self.publisher = node.create_publisher(EncoderFeedback, "/encoder_feedback", 10)

    def process_can_msg(self, msg):
        if msg.is_extended_id or not in_ranges(self.ranges, msg.arbitration_id) or \
                msg.dlc not in (4, 8):
            return False
        now = self.node.get_clock().now().nanoseconds * 1e-9
        data = bytes(msg.data[:msg.dlc])
        for slot, position in enumerate(frames.encoder_positions(data)):
            speed = self.speed.update((msg.arbitration_id, slot), position, now)
            self.publish(msg.arbitration_id, slot, position, speed)
        return True

    def publish(self, can_id, slot, position, speed):
        out = EncoderFeedback()
        out.can_id = can_id
        out.slot = slot
        out.position = float(position)
        out.speed = float(speed)
        self.publisher.publish(out)
