import time
import math
import asyncio

from atlasbuggy import Node

from lms200.messages import OdometryMessage
from babybuggy.quad_encoder import EncoderMessage
from babybuggy.bno055 import Bno055Message


class Odometry(Node):
    def __init__(self, enabled=True):
        super(Odometry, self).__init__(enabled)

        self.encoder_tag = "encoder"
        self.encoder_sub = self.define_subscription(self.encoder_tag, message_type=EncoderMessage)
        self.encoder_queue = None

        self.bno055_tag = "bno055"
        self.bno055_sub = self.define_subscription(self.bno055_tag, message_type=Bno055Message)
        self.bno055_queue = None

        self.prev_angle = 0.0

        self.num_bno_messages = 0
        self.num_enc_messages = 0

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_th = 0.0

        self.odom_position_service = "odom_pos"
        self.define_service(self.odom_position_service, tuple)

    def take(self):
        self.encoder_queue = self.encoder_sub.get_queue()
        self.bno055_queue = self.bno055_sub.get_queue()

    async def loop(self):
        message_num = 0
        prev_t = time.time()

        while True:
            odometry_message = OdometryMessage()

            enc_updated = not self.encoder_queue.empty()
            if enc_updated:
                while not self.encoder_queue.empty():
                    encoder_message = await self.encoder_queue.get()
                    odometry_message.delta_t = encoder_message.dt
                    odometry_message.delta_xy_mm = -encoder_message.delta_arc

                    self.num_enc_messages += 1
            else:
                odometry_message.delta_t = 0.0
                odometry_message.delta_xy_mm = 0.0

            bno_updated = not self.bno055_queue.empty()
            if bno_updated:
                while not self.bno055_queue.empty():
                    bno055_message = await self.bno055_queue.get()
                    current_angle = math.degrees(bno055_message.euler.z)
                    if current_angle - self.prev_angle > 180:
                        current_angle -= 360
                    odometry_message.delta_theta_degrees = current_angle - self.prev_angle
                    self.prev_angle = current_angle

                    self.num_bno_messages += 1

                if not enc_updated:
                    odometry_message.delta_t = bno055_message.timestamp - prev_t
                prev_t = bno055_message.timestamp
            else:
                odometry_message.delta_theta_degrees = 0.0

            if enc_updated or bno_updated:
                odometry_message.timestamp = time.time()
                odometry_message.n = message_num

                message_num += 1

                self.odom_th += odometry_message.delta_theta_degrees
                self.odom_x += math.cos(math.radians(self.odom_th)) * odometry_message.delta_xy_mm
                self.odom_y += math.sin(math.radians(self.odom_th)) * odometry_message.delta_xy_mm

                self.log_to_buffer(time.time(), odometry_message)
                self.broadcast_nowait((self.odom_x, self.odom_y, self.odom_th), self.odom_position_service)
                await self.broadcast(odometry_message)
            else:
                await asyncio.sleep(0.01)

    async def teardown(self):
        self.logger.info("%s BNO055 messages received. %s encoder messages received" % (self.num_bno_messages, self.num_enc_messages))
