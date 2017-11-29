import re
import time
import math
import asyncio

from atlasbuggy import Node, Message

from naboris.hardware_interface import EncoderMessage, Bno055Message


class PoseMessage(Message):
    def __init__(self, timestamp=None, n=None, x_mm=0.0, y_mm=0.0, theta_rad=0.0, delta_xy_mm=0.0,
                 delta_theta_rad=0.0, delta_t=0.0):
        self.x_mm = x_mm
        self.y_mm = y_mm
        self.theta_rad = theta_rad

        self.delta_xy_mm = delta_xy_mm
        self.delta_theta_rad = delta_theta_rad
        self.delta_t = delta_t

        super(PoseMessage, self).__init__(timestamp, n)

    def __str__(self):
        return "%s(t=%s, n=%s, x=%s, y=%s, th=%s)" % (
            self.__class__.__name__, self.timestamp, self.n, self.x_mm, self.y_mm, self.theta_rad)


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
            pose_message = PoseMessage()

            enc_updated = not self.encoder_queue.empty()
            if enc_updated:
                while not self.encoder_queue.empty():
                    encoder_message = await self.encoder_queue.get()
                    pose_message.delta_t = encoder_message.dt
                    pose_message.delta_xy_mm = -encoder_message.delta_arc

                    self.num_enc_messages += 1
            else:
                pose_message.delta_t = 0.0
                pose_message.delta_xy_mm = 0.0

            bno_updated = not self.bno055_queue.empty()
            if bno_updated:
                while not self.bno055_queue.empty():
                    bno055_message = await self.bno055_queue.get()
                    current_angle = bno055_message.euler.z
                    if current_angle - self.prev_angle > math.pi:
                        current_angle -= 2 * math.pi
                    pose_message.delta_theta_rad = current_angle - self.prev_angle
                    self.prev_angle = current_angle

                    self.num_bno_messages += 1

                if not enc_updated:
                    pose_message.delta_t = bno055_message.timestamp - prev_t
                prev_t = bno055_message.timestamp
            else:
                pose_message.delta_theta_rad = 0.0

            if enc_updated or bno_updated:
                pose_message.timestamp = time.time()
                pose_message.n = message_num

                message_num += 1

                self.odom_th += pose_message.delta_theta_rad
                self.odom_x += math.cos(self.odom_th) * pose_message.delta_xy_mm
                self.odom_y += math.sin(self.odom_th) * pose_message.delta_xy_mm

                self.log_to_buffer(time.time(), pose_message)
                await self.broadcast(pose_message)
            else:
                await asyncio.sleep(0.01)

    async def teardown(self):
        self.logger.info("%s BNO055 messages received. %s encoder messages received" % (
            self.num_bno_messages, self.num_enc_messages))
