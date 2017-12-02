import re
import time
import math
import asyncio

from atlasbuggy import Node, Message

from naboris.hardware_interface import EncoderMessage, Bno055Message, dist_between_axles_mm


class PoseMessage(Message):
    def __init__(self, timestamp=None, n=None, x_mm=0.0, y_mm=0.0, theta_rad=0.0, delta_xy_mm=0.0,
                 delta_theta_rad=0.0, delta_t=0.0):
        self.x_mm = x_mm
        self.y_mm = y_mm
        self.theta_rad = theta_rad

        self.delta_xy_mm = delta_xy_mm
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

        self.start_angle = None
        self.prev_angle = 0.0
        self.prev_bno_time = None

        self.prev_enc_time = None

        self.num_bno_messages = 0
        self.num_enc_messages = 0

        self.num_used_bno_messages = 0
        self.num_used_enc_messages = 0

        self.message_num = 0

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_th = 0.0

        self.axle_offset_x = 0.0
        self.axle_offset_y = 0.0

        self.odom_position_service = "odom_pos"
        self.define_service(self.odom_position_service, tuple)

    def take(self):
        self.encoder_queue = self.encoder_sub.get_queue()
        self.bno055_queue = self.bno055_sub.get_queue()

    async def loop(self):
        prev_t = time.time()

        while True:
            enc_updated = not self.encoder_queue.empty()
            bno_updated = not self.bno055_queue.empty()

            if enc_updated or bno_updated:
                pose_message = PoseMessage()

                if enc_updated:
                    encoder_message = None
                    while not self.encoder_queue.empty():
                        encoder_message = await self.encoder_queue.get()
                        self.num_enc_messages += 1

                    self.num_used_enc_messages += 1

                    if self.prev_enc_time is None:
                        self.prev_enc_time = encoder_message.timestamp

                    pose_message.delta_xy_mm = encoder_message.dist_mm
                    pose_message.delta_t = encoder_message.timestamp - self.prev_enc_time
                    self.prev_enc_time = encoder_message.timestamp

                    self.odom_x += math.cos(self.odom_th) * pose_message.delta_xy_mm
                    self.odom_y += math.sin(self.odom_th) * pose_message.delta_xy_mm

                else:
                    pose_message.delta_t = 0.0
                    pose_message.delta_xy_mm = 0.0

                if bno_updated:
                    bno055_message = None
                    while not self.bno055_queue.empty():
                        bno055_message = await self.bno055_queue.get()
                        if self.start_angle is None:
                            self.start_angle = bno055_message.euler.z

                        self.num_bno_messages += 1

                    self.num_used_bno_messages += 1

                    current_angle = bno055_message.euler.z - self.start_angle
                    if current_angle - self.prev_angle > math.pi:
                        current_angle -= 2 * math.pi
                    delta_angle = current_angle - self.prev_angle
                    self.prev_angle = current_angle

                    if self.prev_bno_time is None:
                        self.prev_bno_time = bno055_message.timestamp
                    if not enc_updated:
                        pose_message.delta_t = bno055_message.timestamp - self.prev_bno_time

                    self.prev_bno_time = bno055_message.timestamp

                    self.odom_th += delta_angle

                    # self.axle_offset_x = dist_between_axles_mm / 2 * math.cos(self.odom_th)
                    # self.axle_offset_y = dist_between_axles_mm / 2 * math.sin(self.odom_th)

                    # print(self.axle_offset_x, self.axle_offset_y)

                await self.broadcast_pose(pose_message)
            else:
                await asyncio.sleep(0.01)

    async def broadcast_pose(self, pose_message):
        pose_message.timestamp = time.time()
        pose_message.n = self.message_num
        pose_message.x_mm = self.odom_x
        pose_message.y_mm = self.odom_y
        pose_message.theta_rad = self.odom_th

        # print(pose_message.x_mm, pose_message.y_mm, pose_message.theta_rad)

        self.message_num += 1

        self.log_to_buffer(time.time(), pose_message)
        await self.broadcast(pose_message)

    async def teardown(self):
        self.logger.info("%s BNO055 messages received. %s encoder messages received. %s messages broadcast." % (
            self.num_bno_messages, self.num_enc_messages, self.message_num)
        )
        self.logger.info("%s BNO055 messages used. %s encoder messages used." % (
            self.num_used_bno_messages, self.num_used_enc_messages)
        )

    def reset(self):
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_th = 0.0
        self.prev_angle = 0.0
        self.start_angle = None

        self.logger.info("Reset to initial conditions")
