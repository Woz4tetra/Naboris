import re
import time
import asyncio

from atlasbuggy.device.arduino import Arduino

from naboris.bno055 import *

counts_per_revolution = 12
gear_ratio = 150.58
wheel_radius_mm = 27.0

ticks_to_mm = wheel_radius_mm * 2 * math.pi / (gear_ratio * counts_per_revolution)
dist_between_wheels_mm = 109.0
dist_between_axles_mm = 143.0


class EncoderMessage(Message):
    message_regex = r"EncoderMessage\(t=(\d.*), n=(\d*), enc_time=(\d.*), dist=(-?[0-9]\d*\.\d+)"

    def __init__(self, encoder_time, dist_mm, timestamp=None, n=None):
        super(EncoderMessage, self).__init__(timestamp, n)
        self.encoder_time = encoder_time
        self.dist_mm = dist_mm

    def __str__(self):
        return "%s(t=%s, n=%s, enc_time=%s, dist=%s)" % (
            self.name, self.timestamp, self.n, self.encoder_time, self.dist_mm
        )

    @classmethod
    def parse(cls, message):
        match = re.match(cls.message_regex, message)
        if match is not None:
            timestamp = float(match.group(1))
            n = int(match.group(2))
            dist = float(match.group(3))
            message = cls(dist, timestamp, n)

            return message
        else:
            return None


class HardwareInterface(Arduino):
    def __init__(self, enabled=True):
        super(HardwareInterface, self).__init__("hardware_interface", enabled=enabled)

        self.num_leds = None
        self.temperature = None

        self.led_states = None
        self.servo_value = 90

        self.commanded_speed = 0
        self.commanded_angle = 0
        self.commanded_angular = 0

        self.right_tick = 0
        self.left_tick = 0
        self.prev_right_tick = 0
        self.prev_left_tick = 0
        self.right_updated = False
        self.left_updated = False
        self.enc_packet_num = 0

        self.right_encoder_regex = r"er([\-\d]*)\t([\-\d]*)"
        self.left_encoder_regex = r"el([\-\d]*)\t([\-\d]*)"

        self.encoder_service = "encoder"
        self.define_service(self.encoder_service, message_type=EncoderMessage)

        self.bno055_packet_header = "imu"
        self.bno055_packet_num = 0
        self.bno055 = BNO055()
        self.bno055_service = "bno055"
        self.define_service(self.bno055_service, message_type=Bno055Message)

    async def loop(self):
        self.receive_first(self.first_packet)

        self.start()

        self.set_all_leds(15, 15, 15)
        self.pause(0.5)  # servos don't like being set at the same time as LEDs
        self.look_straight()

        while True:
            while not self.empty():
                packet_time, packets = self.read()

                for packet in packets:
                    try:
                        await self.receive(packet_time, packet)
                    except ValueError:
                        self.logger.exception("Failed to parse packet: %s" % packet)

            await asyncio.sleep(0.01)  # update rate of IMU (100 Hz)

    async def teardown(self):
        await super(HardwareInterface, self).teardown()
        self.stop_motors()
        self.release_motors()

    def write(self, packet):
        self.device_write_queue.put(packet)
        self.log_to_buffer(time.time(), "putting: " + str(packet))

    def receive_first(self, packet):
        data = packet.split("\t")
        assert len(data) == 2

        self.temperature = int(data[0])
        self.num_leds = int(data[1])

        self.led_states = [[0, 0, 0] for _ in range(self.num_leds)]

        self.logger.info("Number of leds: %s" % self.num_leds)

    async def receive(self, packet_time, packet):
        match = re.match(self.right_encoder_regex, packet)
        if match is not None:
            encoder_time = int(match.group(1))
            self.right_tick = -int(match.group(2))  # right encoder reversed
            self.right_updated = True

        match = re.match(self.left_encoder_regex, packet)
        if match is not None:
            encoder_time = int(match.group(1))
            self.left_tick = int(match.group(2))
            self.left_updated = True

        if self.right_updated and self.left_updated:
            encoder_time = float(encoder_time) / 1000

            delta_right = self.right_tick - self.prev_right_tick
            delta_left = self.left_tick - self.prev_left_tick

            self.log_to_buffer(time.time(), "right: %s, left: %s" % (self.right_tick, self.left_tick))
            # print("right: %s, left: %s" % (self.right_tick, self.left_tick))

            delta_dist = ticks_to_mm * (delta_right + delta_left) / 2

            encoder_message = EncoderMessage(encoder_time, delta_dist, time.time(), self.enc_packet_num)
            self.enc_packet_num += 1

            self.log_to_buffer(packet_time, encoder_message)
            await self.broadcast(encoder_message, self.encoder_service)

            self.right_updated = False
            self.left_updated = False

            self.prev_right_tick = self.right_tick
            self.prev_left_tick = self.left_tick


        if packet.startswith(self.bno055_packet_header):
            message = self.bno055.parse_packet(packet_time, packet, self.bno055_packet_num)

            self.log_to_buffer(packet_time, message)
            self.bno055_packet_num += 1
            await self.broadcast(message, self.bno055_service)


    def drive(self, speed=0, angle=0, angular=0):
        self.commanded_angle = angle % 360

        angle = (180 - angle) % 360
        speed = self.constrain_value(speed)
        if angular > 255:
            angular = 255
        elif angular < -255:
            angular = -255

        self.commanded_speed = speed
        self.commanded_angular = angular

        if 0 <= angle < 90:
            fraction_speed = -2 * speed / 90 * angle + speed
            self.command_motors(speed + angular, fraction_speed - angular, fraction_speed + angular, speed - angular)

        elif 90 <= angle < 180:
            fraction_speed = -2 * speed / 90 * (angle - 90) + speed
            self.command_motors(fraction_speed + angular, -speed - angular, -speed + angular, fraction_speed - angular)

        elif 180 <= angle < 270:
            fraction_speed = 2 * speed / 90 * (angle - 180) - speed
            self.command_motors(-speed + angular, fraction_speed - angular, fraction_speed + angular, -speed - angular)

        elif 270 <= angle < 360:
            fraction_speed = 2 * speed / 90 * (angle - 270) - speed
            self.command_motors(fraction_speed + angular, speed - angular, speed + angular, fraction_speed - angular)

        # self.logger.info("speed=%s, angle=%s, angular=%s" % (speed, angle, angular))

    def spin(self, speed):
        self.drive(angular=speed)

    def command_motors(self, m1, m2, m3, m4):
        m1, m2, m3, m4 = int(m1), int(m2), int(m3), int(m4)
        command = "d%04d%04d%04d%04d" % (m1, m2, m3, m4)
        self.write(command)
        # self.logger.info("m1=%s, m2=%s, m3=%s, m4=%s" % (m1, m2, m3, m4))
        # self.logger.info("command: %s" % command)

    def stop_motors(self):
        self.write("h")

    def release_motors(self):
        self.write("r")

    def set_servo(self, servo_value):
        self.write("c%03d" % int(servo_value % 181))

    def look_up(self):
        self.set_servo(150)

    def look_down(self):
        self.set_servo(84)

    def look_straight(self):
        self.set_servo(100)

    @staticmethod
    def constrain_value(value):
        if value < 0:
            value = 0
        if value > 255:
            value = 255
        return value

    def constrain_input(self, rgb):
        if len(rgb) == 1:
            rgb = rgb[0]
        return list(map(self.constrain_value, rgb))

    def set_led(self, led_index, *rgb, show=True):
        r, g, b = self.constrain_input(rgb)
        led_index = int(abs(led_index))
        if led_index >= self.num_leds:
            led_index = self.num_leds - 1
        if led_index < 0:
            led_index = 0

        self.led_states[led_index][0] = r
        self.led_states[led_index][1] = g
        self.led_states[led_index][2] = b

        self.write("o%03d%03d%03d%03d" % (led_index, r, g, b))
        if show:
            self.show()

    def set_leds(self, start, end, *rgb, show=True):
        r, g, b = self.constrain_input(rgb)
        start = int(abs(start))
        end = int(abs(end))
        if start >= self.num_leds:
            start = self.num_leds - 1
        if start < 0:
            start = 0

        if end > self.num_leds:
            end = self.num_leds
        if end < 1:
            end = 1

        for index in range(start, end):
            self.led_states[index][0] = r
            self.led_states[index][1] = g
            self.led_states[index][2] = b

        assert 0 <= start < end <= self.num_leds

        self.write("o%03d%03d%03d%03d%03d" % (start, r, g, b, end))
        if show:
            self.show()

        self.log_to_buffer(time.time(), "led s: %s, end: %s, r: %s, g: %s, b: %s" % (start, end, r, g, b))

    def set_all_leds(self, *rgb, show=True):
        self.set_leds(0, self.num_leds, rgb, show=show)

    def get_led(self, index):
        return tuple(self.led_states[index])

    def show(self):
        self.write("x")
