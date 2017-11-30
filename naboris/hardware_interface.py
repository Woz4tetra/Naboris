import time
import asyncio

from atlasbuggy.device.arduino import Arduino

from naboris.bno055 import *

counts_per_revolution = 12
gear_ratio = 150.58
wheel_radius_mm = 27.0

ticks_to_mm = wheel_radius_mm * 2 * math.pi / (gear_ratio * counts_per_revolution)
dist_between_wheels_mm = 109.0


class EncoderMessage(Message):
    message_regex = r"EncoderMessage\(t=(\d.*), n=(\d*), pt=(\d.*), tick=(-?[0-9]\d*), dist=(-?[0-9]\d*\.\d+), ptick=(-?[0-9]\d*), pdist=(-?[0-9]\d*\.\d+)\)"

    def __init__(self, tick, dist_mm, prev_message=None, timestamp=None, n=None):
        super(EncoderMessage, self).__init__(timestamp, n)

        self.tick = tick
        self.dist_mm = dist_mm

        if prev_message is None:
            self.prev_tick = self.tick
            self.prev_dist_mm = self.dist_mm
            self.prev_timestamp = self.timestamp
        else:
            self.prev_tick = prev_message.tick
            self.prev_dist_mm = prev_message.dist_mm
            self.prev_timestamp = prev_message.timestamp

        self.dt = self.timestamp - self.prev_timestamp
        self.delta_arc = self.dist_mm - self.prev_dist_mm

    def __str__(self):
        return "%s(t=%s, n=%s, pt=%s, tick=%s, dist=%s, ptick=%s, pdist=%s)" % (
            self.name, self.timestamp, self.n, self.prev_timestamp, self.tick, self.dist_mm, self.prev_tick,
            self.prev_dist_mm
        )

    @classmethod
    def parse(cls, message):
        match = re.match(cls.message_regex, message)
        if match is not None:
            timestamp = float(match.group(1))
            n = int(match.group(2))
            prev_time = float(match.group(3))
            tick = int(match.group(4))
            # dist = float(match.group(5))
            dist = tick * ticks_to_mm
            # assert dist == float(match.group(5)), "dist: tick=%s, %s != %s" % (tick, dist, match.group(5))

            prev_tick = int(match.group(6))
            # prev_dist = float(match.group(7))
            prev_dist = prev_tick * ticks_to_mm
            # assert prev_dist == float(match.group(7)), "prev dist: prev_tick=%s, %s != %s" % (prev_tick, prev_dist, match.group(7))

            prev_message = cls(prev_tick, prev_dist, timestamp=prev_time, n=n - 1)
            message = cls(tick, dist, prev_message, timestamp, n)

            return message
        else:
            return None


class HardwareInterface(Arduino):
    def __init__(self, enabled=True):
        super(HardwareInterface, self).__init__("hardware_interface", enabled=enabled)

        self.num_leds = None
        self.temperature = None

        self.led_states = None
        self.turret_yaw = 90
        self.turret_azimuth = 90

        self.commanded_speed = 0
        self.commanded_angle = 0
        self.commanded_angular = 0

        self.right_tick = 0
        self.left_tick = 0
        self.right_updated = False
        self.left_updated = False

        self.right_encoder_message = None
        self.left_encoder_message = None
        self.right_packet_num = 0
        self.left_packet_num = 0

        self.right_encoder_service = "right_encoder"
        self.define_service(self.right_encoder_service, message_type=EncoderMessage)

        self.left_encoder_service = "left_encoder"
        self.define_service(self.left_encoder_service, message_type=EncoderMessage)

        self.bno055_packet_header = "imu"
        self.bno055_packet_num = 0
        self.bno055 = BNO055()
        self.bno055_service = "bno055"
        self.define_service(self.bno055_service, message_type=Bno055Message)

    async def loop(self):
        self.receive_first(self.first_packet)

        self.start()

        self.set_all_leds(15, 15, 15)
        await asyncio.sleep(0.1)  # servos don't like being set at the same time as LEDs
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
        if packet[0] == "e":
            header = packet[1]
            data = packet[2:]
            if header == "r":
                encoder_time, right_tick = data.split("\t")
                right_tick = -int(right_tick)  # right encoder reversed
                right_dist = right_tick * ticks_to_mm

                self.right_encoder_message = EncoderMessage(
                    right_tick, right_dist, self.right_encoder_message,
                    packet_time, self.right_packet_num
                )
                self.right_packet_num += 1
                self.log_to_buffer(self.right_encoder_message.timestamp, self.right_encoder_message)
                await self.broadcast(self.right_encoder_message, self.right_encoder_service)

            elif header == "l":
                encoder_time, left_tick = data.split("\t")
                left_tick = int(left_tick)  # right encoder reversed
                left_dist = left_tick * ticks_to_mm

                self.left_encoder_message = EncoderMessage(
                    left_tick, left_dist, self.left_encoder_message,
                    packet_time, self.left_packet_num
                )
                self.left_packet_num += 1
                self.log_to_buffer(self.left_encoder_message.timestamp, self.left_encoder_message)
                await self.broadcast(self.right_encoder_message, self.left_encoder_service)

            else:
                raise ValueError("Invalid encoder header. Packet: %s" % packet)

        elif packet.startswith(self.bno055_packet_header):
            message = self.bno055.parse_packet(packet_time, packet, self.bno055_packet_num)

            self.log_to_buffer(packet_time, message)
            self.bno055_packet_num += 1
            await self.broadcast(message, self.bno055_service)

        else:
            raise ValueError("Unrecognized packet type: %s" % packet)

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

    def set_turret(self, yaw, azimuth):
        if azimuth < 30:
            azimuth = 30
        if azimuth > 100:
            azimuth = 100

        if yaw < 30:
            yaw = 30
        if yaw > 150:
            yaw = 150

        self.write("c%03d%03d" % (yaw, azimuth))

    def set_yaw(self, yaw):
        self.turret_yaw = yaw
        self.set_turret(self.turret_yaw, self.turret_azimuth)

    def set_azimuth(self, azimuth):
        self.turret_azimuth = azimuth
        self.set_turret(self.turret_yaw, self.turret_azimuth)

    def look_up(self, azimuth=0):
        self.set_azimuth(azimuth)

    def look_down(self, azimuth=180):
        self.set_azimuth(azimuth)

    def look_left(self, yaw=110):
        self.set_yaw(yaw)

    def look_right(self, yaw=70):
        self.set_yaw(yaw)

    def look_straight(self):
        self.turret_yaw = 90
        self.turret_azimuth = 75
        self.set_turret(self.turret_yaw, self.turret_azimuth)

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
