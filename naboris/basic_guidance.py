import math
import time
import asyncio

from atlasbuggy import Node

from naboris.pid import PID
from naboris.odometry import PoseMessage


class BasicGuidance(Node):
    def __init__(self, enabled=True):
        super(BasicGuidance, self).__init__(enabled)

        self.position_tag = "position"
        self.position_sub = self.define_subscription(self.position_tag, message_type=PoseMessage)
        self.position_queue = None

        self.hardware_tag = "hardware"
        self.hardware_sub = self.define_subscription(
            self.hardware_tag, queue_size=None,
            required_methods=(
                "spin",
                "drive",
                "stop_motors"
            )
        )
        self.hardware = None

        self.goal_available_event = asyncio.Event()
        self.goal_x = None
        self.goal_y = None
        self.goal_th = None
        self.end_goal_th = None

        self.current_th = None
        self.current_x = None
        self.current_y = None

        self.angle_pid = PID(1.0, 0.0, "tuning")
        self.dist_pid = PID(1.0, 0.0, "tuning")
        self.angular_pid = PID(1.0, 0.0, "tuning")

        self.refresh_rate = 0.003
        self.dist_error = 5  # mm
        self.angle_error = 0.005  # rad
        self.pid_timeout = 4  # sec

        self.prev_time = time.time()

    def take(self):
        self.position_queue = self.position_sub.get_queue()
        self.hardware = self.hardware_sub.get_producer()

    async def loop(self):
        self.prev_time = time.time()
        while True:
            self.logger.info("Awaiting next goal...")
            await self.goal_available_event.wait()
            self.logger.info("Goal received!")

            if self.goal_x is not None and self.goal_y is not None:
                await self.update_current_state()
                self.goal_th = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)

                if not self.goal_available_event.is_set():
                    await self._rotate_to_angle(self.goal_th)
                if not self.goal_available_event.is_set():
                    await self._drive_to_point()

            if self.end_goal_th is not None and not self.goal_available_event.is_set():
                await self._rotate_to_angle(self.end_goal_th)

            self.logger.info("goto(%s, %s, %s) was successful!" % (self.goal_x, self.goal_y, self.goal_th))
            self.cancel()

    def convert_angle_range(self, angle_error):
        angle_error %= 2 * math.pi

        if angle_error >= math.pi:
            angle_error -= 2 * math.pi

        return angle_error

    def shift_reference_frame(self, x, y, angle):
        x = x * math.cos(angle) + y * math.sin(angle)
        y = x * -math.sin(angle) + y * math.cos(angle)

        return x, y

    async def update_current_state(self):
        pose_message = await self.position_queue.get()
        self.current_x = pose_message.x_mm
        self.current_y = pose_message.y_mm
        self.current_th = pose_message.theta_rad

    async def _rotate_to_angle(self, goal_th):
        await self.update_current_state()
        angle_error = self.convert_angle_range(goal_th - self.current_th)

        self.logger.info("Rotating to: %srad. Current error: %s" % (goal_th, angle_error))

        self.angle_pid.reset()

        start_time = time.time()
        prev_time = time.time()

        while abs(angle_error) > self.angle_error or (time.time() - start_time) < self.pid_timeout:
            await self.update_current_state()
            angle_error = self.convert_angle_range(goal_th - self.current_th)

            current_time = time.time()
            dt = current_time - prev_time
            prev_time = current_time
            motor_power = self.angle_pid.update(angle_error, dt)

            self.hardware.spin(int(motor_power))
            await asyncio.sleep(self.refresh_rate)

            if self.goal_available_event.is_set():
                self.logger.info("Cancelling rotation.")
                break

        self.hardware.stop_motors()

        if (time.time() - start_time) >= self.pid_timeout:
            self.logger.warning("Rotation timed out! Cancelling operation.")
            self.cancel()
        else:
            self.logger.info("Rotation successful! Error is %s" % angle_error)

    async def _drive_to_point(self):
        await self.update_current_state()
        dist_error, lateral_error = self.shift_reference_frame(
            self.goal_x - self.current_x, self.goal_y - self.current_y, self.goal_th
        )

        self.logger.info(
            "Driving to x=%s, y=%s. distance error is %s, lateral error is %s" % (
                self.goal_x, self.goal_y, dist_error, lateral_error)
        )

        self.dist_pid.reset()
        self.angular_pid.reset()

        start_time = time.time()
        prev_time = time.time()

        while abs(dist_error) > self.dist_error or (time.time() - start_time) < self.pid_timeout:
            await self.update_current_state()
            dist_error, lateral_error = self.shift_reference_frame(
                self.goal_x - self.current_x, self.goal_y - self.current_y, self.goal_th
            )

            current_time = time.time()
            dt = current_time - prev_time
            prev_time = current_time
            forward_power = self.dist_pid.update(dist_error, dt)
            angle_power = self.angular_pid.update(lateral_error, dt)

            self.hardware.drive(int(forward_power), angular=int(angle_power))
            await asyncio.sleep(self.refresh_rate)

            if self.goal_available_event.is_set():
                self.logger.info("Cancelling drive.")
                break

        self.hardware.stop_motors()

        if (time.time() - start_time) >= self.pid_timeout:
            self.logger.warning("Drive timed out! Cancelling operation.")
            self.cancel()
        else:
            self.logger.info("Drive successful! Error is %s" % dist_error)

    def cancel(self):
        self.goal_available_event.set()
        self.position_sub.enabled = False

        self.logger.info("Cancelling goal.")

    def goto(self, x=None, y=None, theta=None):
        self.goal_x = x
        self.goal_y = y
        self.end_goal_th = theta

        self.goal_available_event.clear()
        self.position_sub.enabled = True

        self.logger.info("Submitting goal (x=%s, y=%s, th=%s)" % (x, y, theta))
