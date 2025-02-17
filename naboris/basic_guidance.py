import math
import time
import asyncio

from atlasbuggy import Node

from naboris.pid import PID
from naboris.odometry import PoseMessage


class BasicGuidance(Node):
    def __init__(self, enabled=True):
        super(BasicGuidance, self).__init__(enabled)

        # define subscription to node that produces current position
        self.odometry_tag = "position"
        self.odometry_sub = self.define_subscription(self.odometry_tag, message_type=PoseMessage)
        self.odometry_queue = None
        self.odometry = None

        # node that contains motor commands
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

        self.goal_was_cancelled = False
        self.goal_x = None  # goal position x in mm
        self.goal_y = None  # goal position y in mm
        self.goal_th = None  # goal angle theta in radians
        self.end_goal_th = None  # angle to face after position has been reached (radians)

        self.goal_queue = asyncio.Queue()

        self.current_x = 0.0  # current position x in mm
        self.current_y = 0.0  # current position y in mm
        self.current_th = 0.0 # current angle theta in radians

        # self.angle_pid = PID(120.0, 20.0, 0.0)  # solid
        self.angle_pid = PID(250.0, 50.0, 0.0)  # carpet
        self.dist_pid = PID(1.0, 0.0, 0.0)
        # self.angular_pid = PID(120.0, 20.0, 0.0)  # solid
        self.angular_pid = PID(200.0, 50.0, 0.0)  # carpet

        self.refresh_rate = 0.003  # how rapidly the controller should update in seconds (approximate)
        self.dist_error = 5.0  # stopping condition for distance (mm)
        self.angle_error = 0.003  # stopping condition for angle (rad)
        self.pid_timeout = 15.0  # timeout condition (sec)
        self.min_motor_power = 45

    def take(self):
        self.odometry_queue = self.odometry_sub.get_queue()
        self.odometry = self.odometry_sub.get_producer()
        self.hardware = self.hardware_sub.get_producer()
        self.cancel()  # no goal available at the start. Set the corresponding flags

    async def loop(self):
        while True:
            self.logger.info("Awaiting next goal...")
            await self.get_next_goal()
            # await self.goal_available_event.wait()  # wait for a goal to come in
            print("Goal received: (%s, %s, %s)" % (self.goal_x, self.goal_y, self.end_goal_th))

            # check if a position was commanded. It's possible to only command an angle
            if self.goal_x is not None and self.goal_y is not None:

                # calculate goal angle with current and goal position
                await self.update_current_state()
                self.goal_th = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)

                self.logger.debug("Calculated goal angle: %s" % self.goal_th)

                if not self.goal_was_cancelled:  # if goal wasn't cancelled
                    await self._rotate_to_angle(self.goal_th)  # rotate to goal angle

                if not self.goal_was_cancelled:  # if goal wasn't cancelled
                    await self._drive_to_point(self.goal_th)  # go to point and maintain the goal angle

            # if the goal wasn't cancelled and an end angle was specified, rotate to that angle
            if self.end_goal_th is not None and not self.goal_was_cancelled:
                await self._rotate_to_angle(self.end_goal_th)

            print("goto(%s, %s, %s) complete" % (self.goal_x, self.goal_y, self.end_goal_th))
            self.pause_for_next()

    async def get_next_goal(self):
        x, y, theta = await self.goal_queue.get()

        self.goal_x = x
        self.goal_y = y
        self.end_goal_th = theta

        self.goal_was_cancelled = False
        self.odometry_sub.enabled = True

        pose_message = await self.odometry_queue.get()
        self.current_x = pose_message.x_mm
        self.current_y = pose_message.y_mm
        self.current_th = pose_message.theta_rad

    def convert_angle_range(self, angle_error):
        """Convert an angle to a -pi...pi range"""
        angle_error %= 2 * math.pi

        if angle_error >= math.pi:
            angle_error -= 2 * math.pi

        return angle_error

    def shift_reference_frame(self, x, y, angle):
        """Rotate a point to a different reference frame (applies a rotation matrix)"""
        x = x * math.cos(angle) + y * math.sin(angle)
        y = x * -math.sin(angle) + y * math.cos(angle)

        return x, y

    async def update_current_state(self):
        """Update current position if a new value is available"""
        if not self.odometry_queue.empty():
            pose_message = await self.odometry_queue.get()
            self.current_x = pose_message.x_mm
            self.current_y = pose_message.y_mm
            self.current_th = pose_message.theta_rad

    async def _rotate_to_angle(self, goal_th):
        """Rotate to an angle. For internal use."""

        await self.update_current_state()
        angle_error = self.convert_angle_range(goal_th - self.current_th)

        self.logger.info("Rotating to: %srad. Current error: %s" % (goal_th, angle_error))
        self.angle_pid.reset()  # reset pid to initial conditions

        prev_motor_power = 0

        start_time = time.time()
        prev_time = time.time()

        satisfied = False
        # keep updating until angle is minimized or timeout is reached
        while not satisfied:
            await self.update_current_state()
            angle_error = self.convert_angle_range(goal_th - self.current_th)

            # angle_error = math.copysign(math.sqrt(abs(angle_error)), angle_error)

            # apply spinning motor power (-255...255) depending on the angle error
            current_time = time.time()
            dt = current_time - prev_time
            prev_time = current_time
            motor_power = -self.angle_pid.update(angle_error, dt)
            if abs(motor_power) < self.min_motor_power:
                motor_power = math.copysign(self.min_motor_power, motor_power)

            motor_power = int(motor_power)
            if motor_power != prev_motor_power:
                self.hardware.spin(motor_power)
                motor_power = prev_motor_power

            await asyncio.sleep(self.refresh_rate)

            if self.goal_was_cancelled:
                self.logger.info("Cancelling rotation.")
                break

            if (time.time() - start_time) >= self.pid_timeout:
                # cancel the rest of the operations if timeout was reached (goal wasn't achieved)
                self.logger.warning("Rotation timed out! Cancelling operation.")
                self.cancel()
                return

            if abs(angle_error) < self.angle_error:
                self.logger.debug("stopping motors")
                self.hardware.stop_motors()
                await asyncio.sleep(0.25)
                await self.update_current_state()
                self.logger.debug("error is %s after motors settled down." % angle_error)
                if abs(angle_error) < self.angle_error:
                    self.logger.debug("This error is satisfactory")
                    satisfied = True

        self.hardware.stop_motors()
        await asyncio.sleep(0.1)
        self.logger.info("Rotation successful! Error is %s" % angle_error)

    async def _drive_to_point(self, goal_th):
        """Drive to a distance and maintain the current angle. For internal use."""

        await self.update_current_state()
        dist_error, lateral_error = self.shift_reference_frame(
            self.goal_x - self.current_x, self.goal_y - self.current_y, goal_th
        )

        self.logger.info(
            "Driving to x=%s, y=%s. distance error is %s" % (
                self.goal_x, self.goal_y, dist_error)
        )

        # extend the timeout arbitrarily based on distance
        pid_timeout = self.pid_timeout + dist_error * 1.5

        # reset PID's
        self.dist_pid.reset()
        self.angular_pid.reset()

        start_time = time.time()
        prev_time = time.time()

        satisfied = False

        prev_forward_power = 0
        prev_angle_power = 0

        # keep updating until distance is minimized or timeout is reached
        while not satisfied:
            await self.update_current_state()
            distance = math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)
            dist_error, lateral_error = self.shift_reference_frame(
                self.goal_x - self.current_x, self.goal_y - self.current_y, goal_th
            )

            angle_error = self.convert_angle_range(goal_th - self.current_th)

            current_time = time.time()
            dt = current_time - prev_time
            prev_time = current_time

            forward_power = int(self.dist_pid.update(dist_error, dt))
            angle_power = -int(self.angular_pid.update(angle_error, dt))

            if abs(forward_power) < self.min_motor_power:
                forward_power = math.copysign(self.min_motor_power, forward_power)

            # apply forward/backward motor power (-255...255) depending on the distance error
            if forward_power != prev_forward_power or angle_power != prev_angle_power:
                self.hardware.drive(forward_power, -angle_power, angle_power)
                prev_forward_power = forward_power
                prev_angle_power = angle_power

            await asyncio.sleep(self.refresh_rate)

            if self.goal_was_cancelled:
                self.logger.info("Cancelling drive.")
                break

            if (time.time() - start_time) >= pid_timeout:
                # cancel the rest of the operations if timeout was reached (goal wasn't achieved)
                self.logger.warning("Drive timed out! Cancelling operation.")
                self.cancel()
                return

            if abs(dist_error) < self.dist_error:
                self.hardware.stop_motors()
                await asyncio.sleep(0.25)
                await self.update_current_state()
                self.logger.debug("error is %s after motors settled down." % angle_error)
                if abs(dist_error) < self.dist_error:
                    self.logger.debug("This error is satisfactory")
                    satisfied = True

        self.hardware.stop_motors()
        await asyncio.sleep(0.1)

        self.logger.info("Drive successful! Error is %s" % dist_error)

    def pause_for_next(self):
        self.goal_was_cancelled = True
        self.odometry_sub.enabled = False
        self.hardware.stop_motors()

        while not self.odometry_queue.empty():
            self.odometry_queue.get_nowait()

    def cancel(self):
        """Cancel a goal request"""

        while not self.odometry_queue.empty():
            self.odometry_queue.get_nowait()

        while not self.goal_queue.empty():
            self.goal_queue.get_nowait()

        self.pause_for_next()

        self.logger.info("Cancelling goal.")

    def goto(self, x=None, y=None, theta=None):
        """
        Set a goal for the robot to go to.

        Face 180 degrees:
        guidance.goto(theta=math.pi)

        Go to x = 50mm, y = 60mm:
        guidance.goto(50, 60)

        Go to x = 50mm, y = 60mm and face 90 degrees once the robot gets there:
        guidance.goto(50, 60, math.pi / 2)
        """
        self.goal_queue.put_nowait((x, y, theta))

        print("Submitting goal (x=%s, y=%s, th=%s)" % (x, y, theta))

    def reset(self):
        self.angle_pid.reset()
        self.dist_pid.reset()
        self.angular_pid.reset()

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_th = 0.0

        self.odometry.reset()

        self.cancel()

        self.logger.info("Reset to initial conditions")
