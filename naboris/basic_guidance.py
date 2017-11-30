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
        self.position_tag = "position"
        self.position_sub = self.define_subscription(self.position_tag, message_type=PoseMessage)
        self.position_queue = None

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

        self.goal_available_event = asyncio.Event()  # unfreezes the guidance system
        self.goal_x = None  # goal position x in mm
        self.goal_y = None  # goal position y in mm
        self.goal_th = None  # goal angle theta in radians
        self.end_goal_th = None  # angle to face after position has been reached (radians)

        self.current_x = None  # current position x in mm
        self.current_y = None  # current position y in mm
        self.current_th = None  # current angle theta in radians

        self.angle_pid = PID(1.0, 0.0, "tuning")
        self.dist_pid = PID(1.0, 0.0, "tuning")
        self.angular_pid = PID(1.0, 0.0, "tuning")

        self.refresh_rate = 0.003  # how rapidly the controller should update in seconds (approximate)
        self.dist_error = 5.0  # stopping condition for distance (mm)
        self.angle_error = 0.005  # stopping condition for angle (rad)
        self.pid_timeout = 4  # timeout condition (sec)

    def take(self):
        self.position_queue = self.position_sub.get_queue()
        self.hardware = self.hardware_sub.get_producer()
        self.cancel()  # no goal available at the start. Set the corresponding flags

    async def loop(self):
        while True:
            self.logger.info("Awaiting next goal...")
            await self.goal_available_event.wait()  # wait for a goal to come in
            self.logger.info("Goal received!")

            # check if a position was commanded. It's possible to only command an angle
            if self.goal_x is not None and self.goal_y is not None:

                # calculate goal angle with current and goal position
                await self.update_current_state()
                self.goal_th = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)

                if not self.goal_available_event.is_set():  # if goal wasn't cancelled
                    await self._rotate_to_angle(self.goal_th)  # rotate to goal angle
                if not self.goal_available_event.is_set():  # if goal wasn't cancelled
                    await self._drive_to_point(self.goal_th)  # go to point and maintain the goal angle

            # if the goal wasn't cancelled and an end angle was specified, rotate to that angle
            if self.end_goal_th is not None and not self.goal_available_event.is_set():
                await self._rotate_to_angle(self.end_goal_th)

            self.logger.info("goto(%s, %s, %s) was successful!" % (self.goal_x, self.goal_y, self.goal_th))
            self.cancel()

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
        if not self.position_queue.empty():
            pose_message = await self.position_queue.get()
            self.current_x = pose_message.x_mm
            self.current_y = pose_message.y_mm
            self.current_th = pose_message.theta_rad

    async def _rotate_to_angle(self, goal_th):
        """Rotate to an angle. For internal use."""

        await self.update_current_state()
        angle_error = self.convert_angle_range(goal_th - self.current_th)

        self.logger.info("Rotating to: %srad. Current error: %s" % (goal_th, angle_error))

        self.angle_pid.reset()  # reset pid to initial conditions

        start_time = time.time()
        prev_time = time.time()

        # keep updating until angle is minimized or timeout is reached
        while abs(angle_error) > self.angle_error or (time.time() - start_time) < self.pid_timeout:
            await self.update_current_state()
            angle_error = self.convert_angle_range(goal_th - self.current_th)

            # apply spinning motor power (-255...255) depending on the angle error
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

        # cancel the rest of the operations if timeout was reached (goal wasn't achieved)
        if (time.time() - start_time) >= self.pid_timeout:
            self.logger.warning("Rotation timed out! Cancelling operation.")
            self.cancel()
        else:
            self.logger.info("Rotation successful! Error is %s" % angle_error)

    async def _drive_to_point(self, goal_th):
        """Drive to a distance and maintain the current angle. For internal use."""

        await self.update_current_state()
        dist_error = math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)

        self.logger.info(
            "Driving to x=%s, y=%s. distance error is %s" % (
                self.goal_x, self.goal_y, dist_error)
        )

        # reset PID's
        self.dist_pid.reset()
        self.angular_pid.reset()

        start_time = time.time()
        prev_time = time.time()

        # keep updating until distance is minimized or timeout is reached
        while abs(dist_error) > self.dist_error or (time.time() - start_time) < self.pid_timeout:
            await self.update_current_state()
            dist_error = math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)
            angle_error = self.convert_angle_range(goal_th - self.current_th)

            current_time = time.time()
            dt = current_time - prev_time
            prev_time = current_time
            forward_power = self.dist_pid.update(dist_error, dt)
            angle_power = self.angular_pid.update(angle_error, dt)

            # apply forward/backward motor power (-255...255) depending on the distance error
            # apply rotational motor power (-255...255) depending on angle error
            self.hardware.drive(int(forward_power), angular=int(angle_power))
            await asyncio.sleep(self.refresh_rate)

            if self.goal_available_event.is_set():
                self.logger.info("Cancelling drive.")
                break

        self.hardware.stop_motors()

        # cancel the rest of the operations if timeout was reached (goal wasn't achieved)
        if (time.time() - start_time) >= self.pid_timeout:
            self.logger.warning("Drive timed out! Cancelling operation.")
            self.cancel()
        else:
            self.logger.info("Drive successful! Error is %s" % dist_error)

    # async def _drive_to_point(self):
    #     await self.update_current_state()
    #     dist_error, lateral_error = self.shift_reference_frame(
    #         self.goal_x - self.current_x, self.goal_y - self.current_y, self.goal_th
    #     )
    #
    #     self.logger.info(
    #         "Driving to x=%s, y=%s. distance error is %s, lateral error is %s" % (
    #             self.goal_x, self.goal_y, dist_error, lateral_error)
    #     )
    #
    #     self.dist_pid.reset()
    #     self.angular_pid.reset()
    #
    #     start_time = time.time()
    #     prev_time = time.time()
    #
    #     while abs(dist_error) > self.dist_error or (time.time() - start_time) < self.pid_timeout:
    #         await self.update_current_state()
    #         dist_error, lateral_error = self.shift_reference_frame(
    #             self.goal_x - self.current_x, self.goal_y - self.current_y, self.goal_th
    #         )
    #
    #         current_time = time.time()
    #         dt = current_time - prev_time
    #         prev_time = current_time
    #         forward_power = self.dist_pid.update(dist_error, dt)
    #         angle_power = self.angular_pid.update(lateral_error, dt)
    #
    #         self.hardware.drive(int(forward_power), angular=int(angle_power))
    #         await asyncio.sleep(self.refresh_rate)
    #
    #         if self.goal_available_event.is_set():
    #             self.logger.info("Cancelling drive.")
    #             break
    #
    #     self.hardware.stop_motors()
    #
    #     if (time.time() - start_time) >= self.pid_timeout:
    #         self.logger.warning("Drive timed out! Cancelling operation.")
    #         self.cancel()
    #     else:
    #         self.logger.info("Drive successful! Error is %s" % dist_error)

    def cancel(self):
        """Cancel a goal request"""
        self.goal_available_event.set()
        self.position_sub.enabled = False

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
        self.goal_x = x
        self.goal_y = y
        self.end_goal_th = theta

        self.goal_available_event.clear()
        self.position_sub.enabled = True

        self.logger.info("Submitting goal (x=%s, y=%s, th=%s)" % (x, y, theta))
