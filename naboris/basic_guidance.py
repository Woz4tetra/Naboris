import math
import time
import asyncio

from atlasbuggy import Node

from naboris.pid import PID


class BasicGuidance(Node):
    def __init__(self, enabled=True):
        super(BasicGuidance, self).__init__(enabled)

        self.position_tag = "position"
        self.position_sub = self.define_subscription(self.position_tag, "odom_pos")
        self.position_queue = None

        self.hardware_tag = "hardware"
        self.hardware_sub = self.define_subscription(self.hardware_tag, queue_size=None)
        self.hardware = None

        self.goal_x = None
        self.goal_y = None
        self.goal_th = None

        self.angle_pid = PID(1.0, 0.0, "tuning")

        self.prev_time = time.time()

    def take(self):
        self.position_queue = self.position_sub.get_queue()
        self.hardware = self.hardware_sub.get_producer()

    async def loop(self):
        self.prev_time = time.time()
        while True:
            x, y, th = await self.position_queue.get()


    def get_angle_error(self, goal_heading):
        angle_error = goal_heading - self.current_th
        # force the relative angle (goal - current) into -pi...pi range
        angle_error %= 2 * math.pi
        if math.pi <= angle_error <= 2 * math.pi:
            angle_error -= 2 * math.pi

        return angle_error

    # def update_current_values(self):


    def point_to_angle(self, goal_heading):
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        motor_power = int(self.angle_pid.update(angle_error, dt))
        if not (-255 <= motor_power <= 255):
            motor_power = int(math.copysign(255, motor_power))

        self.hardware.spin(motor_power)

    def get_goal_angle(self):
        angle = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        if angle < 0:
            angle += 2 * math.pi

        return angle

    def set_goal(self, x, y, theta=None):
        self.goal_x = x
        self.goal_y = y
        self.goal_th = theta
