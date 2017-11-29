import re
import sys
import asyncio
import traceback
from atlasbuggy import Node


class NaborisCLI(Node):
    def __init__(self, enabled=True, prompt_text=">> "):
        super(NaborisCLI, self).__init__(enabled)
        self.prompt_text = prompt_text
        self.queue = asyncio.Queue()

        self.should_exit = False

        self.hardware_tag = "hardware"
        self.hardware_sub = self.define_subscription(
            self.hardware_tag,
            queue_size=None,
            required_methods=(
                "drive",
                "look_straight",
                "look_up",
                "look_down",
                "look_left",
                "look_right",
                "set_turret",
                "set_all_leds",
                "stop_motors",
            )
        )
        self.hardware = None

        self.sounds_tag = "sounds"
        self.sounds_sub = self.define_subscription(self.sounds_tag, queue_size=None, is_required=False)
        self.sounds = None

        self.guidance_tag = "guidance"
        self.guidance_sub = self.define_subscription(
            self.guidance_tag, queue_size=None,
            required_methods=("goto", "cancel")
        )
        self.guidance = None

        self.available_commands = dict(
            q=self.exit,
            l=self.spin_left,
            r=self.spin_right,
            d=self.drive,
            h=self.help,
            # euler=self.get_orientation,
            goto=self.goto_pos,
            cancel=self.cancel_goto_pos,
            look=self.look,
            s=self.my_stop,
            red=self.red,
            green=self.green,
            blue=self.blue,
            white=self.white,
            rgb=self.rgb,
            hello=self.say_hello,
            alert=self.say_alert,
            sound=self.say_random_sound,
            # startvideo=self.start_new_video,
            # stopvideo=self.stop_recording,
            # photo=self.take_a_photo
        )

    async def setup(self):
        self.event_loop.add_reader(sys.stdin, self.handle_stdin)

    def handle_stdin(self):
        data = sys.stdin.readline()
        asyncio.async(self.queue.put(data))

    async def loop(self):
        while True:
            print("\r%s" % self.prompt_text, end="")
            data = await self.queue.get()
            try:
                self.handle_input(data.strip('\n'))
            except BaseException as error:
                traceback.print_exc()
                print(error)
                self.logger.warning("Failed to parse input: " + repr(data))

            if self.should_exit:
                return

    def take(self):
        self.hardware = self.hardware_sub.get_producer()
        if self.is_subscribed(self.sounds_tag):
            self.sounds = self.sounds_sub.get_producer()

    def spin_left(self, params):
        value = int(params) if len(params) > 0 else 75
        self.hardware.spin(value)

    def spin_right(self, params):
        value = int(params) if len(params) > 0 else 75
        self.hardware.spin(-value)

    def drive(self, params):
        angle = 0
        speed = 75
        angular = 0
        if len(params) > 1:
            values = params.split(" ")

            try:
                if len(values) >= 1:
                    angle = int(values[0])

                if len(values) >= 2:
                    speed = int(values[1])

                if len(values) >= 3:
                    angular = int(values[2])
            except ValueError:
                print("Failed to parse input:", repr(values))
        self.hardware.drive(speed, angle, angular)

    def look(self, params):
        data = params.split(" ")
        if data[0] == "":
            self.hardware.look_straight()
        elif data[0] == "down":
            if len(data) > 1:
                value = int(data[1])
                self.hardware.look_up(value)
            else:
                self.hardware.look_down()
        elif data[0] == "up":
            if len(data) > 1:
                value = int(data[1])
                self.hardware.look_up(value)
            else:
                self.hardware.look_up()
        elif data[0] == "left":
            if len(data) > 1:
                value = int(data[1])
                self.hardware.look_left(value)
            else:
                self.hardware.look_left()
        elif data[0] == "right":
            if len(data) > 1:
                value = int(data[1])
                self.hardware.look_right(value)
            else:
                self.hardware.look_right()
        else:
            if len(data) == 2:
                yaw, azimuth = data
                self.hardware.set_turret(int(yaw), int(azimuth))

    def rgb(self, params):
        r, g, b = [int(x) for x in params.split(" ")]
        self.hardware.set_all_leds(r, g, b)

    def red(self, params):
        value = int(params) if len(params) > 0 else 15
        self.hardware.set_all_leds(value, 0, 0)

    def green(self, params):
        value = int(params) if len(params) > 0 else 15
        self.hardware.set_all_leds(0, value, 0)

    def blue(self, params):
        value = int(params) if len(params) > 0 else 15
        self.hardware.set_all_leds(0, 0, value)

    def white(self, params):
        value = int(params) if len(params) > 0 else 15
        self.hardware.set_all_leds(value, value, value)

    def exit(self, params):
        self.should_exit = True

    def my_stop(self, params):
        self.hardware.stop_motors()

    def say_hello(self, params):
        if self.is_subscribed(self.sounds_tag):
            self.sounds.play("emotes/hello")
        self.hardware.pause(0.5)
        self.hardware.set_all_leds(0, 0, 15)
        self.hardware.look_straight()
        self.hardware.pause(0.25)
        for _ in range(2):
            self.hardware.look_up()
            self.hardware.pause(0.25)
            self.hardware.look_down()
            self.hardware.pause(0.25)
        self.hardware.look_straight()
        self.hardware.set_all_leds(15, 15, 15)

    def say_alert(self, params):
        if self.is_subscribed(self.sounds_tag):
            self.sounds.play("alert/high_alert")
        self.hardware.pause(0.5)
        self.hardware.set_all_leds(15, 0, 0)
        self.hardware.pause(0.05)
        self.hardware.look_straight()
        for _ in range(3):
            self.hardware.look_left()
            self.hardware.pause(0.15)
            self.hardware.look_right()
            self.hardware.pause(0.15)
        self.hardware.look_straight()
        self.hardware.pause(0.1)
        self.hardware.set_all_leds(15, 15, 15)

    def say_random_sound(self, params):
        if self.is_subscribed(self.sounds_tag):
            self.sounds.play_random_sound()

    def goto_pos(self, params):
        if not self.is_subscribed(self.guidance_tag):
            print("autonomous mode not enabled.")
            return
        data = params.split(" ")
        x = None
        y = None
        theta = None
        if len(data) >= 2:
            x = float(data[0])
            y = float(data[1])
            print("going to %0.4f, %0.4f" % (x, y), end="")
        if len(data) >= 3:
            theta = float(data[2])
            print(", %0.4f" % theta, end="")
        print()

        self.guidance.goto(x, y, theta)

    def cancel_goto_pos(self, params):
        self.guidance.cancel()

    def help(self, params):
        print("\nAvailable commands:")
        for command in self.available_commands:
            print("\t%s" % command)

        print(self.prompt_text, end="")

    def check_commands(self, line, commands):
        function = None
        current_command = ""
        for command, fn in commands.items():
            if line.startswith(command) and len(command) > len(current_command):
                function = fn
                current_command = command
        if function is not None:
            function(line[len(current_command):].strip(" "))

    def handle_input(self, line):
        if type(line) == str:
            self.check_commands(line, self.available_commands)
