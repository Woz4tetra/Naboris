from atlasbuggy import Orchestrator, run

from naboris.hardware_interface import HardwareInterface
from naboris.cli import NaborisCLI
from naboris.soundfiles import Sounds
from naboris.basic_guidance import BasicGuidance
from naboris.odometry import Odometry


class NaborisOrchestrator(Orchestrator):
    def __init__(self, event_loop):
        # self.set_default(write=log, level=30)
        super(NaborisOrchestrator, self).__init__(event_loop)

        self.hardware = HardwareInterface()
        self.cli = NaborisCLI()
        self.sounds = Sounds(
            "sounds", "/home/pi/Music/Bastion/",
            ("humming", "curiousity", "nothing", "confusion", "concern", "sleepy", "vibrating"),
            enabled=True
        )
        self.odometry = Odometry()
        self.guidance = BasicGuidance(enabled=True)

        self.add_nodes(self.hardware, self.cli, self.sounds, self.guidance, self.odometry)

        self.subscribe(self.sounds, self.cli, self.cli.sounds_tag)
        self.subscribe(self.hardware, self.cli, self.cli.hardware_tag)
        self.subscribe(self.guidance, self.cli, self.cli.guidance_tag)

        self.subscribe(self.odometry, self.guidance, self.guidance.position_tag)
        self.subscribe(self.hardware, self.guidance, self.guidance.hardware_tag)

        self.subscribe(self.hardware, self.odometry, self.odometry.right_encoder_tag, self.hardware.right_encoder_service)
        self.subscribe(self.hardware, self.odometry, self.odometry.left_encoder_tag, self.hardware.left_encoder_service)
        self.subscribe(self.hardware, self.odometry, self.odometry.bno055_tag, self.hardware.bno055_service)

        # async def loop(self):
        #     while True:
        #         print(self.hardware.left_tick, self.hardware.right_tick)
        #         await asyncio.sleep(0.1)


run(NaborisOrchestrator)
