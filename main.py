from atlasbuggy import Orchestrator, run

from naboris.hardware_interface import HardwareInterface
from naboris.cli import NaborisCLI
from naboris.soundfiles import Sounds
from naboris.basic_guidance import BasicGuidance
from naboris.odometry import Odometry
from naboris.website_client import CommandClient
from naboris.website_client import CameraClient
from naboris.website_client import WebsiteConnection
from naboris.picamera import PiCamera

log = True


class NaborisOrchestrator(Orchestrator):
    def __init__(self, event_loop):
        self.set_default(write=log, level=20)
        super(NaborisOrchestrator, self).__init__(event_loop)

        self.hardware = HardwareInterface()
        self.cli = NaborisCLI(enabled=True)
        self.sounds = Sounds(
            "sounds", "/home/pi/Music/Bastion/",
            ("humming", "curiousity", "nothing", "confusion", "concern", "sleepy", "vibrating"),
            enabled=False
        )

        "10.76.76.1"
        self.odometry = Odometry()
        self.guidance = BasicGuidance(enabled=False)
        self.client = WebsiteClient(720, 480, enabled=False, enable_images=False)
        self.picamera = PiCamera(enabled=False)

        self.add_nodes(self.hardware, self.cli, self.sounds, self.guidance, self.odometry, self.client, self.picamera)

        self.subscribe(self.sounds, self.cli, self.cli.sounds_tag)
        self.subscribe(self.hardware, self.cli, self.cli.hardware_tag)
        self.subscribe(self.guidance, self.cli, self.cli.guidance_tag)
        self.subscribe(self.client, self.cli, self.cli.command_source_tag, self.client.command_service_tag)

        self.subscribe(self.odometry, self.guidance, self.guidance.odometry_tag)
        self.subscribe(self.hardware, self.guidance, self.guidance.hardware_tag)

        self.subscribe(self.hardware, self.odometry, self.odometry.encoder_tag, self.hardware.encoder_service)
        self.subscribe(self.hardware, self.odometry, self.odometry.bno055_tag, self.hardware.bno055_service)

        # async def loop(self):
        #     while True:
        #         print(self.hardware.left_tick, self.hardware.right_tick)
        #         await asyncio.sleep(0.1)


run(NaborisOrchestrator)
