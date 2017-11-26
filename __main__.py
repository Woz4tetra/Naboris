import asyncio

from atlasbuggy import Orchestrator, run

from naboris.hardware_interface import HardwareInterface
from naboris.cli import NaborisCLI
from naboris.soundfiles import Sounds

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
        self.add_nodes(self.hardware, self.cli, self.sounds)

        self.subscribe(self.hardware, self.cli, self.cli.hardware_tag)
        self.subscribe(self.sounds, self.cli, self.cli.sounds_tag)

    # async def loop(self):
    #     while True:
    #         print(self.hardware.left_tick, self.hardware.right_tick)
    #         await asyncio.sleep(0.1)

run(NaborisOrchestrator)
