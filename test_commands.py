from atlasbuggy import Orchestrator, run

from naboris.hardware_interface import HardwareInterface
from naboris.cli import NaborisCLI
from naboris.website_client import WebsiteConnection
from naboris.website_client import CommandClient

log = True


class CommandOrchestrator(Orchestrator):
    def __init__(self, event_loop):
        self.set_default(write=log, level=20)
        super(CommandOrchestrator, self).__init__(event_loop)

        self.hardware = HardwareInterface(enabled=False)
        self.cli = NaborisCLI(enabled=False)

        client = WebsiteConnection("10.76.76.1", user="robot", password="naboris", timeout=5)

        self.command_client = CommandClient(client)

        self.add_nodes(self.hardware, self.cli, self.command_client)

        self.subscribe(self.hardware, self.cli, self.cli.hardware_tag)
        self.subscribe(self.command_client, self.cli, self.cli.command_source_tag)


run(CommandOrchestrator)
