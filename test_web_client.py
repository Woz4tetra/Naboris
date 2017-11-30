from atlasbuggy import Orchestrator, run

from naboris.website_client import NaborisSocketClient

class TestOrchestrator(Orchestrator):
    def __init__(self, event_loop):
        self.set_default(level=10)
        super(TestOrchestrator, self).__init__(event_loop)

        self.client = NaborisSocketClient(640, 480)

        self.add_nodes(self.client)

run(TestOrchestrator)
