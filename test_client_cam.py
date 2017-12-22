from atlasbuggy import Orchestrator, run
from atlasbuggy.opencv import OpenCVViewer

from naboris.website_client import CameraWebsiteClient


class ClientCamOrchestrator(Orchestrator):
    def __init__(self, event_loop):
        self.set_default(write=True, level=10)
        super(ClientCamOrchestrator, self).__init__(event_loop)

        self.client = CameraWebsiteClient(720, 480)
        self.viewer = OpenCVViewer(enabled=True)

        self.add_nodes(self.client, self.viewer)

        self.subscribe(self.client, self.viewer, self.viewer.capture_tag)


run(ClientCamOrchestrator)
