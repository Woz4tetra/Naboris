from atlasbuggy import Orchestrator, run

from naboris.website_client import CameraWebsiteClient
from naboris.picamera import PiCamera

class OrbSlamOrchestrator(Orchestrator):
    def __init__(self, event_loop):
        # self.set_default(write=log, level=20)
        super(NaborisOrchestrator, self).__init__(event_loop)


        self.picamera = PiCamera()
        self.client = CameraWebsiteClient(640, 480)
        self.stereo_cam = StereoCam()
        self.orbslam2 = Orbslam2Node(640, 480)

        self.add_nodes(self.client, self.picamera, self.stereo_cam)

        self.subscribe(self.picamera, self.stereo_cam, self.stereo_cam.camera1)
        self.subscribe(self.client, self.stereo_cam, self.stereo_cam.camera2)
