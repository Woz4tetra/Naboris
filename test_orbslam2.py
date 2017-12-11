import asyncio
from atlasbuggy import Orchestrator, run

from naboris.orbslam2_node import OrbslamNode
from naboris.stereo_cam import StereoCam
from naboris.website_client import CameraWebsiteClient
from naboris.picamera import PiCamera


class OrbSlamOrchestrator(Orchestrator):
    def __init__(self, event_loop):
        # self.set_default(write=log, level=20)
        super(OrbSlamOrchestrator, self).__init__(event_loop)

        width, height = 640, 480
        self.picamera = PiCamera(width, height)
        self.client = CameraWebsiteClient(width, height)
        self.stereo_cam = StereoCam(0.13208)
        self.orbslam2 = OrbslamNode(width, height)

        self.add_nodes(self.client, self.picamera, self.stereo_cam)

        self.subscribe(self.stereo_cam, self.orbslam2, self.orbslam2.stereo_tag)

        self.subscribe(self.picamera, self.stereo_cam, self.stereo_cam.camera1_tag)
        self.subscribe(self.client, self.stereo_cam, self.stereo_cam.camera2_tag)

    async def loop(self):
        while True:
            await asyncio.sleep(0.25)
            if self.orbslam2.initialized:
                print("Tracking state: %s" % self.orbslam2.orb.tracking_state)
                print(self.orbslam2.orb.get_trajectory_points())
                print()
            else:
                print("Orbslam not ready.")
                await asyncio.sleep(0.75)

run(OrbSlamOrchestrator)
