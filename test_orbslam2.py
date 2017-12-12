import asyncio
import numpy as np

from atlasbuggy import Orchestrator, run
from atlasbuggy.opencv import OpenCVPipeline, OpenCVViewer

from naboris.orbslam2_node import OrbslamNode
from naboris.stereo_cam import StereoCam
from naboris.website_client import CameraWebsiteClient
from naboris.picamera import PiCamera


class StereoStitcher(OpenCVPipeline):
    def __init__(self, enabled=True):
        super(StereoStitcher, self).__init__(enabled)

    async def pipeline(self, message):
        return np.concatenate((message.left_image, message.right_image), axis=1)

class OrbSlamOrchestrator(Orchestrator):
    def __init__(self, event_loop):
        self.set_default(write=True, level=20)
        super(OrbSlamOrchestrator, self).__init__(event_loop)

        width, height = 640, 480
        self.picamera = PiCamera(width, height)
        self.client = CameraWebsiteClient(width, height)
        self.stereo_cam = StereoCam(0.13208)
        self.stereo_sticher = StereoStitcher()
        self.viewer = OpenCVViewer()
        self.orbslam2 = OrbslamNode(width, height, enabled=False)

        self.add_nodes(self.orbslam2, self.client, self.picamera, self.stereo_cam, self.viewer, self.stereo_sticher)


        self.subscribe(self.stereo_cam, self.stereo_sticher, self.stereo_sticher.capture_tag)

        self.subscribe(self.stereo_sticher, self.viewer, self.viewer.capture_tag)

        self.subscribe(self.stereo_cam, self.orbslam2, self.orbslam2.stereo_tag)

        self.subscribe(self.picamera, self.stereo_cam, self.stereo_cam.fast_cam_tag)
        self.subscribe(self.client, self.stereo_cam, self.stereo_cam.slow_cam_tag)

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


class ClientCamOrchestrator(Orchestrator):
    def __init__(self, event_loop):
        self.set_default(write=False, level=20)
        super(ClientCamOrchestrator, self).__init__(event_loop)

        self.client = CameraWebsiteClient()
        self.viewer = OpenCVViewer()

        self.add_nodes(self.client, self.viewer)

        self.subscribe(self.client, self.viewer, self.viewer.capture_tag)


# run(OrbSlamOrchestrator)
run(ClientCamOrchestrator)
