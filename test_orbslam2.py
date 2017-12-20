import asyncio
import numpy as np

from atlasbuggy import Orchestrator, run
from atlasbuggy.opencv import OpenCVPipeline, OpenCVViewer, OpenCVRecorder

from naboris.orbslam2_node import OrbslamNode
from naboris.stereo_cam import StereoCam, StereoStitcher
from naboris.website_client import CameraWebsiteClient
from naboris.picamera import PiCamera
from naboris.stereo_image_saver import StereoImageSaver


class OrbSlamOrchestrator(Orchestrator):
    def __init__(self, event_loop):
        self.set_default(write=True, level=20)
        super(OrbSlamOrchestrator, self).__init__(event_loop)

        enable_viewing = True
        enable_recording = False

        width, height = 720, 480
        # width, height = 1296, 730
        max_time_diff = 0.5

        self.picamera = PiCamera(width, height)
        self.client = CameraWebsiteClient(width, height)
        self.stereo_cam = StereoCam(0.13208, max_time_diff=max_time_diff)
        self.stereo_sticher = StereoStitcher(enable_viewing or enable_recording)
        self.viewer = OpenCVViewer(enabled=enable_viewing)
        self.recorder = OpenCVRecorder(directory=None, enabled=enable_recording, width=width, height=height)
        self.orbslam2 = OrbslamNode(width, height, enabled=False, use_viewer=enable_viewing)
        self.image_saver = StereoImageSaver(enabled=True, delay=0.75)

        self.add_nodes(self.orbslam2, self.client, self.picamera, self.stereo_cam, self.viewer, self.stereo_sticher, self.recorder, self.image_saver)

        # image saver
        self.subscribe(self.stereo_cam, self.image_saver, self.image_saver.stereo_tag)

        # stereo_sticher
        self.subscribe(self.stereo_cam, self.stereo_sticher, self.stereo_sticher.capture_tag)

        # viewer
        self.subscribe(self.stereo_sticher, self.viewer, self.viewer.capture_tag)

        # recorder
        self.subscribe(self.stereo_sticher, self.recorder, self.recorder.capture_tag)

        # orbslam2
        self.subscribe(self.stereo_cam, self.orbslam2, self.orbslam2.stereo_tag)

        # stereo_cam
        self.subscribe(self.picamera, self.stereo_cam, self.stereo_cam.fast_cam_tag)
        self.subscribe(self.client, self.stereo_cam, self.stereo_cam.slow_cam_tag)

    async def loop(self):
        while True:
            await asyncio.sleep(0.25)
            if self.orbslam2.initialized:
                print("Tracking state: %s" % self.orbslam2.tracking_state)
                print(self.orbslam2.orb.get_trajectory_points())
                print()
            else:
                print("Orbslam not ready.")
                await asyncio.sleep(0.75)

run(OrbSlamOrchestrator)
