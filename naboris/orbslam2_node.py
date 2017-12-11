from atlasbuggy import Node

from orbslam2 import System, Sensor


class OrbslamNode(Node):
    def __init__(self, width, height, enabled=True):
        super(OrbslamNode, self).__init__(enabled)

        self.orb = System("data/ORBvoc.txt", "data/config/TUM1.yaml", width, height, Sensor.STEREO)

        self.stereo_tag = "stereo"
        self.stereo_sub = self.define_subscription(self.stereo_tag)
        self.stereo_queue = None

        self.initialized = False
        self.tracking_state = None

    def take(self):
        self.stereo_queue = self.stereo_sub.get_queue()

    async def setup(self):
        self.orb.set_use_viewer(True)
        self.orb.initialize()

    async def loop(self):
        while self.orb.is_running():
            while not self.stereo_queue.empty():
                stereo_message = await self.stereo_queue.get()
                self.initialized = self.orb.process_image_stereo(stereo_message.left_image, stereo_message.right_image,
                                                                 stereo_message.timestamp)
                self.tracking_state = self.orb.get_tracking_state()

    async def teardown(self):
        self.orb.shutdown()
