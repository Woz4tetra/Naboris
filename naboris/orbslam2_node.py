class Orbslam2Node(Node):
    def __init__(self, width, height, enabled=True):
        super(Orbslam2Node, self).__init__(enabled)
        self.orb = System("data/ORBvoc.txt", "data/config/TUM1.yaml", width, height, Sensor.STEREO)

    async def setup(self):
        self.orb.set_use_viewer(True)
        self.orb.initialize()

    async def loop(self):
        self.orb.process_image_stereo(image, timestamp)
