import cv2
import time
import json
import asyncio
import numpy as np
from threading import Lock
from http.client import HTTPConnection

from atlasbuggy import Node
from atlasbuggy.opencv import ImageMessage


class NaborisSocketClient(Node):
    def __init__(self, address=("10.76.76.1", 80), enabled=True):
        super(NaborisSocketClient, self).__init__(enabled)
        self.address = address

        self.buffer = b''

        self.width = 640
        self.height = 480
        self.num_frames = 0
        self.current_frame_num = 0

        self.reader = None
        self.writer = None

        self.connection = None
        self.response_lock = Lock()

        self.chunk_size = int(self.width * self.height / 16)

        self.fps = 30.0
        self.length_sec = 0.0

        self.fps_sum = 0.0
        self.fps_avg = 30.0
        self.prev_t = None

    def set_frame(self):
        pass

    def set_pause(self):
        pass

    def get_pause(self):
        return False

    async def setup(self):
        self.connection = HTTPConnection("%s:%s" % (self.address[0], self.address[1]))

    def recv(self, response):
        buf = response.read(self.chunk_size)
        while buf:
            yield buf
            with self.response_lock:
                buf = response.read(self.chunk_size)

    async def loop(self):
        headers = {'Content-type': 'image/jpeg'}
        self.connection.request("GET", "/api/robot/rightcam", headers=headers)
        response = self.connection.getresponse()

        for resp in self.recv(response):
            if len(resp) == 0:
                return

            self.buffer += resp
            response_1 = self.buffer.find(b'\xff\xd8')
            response_2 = self.buffer.find(b'\xff\xd9')

            if response_1 != -1 and response_2 != -1:
                jpg = self.buffer[response_1:response_2 + 2]
                self.buffer = self.buffer[response_2 + 2:]
                image = self.to_image(jpg)

                message = ImageMessage(image, self.current_frame_num)
                await self.broadcast(message)

                self.current_frame_num += 1
                self.num_frames += 1
            await asyncio.sleep(0.0)

            self.poll_for_fps()

    def send_command(self, command):
        with self.response_lock:
            self.logger.debug("sending: %s" % command)
            headers = {'Content-type': 'application/json', 'Accept': 'text/plain'}
            self.connection.request("POST", "/cmd?command=" + str(command), json.dumps(""), headers)
            response = self.connection.getresponse()
            if response.status != 200:
                raise RuntimeError("Response was not OK: %s, %s" % (response.status, response.reason))

    def to_image(self, byte_stream):
        return cv2.imdecode(np.fromstring(byte_stream, dtype=np.uint8), 1)

    def poll_for_fps(self):
        if self.prev_t is None:
            self.prev_t = time.time()
            return 0.0

        self.length_sec = time.time() - self.start_time
        self.num_frames += 1
        self.fps_sum += 1 / (time.time() - self.prev_t)
        self.fps_avg = self.fps_sum / self.num_frames
        self.prev_t = time.time()

        self.fps = self.fps_avg

    async def teardown(self):
        if self.reader is not None:
            self.reader.close()
        if self.writer is not None:
            self.writer.close()