import cv2
import time
import json
import base64
import struct
import asyncio
import numpy as np
import aioprocessing
from http.client import HTTPConnection

from atlasbuggy import Node
from atlasbuggy.opencv.messages import ImageMessage


class CameraWebsiteClient(Node):
    def __init__(self, width, height, address=("10.76.76.1", 80), enabled=True):
        super(CameraWebsiteClient, self).__init__(enabled)
        # http://user:something@10.76.76.1/api/robot/rightcam

        self.address = address

        self.requested_width = width
        self.requested_height = height

        self.width = width
        self.height = height
        self.num_frames = 0

        self.reader = None
        self.writer = None

        # self.connection = None
        # self.response_lock = Lock()

        self.chunk_size = int(self.width * self.height / 8)

        self.fps = 30.0
        self.length_sec = 0.0

        self.fps_sum = 0.0
        self.fps_avg = 30.0
        self.prev_t = None

        self.credentials = base64.b64encode(b'robot:naboris').decode('ascii')

        self.process = aioprocessing.AioProcess(target=self.retrieve_images)
        self.manager = aioprocessing.AioManager()
        self.shared_list = self.manager.list()
        self.image_queue = aioprocessing.AioQueue()
        self.image_lock = aioprocessing.AioLock()
        self.exit_event = aioprocessing.AioEvent()
        self.new_data_event = aioprocessing.AioEvent()

    async def setup(self):
        self.shared_list.extend([b'', 0.0, 0, 0])
        self.process.start()

    def retrieve_images(self):
        connection = HTTPConnection("%s:%s" % (self.address[0], self.address[1]))
        headers = {
            'Content-type':  'image/jpeg',
            'Authorization': 'Basic %s' % self.credentials
        }
        connection.request("GET", "/api/robot/rightcam_time", headers=headers)
        response = connection.getresponse()

        buffer = b''
        timestamp_len = 8
        width_len = 2
        height_len = 2

        for resp in self.recv(response):
            if len(resp) == 0:
                self.logger.info("Response ended. Closing.")
                return

            if self.exit_event.is_set():
                self.logger.info("Exit event set. Closing.")
                return

            buffer += resp
            response_1 = buffer.find(b'\xff\xd8')
            response_2 = buffer.find(b'\xff\xd9')

            if response_1 != -1 and response_2 != -1:
                response_2 += 2
                timestamp_index = response_2 + timestamp_len
                width_index = timestamp_index + width_len
                height_index = width_index + height_len
                if height_index >= len(buffer):
                    continue

                jpg = buffer[response_1:response_2]
                timestamp = struct.unpack('d', buffer[response_2:timestamp_index])[0]
                width = int.from_bytes(buffer[timestamp_index:width_index], 'big')
                height = int.from_bytes(buffer[width_index:height_index], 'big')

                buffer = buffer[height_index:]

                with self.image_lock:
                    self.shared_list[0] = jpg
                    self.shared_list[1] = timestamp
                    self.shared_list[2] = width
                    self.shared_list[3] = height
                    self.new_data_event.set()

    def recv(self, response):
        buf = response.read(self.chunk_size)
        while buf:
            yield buf
            buf = response.read(self.chunk_size)

    async def loop(self):
        while True:
            await self.new_data_event.coro_wait()
            if self.exit_event.is_set():
                return
            with self.image_lock:
                jpg, timestamp, width, height = self.shared_list

            image = self.to_image(jpg)

            if width != self.width or height != self.height:
                image = cv2.resize(image, (self.requested_width, self.requested_height))

                if width != self.width:
                    self.logger.info("Size changed to %s, %s" % (width, height))
                    self.width = width

                if height != self.height:
                    self.logger.info("Size changed to %s, %s" % (width, height))
                    self.height = height

            message = ImageMessage(image, self.num_frames, timestamp=timestamp)
            self.log_to_buffer(time.time(), "Web socket image received: %s" % message)
            self.check_buffer(self.num_frames)

            await self.broadcast(message)

            self.poll_for_fps()
            self.log_to_buffer(time.time(), "image time diff: %s, fps: %s" % (time.time() - timestamp, self.fps))
            self.new_data_event.clear()
            await asyncio.sleep(0.01)

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

        self.exit_event.set()
        self.new_data_event.set()
        await self.process.coro_join()
