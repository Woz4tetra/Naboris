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

        self.response_start_header = b'\xbb\x08'
        self.message_start_header = b'\xde\xad\xbe\xef'
        self.frame_len = 4
        self.timestamp_len = 8
        self.width_len = 2
        self.height_len = 2
        self.endian = 'big'

        self.chunk_size = int(self.width * self.height / 2)

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
        connection.request("GET", "/api/robot/rightcam_meta", headers=headers)
        response = connection.getresponse()

        self.wait_for_header(response)
        buffer = b''

        while True:
            status, buffer = self.get_buffer(response, len(self.message_start_header))

            if status: return

            if buffer == self.message_start_header:
                status, buffer = self.get_buffer(response, self.frame_len)

                if status: return

                frame_size = int.from_bytes(buffer, self.endian)
                if frame_size > self.requested_width * self.requested_height:
                    self.exit_event.set()
                    raise RuntimeError("Frame size is larger than the expected amount (%s > %s). Exiting." % (
                        frame_size, self.requested_width * self.requested_height)
                    )

                status, frame = self.get_buffer(response, frame_size)

                if status: return

                status, buffer = self.get_buffer(response, self.timestamp_len)
                timestamp = struct.unpack('d', buffer)[0]

                if status: return

                status, buffer = self.get_buffer(response, self.width_len)
                width = int.from_bytes(buffer, self.endian)

                if status: return

                status, buffer = self.get_buffer(response, self.height_len)
                height = int.from_bytes(buffer, self.endian)

                if status: return

                buffer = response.read(2)
                if buffer != b'\r\n':
                    self.exit_event.set()
                    raise RuntimeError("Response doesn't end with a newline! (%s)" % (buffer))

                with self.image_lock:
                    self.shared_list[0] = frame
                    self.shared_list[1] = timestamp
                    self.shared_list[2] = width
                    self.shared_list[3] = height
                    self.new_data_event.set()

            else:
                self.exit_event.set()
                raise RuntimeError("Response doesn't match header (received %s != %s)" % (buffer, self.message_start_header))

    def wait_for_header(self, response):
        buffer = b''
        while not self.exit_event.is_set():
            char = response.read(1)
            buffer += char
            if len(buffer) > 100:
                self.logger.error("Still searching for start header. Current response:", buffer)
                buffer = b''

            if char == self.response_start_header[0:1]:
                char = response.read(1)
                buffer += char
                if char == self.response_start_header[1:2]:
                    return

    def get_buffer(self, response, length):
        buffer = response.read(length)
        if len(buffer) == 0:
            self.logger.info("Response ended. Closing.")
            return True, buffer

        if self.exit_event.is_set():
            self.logger.info("Exit event set. Closing.")
            return True, buffer

        return False, buffer

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
