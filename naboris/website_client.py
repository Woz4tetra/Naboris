import cv2
import time
import json
import base64
import struct
import asyncio
import numpy as np
import aioprocessing
from PIL import Image
from io import StringIO
from http.client import HTTPConnection

from atlasbuggy import Node
from atlasbuggy.opencv.messages import ImageMessage


class WebsiteClient(Node):
    def __init__(self, width, height, address=("10.76.76.1", 80), enabled=True, enable_images=True):
        super(WebsiteClient, self).__init__(enabled)
        # http://user:something@10.76.76.1/api/robot/rightcam

        self.address = address

        self.requested_width = width
        self.requested_height = height

        self.width = width
        self.height = height
        self.num_frames = 0

        self.reader = None
        self.writer = None

        self.enable_images = enable_images

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

        # self.manager = aioprocessing.AioSyncManager()

        self.connection = HTTPConnection("%s:%s" % (self.address[0], self.address[1]))

        self.headers = {
            'Content-type':  'image/jpeg',
            'Authorization': 'Basic %s' % self.credentials
        }

        if self.enable_images:
            self.connection.request("GET", "/api/robot/rightcam_meta", headers=self.headers)
            response = self.connection.getresponse()
        else:
            response = None

        self.image_process = aioprocessing.AioProcess(target=self.retrieve_images, args=(response,))
        self.image_queue = aioprocessing.AioQueue()

        self.connection.request("GET", "/cmd", headers=self.headers)
        response = self.connection.getresponse()

        self.command_process = aioprocessing.AioProcess(target=self.retrieve_commands, args=(response,))
        self.command_queue = aioprocessing.AioQueue()

        self.exit_event = aioprocessing.AioEvent()

        self.command_service_tag = "commands"
        self.define_service(self.command_service_tag, str)

    async def setup(self):
        if self.enable_images:
            self.image_process.start()
        self.command_process.start()

    def retrieve_images(self, response):
        self.wait_for_header(response)
        buffer = b''

        prev_time = time.time()
        num_frames = 0
        sum_fps = 0

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

                self.image_queue.put((frame, timestamp, width, height))

                current_time = time.time()
                dt = current_time - prev_time
                prev_time = current_time
                sum_fps += 1 / dt
                num_frames += 1

                self.logger.info("avg fps: %s" % (sum_fps / num_frames))
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
            if self.exit_event.is_set():
                self.logger.info("Exit event set. Loop exiting.")
                return

            if self.enable_images:
                jpg, timestamp, width, height = await self.image_queue.coro_get()

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

            if not self.command_queue.empty():
                command = await self.command_queue.coro_get()
                await self.broadcast(command, self.command_service_tag)
            else:
                if self.enable_images:
                    await asyncio.sleep(0.0)
                else:
                    await asyncio.sleep(0.01)

    def retrieve_commands(self, response):
        assert response.read(2) == self.response_start_header
        try:
            while not self.exit_event.is_set():
                status, buffer = self.get_buffer(response, 4)
                if status: return
                response_len = int.from_bytes(buffer, self.endian)
                if response_len == 0:
                    continue

                status, buffer = self.get_buffer(response, 8)
                if status: return
                batch_timestamp = struct.unpack('d', buffer)[0]

                status, buffer = self.get_buffer(response, response_len)
                if status: return
                batch = buffer.decode()

                self.logger.info("batch time diff: %s" % (time.time() - batch_timestamp))

                sent_commands = False
                packets = batch.split(";\n")
                for packet in packets:
                    if len(packet.strip()) > 0:
                        timestamp, command = packet.split("-")
                        timestamp = float(timestamp)
                        self.logger.info("command received: %s, time diff: %s" % (command, time.time() - timestamp))
                        self.command_queue.put(command)

                        sent_commands = True
                if not sent_commands:
                    time.sleep(0.01)

        except BaseException:
            self.exit_event.set()
            raise

    def to_image(self, byte_stream):
        t0 = time.time()

        # TODO: SUPER slow... causing a rate of 25 fps to dip to 5 fps
        image = cv2.imdecode(np.fromstring(byte_stream, dtype=np.uint8), 1)
        # image = cv2.imdecode(np.asarray(byte_stream, dtype=np.uint8), 1)
        # image = np.array(Image.open(StringIO(byte_stream)))
        t1 = time.time()
        print(t1 - t0)
        return image

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
        self.logger.info("exit event set")

        if self.enable_images:
            self.logger.info("waiting for image process...")
            await self.image_process.coro_join()

        self.logger.info("waiting for command process...")
        await self.command_process.coro_join()
        self.logger.info("done!")
