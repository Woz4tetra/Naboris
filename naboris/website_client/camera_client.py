import cv2
import time
import struct
import numpy as np

from atlasbuggy.opencv.messages import ImageMessage

from .base_client import BaseClient


class CameraClient(BaseClient):
    def __init__(self, client, width, height, enabled=True):
        super(CameraClient, self).__init__("/api/robot/rightcam_meta", "image/jpeg", client, enabled)

        self.width = width
        self.height = height

        self.requested_width = width
        self.requested_height = height

        self.endian = "big"
        self.frame_len = 4
        self.timestamp_len = 8
        self.width_len = 2
        self.height_len = 2

    def parse_response(self, response):
        status, buffer = self.get_buffer(response, len(self.message_start_header))
        if status: return buffer

        if buffer == self.message_start_header:
            status, buffer = self.get_buffer(response, self.frame_len)
            if status: return buffer

            frame_size = int.from_bytes(buffer, self.endian)
            if frame_size > self.requested_width * self.requested_height:
                self.exit_event.set()
                raise RuntimeError(
                    "Frame size is larger than the expected amount (%s > %s). Exiting." % (
                        frame_size, self.requested_width * self.requested_height)
                )

            status, frame = self.get_buffer(response, frame_size)
            if status: return buffer

            status, buffer = self.get_buffer(response, self.timestamp_len)
            timestamp = struct.unpack('d', buffer)[0]
            if status: return buffer

            status, buffer = self.get_buffer(response, self.width_len)
            width = int.from_bytes(buffer, self.endian)
            if status: return buffer

            status, buffer = self.get_buffer(response, self.height_len)
            height = int.from_bytes(buffer, self.endian)
            if status: return buffer

            buffer = response.read(2)
            if buffer != b'\r\n':
                self.exit_event.set()
                raise RuntimeError("Response doesn't end with a newline! (%s)" % (buffer))

            return timestamp, frame, width, height

    def parse_result(self, result):
        jpg, timestamp, width, height = result
        image = self.to_image(jpg)

        if width != self.width or height != self.height:
            image = cv2.resize(image, (self.requested_width, self.requested_height))

            if width != self.width:
                self.logger.info("Size changed to %s, %s" % (width, height))
                self.width = width

            if height != self.height:
                self.logger.info("Size changed to %s, %s" % (width, height))
                self.height = height

        return ImageMessage(image, self.num_messages, timestamp=timestamp)

    def to_image(self, byte_stream):
        t0 = time.time()

        # TODO: SUPER slow... causing a rate of 25 fps to dip to 5 fps
        image = cv2.imdecode(np.fromstring(byte_stream, dtype=np.uint8), 1)
        # image = cv2.imdecode(np.asarray(byte_stream, dtype=np.uint8), 1)
        # image = np.array(Image.open(StringIO(byte_stream)))
        t1 = time.time()
        print(t1 - t0)
        return image
