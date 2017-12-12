import time
import json
import base64
import struct
import asyncio
from threading import Lock
from http.client import HTTPConnection

from atlasbuggy import Node


class TimestampWebsiteClient(Node):
    def __init__(self, address=("10.76.76.1", 80), enabled=True):
        super(TimestampWebsiteClient, self).__init__(enabled)
        self.address = address

        self.buffer = b''
        self.reader = None
        self.writer = None

        self.connection = None
        self.response_lock = Lock()

        self.chunk_size = 2

        self.credentials = base64.b64encode(b'robot:naboris').decode('ascii')

    async def setup(self):
        self.connection = HTTPConnection("%s:%s" % (self.address[0], self.address[1]))

    def recv(self, response):
        buf = response.read(self.chunk_size)
        while buf:
            yield buf
            with self.response_lock:
                buf = response.read(self.chunk_size)

    async def loop(self):
        headers = {
            'Content-type': 'image/jpeg',
            'Authorization' : 'Basic %s' % self.credentials
        }
        self.connection.request("GET", "/api/robot/timestamp", headers=headers)
        response = self.connection.getresponse()

        for resp in self.recv(response):
            if len(resp) == 0:
                return

            self.buffer += resp
            response_1 = self.buffer.rfind(b'\xff\xd8')
            response_2 = self.buffer.rfind(b'\xff\xd9')

            if response_1 != -1 and response_2 != -1:
                response_1 += 2
                timestamp = struct.unpack('d', self.buffer[response_1:response_2])[0]
                response_2 += 2
                self.buffer = self.buffer[response_2:]

                print("time diff: %s" % (time.time() - timestamp))
            await asyncio.sleep(0.0)

        self.logger.info("Response ended. Closing.")

    async def teardown(self):
        if self.reader is not None:
            self.reader.close()
        if self.writer is not None:
            self.writer.close()
