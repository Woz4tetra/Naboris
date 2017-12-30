import time
# import math
# import fcntl
# import array
# import termios
import select
import asyncio
import aioprocessing

from atlasbuggy import Node


class BaseClient(Node):
    def __init__(self, uri, content_type, client, enabled=True):
        super(BaseClient, self).__init__(enabled)

        self.client = client

        self.fps = 30.0
        self.length_sec = 0.0

        self.fps_sum = 0.0
        self.fps_avg = 30.0
        self.prev_t = None
        self.num_messages = 1

        self.response_start_header = b'\xbb\x08'
        self.message_start_header = b'\xde\xad\xbe\xef'

        self.logger.info("Getting URI response '%s'..." % uri)
        response = self.client.get_response(uri, content_type)
        self.logger.info("response: %s" % str(response.__dict__))

        self.process = aioprocessing.AioProcess(target=self.retrieve_messages, args=(response, self.client.connection.sock))
        self.queue = aioprocessing.AioQueue()

        self.exit_event = aioprocessing.AioEvent()

    async def setup(self):
        self.process.start()

    def retrieve_messages(self, response, sock):
        self.wait_for_header(response)

        while True:
            result = self.parse_response(response, sock)
            self.queue.put(result)
            if isinstance(result, BaseException) or result is None:
                self.logger.info("No result parse_response. Exiting.")
                self.close(response)
                return

    def close(self, response):
        self.logger.info("Closing process.")
        self.exit_event.set()
        response.close()
        self.client.close()

    def parse_response(self, response, sock):
        raise NotImplementedError("Please override this method")

    def parse_result(self, result):
        return result

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
                    self.logger.info("Found the header. Proceeding.")
                    return

    def get_buffer(self, response, sock, length):
        t0 = time.time()
        while True:
            ta = time.time()
            readable, writable, exceptional = select.select([sock], [], [], 0.05)
            if len(readable) > 0:
                break
            tb = time.time()

            print(tb - ta)

            if self.exit_event.is_set():
                return True, None
            if time.time() - t0 > 5:
                print("waiting...")
                t0 = time.time()
            time.sleep(0.05)

        try:
            buffer = response.read(length)
        except BaseException as error:
            return True, error

        if len(buffer) == 0:
            self.logger.info("Response ended. Closing.")
            return True, None

        if self.exit_event.is_set():
            self.logger.info("Exit event set. Closing.")
            return True, None

        return False, buffer

    def poll_for_fps(self):
        if self.prev_t is None:
            self.prev_t = time.time()
            return 0.0

        self.length_sec = time.time() - self.start_time
        self.num_messages += 1
        self.fps_sum += 1 / (time.time() - self.prev_t)
        self.fps_avg = self.fps_sum / self.num_messages
        self.prev_t = time.time()

        self.fps = self.fps_avg

    async def loop(self):
        while True:
            if self.exit_event.is_set():
                self.logger.info("Exit event set. Loop exiting.")
                return

            result = await self.queue.coro_get()
            if isinstance(result, BaseException):
                self.logger.info("Result from queue is and error. Exiting.")
                raise result

            self.log_to_buffer(time.time(), "Message received: %s" % str(result))
            self.check_buffer(self.num_messages)

            message = self.parse_result(result)
            if message is not None:
                await self.broadcast(message)
                self.poll_for_fps()
            else:
                await asyncio.sleep(0.0)

    async def teardown(self):
        self.exit_event.set()
        self.logger.info("exit event set")

        self.logger.info("waiting for process...")
        # self.process.delegate.terminate(self.process.delegate)
        await self.process.coro_join()
        self.logger.info("done!")

