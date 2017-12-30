import time
import struct

from .base_client import BaseClient


class CommandClient(BaseClient):
    def __init__(self, client, enabled=True):
        super(CommandClient, self).__init__("/cmd", "text/json", client, enabled)

        self.message_len = 4
        self.timestamp_len = 8

        self.endian = "big"

    def parse_response(self, response, sock):
        status, buffer = self.get_buffer(response, sock, len(self.message_start_header))
        if status: return buffer

        if buffer == self.message_start_header:
            status, buffer = self.get_buffer(response, sock, self.message_len)
            if status: return buffer
            command_len = int.from_bytes(buffer, self.endian)
            if command_len == 0:
                return ""

            status, buffer = self.get_buffer(response, sock, self.timestamp_len)
            if status: return buffer
            batch_timestamp = struct.unpack('d', buffer)[0]

            status, buffer = self.get_buffer(response, sock, command_len)
            if status: return buffer
            command_timestamp = struct.unpack('d', buffer[0:8])[0]

            command = buffer[8:].decode()

            print(
                "send diff: %s, recv diff: %s, command: %s" % (
                    batch_timestamp - command_timestamp, time.time() - command_timestamp, command)
            )
            return command
        else:
            return ""
