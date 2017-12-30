import base64
import aiohttp
from http.client import HTTPConnection


class WebsiteConnection:
    def __init__(self, address, port=80, user=None, password=None, **connection_kwargs):
        self.address = address
        self.port = port

        self.use_credentials = user is not None and password is not None

        if self.use_credentials:
            user_password = ("%s:%s" % (user, password)).encode()
            self.credentials = base64.b64encode(user_password).decode('ascii')
        else:
            self.credentials = ""

        self.connection = HTTPConnection("%s:%s" % (self.address, self.port), **connection_kwargs)

    def get_response(self, uri, content_type):
        headers = {
            'Content-type': content_type,
        }
        if self.use_credentials:
            headers['Authorization'] = 'Basic %s' % self.credentials

        self.connection.request("GET", uri, headers=headers)
        return self.connection.getresponse()

    def close(self):
        self.connection.close()
