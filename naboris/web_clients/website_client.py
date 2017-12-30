
class WebsiteConnection:
    def __init__(self, address, port=80, user=None, password=None):
        self.address = address
        self.port = port

        self.use_credentials = user is not None and password is not None

        if self.use_credentials:
            self.credentials = base64.b64encode(b'robot:naboris').decode('ascii')
        else:
            self.credentials = ""

        self.connection = HTTPConnection("%s:%s" % (self.address, self.port))

    def get_response(self, uri, content_type):
        headers = {
            'Content-type':  content_type,
        }
        if self.use_credentials:
            self.connection['Authorization'] = 'Basic %s' % self.credentials

        self.connection.request("GET", uri, headers=headers)
        response = self.connection.getresponse()
