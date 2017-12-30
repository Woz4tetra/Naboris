
from .website_client import WebsiteClient

from atlasbuggy import Node

class BaseClient(Node):
    def __init__(self, client, enabled=True):
        
