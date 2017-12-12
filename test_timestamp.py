import asyncio
import numpy as np

from atlasbuggy import Orchestrator, run
from naboris.website_client import TimestampWebsiteClient


class TimestampOrchestrator(Orchestrator):
    def __init__(self, event_loop):
        self.set_default(write=False, level=20)
        super(TimestampOrchestrator, self).__init__(event_loop)

        self.timestamp_client = TimestampWebsiteClient()
        self.add_nodes(self.timestamp_client)

run(TimestampOrchestrator)
