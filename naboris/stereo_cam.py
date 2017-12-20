import time
import asyncio

from atlasbuggy import Node
from atlasbuggy.opencv import OpenCVPipeline, StereoImageMessage


class StereoCam(Node):
    def __init__(self, cam_separation_dist, enabled=True, fast_cam_is_left=False,
                 max_time_diff=0.1):
        super(StereoCam, self).__init__(enabled)

        self.fast_cam_tag = "fast_cam"
        self.fast_cam_sub = self.define_subscription(self.fast_cam_tag)
        self.fast_cam_queue = None

        self.slow_cam_tag = "slow_cam"
        self.slow_cam_sub = self.define_subscription(self.slow_cam_tag)
        self.slow_cam_queue = None

        self.cam_separation_dist = cam_separation_dist
        self.max_time_diff = max_time_diff
        self.fast_cam_is_left = fast_cam_is_left
        self.fast_camera_buffer = []
        self.slow_camera_message = None

        self.fps = 30.0
        self.length_sec = 0.0
        self.fps_sum = 0.0
        self.fps_avg = 0.0
        self.prev_t = None
        self.num_frames = 0

    def take(self):
        self.fast_cam_queue = self.fast_cam_sub.get_queue()
        self.slow_cam_queue = self.slow_cam_sub.get_queue()

    async def loop(self):
        message_counter = 0
        while True:
            self.fast_camera_buffer.append(await self.fast_cam_queue.get())
            while not self.slow_cam_queue.empty():
                self.slow_camera_message = await self.slow_cam_queue.get()

                match_index, matched_fast_message = min(
                    enumerate(self.fast_camera_buffer),
                    key=lambda element: abs(self.slow_camera_message.timestamp - element[1].timestamp)
                )

                message_max_time_diff = matched_fast_message.timestamp - self.slow_camera_message.timestamp
                if abs(message_max_time_diff) < self.max_time_diff:
                    # eliminate all images before the index
                    self.fast_camera_buffer = self.fast_camera_buffer[match_index:]

                    if self.fast_cam_is_left:
                        right_message = matched_fast_message
                        left_message = self.slow_camera_message
                    else:
                        left_message = matched_fast_message
                        right_message = self.slow_camera_message

                    stereo_message = StereoImageMessage(
                        left_message.image, right_message.image, message_counter, self.cam_separation_dist,
                        left_timestamp=left_message.timestamp, right_timestamp=right_message.timestamp
                    )
                    self.log_to_buffer(time.time(), stereo_message)
                    self.check_buffer(message_counter)
                    self.log_to_buffer(time.time(), "slow message diff: %s" % (
                        self.slow_camera_message.timestamp - self.fast_camera_buffer[-1].timestamp)
                    )
                    message_counter += 1
                    await self.broadcast(stereo_message)
                    self.poll_for_fps()
                else:
                    self.logger.warning(
                        "Time difference abs(%ss) is greater than %ss" % (message_max_time_diff, self.max_time_diff))

            await asyncio.sleep(0.01)

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

class StereoStitcher(OpenCVPipeline):
    def __init__(self, enabled=True):
        super(StereoStitcher, self).__init__(enabled)

    async def pipeline(self, message):
        return np.concatenate((message.left_image, message.right_image), axis=1)
