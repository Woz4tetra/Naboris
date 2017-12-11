import asyncio

from atlasbuggy import Node
from atlasbuggy.opencv.messages import StereoImageMessage


class StereoCam(Node):
    def __init__(self, cam_separation_dist, enabled=True, camera2_is_slow=True, camera1_is_left=True,
                 time_diff=0.001):
        super(StereoCam, self).__init__(enabled)

        self.camera1_tag = "camera1"
        self.camera1_sub = self.define_subscription(self.camera1_tag)
        self.camera1_queue = None

        self.camera2_tag = "camera2"
        self.camera2_sub = self.define_subscription(self.camera2_tag)
        self.camera2_queue = None

        self.cam_separation_dist = cam_separation_dist
        self.time_diff = time_diff
        self.camera2_is_slow = camera2_is_slow
        self.camera1_is_left = camera1_is_left
        self.slow_cam_is_left = camera1_is_left and camera2_is_slow
        self.slow_camera_buffer = []
        self.fast_camera_message = None

    def take(self):
        self.camera1_queue = self.camera1_sub.get_queue()
        self.camera2_queue = self.camera2_sub.get_queue()

    async def loop(self):
        message_counter = 0
        while True:
            while not self.camera1_queue.empty():
                if not self.camera2_is_slow:
                    self.slow_camera_buffer.append(await self.camera1_queue.get())
                else:
                    self.fast_camera_message = await self.camera1_queue.get()
                    assert self.camera1_queue.empty()

            while not self.camera2_queue.empty():
                if self.camera2_is_slow:
                    self.slow_camera_buffer.append(await self.camera2_queue.get())
                else:
                    self.fast_camera_message = await self.camera2_queue.get()
                    assert self.camera2_queue.empty()

            match_index, slow_cam_message = min(
                enumerate(self.slow_camera_buffer),
                lambda image_message: self.fast_camera_message.timestamp - image_message.timestamp
            )
            message_time_diff = self.fast_camera_message.timestamp - slow_cam_message.timestamp
            if abs(message_time_diff) < self.time_diff:
                self.slow_camera_buffer = self.slow_camera_buffer[
                                          match_index + 1:]  # eliminate all images before the index

                if self.slow_cam_is_left:
                    left_image = slow_cam_message.image
                    right_image = self.fast_camera_message.image
                else:
                    right_image = slow_cam_message.image
                    left_image = self.fast_camera_message.image

                stereo_message = StereoImageMessage(left_image, right_image, message_counter, self.cam_separation_dist)
                message_counter += 1

                await self.broadcast(stereo_message)
            else:
                self.logger.warning(
                    "Time difference abs(%ss) is greater than %ss" % (message_time_diff, self.time_diff))
                await asyncio.sleep(0.01)
