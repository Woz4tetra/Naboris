import os
import random
import asyncio
from subprocess import Popen, PIPE, DEVNULL

from atlasbuggy import Node

class Sounds(Node):
    def __init__(self, name, sound_directory, random_sound_folders, enabled=True):
        super(Sounds, self).__init__(enabled)

        self.random_sound_folders = random_sound_folders
        self.directory = os.path.abspath(sound_directory)
        self.file_types = ["wav", "ogg", "mp3"]
        self.tunes = {}
        self.paths = {}

        for dirpath, dirnames, filenames in os.walk(self.directory):
            for tune_name in filenames:
                if any([tune_name.endswith(file_type) for file_type in self.file_types]):
                    ext_index = tune_name.rfind(".")
                    name = os.path.join(dirpath[len(self.directory) + 1:], tune_name[:ext_index])
                    full_path = os.path.join(self.directory, dirpath, tune_name)

                    if os.path.isfile(full_path):
                        self.paths[name] = full_path
                    else:
                        raise FileNotFoundError("Couldn't find the file: '%s'" % full_path)

    def play(self, tune_name):
        if tune_name not in self.tunes:
            self.tunes[tune_name] = Player(self.paths[tune_name])
        self.tunes[tune_name].start()

    def list_sounds(self, directory="."):
        sounds = os.listdir(os.path.join(self.directory, directory))
        result = []
        for tune_name in sounds:
            ext_index = tune_name.rfind(".")
            result.append(os.path.join(directory, tune_name[:ext_index]))

        return result

    def is_playing(self, tune_name):
        return self.tunes[tune_name].is_running()

    def stop(self, tune_name):
        self.tunes[tune_name].stop()

    def stop_all(self):
        for tune_name in self.tunes.keys():
            self.tunes[tune_name].stop()

    async def loop(self):
        self.play_random_sound()
        while True:
            await asyncio.sleep(random.randint(30, 120))
            self.play_random_sound()

    def play_random_sound(self):
        folder = random.choice(self.random_sound_folders)
        sound = random.choice(self.list_sounds(folder))
        self.play(sound)


class Player:
    """
    1       Increase Speed
    2       Decrease Speed
    j       Previous Audio stream
    k       Next Audio stream
    i       Previous Chapter
    o       Next Chapter
    n       Previous Subtitle stream
    m       Next Subtitle stream
    s       Toggle subtitles
    q       Exit OMXPlayer
    Space or p  Pause/Resume
    -       Decrease Volume
    +       Increase Volume
    Left    Seek -30
    Right   Seek +30
    Down    Seek -600
    Up      Seek +600"""

    toggle_command = b'p'
    quit_command = b'q'

    def __init__(self, sound):
        self.sound = sound
        self.process = None
        self.output = None

    def start(self):
        self.stop()
        self.process = Popen(['omxplayer', self.sound], stdin=PIPE,
                             stdout=DEVNULL, close_fds=True, bufsize=0)
        self.output = None

    def is_running(self):
        if self.process is not None:
            self.output = self.process.poll()

        return self.output is None

    def stop(self):
        if not self.is_running():
            self.process = None

        if self.process is not None:
            self.output = 0
            try:
                self.process.stdin.write(Player.quit_command)  # send quit command
                self.process.terminate()
                self.process.wait()  # -> move into background thread if necessary
            except EnvironmentError as e:
                #    logger.error("can't stop %s: %s", self.sound, e)
                pass
            else:
                self.process = None

    def toggle(self):
        p = self.process
        if p is not None:
            try:
                p.stdin.write(Player.toggle_command)  # pause/unpause
            except EnvironmentError as e:
                print("can't toggle %s: %s", self.sound, e)
