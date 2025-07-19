import mmap
import os
import tempfile
import numpy as np
from nicegui import ui, app
from PIL import Image
import time
import subprocess
import signal
import mmap


class ImageReader:
    def __init__(self, width: int, height: int):
        self._file = tempfile.NamedTemporaryFile(mode="wb", delete=True)
        self._im = np.zeros((height, width), dtype=np.uint8)
        self._data = bytearray(0)

    @property
    def file(self):
        return self._file.name

    def open(self):
        self._file = self._file.__enter__()
        self._file.write(bytes(self._im.size+1))
        self._mm = mmap.mmap(self._file.fileno(), 0)

    def close(self):
        self._file.__exit__(None, None, None)

    def update(self):
        while bool(self._mm[0]):
            # wait for region to be unlocked
            time.sleep(0.001)            
        self._im = np.ndarray(self._im.shape, buffer=self._mm[1:], dtype=np.uint8)

    @property
    def image(self):
        return self._im


class imgen:
    def __init__(self, ui, img, reader):
        self.ui = ui
        self.img = img
        self.reader = reader

    def __call__(self):
        reader.update()
        self.img.set_source(Image.fromarray(reader.image))
        ui.timer(0.1, self, once=True)


class starter:
    def __init__(self, reader, feed, feed_args):
        self.reader = reader
        self.feed = feed
        self.feed_args = feed_args

    def __call__(self):
        self.feed[0] = subprocess.Popen(self.feed_args, stdout = subprocess.PIPE)
        while True:
            line = self.feed[0].stdout.readline().decode()
            print(line)
            if "Started random image feed" in line:
                break

        self.reader.open()


class closer:
    def __init__(self, reader, feed):
        self.reader = reader
        self.feed = feed

    def __call__(self):
        self.reader.close()
        self.feed[0].send_signal(signal.SIGINT)


if __name__ in {"__main__", "__mp_main__"}:
    width = 480
    height = 480

    ui.markdown("Test Page\n=========\n\nThis is a test page for [NiceGUI](https://nicegui.io/).\n")
    
    reader = ImageReader(width, height)
    img = ui.interactive_image(Image.fromarray(np.zeros((100, 100), dtype=np.uint8)))
    ui.timer(1.0, imgen(ui, img, reader), once=True)

    feed = [None]

    app.on_startup(starter(reader, feed, ("./build/testfeed", str(width), str(height), reader.file)))
    app.on_shutdown(closer(reader, feed))

    ui.run()