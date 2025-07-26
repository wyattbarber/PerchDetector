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

_cv_type_to_numpy = {
    0: np.uint8,
    1: np.int8,
    2: np.uint16,
    3: np.int16,
    4: np.uint32,
    5: np.int32,
    6: np.float32,
    7: np.float64
}


class ImageReader:
    def __init__(self, name: str):
        self._name = name
        self._file = tempfile.NamedTemporaryFile(mode="wb", delete=True)
        self._im = None
        self._data = bytearray(0)
        self._filename = None
        self._valid = False

    @property
    def file(self):
        if self._filename is None:
            # Generate a filename in the tmp directory
            path = tempfile.gettempdir()
            prefix = tempfile.gettempprefix()
            name = self._name
            self._filename = os.path.join(path, prefix+"_img_stream_mmap_"+self._name)
        return self._filename

    @property
    def opened(self):
        return self._valid

    def open(self):
        self._file = open(self.file, "rb")
        self._mm = mmap.mmap(self._file.fileno(), 0, prot=mmap.PROT_READ)
        self._shape = (
            int.from_bytes(self._mm[2:4], byteorder='little'), 
            int.from_bytes(self._mm[4:6], byteorder='little')
        )
        print(f"Opened memory map for image of size {self._shape}")
        self._valid = True

    def close(self):
        self._valid = False
        self._file.close()

    def update(self):
        while bool(self._mm[0]):
            # wait for region to be unlocked
            time.sleep(0.001)        
        self._im = np.ndarray(self._shape, buffer=self._mm[6:], dtype=_cv_type_to_numpy[self._mm[1]])
    @property
    def image(self):
        return self._im


class imgen:
    def __init__(self, ui, img, reader):
        self.ui = ui
        self.img = img
        self.reader = reader

    def __call__(self):
        if reader.opened:
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
            if "Started image feed" in line:
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
    ui.markdown("Test Page\n=========\n\nThis is a test page.\n")
    
    reader = ImageReader("static")
    img = ui.interactive_image(Image.fromarray(np.zeros((100, 100), dtype=np.uint8)))
    ui.timer(1.0, imgen(ui, img, reader), once=True)

    feed = [None]

    app.on_startup(starter(reader, feed, ("./build/camwrapper", reader.file)))
    app.on_shutdown(closer(reader, feed))

    ui.run()