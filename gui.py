import mmap
import os
import tempfile
import numpy as np
from nicegui import ui, app
from PIL import Image
import time
import subprocess
import signal

_cv_type_to_numpy = {
    0: np.uint8,
    1: np.int8,
    2: np.uint16,
    3: np.int16,
    4: np.int32,
    5: np.float32,
    6: np.float64
}


class ImageReader:
    def __init__(self, name: str, rescaler = None):
        self._name = name
        self._file = tempfile.NamedTemporaryFile(mode="wb", delete=True)
        self._im = None
        self._data = bytearray(0)
        self._filename = None
        self._valid = False
        self._rescaler = rescaler

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
        self._im = np.ndarray(self._shape, buffer=self._mm[6:], dtype=_cv_type_to_numpy[self._mm[1]])[::2,::2] # Scale images by 1/2
        
    @property
    def image(self):
        if self._rescaler is not None:
           return self._rescaler(self._im)
        else:
            return self._im


class imgen:
    def __init__(self, ui, img, reader):
        self.ui = ui
        self.img = img
        self.reader = reader

    def __call__(self):
        if self.reader.opened:
            self.reader.update()
            self.img.set_source(Image.fromarray(self.reader.image))
        ui.timer(0.1, self, once=True)


class starter:
    def __init__(self, readers, processes, args):
        self.readers = readers
        self.processes = processes
        self.args = args

    def __call__(self):
        self.processes.append(subprocess.Popen(self.args, stdout = subprocess.PIPE))
        self.pidx = len(self.processes) - 1
        while True:
            line = self.processes[self.pidx].stdout.readline().decode()
            print(line)
            retcode = self.processes[self.pidx].poll()
            if retcode is not None:
                raise RuntimeError(f"Image feed terminated with returncode {retcode}")
            elif "Started image feed" in line:
                break
        
        for reader in self.readers:
            reader.open()


class closer:
    def __init__(self, readers, processes):
        self.readers = readers
        self.processes = processes

    def __call__(self):
        for reader in self.readers:
            reader.close()
        for proc in self.processes:
            proc.send_signal(signal.SIGINT)


def make_title(title):
    return ui.markdown(f"{title}\n{''.join(['='] * len(title))}\n\n")


def make_subtitle(subtitle):
    return ui.markdown(f"{subtitle}\n{''.join(['-'] * len(subtitle))}\n\n")


def float_rescale(im):
    out = im.copy()
    out[np.isnan(out)] = 0
    _min = out.min()
    _max = out.max()
    shifted = out - _min
    scaled = shifted / (_max - _min)
    return np.floor(scaled * 255).astype(np.uint8)

if __name__ in {"__main__", "__mp_main__"}:
    make_title("Perch Detector Test and Visualization")
    
    with ui.row():
        with ui.column():            
            make_subtitle("Grayscale Image")
            image_reader = ImageReader("grayscale")
            img = ui.interactive_image(Image.fromarray(np.zeros((100, 100), dtype=np.uint8)))
            ui.timer(1.0, imgen(ui, img, image_reader), once=True)

        with ui.column():            
            make_subtitle("Depth Map")
            depth_reader = ImageReader("depth", float_rescale)
            depth = ui.interactive_image(Image.fromarray(np.zeros((100, 100), dtype=np.uint8)))
            ui.timer(1.0, imgen(ui, depth, depth_reader), once=True)

    processes = []

    app.on_startup(starter((image_reader, depth_reader), processes, ("./build/main", "--image-file", image_reader.file, "--depth-file", depth_reader.file)))
    app.on_shutdown(closer((image_reader, depth_reader), processes))

    ui.run()