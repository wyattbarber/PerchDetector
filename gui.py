import mmap
import os
import tempfile
import numpy as np
from nicegui import ui, app
from PIL import Image
import time
import subprocess
import signal
import select

_c_to_numpy = {
    "unsigned char": np.uint8,
    "char": np.int8,
    "float": np.float32,
    "double": np.float64,
}

def c_typename_to_numpy(typename: str):
    idx_low = typename.index('[')
    idx_high = typename.index(']')
    dtype = typename[:idx_low].strip()
    size = typename[idx_low+1:idx_high]
    return (_c_to_numpy[dtype], size)


class ImageReader:
    def __init__(self):
        self._valid = False

    def configure(self, filename: str, dtype: np.dtype, shape: tuple):
        self._file = open(filename, "rb")
        self._mm = mmap.mmap(self._file.fileno(), 0, prot=mmap.PROT_READ)
        self._shape = shape
        self._dtype = dtype
        self._valid = True

    def close(self):
        self._valid = False
        self._file.close()
        
    @property
    def image(self):
        if self._valid:
            while self._mm[0] != 0:
                pass
            data = np.ndarray(self._shape, buffer=self._mm[1:], dtype=self._dtype).copy()
            if self._dtype != np.uint8:
                # Scale data
                data -= np.min(data)
                data /= np.max(data)
                data *= 255
            return Image.fromarray(data.astype(np.uint8))
        else:
            return Image.fromarray(np.zeros((600, 600)).astype(np.uint8))


class ProcManager:
    def __init__(self, *args: str):
        self._args = args

    def init(self):
        self.process = subprocess.Popen(
            [os.path.join(os.path.dirname(__file__), "build", "main"), *self._args],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        os.set_blocking(self.process.stdout.fileno(), False)
        waiting = True
        while waiting:
            for line in self.process.stdout:
                if line.strip() == b">>>":
                    waiting = False
                    break
        self.process.stdout.flush()

    def close(self):
        self.process.communicate(input=b"exit\n", timeout=10.0)

    def get_lines_out(self, cmd: str):
        out = []
        self.process.stdin.write((cmd+"\n").encode())
        self.process.stdin.flush()
        waiting = True
        while waiting:
            for line in self.process.stdout:
                if line.strip() == b">>>":
                    waiting = False
                    break
                else:
                    out.append(line.decode())
        return out

    def tasks(self):
        """
        Lists all tasks and their status.
        """
        out = []
        for line in self.get_lines_out("status"):
            parts = line.split()
            out.append((parts[0], parts[1]))
        return out

    def commands(self, task: str):
        return self.get_lines_out(f"commands {task}")[0].split()
    
    def start_task(self, task: str):
        res = self.get_lines_out(f"autostart {task}")[-1]
        return res == f"Task {task} has been started."
    
    def stop_task(self, task: str):
        self.get_lines_out(f"autostop {task}")


class UIManager:
    def __init__(self, pm: ProcManager, memman: ImageReader):
        self._pm = pm
        self._mem = memman
        self._image_path = "/tmp/gui_img_placeholder.png"

    def image_window(self, container):
        self._mem.image.save(self._image_path) 
        with container:
            with ui.column():
                self._button = ui.button(text="Refresh View", on_click=self._img_callback)
                self._image = ui.interactive_image(source=self._image_path)

    def _img_callback(self):
        self._mem.image.save(self._image_path)
        self._image.force_reload()


class startup():
    def __init__(self, pm: ProcManager, im: ImageReader, um: UIManager):
        self._pm = pm
        self._im = im
        self._um = um

    def __call__(self):
        self._pm.init()
        self._pm.start_task("right_feed")
        file = "/tmp/right-feed"
        dt, s = c_typename_to_numpy(pm.get_lines_out(f"right_feed datatype")[0])
        shape = [int(d) for d in pm.get_lines_out("right_feed dimensions")[0].split()]
        self._pm.get_lines_out(f"right_feed map {file}")
        self._im.configure(file, dt, shape)
        print("Configured memory map")


class shutdown():
    def __init__(self, pm: ProcManager, im: ImageReader, um: UIManager):
        self._pm = pm
        self._im = im
        self._um = um

    def __call__(self):
        self._im.close()
        self._pm.close()



if __name__ in {"__main__", "__mp_main__"}:   
    pm = ProcManager()
    im = ImageReader()
    um = UIManager(pm, im)

    @ui.page('/')
    def main():
        um.image_window(ui.row())

    app.on_startup(startup(pm, im, um))
    app.on_shutdown(shutdown(pm, im, um))
    ui.run(reload=False)