import socket
from threading import Thread
import numpy as np
from nicegui import ui
from PIL import Image
import time


class ImageReader:
    def __init__(self, port: int, width: int, height: int):
        self._client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._port = port
        self._th = Thread(target=self._update)
        self._im = np.zeros((height, width))

    def open():
        self._client.connect("localhost", self._port)
        self._th.start()        
        self._run = True

    def close():
        self._run = False
        self._th.join(timeout=1.0)
        self._client.__exit__(None, None, None)

    def _update():
        data = self._client.recv(self._im.size)
        self._im = np.ndarray(self._im.shape, buffer=data)

    @property
    def image(self):
        return self._im


testdoc = """
Test Page
=========

This is a test page for [NiceGUI](https://nicegui.io/).
"""


def imgen(uiimg):
    while True:
        image = Image.fromarray(np.random.randint(0, 255, (100, 100), dtype=np.uint8))
        img.set_source(image)
        img.force_reload()
        time.sleep(0.01)


if __name__ in {"__main__", "__mp_main__"}:
    ui.markdown(testdoc)
    image = Image.fromarray(np.random.randint(0, 255, (100, 100), dtype=np.uint8))
    img = ui.image(image)
    th = Thread(target=imgen, args=[img])
    th.start()
    ui.run()
    th.join(timeout=1.0)