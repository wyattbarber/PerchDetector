import mmap
import os
import traceback
import numpy as np
from nicegui import ui, app
from PIL import Image
import time
import subprocess
import threading
import asyncio
from pprint import pformat


_c_to_numpy = {
    "unsigned char": np.uint8,
    "char": np.int8,
    "float": np.float32,
    "double": np.float64,
    "short": np.int16,
    "unsigned short": np.uint16
}

def c_typename_to_numpy(typename: str):
    idx_low = typename.index('[')
    idx_high = typename.index(']')
    dtype = typename[:idx_low].strip()
    size = typename[idx_low+1:idx_high]
    return (_c_to_numpy[dtype], size)


class ImageReader:
    def __init__(self, displaysize):
        self._valid = False
        self._reshape = displaysize

    def configure(self, filename: str, dtype: np.dtype, shape: tuple):
        self._valid = False
        self._file = open(filename, "rb")
        self._mm = mmap.mmap(self._file.fileno(), 0, prot=mmap.PROT_READ)
        self._shape = shape
        self._dtype = dtype
        self._valid = True

    def close(self):
        self._valid = False
        if hasattr(self, "_file") and hasattr(self._file, "close"):
            self._file.close()
        
    @property
    def image(self):
        if self._valid:
            while self._mm[0] != 0:
                pass
            data = np.ndarray(self._shape, buffer=self._mm[1:], dtype=self._dtype).copy()
            if self._dtype != np.uint8:
                # Scale data
                data = data.astype(np.float64)
                data -= np.min(data)
                data /= np.max(data)
                data *= 255
            return Image.fromarray(data.astype(np.uint8)).resize((self._reshape[1], self._reshape[0]))
        else:
            return Image.fromarray(np.zeros((600, 600)).astype(np.uint8))

    @property
    def data_string(self):
        if self._valid:
            while self._mm[0] != 0:
                pass
            return pformat(np.ndarray(self._shape, buffer=self._mm[1:], dtype=self._dtype))
        else:
            return "[]"

    @property
    def raw_data(self):
        if self._valid:
            while self._mm[0] != 0:
                pass
            return np.ndarray(self._shape, buffer=self._mm[1:], dtype=self._dtype).copy()
        else:
            return np.ones((3, 3)).astype(np.uint8)


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

    async def get_lines_out_async(self, cmd: str):
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


class EventThread:
    def __init__(self):
        self._loop = asyncio.new_event_loop()
        self._th = threading.Thread(target=self._loop.run_forever)
        self._th.start()

    def close(self):    
        self._loop.stop()
        self._th.join(timeout=1.0)

    def submit(self, coro):
        asyncio.run_coroutine_threadsafe(coro, self._loop)


class FeedSelection:
    def __init__(self, container, refresher, pm: ProcManager, em: EventThread, mem: ImageReader):
        with container:
            self._button = ui.button(text="Refresh", on_click=refresher)
            self._feed_select = ui.select(options=[""], on_change=self._feed_select_cback)
        self._pm = pm
        self._em = em
        self._mem = mem
        self._task_prev = ""
    
    def _feed_select_cback(self):
        self._em.submit(self._configure_feed())

    async def _configure_feed(self):
        task = self._feed_select.value.strip()

        if self._task_prev != "":
            await self._pm.get_lines_out_async(f"{self._task_prev} unmap")

        if task != "":        
            file = f"/tmp/stereo_gui/{task}"
            await self._pm.get_lines_out_async(f"{task} map {file}")
            feed_type = (await self._pm.get_lines_out_async(f"{task} datatype"))[0]
            dt, s = c_typename_to_numpy(feed_type)
            dims = (await self._pm.get_lines_out_async(f"{task} dimensions"))[0].split()
            shape = [int(d) for d in dims]
            self._mem.configure(file, dt, shape)

        self._task_prev = task
    
    def update_feed_options(self, options):
        self._feed_select.set_options(["", *options])

    @property
    def feed(self):
        return self._feed_select.value


class UIManager:
    def __init__(self, pm: ProcManager, memman: ImageReader, eventman: EventThread):
        self._pm = pm
        self._mem = memman
        self._eventman = eventman
        if not os.path.exists("/tmp/stereo_gui"):
            os.makedirs("/tmp/stereo_gui")
        self._task_prev = ""

    def image_window(self, container):
        with container:
            self._image_select = FeedSelection(ui.row(), lambda: self._image.set_source(self._mem.image), self._pm, self._eventman, self._mem)
            self._image = ui.interactive_image(source=self._mem.image)

    def data_window(self, container):
        with container:
            self._data_select = FeedSelection(ui.row(), self._data_callback, self._pm, self._eventman, self._mem)
            with ui.card().classes("w-full"):
                self._data_view = ui.label()

    async def _update_feed_options(self):
        feed_tasks = []
        for task in [x[0] for x in self._pm.tasks() if x[1] == "running"]:
            cmds = self._pm.commands(task)
            if ("map" in cmds) and ("unmap" in cmds):
                feed_tasks.append(task)
        self._image_select.update_feed_options(feed_tasks)
        self._data_select.update_feed_options(feed_tasks)
        self._pt_cloud_select.update_feed_options(feed_tasks)
   
    def _img_callback(self):
        self._image.set_source(self._mem.image)

    def _data_callback(self):
        self._data_view.text = self._mem.data_string
        self._data_view.update()

    def task_list(self, container):
        self._task_status = {}
        with container:
            with ui.column():
                for task, status in self._pm.tasks():
                    with ui.card() as card:
                        card.on("click", self._make_task_toggler(task))
                        self._task_status[task] = [ui.html(f"<b>{task}</b>: {status}"), status]

    def _make_task_toggler(self, task):
        def _inner():
            self._eventman.submit(self._toggle_task(task))
        return _inner

    async def _toggle_task(self, task):
        # Toggle state
        _, status = self._task_status[task]
        if status == "stopped":
            self._pm.start_task(task)
        else:
            self._pm.stop_task(task)
        # Update all states
        for t, status in self._pm.tasks():
            self._task_status[t][0].set_content(f"<b>{t}</b>: {status}")
            self._task_status[t][1] = status
        # Update other data dependent on task info
        await self._update_feed_options()

    def log_view(self, container):
        with container:
            self._log_refresh = ui.button(text="Refresh", on_click=lambda: self._eventman.submit(self._load_log()))
            with ui.card().classes("w-full"):
                self._log_view = ui.html()

    async def _load_log(self):
        log = await self._pm.get_lines_out_async("log")
        self._log_view.set_content(
            "</br>".join(log)
        )

    def point_cloud_scene(self, container):
        with container:
            self._pt_cloud_select = FeedSelection(ui.row(), self._pt_cloud_callback, self._pm, self._eventman, self._mem)
            self._3d_scene = ui.scene(background_color='#222').classes("w-full")

    def _pt_cloud_callback(self):
        ptc = self._mem.raw_data
        if ptc.shape[1] == 3:
            self._3d_scene.point_cloud(ptc, np.ones(ptc.shape), point_size=0.1)
        else:
            ui.notify(f"Cannot render data of shape {ptc.shape} as point cloud")

class startup():
    def __init__(self, pm: ProcManager, im: ImageReader, um: UIManager, em: EventThread):
        self._pm = pm
        self._im = im
        self._um = um
        self._em = em

    def __call__(self):
        self._pm.init()


class shutdown():
    def __init__(self, pm: ProcManager, im: ImageReader, um: UIManager, em: EventThread):
        self._pm = pm
        self._im = im
        self._um = um
        self._em = em

    def __call__(self):
        self._im.close()
        self._pm.close()
        self._em.close()


def handle_exc(e: Exception):
    ui.notify(str(e))
    traceback.print_exc()


if __name__ in {"__main__", "__mp_main__"}:   
    pm = ProcManager(
        "--calibrations", "calibrations"
    )
    im = ImageReader((600, 800))
    em = EventThread()
    um = UIManager(pm, im, em)

    @ui.page('/')
    def main():
        with ui.splitter(value=75).classes("w-full") as splitter:
            um.task_list(splitter.after)

            with splitter.before:
                with ui.tabs().classes("w-full") as tabs:
                    feeds = ui.tab("Image Capture")
                    data = ui.tab("Raw Data")
                    log = ui.tab("Logs")
                    cloud = ui.tab("Point Cloud")
                    graph = ui.tab("Graph")
                    rates = ui.tab("Rates")

                with ui.tab_panels(tabs, value=feeds).classes("w-full"):
                    with ui.tab_panel(feeds).classes("w-full") as tb:
                        um.image_window(tb)

                    with ui.tab_panel(data).classes("w-full") as tb:
                        um.data_window(tb)

                    with ui.tab_panel(log).classes("w-full") as tb:
                        um.log_view(tb)

                    with ui.tab_panel(cloud).classes("w-full") as tb:
                        um.point_cloud_scene(tb)

                    with ui.tab_panel(graph).classes("w-full") as tb:
                        ui.markdown("_To-Do_")
                        
                    with ui.tab_panel(rates).classes("w-full") as tb:
                        ui.markdown("_To-Do_")

    app.on_startup(startup(pm, im, um, em))
    app.on_shutdown(shutdown(pm, im, um, em))
    app.on_exception(handle_exc)
    ui.run(reload=False)