import mmap
import os
import traceback
import numpy as np
from nicegui import ui, app, background_tasks, run
from PIL import Image
import sys
import subprocess
import asyncio
import threading
import open3d
from typing import Dict, Any
import cv2
from matplotlib.pyplot import get_cmap
import json
import shutil


_c_to_numpy = {
    "unsigned char": np.uint8,
    "char": np.int8,
    "float": np.float32,
    "double": np.float64,
    "short": np.int16,
    "unsigned short": np.uint16
}

def c_typename_to_numpy(typename: str):
    try:
        idx_low = typename.index('[')
        idx_high = typename.index(']')
        dtype = typename[:idx_low].strip()
        size = typename[idx_low+1:idx_high]
    except Exception as e:
        raise RuntimeError(f"Cannot handle feed type {typename}.")
    return (_c_to_numpy[dtype], size)


class MapReader:
    def __init__(self):
        self._valid = False

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
  
    async def data(self):
        if self._valid:
            # while self._mm[0] != 0:
            #     await asyncio.sleep(0.001)
            return np.ndarray(self._shape, buffer=self._mm[1:], dtype=self._dtype).copy()
        else:
            return np.ones((3, 3)).astype(np.uint8)


class ProcManager:
    def __init__(self, *args: str):
        self._args = args
        self._lock = threading.Lock()

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
        return self._get_lines_out(cmd)

    def _get_lines_out(self, cmd: str):
        out = []
        
        with self._lock:
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
        for line in self._get_lines_out("status"):
            parts = line.split()
            out.append((parts[0], parts[1]))
        return out

    def commands(self, task: str):
        x = self._get_lines_out(f"commands {task}")[0]
        return x.split()
    
    def start_task(self, task: str):
        res = self._get_lines_out(f"autostart {task}")[-1]
        return res == f"Task {task} has been started."

    def stop_task(self, task: str):
        self._get_lines_out(f"autostop {task}")


class FeedSelection:
    def __init__(self, container, refresher, pm: ProcManager):
        with container:
            self._button = ui.button(text="Refresh", on_click=lambda: background_tasks.create(self._refresh()))
            self._feed_select = ui.select(options=[""], on_change=lambda: background_tasks.create(self._configure_feed()))
        self._pm = pm
        self._mem = MapReader()
        self._task_prev = ""
        self._refresher = refresher
    
    async def _refresh(self):
        self._button.disable()
        await self._refresher()
        self._button.enable()

    async def _configure_feed(self):
        task = self._feed_select.value

        if (self._task_prev is not None) and (self._task_prev != ""):
            await asyncio.to_thread(self._pm.get_lines_out, f"{self._task_prev} unmap")

        if (task is not None) and (task != ""):        
            file = f"/tmp/stereo_gui/{task}"
            await asyncio.to_thread(self._pm.get_lines_out, f"{task} map {file}")
            feed_type = (await  asyncio.to_thread(self._pm.get_lines_out, f"{task} datatype"))[0].strip()
            self._dt, s = c_typename_to_numpy(feed_type)
            dims = (await  asyncio.to_thread(self._pm.get_lines_out, f"{task} dimensions"))[0].split()
            self._shape = [int(d) for d in dims]
            self._mem.configure(file, self._dt, self._shape)

            self._task_prev = task
    
    def update_feed_options(self, options):
        self._feed_select.set_options(["", *options])

    def close(self):
        self._mem.close()

    @property
    def feed(self):
        return self._feed_select.value
    
    async def data(self):
        return await self._mem.data()
    
    @property
    def dtype(self):
        return self._dt

    @property
    def shape(self):
        return self._shape


class ResultsViewer:
    _file_path = "/tmp/stereo_gui_results"

    def __init__(self, pm: ProcManager, calibration_path: str):
        self._pm = pm
        self._cmap = get_cmap()
        self._counter_in = 0
        self._counter_out = 0
        self._file = None
        self._mm = None
        self._latest_file = None

        with open(f"{calibration_path}/left.json") as file:
            cal_data_left = json.load(file)
        with open(f"{calibration_path}/right.json") as file:
            cal_data_right = json.load(file)
        with open(f"{calibration_path}/stereo_calibration.json") as file:
            cal_data_stereo = json.load(file)

        _, _, self._Pn1, _, _, _, _ = cv2.stereoRectify(
            np.array(cal_data_left["intrinsic"]).reshape((3,3)), np.array(cal_data_left["distortion"]),
            np.array(cal_data_right["intrinsic"]).reshape((3,3)), np.array(cal_data_right["distortion"]),
            (800,600),
            np.array(cal_data_stereo["rotation"]).reshape((3,3)), np.array(cal_data_stereo["translation"]),
            flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=0
        )

    async def update(self):
        await asyncio.to_thread(self._pm.get_lines_out, f"detector save {self._file_path}/map{self._counter_in}")
        report = await asyncio.to_thread(self._pm.get_lines_out, f"detector report --json")
        with open(f"{self._file_path}/report{self._counter_in}.json", "w") as file:
            file.write("".join(report))

        if hasattr(self, "_file") and hasattr(self._file, "close"):
            self._file.close()
        self._file = open(f"{self._file_path}/map{self._counter_in}", "rb")
        self._counter_in += 1
        return mmap.mmap(self._file.fileno(), 0, prot=mmap.PROT_READ)

    def load(self, data: bytes):
        n = int.from_bytes(data[
            (3*4) + (3*4) : (3*4) + (3*4) + 4
        ], byteorder="little")
        w = int.from_bytes(data[
            (3*4) + (3*4) + 4 + (n * 3 * 4): (3*4) + (3*4) + 4 + (n * 3 * 4) + 4
        ], byteorder="little")
        h = int.from_bytes(data[
            (3*4) + (3*4) + 4 + (n * 3 * 4) + 4 : (3*4) + (3*4) + 4 + (n * 3 * 4) + 4 + 4
        ], byteorder="little")

        start_cloud = (3*4) + (3*4) + 4
        end_cloud = start_cloud + (n * 3 * 4)
        start_left = end_cloud + 4 + 4
        end_left = start_left + (w * h)
        end_right = end_left + (w * h)
        end_confidence = end_right + (w * h)
        end_disparity = end_confidence + (w * h * 2)
        nl = data[end_disparity]
        start_rejects = end_disparity + 1
        start_assignments = start_rejects + nl*4*6
        
        cloud = np.ndarray((n, 3), dtype=np.float32, buffer=data[start_cloud:end_cloud]).transpose()
        left = np.ndarray((h,w), dtype=np.uint8, buffer=data[start_left:end_left])
        ids = np.ndarray((n, 1), dtype=np.int8, buffer=data[start_assignments:])
        anchor = np.ndarray((3, 1), buffer = data[0:3*4], dtype=np.float32)
        dir = np.ndarray((3, 1), buffer = data[3*4:3*4 + 3*8], dtype=np.float32)

        return (left, cloud, ids, nl+1, anchor, dir)

    def render(self, image, pointcloud, line_ids, n_lines, anchor, dir):
        out = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        # Draw clusters of points assigned to lines
        image_points, _= cv2.projectPoints(pointcloud, np.zeros((3,1)), np.zeros((3,1)), self._Pn1[:,:3], np.zeros((1,5)))
        for i in range(pointcloud.shape[1]):
            if line_ids[i,0] >= 0:
                x = int(image_points[i,0,0])
                y = int(image_points[i,0,1])
                out[y,x,:] = (np.asarray(self._cmap(line_ids[i,0] / n_lines)[:3]) * 255).astype(np.uint8)
        # Draw selected line
        pt1 = anchor + dir
        pt2 = anchor - dir        
        image_points, _= cv2.projectPoints(np.stack([pt1, pt2], axis=1), np.zeros((3,1)), np.zeros((3,1)), self._Pn1[:,:3], np.zeros((1,5)))
        cv2.line(out, 
            (int(image_points[0,0,0]), int(image_points[0,0,1])), 
            (int(image_points[1,0,0]), int(image_points[1,0,1])), 
            (255,0,0), 3)
        return out
    
    def save(self, image):
        Image.fromarray(image).resize((800,600)).save(f"{self._file_path}/view{self._counter_out}.png")
        self._latest_file = f"{self._file_path}/view{self._counter_out}.png"
        self._counter_out += 1

    @property
    def latest(self):
        return self._latest_file

    @property
    def report(self):
        return f"{self._file_path}/report{self._counter_in-1}.json"


pm: ProcManager = None
elements: Dict[str, Any] = {}


async def _img_callback():
        global elements
        img = await elements["image_select"].data()
        if elements["image_select"].dtype != np.uint8:
            img = img.astype(np.float64)
            img -= np.min(img)
            m = np.max(img)
            if m > 0:
                img /= m
            img *= 255
        elements["image"].set_source(Image.fromarray(img.astype(np.uint8)).resize((800,600)))

def image_window(container):
    global elements, pm

    with container:
        elements["image_select"] = FeedSelection(ui.row(), _img_callback, pm)
        elements["image"] = ui.interactive_image()
    

async def _data_callback():
    global elements
    elements["data_view"].text = str(await elements["data_select"].data())
    elements["data_view"].update()

def data_window(container):
    global pm, elements

    with container:
        elements["data_select"] = FeedSelection(ui.row(), _data_callback, pm)
        with ui.card().classes("w-full"):
            elements["data_view"] = ui.label()


async def _update_feed_options():
    global elements, pm
    feed_tasks = []
    tasks = await asyncio.to_thread(pm.tasks)
    for task in [x[0] for x in tasks if x[1] == "running"]:
        cmds = await  asyncio.to_thread(pm.commands, task)
        if ("map" in cmds) and ("unmap" in cmds):
            feed_tasks.append(task)
    for item in elements.values():
        if isinstance(item, FeedSelection):
            item.update_feed_options(feed_tasks)

async def _toggle_task(task):
    global elements, pm
    # Toggle state
    _, status = elements["task_status"][task]
    if status == "stopped":
        await asyncio.to_thread(pm.start_task, task)
    else:
        await asyncio.to_thread(pm.stop_task, task)
    # Update all states
    tasks = await  asyncio.to_thread(pm.tasks)
    for t, status in tasks:
        elements["task_status"][t][0].set_content(f"<b>{t}</b>: {status}")
        elements["task_status"][t][1] = status
    # Update other data dependent on task info
    await _update_feed_options()

def _make_task_toggler(task):
    def _inner():
        background_tasks.create(_toggle_task(task))
    return _inner

def task_list(container):
    global pm, elements

    elements["task_status"] = {}

    with container:
        with ui.column():
            for task, status in pm.tasks():
                with ui.card() as card:
                    card.on("click", _make_task_toggler(task))
                    elements["task_status"][task] = [ui.html(f"<b>{task}</b>: {status}"), status]



async def _load_log():
    global elements

    log = await asyncio.to_thread(pm.get_lines_out, "log")
    elements["log_view"].set_content(
        "</br>".join(log)
    )

def log_view(container):
    global pm, elements

    with container:
        elements["log_refresh"] = ui.button(text="Refresh", on_click=_load_log)
        with ui.card().classes("w-full"):
            elements["log_view"] = ui.html()


async def _pt_cloud_callback_get():
    global elements

    return await elements["pt_cloud_select"].data()

def _pt_cloud_callback_filter(x):
    global elements
    data = x.T
    data = data[~np.isnan(data).any(axis=1)]
    data = data[~np.isinf(data).any(axis=1)]

    if data.shape[0] > elements["point_cloud_n_max"]:
        o3p = open3d.geometry.PointCloud()
        o3p.points = open3d.utility.Vector3dVector(data)
        ptc = np.asarray(o3p.farthest_point_down_sample(elements["point_cloud_n_max"]).points)
    else:
        ptc = data
    return (ptc @ elements["point_cloud_rotation"].T) / 100.0 

async def _pt_cloud_callback():
    global elements

    data = await _pt_cloud_callback_get()
    ptc = await run.cpu_bound(_pt_cloud_callback_filter, data)
    if ptc.shape[1] == 3:
        elements["point_cloud"].set_points(ptc, elements["point_cloud_colors"][:ptc.shape[0],:])
        print(f"Filtered cloud shape from {data.shape} to {ptc.shape}")
    else:
        raise RuntimeError(f"Cannot render data of shape {ptc.shape} as point cloud")

def point_cloud_scene(container):
    global pm, elements

    with container:
        elements["pt_cloud_select"] = FeedSelection(ui.row(), _pt_cloud_callback, pm)
        elements["3d_scene"] = ui.scene(background_color='#222').classes("w-full")
        elements["point_cloud"] = elements["3d_scene"].point_cloud([], [], point_size=1)
         # Preallocate point cloud transformation and color matrices
        elements["point_cloud_rotation"] = open3d.geometry.get_rotation_matrix_from_xyz([np.pi/2, 0, -np.pi])
        elements["point_cloud_n_max"] = 1000
        elements["point_cloud_colors"] = np.ones((elements["point_cloud_n_max"], 3))


async def _load_results():
    global elements
    rvd = elements["results_viewer_datasource"]
    rv = elements["results_view"]

    data = await rvd.update()
    img = rvd.render(*rvd.load(data))
    rvd.save(img)

    rv.set_source(rvd.latest)

def _save_results():
    global elements
    ui.download.file(elements["results_viewer_datasource"].latest)
    ui.download.file(elements["results_viewer_datasource"].report)

def results_window(container):
    global pm, elements

    with container:
        elements["results_viewer_datasource"] = ResultsViewer(pm, "calibrations")
        with ui.row():
            elements["results_reload"] = ui.button("Load Detection Results", on_click=_load_results)
            elements["results_save"] = ui.button("Save Detection Results", on_click=_save_results)
        elements["results_view"] = ui.image("").classes("w-full")


def generic_command_scene(container):
    global pm, elements

    with container:
        with ui.card():
            with ui.row():
                elements["generic_command_input"] = ui.input()
                ui.button("Run").on_change(_run_generic_cmd)
        with ui.card():
            elements["generic_command_output"] = ui.label()

async def _run_generic_cmd():
    global pm, elements
    cmd = elements["generic_command_input"].value
    res = await asyncio.to_thread(pm.get_lines_out, cmd)
    elements["generic_command_output"].text = "<br/>".join(out)


def startup():
    global pm
    dirs = ["/tmp/stereo_gui/", "/tmp/stereo_gui_results/"]
    for d in dirs:
        if os.path.exists(d):
            shutil.rmtree(d)
        os.makedirs(d)
    pm.init()


def shutdown():
    global pm
    pm.close()


class handle_exc:
    def __init__(self, container):
        self._container = container
    def __call__(self, e: Exception):
        with self._container:
            ui.notify(str(e))
        traceback.print_exc()


if __name__ in {"__main__", "__mp_main__"}:   
    pm = ProcManager(
        "--calibrations", "calibrations"
    )

    @ui.page('/')
    def main():
        with ui.splitter(value=75).classes("w-full") as splitter:
            task_list(splitter.after)

            with splitter.before:
                with ui.tabs().classes("w-full") as tabs:
                    results = ui.tab("Detection Results")
                    feeds = ui.tab("Image Capture")
                    data = ui.tab("Raw Data")
                    log = ui.tab("Logs")
                    cloud = ui.tab("Point Cloud")
                    command = ui.tab("Command")
                    graph = ui.tab("Graph")
                    rates = ui.tab("Rates")

                with ui.tab_panels(tabs, value=results).classes("w-full"):
                    with ui.tab_panel(results).classes("w-full") as tb:
                        results_window(tb)

                    with ui.tab_panel(feeds).classes("w-full") as tb:
                        image_window(tb)

                    with ui.tab_panel(data).classes("w-full") as tb:
                        data_window(tb)

                    with ui.tab_panel(log).classes("w-full") as tb:
                        log_view(tb)

                    with ui.tab_panel(cloud).classes("w-full") as tb:
                        point_cloud_scene(tb)
                                        
                    with ui.tab_panel(command).classes("w-full") as tb:
                        generic_command_scene(tb)

                    with ui.tab_panel(graph).classes("w-full") as tb:
                        ui.markdown("_To-Do_")
                        
                    with ui.tab_panel(rates).classes("w-full") as tb:
                        ui.markdown("_To-Do_")

        app.on_exception(handle_exc(splitter))

    np.set_printoptions(threshold=20)

    app.on_startup(startup)
    app.on_shutdown(shutdown)
    ui.run(reload=False)