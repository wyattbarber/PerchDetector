import json
import serial
import time
import argparse


class Interface:
    _ser: serial.Serial
    _user: str
    _password: str

    def __init__(self, port: str, user: str, password: str):
        self._ser = serial.Serial(port, 11250)
        self._user = user
        self._password = password

    def __enter__(self):
        self._ser = self._ser.__enter__()
        self.wait_for("")
        self._ser.write((self._user + "\n").encode())
        self.wait_for("")
        self._ser.write((self._password + "\n").encode())
        self.wait_for("")
        self._ser.write(
            "./PerchDetector/build/main --calibrations ./PerchDetector/calibrations\n".encode()
        )
        self.wait_for(">>>")
        return self
    
    def __exit__(self, *args):
        self._ser.write(
            "exit\n".encode()
        )
        self.wait_for("Perch detector CLI shutdown")
        return self._ser.__exit__(*args)
    
    def wait_for(self, s: str) -> str:
        x = self._ser.read_until(s.encode()).decode()
        i = x.index(s)
        return x[:i]
    
    def start_detector(self) -> bool:
        self._ser.write(
            "autostart detector\n"
        )
        res = self.wait_for(">>>")
        check = "Task detector has been started."
        return res[-len(check):] == check
    
    def get_detection_status(self) -> dict:
        self._ser.write(
            "detector report --json\n"
        )
        return json.loads(self.wait_for(">>>"))
    
    def wait_for_detection(self) -> float:
        valid = False
        ts = time.time()
        while not valid:
            valid = self.get_detection_status()["valid"]
        return time.time() - ts


def parser(args):
    p = argparse.ArgumentParser()
    p.add_argument(
        "port", 
        type=str,
        required=True,
        help="COM port name the device is attached to."
    )
    p.add_argument(
        "user", 
        type=str,
        required=True,
        help="Username for serial terminal login"
    )
    p.add_argument(
        "password", 
        type=str,
        required=True,
        help="Password for serial terminal login."
    )
    p.add_argument(
        "output", 
        type=str,
        required=True,
        help="Output JSON file for results."
    )
    p.add_argument(
        "distance", 
        type=float,
        required=True,
        help="Vertical distance to the test object for this trial, in mm."
    )
    p.add_argument(
        "offset", 
        type=float,
        required=True,
        help="Horizontal distance to the test object for this trial, in mm."
    )
    p.add_argument(
        "elevation", 
        type=float,
        required=True,
        help="Angle between the test object and the camera plane for this trial, in degrees."
    )
    p.add_argument(
        "azimuth", 
        type=float,
        required=True,
        help="Angle between the test object and the camera y axis for this trial, in degrees."
    )
    p.add_argument(
        "width", 
        type=float,
        required=True,
        help="Width of the test object, in mm."
    )
    p.add_argument(
        "collection-time", 
        type=float,
        required=False,
        default=10.0,
        help="Time to run data collection for, in seconds"
    )
    return p.parse_args(args)


if __name__ == "__main__":
    import sys
    import datetime

    args = parser(sys.argv[1:])

    data = {
        "date": str(datetime.date().today()),
        "time": str(datetime.time()),
        "truth": {
            "distance": args.distance,
            "offset": args.offset,
            "elevation": args.elevation,
            "azimuth": args.azimuth,
            "width": args.width
        },
        "observations": {
            "startup_time": 0.0,
            "timestamp": [],
            "distance": [],
            "offset": [],
            "elevation": [],
            "azimuth": [],
            "width":  []
        }
    }
    with Interface(args.port, args.user, args.password) as iface:
        if not iface.start_detector():
            raise RuntimeError("Failed to start detection task.")
        
        data["observations"]["startup_time"] = iface.wait_for_detection()

        ts = time.time()
        while (time.time() - ts) <= args.collection_time:
            observation = iface.get_detection_status()
            data["observations"]["timestamp"].append(time.time())
            data["observations"]["distance"].append(observation["distance"])
            data["observations"]["offset"].append(observation["anchor"][0])
            data["observations"]["elevation"].append(observation["angle_cam_plane"])
            data["observations"]["azimuth"].append(observation["angle_cam_vertical"])
            data["observations"]["width"].append(observation["width"])

    with open(args.output, "w") as file:
        json.dump(data, file, indent=4)