import json
import serial
import time
import argparse
import codecs


class Interface:
    _ser: serial.Serial
    _user: str
    _password: str

    def __init__(self, port: str, user: str, password: str, logfile: str = None):
        self._ser = serial.Serial(port, 115200, timeout=0.1)
        self._user = user
        self._password = password
        self._logfile = logfile

    def __enter__(self):
        self._ser = self._ser.__enter__()
        time.sleep(1.0)
        self._ser.write(b'\x04')
        self.write("\n")
        print("Logging in...")
        self.wait_for("login:")
        self.write(self._user + "\n")
        self.wait_for("Password:")
        self.write(self._password + "\n")
        self.wait_for(":~$")
        print("Starting program...")
        self.write(
            "./PerchDetector/build/main --calibrations ./PerchDetector/calibrations\n"
        )
        self.wait_for(">>>")
        print("Program started.")
        return self
    
    def __exit__(self, *args):
        print("Waiting to close cleanly...")
        self.write("\n")
        self.wait_for(">>>")
        self.write("exit\n")
        print("Exiting program...")
        self.wait_for(":~$")
        print("Logging out...")
        self._ser.write(b'\x04')
        self._ser.__exit__(*args)
    
    def write(self, data: str):
        if self._logfile is not None:
            with open(self._logfile, 'a') as log:
                log.write(data)
        self._ser.write(data.encode())

    def wait_for(self, s: str) -> str:
        x = ""
        d = codecs.getincrementaldecoder("utf-8")()
        while not x.strip().endswith(s):
            b = self._ser.read(10)
            c = d.decode(b)
            if len(c) >= 1:
                x += c
                if self._logfile is not None:
                    with open(self._logfile, 'a') as log:
                        log.write(c)
        i = x.index(s)        
        return x[:i]
    
    def start_detector(self) -> bool:
        self.write("autostart detector\n")
        res = self.wait_for(">>>")
        check = "Task detector has been started."
        i = res.index(check)
        return True
    
    def get_detection_status(self) -> dict:
        self.write("detector report --json\n")
        x = self.wait_for(">>>").strip()
        i = x.index("{")
        return json.loads(x[i:])
    
    def wait_for_detection(self) -> float:
        valid = False
        ts = time.time()
        while not valid:
            valid = self.get_detection_status()["valid"]
            if (time.time() - ts) > (5 * 60):
                raise TimeoutError("Failed to get valid detection in 5 minutes")
        return time.time() - ts


def parser(args):
    p = argparse.ArgumentParser()
    p.add_argument(
        "--port", 
        type=str,
        required=True,
        help="COM port name the device is attached to."
    )
    p.add_argument(
        "--user", 
        type=str,
        required=True,
        help="Username for serial terminal login"
    )
    p.add_argument(
        "--password", 
        type=str,
        required=True,
        help="Password for serial terminal login."
    )
    p.add_argument(
        "--output", 
        type=str,
        required=True,
        help="Output JSON file for results."
    )
    p.add_argument(
        "--distance", 
        type=float,
        required=True,
        help="Vertical distance to the test object for this trial, in mm."
    )
    p.add_argument(
        "--offset", 
        type=float,
        required=True,
        help="Horizontal distance to the test object for this trial, in mm."
    )
    p.add_argument(
        "--elevation", 
        type=float,
        required=True,
        help="Angle between the test object and the camera plane for this trial, in degrees."
    )
    p.add_argument(
        "--azimuth", 
        type=float,
        required=True,
        help="Angle between the test object and the camera y axis for this trial, in degrees."
    )
    p.add_argument(
        "--width", 
        type=float,
        required=True,
        help="Width of the test object, in mm."
    )
    p.add_argument(
        "--collection-time", 
        type=float,
        required=False,
        default=2.0,
        help="Time to run data collection for, in minutes"
    )
    p.add_argument(
        "--serial-log", 
        type=str,
        required=False,
        default=None,
        help="Log file for serial traffic."
    )
    return p.parse_args(args)


if __name__ == "__main__":
    import sys
    import datetime

    args = parser(sys.argv[1:])

    data = {
        "date": str(datetime.date.today()),
        "time": str(datetime.time()),
        "truth": {
            "distance": args.distance,
            "offset": args.offset,
            "elevation": args.elevation,
            "azimuth": args.azimuth,
            "width": args.width
        },
        "observations": {
            "startup_time": -1.0,
            "timestamp": [],
            "distance": [],
            "offset": [],
            "elevation": [],
            "azimuth": [],
            "width":  []
        }
    }
    with Interface(args.port, args.user, args.password, logfile=args.serial_log) as iface:
        if not iface.start_detector():
            raise RuntimeError("Failed to start detection task.")
        print("Started task.")
        
        data["observations"]["startup_time"] = iface.wait_for_detection()
        print("Initial detection valid.")

        ts = time.time()
        while (time.time() - ts) <= (args.collection_time * 60):
            observation = iface.get_detection_status()
            data["observations"]["timestamp"].append(time.time())
            data["observations"]["distance"].append(observation["distance"])
            data["observations"]["offset"].append(observation["anchor"][0])
            data["observations"]["elevation"].append(observation["angle_cam_plane"])
            data["observations"]["azimuth"].append(observation["angle_cam_vertical"])
            data["observations"]["width"].append(observation["width"])

    with open(args.output, "w") as file:
        json.dump(data, file, indent=4)