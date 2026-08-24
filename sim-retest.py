import subprocess
import os
import sys
import glob
import time
import threading


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
        print("Killing simulation")
        self.process.terminate()
        self.process.wait()
            
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

    def __enter__(self):
        self.init()
        return self

    def __exit__(self, *args):
        self.close()


def main(inpath: str, pattern: str, outpath: str):
    files = glob.glob(os.path.join(inpath, pattern))
    print(f"Identified {len(files)} files to retest")
    
    with ProcManager("--calibrations", "./calibrations") as pm:
        print("Started simulation")

        pm.start_task("detector")
        print("Started detector task")

        for src in files:
            basename = os.path.basename(src)
            dst = os.path.join(outpath, basename)

            print("Retesting " + basename)

            pm.get_lines_out("sim-mgr set-source " + src)
            time.sleep(60.0)
            pm.get_lines_out("detector save " + dst)
    

if __name__ == "__main__":
    main(*sys.argv[1:])