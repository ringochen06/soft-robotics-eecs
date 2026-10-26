"""Serial interface to the test bench firmware (firmware/testbench).

The firmware prints these line types:
  I,<name>,<version>,...   firmware info
  H,<col>,<col>,...        column names for the D lines
  D,<val>,<val>,...        one telemetry sample
  OK ... / ERR ...         reply to a command
  # ...                    status message
"""

import queue
import threading
import time

import serial
from serial.tools import list_ports

# USB vendor IDs: Arduino, Arduino.org, SparkFun
ARDUINO_VIDS = {0x2341, 0x2A03, 0x1B4F}


class TestBenchError(RuntimeError):
    pass


def find_port():
    """Return the first port that looks like an Arduino, or None."""
    ports = list(list_ports.comports())
    for p in ports:
        if p.vid in ARDUINO_VIDS:
            return p.device
    for p in ports:
        if any(s in p.device for s in ("usbmodem", "usbserial", "ttyACM", "ttyUSB")):
            return p.device
    return None


def _convert(text):
    if text == "":
        return None
    for cast in (int, float):
        try:
            return cast(text)
        except ValueError:
            pass
    return text


class TestBench:
    """Connection to the board. Samples arrive on `samples` as dicts."""

    def __init__(self, port=None, baud=115200, boot_wait=2.0):
        self.port = port or find_port()
        if not self.port:
            raise TestBenchError("No Arduino found. Plug it in or pass --port.")
        self.ser = serial.Serial(self.port, baud, timeout=0.1)
        # Uno-style boards reset when the port opens; give them time to boot.
        time.sleep(boot_wait)
        self.ser.reset_input_buffer()
        self.columns = None
        self.info = None
        self.samples = queue.Queue()
        self.replies = queue.Queue()
        self.messages = queue.Queue()
        self._stop = threading.Event()
        self._write_lock = threading.Lock()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self.request_info()

    # ------------------------------------------------------------ reading

    def _read_loop(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(256)
            except (serial.SerialException, OSError):
                self.messages.put("# serial connection lost")
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode(errors="replace").strip()
                if line:
                    self._handle(line, time.time())

    def _handle(self, line, t):
        if line.startswith("D,"):
            if not self.columns:
                return
            values = line.split(",")[1:]
            if len(values) != len(self.columns):
                return  # partial line, e.g. right after connecting
            sample = {"host_time": t}
            sample.update({name: _convert(v) for name, v in zip(self.columns, values)})
            self.samples.put(sample)
        elif line.startswith("H,"):
            self.columns = line.split(",")[1:]
        elif line.startswith("I,"):
            self.info = line[2:]
        elif line.startswith("OK") or line.startswith("ERR"):
            self.replies.put(line)
        else:
            self.messages.put(line)

    # ------------------------------------------------------------ writing

    def write(self, text):
        with self._write_lock:
            self.ser.write((text.strip() + "\n").encode())

    def command(self, text, timeout=5.0):
        """Send a command and return its OK reply. Raises TestBenchError on ERR."""
        while not self.replies.empty():
            self.replies.get_nowait()
        self.write(text)
        try:
            reply = self.replies.get(timeout=timeout)
        except queue.Empty:
            raise TestBenchError(f"No reply to {text!r}")
        if reply.startswith("ERR"):
            raise TestBenchError(f"{text!r}: {reply}")
        return reply

    def request_info(self, timeout=5.0):
        self.columns = None
        self.write("INFO")
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.columns:
                return self.info
            time.sleep(0.05)
        raise TestBenchError(
            f"No reply from {self.port}. Is the testbench firmware flashed?"
        )

    def drain(self):
        """Throw away queued samples (e.g. before starting a measurement)."""
        while not self.samples.empty():
            self.samples.get_nowait()

    def collect(self, seconds):
        """Return the samples received over the next `seconds` seconds."""
        self.drain()
        out = []
        deadline = time.time() + seconds
        while time.time() < deadline:
            try:
                out.append(self.samples.get(timeout=0.1))
            except queue.Empty:
                pass
        return out

    def stop_all(self):
        try:
            self.command("STOP", timeout=2.0)
        except TestBenchError:
            pass

    def close(self):
        self.stop_all()
        self._stop.set()
        self._reader.join(timeout=1.0)
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
