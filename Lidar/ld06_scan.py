import serial
import struct
from statistics import median
import time
import threading

# Change this to the sensor's serial port.
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 230400
MIN_CONFIDENCE = 150
MOUNT_OFFSET = 270


def crc8(data: bytes) -> int:
    """Calculate the CRC-8 used by the LD06."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def read_packets(ser, min_confidence=MIN_CONFIDENCE):
    """Yield (start_angle, points) for each valid packet."""
    while True:
        if ser.read(1) != b"\x54":
            continue
        if ser.read(1) != b"\x2c":
            continue

        payload = ser.read(45)
        if len(payload) < 45:
            continue

        packet = b"\x54\x2c" + payload
        if crc8(packet[:46]) != packet[46]:
            continue

        _, start_raw = struct.unpack("<HH", payload[0:4])
        end_raw = struct.unpack("<H", payload[40:42])[0]
        start_deg = start_raw / 100.0
        end_deg = end_raw / 100.0
        step = ((end_deg - start_deg) % 360.0) / 11.0

        points = []
        for index in range(12):
            offset = 4 + index * 3
            distance, confidence = struct.unpack("<HB", payload[offset:offset + 3])
            if distance > 0 and confidence >= min_confidence:
                angle = (start_deg + index * step) % 360.0
                points.append((angle, distance, confidence))

        yield start_deg, points


def read_scans(ser, min_confidence=MIN_CONFIDENCE):
    """Yield one full 360-degree scan (a list of points) at a time."""
    scan = []
    last_start = None
    for start_deg, points in read_packets(ser, min_confidence):
        if last_start is not None and start_deg < last_start and scan:
            yield scan
            scan = []
        scan.extend(points)
        last_start = start_deg


def _angle_difference(angle, target):
    return abs((angle - target + 180) % 360 - 180)


def side_distance(
    scan,
    window_degrees=10,
    mount_offset=MOUNT_OFFSET,
    min_confidence=MIN_CONFIDENCE,
):
    """Return (left_mm, right_mm), using median readings around each side."""
    distances_by_side = []
    for target_angle in ((315 + mount_offset) % 360, (45 + mount_offset) % 360):
        distances = [
            distance
            for angle, distance, confidence in scan
            if confidence >= min_confidence
            and _angle_difference(angle, target_angle) <= window_degrees
        ]
        distances_by_side.append(int(median(distances)) if distances else None)
    return tuple(distances_by_side)


def front_distance(
    scan,
    window_degrees=10,
    mount_offset=MOUNT_OFFSET,
    min_confidence=MIN_CONFIDENCE,
):
    """Return the median front distance, or None if there are no readings."""
    target_angle = mount_offset % 360
    distances = [
        distance
        for angle, distance, confidence in scan
        if confidence >= min_confidence
        and _angle_difference(angle, target_angle) <= window_degrees
    ]
    return int(median(distances)) if distances else None


class LiDAR_Scan:
    """Manage the LD06 serial connection and provide scan-reading helpers."""

    def __init__(
        self,
        serial_port=SERIAL_PORT,
        baud_rate=BAUD_RATE,
        min_confidence=MIN_CONFIDENCE,
        mount_offset=MOUNT_OFFSET,
        timeout=1,
    ):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.min_confidence = min_confidence
        self.mount_offset = mount_offset
        self.timeout = timeout
        self.ser = None
        self._scans = None
        self._latest = None
        self._lock = threading.Lock()  # Lock for thread-safe access to the serial port
        self._running = False  # Flag to control the background thread
        self._thread = None  # Background thread for reading scans

    def start(self):
        """Open the serial port and prepare the scan iterator."""
        if self.ser is not None and self.ser.is_open:
            return
        self.ser = serial.Serial(
            self.serial_port,
            baudrate=self.baud_rate,
            timeout=self.timeout,
        )
        self._latest = None
        self._running = True
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()  # Start the background thread to read scans

    def _reader(self):
        try:
            for scan in read_scans(self.ser, self.min_confidence):
                if not self._running:
                    break
                with self._lock:
                    self._latest = scan
        except Exception:
            pass  # Handle exceptions silently for now

    def get_latest_scan(self, wait=1.0):
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            with self._lock:
                if self._latest is not None:
                    return self._latest
            time.sleep(0.1)
        return None
    
    def close(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.5)  # Wait for the background thread to finish
            self._thread = None
        if self.ser is not None:
            self.ser.close()
            self.ser = None
        self._scans = None

    def _require_started(self):
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("LiDAR scanner is not started")
        return self.ser

    def read_packets(self):
        """Yield decoded packets from the open sensor."""
        return read_packets(self._require_started(), self.min_confidence)

    def read_scans(self):
        """Yield complete scans from the open sensor."""
        ser = self._require_started()
        if self._scans is None:
            self._scans = iter(read_scans(ser, self.min_confidence))
        return self._scans

    def get_scan(self):
        """Return the next complete scan."""
        return next(self.read_scans())

    def side_distance(self, scan=None, window_degrees=10):
        """Return (left_mm, right_mm) from a supplied or newly read scan."""
        if scan is None:
            scan = self.get_latest_scan()
        if scan is None:
            return None, None
        return side_distance(
            scan,
            window_degrees,
            self.mount_offset,
            self.min_confidence,
        )

    def front_distance(self, scan=None, window_degrees=10):
        """Return the front distance from a supplied or newly read scan."""
        if scan is None:
            scan = self.get_latest_scan()
        if scan is None:
            return None
        return front_distance(
            scan,
            window_degrees,
            self.mount_offset,
            self.min_confidence,
        )

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


if __name__ == "__main__":
    try:
        with LiDAR_Scan() as scanner:
            while True:
                print(scanner.side_distance())
                time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nStopped.")
