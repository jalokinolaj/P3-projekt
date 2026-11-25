import serial
import struct
from statistics import median

# Change this to the sensor's serial port.
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 230400
MIN_CONFIDENCE = 100
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
    for target_angle in ((270 + mount_offset) % 360, (90 + mount_offset) % 360):
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

    def start(self):
        """Open the serial port and prepare the scan iterator."""
        if self.ser is not None and self.ser.is_open:
            return
        self.ser = serial.Serial(
            self.serial_port,
            baudrate=self.baud_rate,
            timeout=self.timeout,
        )
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
            scan = self.get_scan()
        return side_distance(
            scan,
            window_degrees,
            self.mount_offset,
            self.min_confidence,
        )

    def front_distance(self, scan=None, window_degrees=10):
        """Return the front distance from a supplied or newly read scan."""
        if scan is None:
            scan = self.get_scan()
        return front_distance(
            scan,
            window_degrees,
            self.mount_offset,
            self.min_confidence,
        )

    def close(self):
        """Close the serial port and reset the scan iterator."""
        if self.ser is not None:
            self.ser.close()
            self.ser = None
        self._scans = None

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


if __name__ == "__main__":
    try:
        with LiDAR_Scan() as scanner:
            for number, scan in enumerate(scanner.read_scans(), start=1):
                print(f"Scan {number}: {len(scan)} points")
    except KeyboardInterrupt:
        print("\nStopped.")
