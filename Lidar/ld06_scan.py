import serial
import struct

# CHANGE THIS to your port. Windows: 'COM5' (see Device Manager -> Ports)
SERIAL_PORT = 'COM5'
BAUD_RATE = 230400
MIN_CONFIDENCE = 100  # ignore weaker readings


def crc8(data: bytes) -> int:
    """CRC-8 used by the LD06 (polynomial 0x4D, start value 0)."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def read_packets(ser):
    """Yield (start_angle, points) per valid packet.
    points = list of (angle, dist_mm, confidence)."""
    while True:
        if ser.read(1) != b'\x54':
            continue
        if ser.read(1) != b'\x2c':
            continue

        payload = ser.read(45)
        if len(payload) < 45:
            continue

        # CRC covers header + length byte + first 44 payload bytes
        packet = b'\x54\x2c' + payload
        if crc8(packet[:46]) != packet[46]:
            continue  # corrupted, skip it

        _, start_raw = struct.unpack('<HH', payload[0:4])
        end_raw = struct.unpack('<H', payload[40:42])[0]

        start_deg = start_raw / 100.0
        end_deg = end_raw / 100.0
        step = ((end_deg - start_deg) % 360.0) / 11.0

        points = []
        for i in range(12):
            offset = 4 + i * 3
            dist, conf = struct.unpack('<HB', payload[offset:offset + 3])
            if dist > 0 and conf >= MIN_CONFIDENCE:
                points.append(((start_deg + i * step) % 360.0, dist, conf))
        yield start_deg, points


def read_scans(ser):
    """Yield one full 360-degree scan (a list of points) at a time."""
    scan = []
    last_start = None
    for start_deg, points in read_packets(ser):
        # Angle wrapped around -> a full rotation just finished
        if last_start is not None and start_deg < last_start and scan:
            yield scan
            scan = []
        scan.extend(points)
        last_start = start_deg


if __name__ == '__main__':
    ser = serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
    try:
        for n, scan in enumerate(read_scans(ser), start=1):
            print(f"Scan {n}: {len(scan)} points")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()
