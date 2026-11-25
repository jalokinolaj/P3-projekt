import serial
from ld06_scan import read_scans, SERIAL_PORT, BAUD_RATE
from ld06_topo import measured_view

HEADING = 0  # direction the sensor "faces" in compass degrees (0 = north)

ser = serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
print("Printing every 5th scan. Ctrl+C to stop.\n")
try:
    for n, scan in enumerate(read_scans(ser), start=1):
        if n % 5:
            continue
        nearest = min(scan, key=lambda p: p[1])  # closest point (angle, dist, conf)
        view = measured_view(scan, HEADING)
        text = "  ".join(
            f"{d[0].upper()}={round(v):4d}" if v else f"{d[0].upper()}=----"
            for d, v in view.items()
        )
        print(f"{text}  | nearest: {nearest[1]} mm at {nearest[0]:.0f} deg")
except KeyboardInterrupt:
    print("\nStopped.")
finally:
    ser.close()
