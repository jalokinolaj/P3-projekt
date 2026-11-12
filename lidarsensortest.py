import serial
import struct

# Change this to your port:
# Linux: '/dev/ttyUSB0' | macOS: '/dev/tty.usbserial-0001' | Windows: 'COM12'
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 230400

def read_ld06():
    ser = serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
    print(f"Reading LD06 on {SERIAL_PORT}... Press Ctrl+C to stop.\n")
    print(f"{'Angle (°)':>10} | {'Distance (mm)':>13} | {'Confidence':>10}")
    print("-" * 40)

    try:
        while True:
            # Find the 2-byte packet header (0x54 0x2C)
            if ser.read(1) == b'\x54' and ser.read(1) == b'\x2c':
                payload = ser.read(45)
                if len(payload) < 45:
                    continue

                # Unpack start and end angles
                _, start_angle = struct.unpack('<HH', payload[0:4])
                end_angle, _, _ = struct.unpack('<HHB', payload[40:45])

                start_deg = start_angle / 100.0
                end_deg = end_angle / 100.0
                angle_step = ((end_deg - start_deg) % 360.0) / 11.0

                # Extract and print all 12 points in the packet
                for i in range(12):
                    offset = 4 + (i * 3)
                    dist_mm, confidence = struct.unpack('<HB', payload[offset:offset+3])
                    angle = (start_deg + i * angle_step) % 360.0
                    
                    # Only print valid readings (distance > 0)
                    if dist_mm > 0:
                        print(f"{angle:10.2f} | {dist_mm:13d} | {confidence:10d}")

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()

if __name__ == '__main__':
    read_ld06()

