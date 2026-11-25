"""Live: read the real lidar and say which node it looks like. Needs the sensor."""
import serial
from ld06_scan import read_scans, SERIAL_PORT, BAUD_RATE
from ld06_match import load_grid, rank_nodes, check_node

GRID_FILE = "warehouse_map.csv"
EXPECTED_NODE = "S0"   # the node you put the robot on
HEADING = 0            # compass direction the robot faces (0 = north)

grid = load_grid(GRID_FILE)
ser = serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
print(f"Expecting {EXPECTED_NODE}, heading {HEADING}. Ctrl+C to stop.\n")
try:
    for n, scan in enumerate(read_scans(ser), start=1):
        if n % 5:
            continue
        ok, err = check_node(grid, scan, HEADING, EXPECTED_NODE)
        best = [(round(e), name) for e, name in rank_nodes(grid, scan, HEADING)[:3]]
        err_text = "no data" if err is None else f"{err:.0f} mm"
        print(f"At {EXPECTED_NODE}? {ok} (error {err_text})  closest nodes: {best}")
except KeyboardInterrupt:
    print("\nStopped.")
finally:
    ser.close()
