import statistics

CELL_MM = 100          # one grid cell = 10 cm
DIRECTIONS = {"north": 0, "east": 90, "south": 180, "west": 270}
STEP = {"north": (-1, 0), "east": (0, 1), "south": (1, 0), "west": (0, -1)}
MOUNT_OFFSET = 0       # degrees between robot front and lidar 0 deg
WINDOW_DEG = 10        # look +/- this many degrees around each direction
MAX_MM = 3000          # cap distances so far-away walls don't dominate
TOLERANCE_MM = 150     # how far off a direction may be and still "match"


def is_wall(grid, r, c):
    """Out of bounds counts as wall. Walls are 1; 0 and node labels are free."""
    if r < 0 or c < 0 or r >= len(grid) or c >= len(grid[0]):
        return True
    return grid[r][c] == 1


def expected_view(grid, r, c):
    """Distance (mm) to the nearest wall in each compass direction, from a cell."""
    view = {}
    for name, (dr, dc) in STEP.items():
        steps = 0
        rr, cc = r + dr, c + dc
        while not is_wall(grid, rr, cc):
            steps += 1
            rr, cc = rr + dr, cc + dc
        # the wall face is half a cell beyond the last free cell
        view[name] = min((steps + 0.5) * CELL_MM, MAX_MM)
    return view


def measured_view(scan, heading_deg):
    """Median lidar distance (mm) per compass direction, given the robot's heading.
    scan is a list of (angle, dist_mm, confidence). Missing directions are None."""
    view = {}
    for name, compass in DIRECTIONS.items():
        target = (compass - heading_deg + MOUNT_OFFSET) % 360
        dists = []
        for angle, dist, _conf in scan:
            diff = abs((angle - target + 180) % 360 - 180)
            if diff <= WINDOW_DEG:
                dists.append(dist)
        view[name] = min(statistics.median(dists), MAX_MM) if dists else None
    return view


def view_error(measured, expected):
    """Average mismatch in mm over the directions we actually measured."""
    errors = [abs(measured[d] - expected[d])
              for d in DIRECTIONS if measured[d] is not None]
    return sum(errors) / len(errors) if errors else None


def node_cells(grid):
    """Find every labelled node cell: {'I3': (row, col), ...}."""
    return {cell: (r, c)
            for r, row in enumerate(grid)
            for c, cell in enumerate(row)
            if isinstance(cell, str)}


def check_node(grid, scan, heading_deg, expected_node):
    """Returns (matches, error_mm) for the node we expect to be at."""
    r, c = node_cells(grid)[expected_node]
    err = view_error(measured_view(scan, heading_deg), expected_view(grid, r, c))
    return err is not None and err <= TOLERANCE_MM, err


def best_matches(grid, scan, heading_deg, top=3):
    """Rank all nodes by how well they match the scan (lowest error first)."""
    measured = measured_view(scan, heading_deg)
    ranked = []
    for name, (r, c) in node_cells(grid).items():
        err = view_error(measured, expected_view(grid, r, c))
        if err is not None:
            ranked.append((err, name))
    return sorted(ranked)[:top]


if __name__ == "__main__":
    import serial
    from ld06_scan import read_scans, SERIAL_PORT, BAUD_RATE

    # TODO: replace with your real grid
    grid = [
        [1, 1, 1, 1, 1],
        [1, "I1", 0, "I2", 1],
        [1, 0, 1, 0, 1],
        [1, "I3", 0, "I4", 1],
        [1, 1, 1, 1, 1],
    ]
    expected_node = "I1"
    heading = 0  # compass heading in degrees

    ser = serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
    try:
        for scan in read_scans(ser):
            ok, err = check_node(grid, scan, heading, expected_node)
            print(f"At {expected_node}? {ok} (error {err:.0f} mm)"
                  if err is not None else "Not enough data")
            print("Closest nodes:", best_matches(grid, scan, heading))
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()
